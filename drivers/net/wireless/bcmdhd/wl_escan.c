
#if defined(WL_ESCAN)
#include <bcmendian.h>
#include <linux/if_arp.h>
#include <asm/uaccess.h>
#include <wl_android.h>
#include <wl_escan.h>
#include <dhd_config.h>

#define ESCAN_ERROR(name, arg1, args...) \
	do { \
		if (iw_msg_level & WL_ERROR_LEVEL) { \
			printk(KERN_ERR "[dhd-%s] ESCAN-ERROR) %s : " arg1, name, __func__, ## args); \
		} \
	} while (0)
#define ESCAN_TRACE(name, arg1, args...) \
	do { \
		if (iw_msg_level & WL_TRACE_LEVEL) { \
			printk(KERN_ERR "[dhd-%s] ESCAN-TRACE) %s : " arg1, name, __func__, ## args); \
		} \
	} while (0)
#define ESCAN_SCAN(name, arg1, args...) \
	do { \
		if (iw_msg_level & WL_SCAN_LEVEL) { \
			printk(KERN_ERR "[dhd-%s] ESCAN-SCAN) %s : " arg1, name, __func__, ## args); \
		} \
	} while (0)

/* IOCTL swapping mode for Big Endian host with Little Endian dongle.  Default to off */
#define htod32(i) (i)
#define htod16(i) (i)
#define dtoh32(i) (i)
#define dtoh16(i) (i)
#define htodchanspec(i) (i)
#define dtohchanspec(i) (i)
#define WL_EXTRA_BUF_MAX 2048

#define wl_escan_get_buf(a) ((wl_scan_results_t *) (a)->escan_buf)

#define for_each_bss(list, bss, __i)	\
	for (__i = 0; __i < list->count && __i < IW_MAX_AP; __i++, bss = next_bss(list, bss))

#define wl_escan_set_sync_id(a) ((a) = htod16(0x1234))

#ifdef ESCAN_BUF_OVERFLOW_MGMT
#define BUF_OVERFLOW_MGMT_COUNT 3
typedef struct {
	int RSSI;
	int length;
	struct ether_addr BSSID;
} removal_element_t;
#endif /* ESCAN_BUF_OVERFLOW_MGMT */

/* Return a new chanspec given a legacy chanspec
 * Returns INVCHANSPEC on error
 */
static chanspec_t
wl_chspec_from_legacy(chanspec_t legacy_chspec)
{
	chanspec_t chspec;

	/* get the channel number */
	chspec = LCHSPEC_CHANNEL(legacy_chspec);

	/* convert the band */
	if (LCHSPEC_IS2G(legacy_chspec)) {
		chspec |= WL_CHANSPEC_BAND_2G;
	} else {
		chspec |= WL_CHANSPEC_BAND_5G;
	}

	/* convert the bw and sideband */
	if (LCHSPEC_IS20(legacy_chspec)) {
		chspec |= WL_CHANSPEC_BW_20;
	} else {
		chspec |= WL_CHANSPEC_BW_40;
		if (LCHSPEC_CTL_SB(legacy_chspec) == WL_LCHANSPEC_CTL_SB_LOWER) {
			chspec |= WL_CHANSPEC_CTL_SB_L;
		} else {
			chspec |= WL_CHANSPEC_CTL_SB_U;
		}
	}

	if (wf_chspec_malformed(chspec)) {
		ESCAN_ERROR("wlan", "wl_chspec_from_legacy: output chanspec (0x%04X) malformed\n",
			chspec);
		return INVCHANSPEC;
	}

	return chspec;
}

/* Return a legacy chanspec given a new chanspec
 * Returns INVCHANSPEC on error
 */
static chanspec_t
wl_chspec_to_legacy(chanspec_t chspec)
{
	chanspec_t lchspec;

	if (wf_chspec_malformed(chspec)) {
		ESCAN_ERROR("wlan", "wl_chspec_to_legacy: input chanspec (0x%04X) malformed\n",
			chspec);
		return INVCHANSPEC;
	}

	/* get the channel number */
	lchspec = CHSPEC_CHANNEL(chspec);

	/* convert the band */
	if (CHSPEC_IS2G(chspec)) {
		lchspec |= WL_LCHANSPEC_BAND_2G;
	} else {
		lchspec |= WL_LCHANSPEC_BAND_5G;
	}

	/* convert the bw and sideband */
	if (CHSPEC_IS20(chspec)) {
		lchspec |= WL_LCHANSPEC_BW_20;
		lchspec |= WL_LCHANSPEC_CTL_SB_NONE;
	} else if (CHSPEC_IS40(chspec)) {
		lchspec |= WL_LCHANSPEC_BW_40;
		if (CHSPEC_CTL_SB(chspec) == WL_CHANSPEC_CTL_SB_L) {
			lchspec |= WL_LCHANSPEC_CTL_SB_LOWER;
		} else {
			lchspec |= WL_LCHANSPEC_CTL_SB_UPPER;
		}
	} else {
		/* cannot express the bandwidth */
		char chanbuf[CHANSPEC_STR_LEN];
		ESCAN_ERROR("wlan", "wl_chspec_to_legacy: unable to convert chanspec %s "
			"(0x%04X) to pre-11ac format\n",
			wf_chspec_ntoa(chspec, chanbuf), chspec);
		return INVCHANSPEC;
	}

	return lchspec;
}

/* given a chanspec value from the driver, do the endian and chanspec version conversion to
 * a chanspec_t value
 * Returns INVCHANSPEC on error
 */
static chanspec_t
wl_chspec_driver_to_host(int ioctl_ver, chanspec_t chanspec)
{
	chanspec = dtohchanspec(chanspec);
	if (ioctl_ver == 1) {
		chanspec = wl_chspec_from_legacy(chanspec);
	}

	return chanspec;
}

/* given a chanspec value, do the endian and chanspec version conversion to
 * a chanspec_t value
 * Returns INVCHANSPEC on error
 */
static chanspec_t
wl_chspec_host_to_driver(int ioctl_ver, chanspec_t chanspec)
{
	if (ioctl_ver == 1) {
		chanspec = wl_chspec_to_legacy(chanspec);
		if (chanspec == INVCHANSPEC) {
			return chanspec;
		}
	}
	chanspec = htodchanspec(chanspec);

	return chanspec;
}

/* given a channel value, do the endian and chanspec version conversion to
 * a chanspec_t value
 * Returns INVCHANSPEC on error
 */
static chanspec_t
wl_ch_host_to_driver(int ioctl_ver, s32 bssidx, u16 channel)
{
	chanspec_t chanspec;

	chanspec = channel & WL_CHANSPEC_CHAN_MASK;

	if (channel <= CH_MAX_2G_CHANNEL)
		chanspec |= WL_CHANSPEC_BAND_2G;
	else
		chanspec |= WL_CHANSPEC_BAND_5G;

	chanspec |= WL_CHANSPEC_BW_20;

	chanspec |= WL_CHANSPEC_CTL_SB_NONE;

	return wl_chspec_host_to_driver(ioctl_ver, chanspec);
}

static inline struct wl_bss_info *next_bss(struct wl_scan_results *list,
	struct wl_bss_info *bss)
{
	return bss = bss ?
		(struct wl_bss_info *)((uintptr) bss + dtoh32(bss->length)) : list->bss_info;
}

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

static int
rssi_to_qual(int rssi)
{
	if (rssi <= WL_IW_RSSI_NO_SIGNAL)
		return 0;
	else if (rssi <= WL_IW_RSSI_VERY_LOW)
		return 1;
	else if (rssi <= WL_IW_RSSI_LOW)
		return 2;
	else if (rssi <= WL_IW_RSSI_GOOD)
		return 3;
	else if (rssi <= WL_IW_RSSI_VERY_GOOD)
		return 4;
	else
		return 5;
}

static s32
wl_escan_inform_bss(struct wl_escan_info *escan)
{
	struct wl_scan_results *bss_list;
	s32 err = 0;
#if defined(RSSIAVG)
	int rssi;
#endif

	bss_list = escan->bss_list;

	/* Delete disconnected cache */
#if defined(BSSCACHE)
	wl_delete_disconnected_bss_cache(&escan->g_bss_cache_ctrl,
		(u8*)&escan->disconnected_bssid);
#if defined(RSSIAVG)
	wl_delete_disconnected_rssi_cache(&escan->g_rssi_cache_ctrl,
		(u8*)&escan->disconnected_bssid);
#endif
#endif

	/* Update cache */
#if defined(RSSIAVG)
	wl_update_rssi_cache(&escan->g_rssi_cache_ctrl, bss_list);
	if (!in_atomic())
		wl_update_connected_rssi_cache(escan->dev, &escan->g_rssi_cache_ctrl, &rssi);
#endif
#if defined(BSSCACHE)
	wl_update_bss_cache(&escan->g_bss_cache_ctrl,
#if defined(RSSIAVG)
		&escan->g_rssi_cache_ctrl,
#endif
		bss_list);
#endif

	/* delete dirty cache */
#if defined(RSSIAVG)
	wl_delete_dirty_rssi_cache(&escan->g_rssi_cache_ctrl);
	wl_reset_rssi_cache(&escan->g_rssi_cache_ctrl);
#endif
#if defined(BSSCACHE)
	wl_delete_dirty_bss_cache(&escan->g_bss_cache_ctrl);
	wl_reset_bss_cache(&escan->g_bss_cache_ctrl);
	if (escan->autochannel)
		wl_ext_get_best_channel(escan->dev, &escan->g_bss_cache_ctrl,
			escan->ioctl_ver &escan->best_2g_ch, &escan->best_5g_ch);
#else
	if (escan->autochannel)
		wl_ext_get_best_channel(escan->dev, bss_list, escan->ioctl_ver,
			&escan->best_2g_ch, &escan->best_5g_ch);
#endif

	ESCAN_TRACE(escan->dev->name, "scanned AP count (%d)\n", bss_list->count);

	return err;
}

static wl_scan_params_t *
wl_escan_alloc_params(struct wl_escan_info *escan, int channel,
	int nprobes, int *out_params_size)
{
	wl_scan_params_t *params;
	int params_size;
	int num_chans;
	int bssidx = 0;

	*out_params_size = 0;

	/* Our scan params only need space for 1 channel and 0 ssids */
	params_size = WL_SCAN_PARAMS_FIXED_SIZE + 1 * sizeof(uint16);
	params = (wl_scan_params_t*) kzalloc(params_size, GFP_KERNEL);
	if (params == NULL) {
		ESCAN_ERROR(escan->dev->name, "mem alloc failed (%d bytes)\n", params_size);
		return params;
	}
	memset(params, 0, params_size);
	params->nprobes = nprobes;

	num_chans = (channel == 0) ? 0 : 1;

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = DOT11_SCANTYPE_ACTIVE;
	params->nprobes = htod32(1);
	params->active_time = htod32(-1);
	params->passive_time = htod32(-1);
	params->home_time = htod32(10);
	if (channel == -1)
		params->channel_list[0] = htodchanspec(channel);
	else
		params->channel_list[0] = wl_ch_host_to_driver(escan->ioctl_ver, bssidx, channel);

	/* Our scan params have 1 channel and 0 ssids */
	params->channel_num = htod32((0 << WL_SCAN_PARAMS_NSSID_SHIFT) |
		(num_chans & WL_SCAN_PARAMS_COUNT_MASK));

	*out_params_size = params_size;	/* rtn size to the caller */
	return params;
}

static void
wl_escan_abort(struct wl_escan_info *escan)
{
	wl_scan_params_t *params = NULL;
	s32 params_size = 0;
	s32 err = BCME_OK;
	if (!in_atomic()) {
		/* Our scan params only need space for 1 channel and 0 ssids */
		params = wl_escan_alloc_params(escan, -1, 0, &params_size);
		if (params == NULL) {
			ESCAN_ERROR(escan->dev->name, "scan params allocation failed \n");
			err = -ENOMEM;
		} else {
			/* Do a scan abort to stop the driver's scan engine */
			err = wldev_ioctl(escan->dev, WLC_SCAN, params, params_size, true);
			if (err < 0) {
				ESCAN_ERROR(escan->dev->name, "scan abort  failed \n");
			}
			kfree(params);
		}
	}
}

static s32
wl_escan_notify_complete(struct wl_escan_info *escan, bool fw_abort)
{
	s32 err = BCME_OK;
	int cmd = 0;
#if WIRELESS_EXT > 13
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];

	memset(extra, 0, sizeof(extra));
#endif

	ESCAN_TRACE(escan->dev->name, "Enter\n");

	if (fw_abort && !in_atomic())
		wl_escan_abort(escan);

	if (timer_pending(&escan->scan_timeout))
		del_timer_sync(&escan->scan_timeout);
#if defined(ESCAN_RESULT_PATCH)
	escan->bss_list = wl_escan_get_buf(escan);
	wl_escan_inform_bss(escan);
#endif /* ESCAN_RESULT_PATCH */

#if WIRELESS_EXT > 13
#if WIRELESS_EXT > 14
	cmd = SIOCGIWSCAN;
#endif
	ESCAN_TRACE(escan->dev->name, "event WLC_E_SCAN_COMPLETE\n");
	// terence 20150224: fix "wlan0: (WE) : Wireless Event too big (65306)"
	memset(&wrqu, 0, sizeof(wrqu));
	if (cmd) {
		if (cmd == SIOCGIWSCAN) {
			wireless_send_event(escan->dev, cmd, &wrqu, NULL);
		} else
			wireless_send_event(escan->dev, cmd, &wrqu, extra);
	}
#endif

	return err;
}

#ifdef ESCAN_BUF_OVERFLOW_MGMT
static void
wl_escan_find_removal_candidate(struct wl_escan_info *escan,
	wl_bss_info_t *bss, removal_element_t *candidate)
{
	int idx;
	for (idx = 0; idx < BUF_OVERFLOW_MGMT_COUNT; idx++) {
		int len = BUF_OVERFLOW_MGMT_COUNT - idx - 1;
		if (bss->RSSI < candidate[idx].RSSI) {
			if (len)
				memcpy(&candidate[idx + 1], &candidate[idx],
					sizeof(removal_element_t) * len);
			candidate[idx].RSSI = bss->RSSI;
			candidate[idx].length = bss->length;
			memcpy(&candidate[idx].BSSID, &bss->BSSID, ETHER_ADDR_LEN);
			return;
		}
	}
}

static void
wl_escan_remove_lowRSSI_info(struct wl_escan_info *escan,
	wl_scan_results_t *list, removal_element_t *candidate, wl_bss_info_t *bi)
{
	int idx1, idx2;
	int total_delete_len = 0;
	for (idx1 = 0; idx1 < BUF_OVERFLOW_MGMT_COUNT; idx1++) {
		int cur_len = WL_SCAN_RESULTS_FIXED_SIZE;
		wl_bss_info_t *bss = NULL;
		if (candidate[idx1].RSSI >= bi->RSSI)
			continue;
		for (idx2 = 0; idx2 < list->count; idx2++) {
			bss = bss ? (wl_bss_info_t *)((uintptr)bss + dtoh32(bss->length)) :
				list->bss_info;
			if (!bcmp(&candidate[idx1].BSSID, &bss->BSSID, ETHER_ADDR_LEN) &&
				candidate[idx1].RSSI == bss->RSSI &&
				candidate[idx1].length == dtoh32(bss->length)) {
				u32 delete_len = dtoh32(bss->length);
				ESCAN_TRACE(escan->dev->name,
					"delete scan info of %pM to add new AP\n", &bss->BSSID);
				if (idx2 < list->count -1) {
					memmove((u8 *)bss, (u8 *)bss + delete_len,
						list->buflen - cur_len - delete_len);
				}
				list->buflen -= delete_len;
				list->count--;
				total_delete_len += delete_len;
				/* if delete_len is greater than or equal to result length */
				if (total_delete_len >= bi->length) {
					return;
				}
				break;
			}
			cur_len += dtoh32(bss->length);
		}
	}
}
#endif /* ESCAN_BUF_OVERFLOW_MGMT */

s32
wl_escan_handler(struct wl_escan_info *escan,
	const wl_event_msg_t *e, void *data)
{
	s32 err = BCME_OK;
	s32 status = ntoh32(e->status);
	wl_bss_info_t *bi;
	wl_escan_result_t *escan_result;
	wl_bss_info_t *bss = NULL;
	wl_scan_results_t *list;
	u32 bi_length;
	u32 i;
	u16 channel;

	ESCAN_TRACE(escan->dev->name, "enter event type : %d, status : %d \n",
		ntoh32(e->event_type), ntoh32(e->status));

	mutex_lock(&escan->usr_sync);
	escan_result = (wl_escan_result_t *)data;

	if (escan->escan_state != ESCAN_STATE_SCANING) {
		ESCAN_TRACE(escan->dev->name, "Not my scan\n");
		goto exit;
	}

	if (status == WLC_E_STATUS_PARTIAL) {
		ESCAN_TRACE(escan->dev->name, "WLC_E_STATUS_PARTIAL \n");
		if (!escan_result) {
			ESCAN_ERROR(escan->dev->name, "Invalid escan result (NULL pointer)\n");
			goto exit;
		}
		if (dtoh16(escan_result->bss_count) != 1) {
			ESCAN_ERROR(escan->dev->name, "Invalid bss_count %d: ignoring\n",
				escan_result->bss_count);
			goto exit;
		}
		bi = escan_result->bss_info;
		if (!bi) {
			ESCAN_ERROR(escan->dev->name, "Invalid escan bss info (NULL pointer)\n");
			goto exit;
		}
		bi_length = dtoh32(bi->length);
		if (bi_length != (dtoh32(escan_result->buflen) - WL_ESCAN_RESULTS_FIXED_SIZE)) {
			ESCAN_ERROR(escan->dev->name, "Invalid bss_info length %d: ignoring\n",
				bi_length);
			goto exit;
		}

		/* +++++ terence 20130524: skip invalid bss */
		channel =
			bi->ctl_ch ? bi->ctl_ch :
			CHSPEC_CHANNEL(wl_chspec_driver_to_host(escan->ioctl_ver, bi->chanspec));
		if (!dhd_conf_match_channel(escan->pub, channel))
			goto exit;
		/* ----- terence 20130524: skip invalid bss */

		{
			int cur_len = WL_SCAN_RESULTS_FIXED_SIZE;
#ifdef ESCAN_BUF_OVERFLOW_MGMT
			removal_element_t candidate[BUF_OVERFLOW_MGMT_COUNT];
			int remove_lower_rssi = FALSE;

			bzero(candidate, sizeof(removal_element_t)*BUF_OVERFLOW_MGMT_COUNT);
#endif /* ESCAN_BUF_OVERFLOW_MGMT */

			list = wl_escan_get_buf(escan);
#ifdef ESCAN_BUF_OVERFLOW_MGMT
			if (bi_length > ESCAN_BUF_SIZE - list->buflen)
				remove_lower_rssi = TRUE;
#endif /* ESCAN_BUF_OVERFLOW_MGMT */

			ESCAN_TRACE(escan->dev->name, "%s(%pM) RSSI %d flags 0x%x length %d\n",
				bi->SSID, &bi->BSSID, bi->RSSI, bi->flags, bi->length);
			for (i = 0; i < list->count; i++) {
				bss = bss ? (wl_bss_info_t *)((uintptr)bss + dtoh32(bss->length))
					: list->bss_info;
#ifdef ESCAN_BUF_OVERFLOW_MGMT
				ESCAN_TRACE(escan->dev->name,
					"%s(%pM), i=%d bss: RSSI %d list->count %d\n",
					bss->SSID, &bss->BSSID, i, bss->RSSI, list->count);

				if (remove_lower_rssi)
					wl_escan_find_removal_candidate(escan, bss, candidate);
#endif /* ESCAN_BUF_OVERFLOW_MGMT */
				if (!bcmp(&bi->BSSID, &bss->BSSID, ETHER_ADDR_LEN) &&
						(CHSPEC_BAND(wl_chspec_driver_to_host(escan->ioctl_ver, bi->chanspec))
						== CHSPEC_BAND(wl_chspec_driver_to_host(escan->ioctl_ver, bss->chanspec))) &&
						bi->SSID_len == bss->SSID_len &&
						!bcmp(bi->SSID, bss->SSID, bi->SSID_len)) {

					/* do not allow beacon data to update
					*the data recd from a probe response
					*/
					if (!(bss->flags & WL_BSS_FLAGS_FROM_BEACON) &&
						(bi->flags & WL_BSS_FLAGS_FROM_BEACON))
						goto exit;

					ESCAN_TRACE(escan->dev->name,
						"%s(%pM), i=%d prev: RSSI %d flags 0x%x, "
						"new: RSSI %d flags 0x%x\n",
						bss->SSID, &bi->BSSID, i, bss->RSSI, bss->flags,
						bi->RSSI, bi->flags);

					if ((bss->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL) ==
						(bi->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL)) {
						/* preserve max RSSI if the measurements are
						* both on-channel or both off-channel
						*/
						ESCAN_TRACE(escan->dev->name,
							"%s(%pM), same onchan, RSSI: prev %d new %d\n",
							bss->SSID, &bi->BSSID, bss->RSSI, bi->RSSI);
						bi->RSSI = MAX(bss->RSSI, bi->RSSI);
					} else if ((bss->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL) &&
						(bi->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL) == 0) {
						/* preserve the on-channel rssi measurement
						* if the new measurement is off channel
						*/
						ESCAN_TRACE(escan->dev->name,
							"%s(%pM), prev onchan, RSSI: prev %d new %d\n",
							bss->SSID, &bi->BSSID, bss->RSSI, bi->RSSI);
						bi->RSSI = bss->RSSI;
						bi->flags |= WL_BSS_FLAGS_RSSI_ONCHANNEL;
					}
					if (dtoh32(bss->length) != bi_length) {
						u32 prev_len = dtoh32(bss->length);

						ESCAN_TRACE(escan->dev->name,
							"bss info replacement occured(bcast:%d->probresp%d)\n",
							bss->ie_length, bi->ie_length);
						ESCAN_TRACE(escan->dev->name,
							"%s(%pM), replacement!(%d -> %d)\n",
							bss->SSID, &bi->BSSID, prev_len, bi_length);

						if (list->buflen - prev_len + bi_length
							> ESCAN_BUF_SIZE) {
							ESCAN_ERROR(escan->dev->name,
								"Buffer is too small: keep the previous result "
								"of this AP\n");
							/* Only update RSSI */
							bss->RSSI = bi->RSSI;
							bss->flags |= (bi->flags
								& WL_BSS_FLAGS_RSSI_ONCHANNEL);
							goto exit;
						}

						if (i < list->count - 1) {
							/* memory copy required by this case only */
							memmove((u8 *)bss + bi_length,
								(u8 *)bss + prev_len,
								list->buflen - cur_len - prev_len);
						}
						list->buflen -= prev_len;
						list->buflen += bi_length;
					}
					list->version = dtoh32(bi->version);
					memcpy((u8 *)bss, (u8 *)bi, bi_length);
					goto exit;
				}
				cur_len += dtoh32(bss->length);
			}
			if (bi_length > ESCAN_BUF_SIZE - list->buflen) {
#ifdef ESCAN_BUF_OVERFLOW_MGMT
				wl_escan_remove_lowRSSI_info(escan, list, candidate, bi);
				if (bi_length > ESCAN_BUF_SIZE - list->buflen) {
					ESCAN_TRACE(escan->dev->name,
						"RSSI(%pM) is too low(%d) to add Buffer\n",
						&bi->BSSID, bi->RSSI);
					goto exit;
				}
#else
				ESCAN_ERROR(escan->dev->name, "Buffer is too small: ignoring\n");
				goto exit;
#endif /* ESCAN_BUF_OVERFLOW_MGMT */
			}

			if (strlen(bi->SSID) == 0) { // terence: fix for hidden SSID
				ESCAN_SCAN(escan->dev->name, "Skip hidden SSID %pM\n", &bi->BSSID);
				goto exit;
			}

			memcpy(&(((char *)list)[list->buflen]), bi, bi_length);
			list->version = dtoh32(bi->version);
			list->buflen += bi_length;
			list->count++;
		}
	}
	else if (status == WLC_E_STATUS_SUCCESS) {
		escan->escan_state = ESCAN_STATE_IDLE;
		ESCAN_TRACE(escan->dev->name, "ESCAN COMPLETED\n");
		escan->bss_list = wl_escan_get_buf(escan);
		ESCAN_TRACE(escan->dev->name, "SCAN COMPLETED: scanned AP count=%d\n",
			escan->bss_list->count);
		wl_escan_inform_bss(escan);
		wl_escan_notify_complete(escan, false);
	} else if ((status == WLC_E_STATUS_ABORT) || (status == WLC_E_STATUS_NEWSCAN) ||
		(status == WLC_E_STATUS_11HQUIET) || (status == WLC_E_STATUS_CS_ABORT) ||
		(status == WLC_E_STATUS_NEWASSOC)) {
		/* Handle all cases of scan abort */
		escan->escan_state = ESCAN_STATE_IDLE;
		ESCAN_TRACE(escan->dev->name, "ESCAN ABORT reason: %d\n", status);
		escan->bss_list = wl_escan_get_buf(escan);
		ESCAN_TRACE(escan->dev->name, "SCAN ABORT: scanned AP count=%d\n",
			escan->bss_list->count);
		wl_escan_inform_bss(escan);
		wl_escan_notify_complete(escan, false);
	} else if (status == WLC_E_STATUS_TIMEOUT) {
		ESCAN_ERROR(escan->dev->name, "WLC_E_STATUS_TIMEOUT\n");
		ESCAN_ERROR(escan->dev->name, "reason[0x%x]\n", e->reason);
		if (e->reason == 0xFFFFFFFF) {
			wl_escan_notify_complete(escan, true);
		}
		escan->escan_state = ESCAN_STATE_IDLE;
	} else {
		ESCAN_ERROR(escan->dev->name, "unexpected Escan Event %d : abort\n", status);
		escan->escan_state = ESCAN_STATE_IDLE;
		escan->bss_list = wl_escan_get_buf(escan);
		ESCAN_TRACE(escan->dev->name, "SCAN ABORTED(UNEXPECTED): scanned AP count=%d\n",
			escan->bss_list->count);
		wl_escan_inform_bss(escan);
		wl_escan_notify_complete(escan, false);
	}
exit:
	mutex_unlock(&escan->usr_sync);
	return err;
}

static int
wl_escan_prep(struct wl_escan_info *escan, wl_uint32_list_t *list,
	wl_scan_params_t *params, wlc_ssid_t *ssid)
{
	int err = 0;
	wl_scan_results_t *results;
	s32 offset;
	char *ptr;
	int i = 0, j = 0;
	wlc_ssid_t ssid_tmp;
	u32 n_channels = 0;
	uint channel;
	chanspec_t chanspec;

	results = wl_escan_get_buf(escan);
	results->version = 0;
	results->count = 0;
	results->buflen = WL_SCAN_RESULTS_FIXED_SIZE;
	escan->escan_state = ESCAN_STATE_SCANING;

	/* Arm scan timeout timer */
	mod_timer(&escan->scan_timeout, jiffies + msecs_to_jiffies(WL_ESCAN_TIMER_INTERVAL_MS));

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = 0;
	params->nprobes = -1;
	params->active_time = -1;
	params->passive_time = -1;
	params->home_time = -1;
	params->channel_num = 0;

	params->nprobes = htod32(params->nprobes);
	params->active_time = htod32(params->active_time);
	params->passive_time = htod32(params->passive_time);
	params->home_time = htod32(params->home_time);

	n_channels = dtoh32(list->count);
	/* Copy channel array if applicable */
	ESCAN_SCAN(escan->dev->name, "### List of channelspecs to scan ###\n");
	if (n_channels > 0) {
		for (i = 0; i < n_channels; i++) {
			channel = dtoh32(list->element[i]);
			if (!dhd_conf_match_channel(escan->pub, channel))
				continue;
			chanspec = WL_CHANSPEC_BW_20;
			if (chanspec == INVCHANSPEC) {
				ESCAN_ERROR(escan->dev->name, "Invalid chanspec! Skipping channel\n");
				continue;
			}
			if (channel <= CH_MAX_2G_CHANNEL) {
				chanspec |= WL_CHANSPEC_BAND_2G;
			} else {
				chanspec |= WL_CHANSPEC_BAND_5G;
			}
			params->channel_list[j] = channel;
			params->channel_list[j] &= WL_CHANSPEC_CHAN_MASK;
			params->channel_list[j] |= chanspec;
			ESCAN_SCAN(escan->dev->name, "Chan : %d, Channel spec: %x\n",
				channel, params->channel_list[j]);
			params->channel_list[j] = wl_chspec_host_to_driver(escan->ioctl_ver,
				params->channel_list[j]);
			j++;
		}
	} else {
		ESCAN_SCAN(escan->dev->name, "Scanning all channels\n");
	}

	if (ssid && ssid->SSID_len) {
		/* Copy ssid array if applicable */
		ESCAN_SCAN(escan->dev->name, "### List of SSIDs to scan ###\n");
		offset = offsetof(wl_scan_params_t, channel_list) + n_channels * sizeof(u16);
		offset = roundup(offset, sizeof(u32));
		ptr = (char*)params + offset;

		ESCAN_SCAN(escan->dev->name, "0: Broadcast scan\n");
		memset(&ssid_tmp, 0, sizeof(wlc_ssid_t));
		ssid_tmp.SSID_len = 0;
		memcpy(ptr, &ssid_tmp, sizeof(wlc_ssid_t));
		ptr += sizeof(wlc_ssid_t);

		memset(&ssid_tmp, 0, sizeof(wlc_ssid_t));
		ssid_tmp.SSID_len = ssid->SSID_len;
		memcpy(ssid_tmp.SSID, ssid->SSID, ssid->SSID_len);
		memcpy(ptr, &ssid_tmp, sizeof(wlc_ssid_t));
		ptr += sizeof(wlc_ssid_t);
		ESCAN_SCAN(escan->dev->name, "1: scan for %s size=%d\n",
			ssid_tmp.SSID, ssid_tmp.SSID_len);
		/* Adding mask to channel numbers */
		params->channel_num =
	        htod32((2 << WL_SCAN_PARAMS_NSSID_SHIFT) |
	               (n_channels & WL_SCAN_PARAMS_COUNT_MASK));
	}
	else {
		ESCAN_SCAN(escan->dev->name, "Broadcast scan\n");
	}

	return err;
}

static int
wl_escan_reset(struct wl_escan_info *escan)
{
	if (timer_pending(&escan->scan_timeout))
		del_timer_sync(&escan->scan_timeout);
	escan->escan_state = ESCAN_STATE_IDLE;

	return 0;
}

static void
wl_escan_timeout(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	struct timer_list *t
#else
	unsigned long data
#endif
)
{
	wl_event_msg_t msg;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	struct wl_escan_info *escan = from_timer(escan, t, scan_timeout);
#else
	struct wl_escan_info *escan = (struct wl_escan_info *)data;
#endif
	struct wl_scan_results *bss_list;
	struct wl_bss_info *bi = NULL;
	s32 i;
	u32 channel;

	if (!escan->dev) {
		ESCAN_ERROR("wlan", "No dev present\n");
		return;
	}

	bss_list = wl_escan_get_buf(escan);
	if (!bss_list) {
		ESCAN_ERROR(escan->dev->name,
			"bss_list is null. Didn't receive any partial scan results\n");
	} else {
		ESCAN_ERROR(escan->dev->name, "scanned AP count (%d)\n", bss_list->count);
		bi = next_bss(bss_list, bi);
		for_each_bss(bss_list, bi, i) {
			channel = wf_chspec_ctlchan(wl_chspec_driver_to_host(escan->ioctl_ver,
				bi->chanspec));
			ESCAN_ERROR(escan->dev->name, "SSID :%s  Channel :%d\n", bi->SSID, channel);
		}
	}

	bzero(&msg, sizeof(wl_event_msg_t));
	ESCAN_ERROR(escan->dev->name, "timer expired\n");

	msg.event_type = hton32(WLC_E_ESCAN_RESULT);
	msg.status = hton32(WLC_E_STATUS_TIMEOUT);
	msg.reason = 0xFFFFFFFF;
	wl_ext_event_send(escan->pub->event_params, &msg, NULL);
}

int
wl_escan_set_scan(struct net_device *dev, struct wl_escan_info *escan,
	struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	s32 err = BCME_OK;
	s32 params_size = (WL_SCAN_PARAMS_FIXED_SIZE + OFFSETOF(wl_escan_params_t, params));
	wl_escan_params_t *params = NULL;
	scb_val_t scbval;
	static int cnt = 0;
	wlc_ssid_t ssid;
	u32 n_channels = 0;
	wl_uint32_list_t *list;
	u8 valid_chan_list[sizeof(u32)*(WL_NUMCHANNELS + 1)];

	ESCAN_TRACE(escan->dev->name, "Enter \n");

	if (escan->escan_state == ESCAN_STATE_DOWN) {
		ESCAN_ERROR(escan->dev->name, "STATE is down\n");
		return -EIO;
	}
	mutex_lock(&escan->usr_sync);

	/* default Broadcast scan */
	memset(&ssid, 0, sizeof(ssid));

#if WIRELESS_EXT > 17
	/* check for given essid */
	if (wrqu->data.length == sizeof(struct iw_scan_req)) {
		if (wrqu->data.flags & IW_SCAN_THIS_ESSID) {
			struct iw_scan_req *req = (struct iw_scan_req *)extra;
			ssid.SSID_len = MIN(sizeof(ssid.SSID), req->essid_len);
			memcpy(ssid.SSID, req->essid, ssid.SSID_len);
			ssid.SSID_len = htod32(ssid.SSID_len);
		}
	}
#endif
	if (escan->escan_state == ESCAN_STATE_SCANING) {
		ESCAN_ERROR(escan->dev->name, "Scanning already\n");
		goto exit;
	}

	/* if scan request is not empty parse scan request paramters */
	memset(valid_chan_list, 0, sizeof(valid_chan_list));
	list = (wl_uint32_list_t *)(void *) valid_chan_list;
	list->count = htod32(WL_NUMCHANNELS);
	err = wldev_ioctl(escan->dev, WLC_GET_VALID_CHANNELS, valid_chan_list,
		sizeof(valid_chan_list), false);
	if (err != 0) {
		ESCAN_ERROR(escan->dev->name, "get channels failed with %d\n", err);
		goto exit;
	}
	n_channels = dtoh32(list->count);
	/* Allocate space for populating ssids in wl_escan_params_t struct */
	if (dtoh32(list->count) % 2)
		/* If n_channels is odd, add a padd of u16 */
		params_size += sizeof(u16) * (n_channels + 1);
	else
		params_size += sizeof(u16) * n_channels;
	if (ssid.SSID_len) {
		params_size += sizeof(struct wlc_ssid) * 2;
	}

	params = (wl_escan_params_t *) kzalloc(params_size, GFP_KERNEL);
	if (params == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	wl_escan_prep(escan, list, &params->params, &ssid);

	params->version = htod32(ESCAN_REQ_VERSION);
	params->action =  htod16(WL_SCAN_ACTION_START);
	wl_escan_set_sync_id(params->sync_id);
	if (params_size + sizeof("escan") >= WLC_IOCTL_MEDLEN) {
		ESCAN_ERROR(escan->dev->name, "ioctl buffer length not sufficient\n");
		kfree(params);
		err = -ENOMEM;
		goto exit;
	}
	params->params.scan_type = DOT11_SCANTYPE_ACTIVE;
	ESCAN_TRACE(escan->dev->name, "Passive scan_type %d\n", params->params.scan_type);

	WL_MSG(dev->name, "LEGACY_SCAN\n");
	err = wldev_iovar_setbuf(dev, "escan", params, params_size,
		escan->escan_ioctl_buf, WLC_IOCTL_MEDLEN, NULL);
	if (unlikely(err)) {
		if (err == BCME_EPERM)
			/* Scan Not permitted at this point of time */
			ESCAN_TRACE(escan->dev->name, "Escan not permitted at this time (%d)\n", err);
		else
			ESCAN_ERROR(escan->dev->name, "Escan set error (%d)\n", err);
		wl_escan_reset(escan);
	}
	kfree(params);

exit:
	if (unlikely(err)) {
		/* Don't print Error incase of Scan suppress */
		if ((err == BCME_EPERM))
			ESCAN_TRACE(escan->dev->name, "Escan failed: Scan Suppressed \n");
		else {
			cnt++;
			ESCAN_ERROR(escan->dev->name, "error (%d), cnt=%d\n", err, cnt);
			// terence 20140111: send disassoc to firmware
			if (cnt >= 4) {
				memset(&scbval, 0, sizeof(scb_val_t));
				wldev_ioctl(dev, WLC_DISASSOC, &scbval, sizeof(scb_val_t), true);
				ESCAN_ERROR(escan->dev->name, "Send disassoc to break the busy\n");
				cnt = 0;
			}
		}
	} else {
		cnt = 0;
	}
	mutex_unlock(&escan->usr_sync);
	return err;
}

static int
wl_escan_merge_scan_results(struct wl_escan_info *escan,
	struct iw_request_info *info, char *extra, wl_bss_info_t *bi, int *len, int max_size)
{
	s32 err = BCME_OK;
	struct iw_event	iwe;
	int j;
	char *event = extra, *end = extra + max_size - WE_ADD_EVENT_FIX, *value;
	int16 rssi;
	int channel;

	/* overflow check cover fields before wpa IEs */
	if (event + ETHER_ADDR_LEN + bi->SSID_len + IW_EV_UINT_LEN + IW_EV_FREQ_LEN +
		IW_EV_QUAL_LEN >= end) {
		err = -E2BIG;
		goto exit;
	}

#if defined(RSSIAVG)
	rssi = wl_get_avg_rssi(&escan->g_rssi_cache_ctrl, &bi->BSSID);
	if (rssi == RSSI_MINVAL)
		rssi = MIN(dtoh16(bi->RSSI), RSSI_MAXVAL);
#else
	// terence 20150419: limit the max. rssi to -2 or the bss will be filtered out in android OS
	rssi = MIN(dtoh16(bi->RSSI), RSSI_MAXVAL);
#endif
	channel = wf_chspec_ctlchan(wl_chspec_driver_to_host(escan->ioctl_ver, bi->chanspec));
	ESCAN_SCAN(escan->dev->name, "BSSID=%pM, channel=%3d, RSSI=%2d, SSID=\"%s\"\n",
		&bi->BSSID, channel, rssi, bi->SSID);

	/* First entry must be the BSSID */
	iwe.cmd = SIOCGIWAP;
	iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
	memcpy(iwe.u.ap_addr.sa_data, &bi->BSSID, ETHER_ADDR_LEN);
	event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_ADDR_LEN);

	/* SSID */
	iwe.u.data.length = dtoh32(bi->SSID_len);
	iwe.cmd = SIOCGIWESSID;
	iwe.u.data.flags = 1;
	event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, bi->SSID);

	/* Mode */
	if (dtoh16(bi->capability) & (DOT11_CAP_ESS | DOT11_CAP_IBSS)) {
		iwe.cmd = SIOCGIWMODE;
		if (dtoh16(bi->capability) & DOT11_CAP_ESS)
			iwe.u.mode = IW_MODE_INFRA;
		else
			iwe.u.mode = IW_MODE_ADHOC;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_UINT_LEN);
	}

	/* Channel */
	iwe.cmd = SIOCGIWFREQ;
#if 1
	iwe.u.freq.m = wf_channel2mhz(channel, channel <= CH_MAX_2G_CHANNEL ?
			WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
#else
	iwe.u.freq.m = wf_channel2mhz(bi->n_cap ?
			bi->ctl_ch : CHSPEC_CHANNEL(bi->chanspec),
			CHSPEC_CHANNEL(bi->chanspec) <= CH_MAX_2G_CHANNEL ?
			WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
#endif
	iwe.u.freq.e = 6;
	event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_FREQ_LEN);

	/* Channel quality */
	iwe.cmd = IWEVQUAL;
	iwe.u.qual.qual = rssi_to_qual(rssi);
	iwe.u.qual.level = 0x100 + rssi;
	iwe.u.qual.noise = 0x100 + bi->phy_noise;
	event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_QUAL_LEN);

	wl_iw_handle_scanresults_ies(&event, end, info, bi);

	/* Encryption */
	iwe.cmd = SIOCGIWENCODE;
	if (dtoh16(bi->capability) & DOT11_CAP_PRIVACY)
		iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
	else
		iwe.u.data.flags = IW_ENCODE_DISABLED;
	iwe.u.data.length = 0;
	event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)event);

	/* Rates */
	if (bi->rateset.count <= sizeof(bi->rateset.rates)) {
		if (event + IW_MAX_BITRATES*IW_EV_PARAM_LEN >= end) {
			err = -E2BIG;
			goto exit;
		}
		value = event + IW_EV_LCP_LEN;
		iwe.cmd = SIOCGIWRATE;
		/* Those two flags are ignored... */
		iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
		for (j = 0; j < bi->rateset.count && j < IW_MAX_BITRATES; j++) {
			iwe.u.bitrate.value = (bi->rateset.rates[j] & 0x7f) * 500000;
			value = IWE_STREAM_ADD_VALUE(info, event, value, end, &iwe,
				IW_EV_PARAM_LEN);
		}
		event = value;
	}
	*len = event - extra;
	if (*len < 0)
		ESCAN_ERROR(escan->dev->name, "==> Wrong size\n");

exit:
	return err;
}

int
wl_escan_get_scan(struct net_device *dev, struct wl_escan_info *escan,
	struct iw_request_info *info, struct iw_point *dwrq, char *extra)
{
	s32 err = BCME_OK;
	int i = 0;
	int len_prep = 0, len_ret = 0;
	wl_bss_info_t *bi = NULL;
	struct wl_scan_results *bss_list;
	__u16 buflen_from_user = dwrq->length;
#if defined(BSSCACHE)
	wl_bss_cache_t *node;
#endif
	char *buf = NULL;
	struct ether_addr cur_bssid;

	ESCAN_TRACE(escan->dev->name, "SIOCGIWSCAN, len=%d\n", dwrq->length);

	if (!extra)
		return -EINVAL;

	mutex_lock(&escan->usr_sync);

	/* Check for scan in progress */
	if (escan->escan_state == ESCAN_STATE_SCANING) {
		ESCAN_TRACE(escan->dev->name, "SIOCGIWSCAN GET still scanning\n");
		err = -EAGAIN;
		goto exit;
	}
	if (!escan->bss_list) {
		ESCAN_ERROR(escan->dev->name, "scan not ready\n");
		err = -EAGAIN;
		goto exit;
	}

	err = wldev_ioctl(dev, WLC_GET_BSSID, &cur_bssid, sizeof(cur_bssid), false);
	if (err != BCME_NOTASSOCIATED && memcmp(&ether_null, &cur_bssid, ETHER_ADDR_LEN)) {
		// merge current connected bss
		buf = kzalloc(WL_EXTRA_BUF_MAX, GFP_ATOMIC);
		if (!buf) {
			ESCAN_ERROR(escan->dev->name, "buffer alloc failed.\n");
			err = BCME_NOMEM;
			goto exit;
		}
		*(u32 *)buf = htod32(WL_EXTRA_BUF_MAX);
		err = wldev_ioctl(dev, WLC_GET_BSS_INFO, buf, WL_EXTRA_BUF_MAX, false);
		if (unlikely(err)) {
			ESCAN_ERROR(escan->dev->name, "Could not get bss info %d\n", err);
			goto exit;
		}
		bi = (struct wl_bss_info *)(buf + 4);
		len_prep = 0;
		err = wl_escan_merge_scan_results(escan, info, extra+len_ret, bi,
			&len_prep, buflen_from_user-len_ret);
		len_ret += len_prep;
		if (err)
			goto exit;
		bi = NULL;
	}

#if defined(BSSCACHE)
	bss_list = &escan->g_bss_cache_ctrl.m_cache_head->results;
	node = escan->g_bss_cache_ctrl.m_cache_head;
	for (i=0; node && i<IW_MAX_AP; i++)
#else
	bss_list = escan->bss_list;
	bi = next_bss(bss_list, bi);
	for_each_bss(bss_list, bi, i)
#endif
	{
#if defined(BSSCACHE)
		bi = node->results.bss_info;
#endif
		if (!memcmp(&bi->BSSID, &cur_bssid, ETHER_ADDR_LEN)) {
			ESCAN_SCAN(escan->dev->name, "skip connected AP %pM\n", &cur_bssid);
#if defined(BSSCACHE)
			node = node->next;
#endif
			continue;
		}
		len_prep = 0;
		err = wl_escan_merge_scan_results(escan, info, extra+len_ret, bi,
			&len_prep, buflen_from_user-len_ret);
		len_ret += len_prep;
		if (err)
			goto exit;
#if defined(BSSCACHE)
		node = node->next;
#endif
	}

	if ((len_ret + WE_ADD_EVENT_FIX) < dwrq->length)
		dwrq->length = len_ret;

	dwrq->flags = 0;	/* todo */
exit:
	kfree(buf);
	dwrq->length = len_ret;
	ESCAN_SCAN(escan->dev->name, "scanned AP count (%d)\n", i);
	mutex_unlock(&escan->usr_sync);
	return err;
}

static void
wl_escan_deinit(struct wl_escan_info *escan)
{
	ESCAN_TRACE(escan->dev->name, "Enter\n");

	del_timer_sync(&escan->scan_timeout);
	escan->escan_state = ESCAN_STATE_DOWN;

#if defined(RSSIAVG)
	wl_free_rssi_cache(&escan->g_rssi_cache_ctrl);
#endif
#if defined(BSSCACHE)
	wl_free_bss_cache(&escan->g_bss_cache_ctrl);
#endif
}

static s32
wl_escan_init(struct wl_escan_info *escan)
{
	ESCAN_TRACE(escan->dev->name, "Enter\n");

	/* Init scan_timeout timer */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	timer_setup(&escan->scan_timeout, wl_escan_timeout, 0);
#else
	init_timer(&escan->scan_timeout);
	escan->scan_timeout.data = (unsigned long) escan;
	escan->scan_timeout.function = wl_escan_timeout;
#endif
	escan->escan_state = ESCAN_STATE_IDLE;

	return 0;
}

void
wl_escan_down(struct wl_escan_info *escan)
{
	ESCAN_TRACE(escan->dev->name, "Enter\n");
	wl_escan_deinit(escan);
}

int
wl_escan_up(struct net_device *net, struct wl_escan_info *escan)
{
	s32 val = 0;
	int err;

	ESCAN_TRACE(escan->dev->name, "Enter\n");
	err = wl_escan_init(escan);
	if (err) {
		ESCAN_ERROR(escan->dev->name, "wl_escan_init err %d\n", err);
		return err;
	}

	if (!escan->ioctl_ver) {
		val = 1;
		if ((err = wldev_ioctl(net, WLC_GET_VERSION, &val, sizeof(int), false) < 0)) {
			ESCAN_ERROR(escan->dev->name, "WLC_GET_VERSION failed, err=%d\n", err);
			return err;
		}
		val = dtoh32(val);
		if (val != WLC_IOCTL_VERSION && val != 1) {
			ESCAN_ERROR(escan->dev->name,
				"Version mismatch, please upgrade. Got %d, expected %d or 1\n",
				val, WLC_IOCTL_VERSION);
			return err;
		}
		escan->ioctl_ver = val;
	}

	return 0;
}

void
wl_escan_detach(struct wl_escan_info *escan)
{
	ESCAN_TRACE(escan->dev->name, "Enter\n");

	wl_escan_deinit(escan);
	if (escan->escan_ioctl_buf) {
		kfree(escan->escan_ioctl_buf);
		escan->escan_ioctl_buf = NULL;
	}
}

int
wl_escan_attach(struct net_device *dev, dhd_pub_t *dhdp,
	struct wl_escan_info *escan)
{
	int ret = 0;

	ESCAN_TRACE(escan->dev->name, "Enter\n");

	/* we only care about main interface so save a global here */
	escan->dev = dev;
	escan->pub = dhdp;
	escan->escan_state = ESCAN_STATE_DOWN;

	escan->escan_ioctl_buf = (void *)kzalloc(WLC_IOCTL_MAXLEN, GFP_KERNEL);
	if (unlikely(!escan->escan_ioctl_buf)) {
		ESCAN_ERROR(escan->dev->name, "Ioctl buf alloc failed\n");
		ret = -ENOMEM;
		goto exit;
	}
	ret = wl_escan_init(escan);
	if (ret) {
		ESCAN_ERROR(escan->dev->name, "wl_escan_init err %d\n", ret);
		goto exit;
	}
	mutex_init(&escan->usr_sync);

	return 0;

exit:
	wl_escan_detach(escan);
	return ret;
}

#endif /* WL_ESCAN */

