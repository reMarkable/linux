/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define _IOCTL_LINUX_C_

#include <net/iw_handler.h>
#include <linux/if_arp.h>

#include <drv_types.h>
#include <rtw_ap.h>

#include <odm_precomp.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

#define _drv_always_		1
#undef DBG_871X_LEVEL
static inline void DBG_871X_LEVEL(const int level, const char *fmt, ...)
{
}

#define RTL_IOCTL_WPA_SUPPLICANT	(SIOCIWFIRSTPRIV+30)

#define SCAN_ITEM_SIZE 768
#define MAX_CUSTOM_LEN 64
#define RATE_COUNT 4

/* combo scan */
#define WEXT_CSCAN_AMOUNT 9
#define WEXT_CSCAN_BUF_LEN		360
#define WEXT_CSCAN_HEADER		"CSCAN S\x01\x00\x00S\x00"
#define WEXT_CSCAN_HEADER_SIZE		12
#define WEXT_CSCAN_SSID_SECTION		'S'
#define WEXT_CSCAN_CHANNEL_SECTION	'C'
#define WEXT_CSCAN_NPROBE_SECTION	'N'
#define WEXT_CSCAN_ACTV_DWELL_SECTION	'A'
#define WEXT_CSCAN_PASV_DWELL_SECTION	'P'
#define WEXT_CSCAN_HOME_DWELL_SECTION	'H'
#define WEXT_CSCAN_TYPE_SECTION		'T'


extern uint8_t key_2char2num(uint8_t hch, uint8_t lch);
extern uint8_t str_2char2num(uint8_t hch, uint8_t lch);
extern uint8_t convert_ip_addr(uint8_t hch, uint8_t mch, uint8_t lch);

u32 rtw_rates[] = {	1000000,
			2000000,
			5500000,
			11000000,
			6000000,
			9000000,
			12000000,
			18000000,
			24000000,
			36000000,
			48000000,
			54000000};

static const char * const iw_operation_mode[] = {
	"Auto", "Ad-Hoc", "Managed",  "Master", "Repeater", "Secondary", "Monitor"
};

static int hex2num_i(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

static int hex2byte_i(const char *hex)
{
	int a, b;
	a = hex2num_i(*hex++);
	if (a < 0)
		return -1;
	b = hex2num_i(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}

/**
 * hwaddr_aton - Convert ASCII string to MAC address
 * @txt: MAC address as a string (e.g., "00:11:22:33:44:55")
 * @addr: Buffer for the MAC address (ETH_ALEN = 6 bytes)
 * Returns: 0 on success, -1 on failure (e.g., string not a MAC address)
 */
static int hwaddr_aton_i(const char *txt, uint8_t *addr)
{
	int i;

	for (i = 0; i < 6; i++) {
		int a, b;

		a = hex2num_i(*txt++);
		if (a < 0)
			return -1;
		b = hex2num_i(*txt++);
		if (b < 0)
			return -1;
		*addr++ = (a << 4) | b;
		if (i < 5 && *txt++ != ':')
			return -1;
	}

	return 0;
}

static void indicate_wx_custom_event(struct rtl_priv *rtlpriv, char *msg)
{
	uint8_t *buff;
	union iwreq_data wrqu;

	if (strlen(msg) > IW_CUSTOM_MAX) {
		DBG_871X("%s strlen(msg):%zu > IW_CUSTOM_MAX:%u\n", __FUNCTION__ , strlen(msg), IW_CUSTOM_MAX);
		return;
	}

	buff = rtw_zmalloc(IW_CUSTOM_MAX+1);
	if (!buff)
		return;

	memcpy(buff, msg, strlen(msg));

	memset(&wrqu, 0, sizeof(wrqu));
	wrqu.data.length = strlen(msg);

	DBG_871X("%s %s\n", __FUNCTION__, buff);
	wireless_send_event(rtlpriv->ndev, IWEVCUSTOM, &wrqu, buff);

	rtw_mfree(buff);
}


static void request_wps_pbc_event(struct rtl_priv *rtlpriv)
{
	uint8_t *buff, *p;
	union iwreq_data wrqu;


	buff = rtw_malloc(IW_CUSTOM_MAX);
	if (!buff)
		return;

	memset(buff, 0, IW_CUSTOM_MAX);

	p = buff;

	p += sprintf(p, "WPS_PBC_START.request=true");

	memset(&wrqu, 0, sizeof(wrqu));

	wrqu.data.length = p-buff;

	wrqu.data.length = (wrqu.data.length < IW_CUSTOM_MAX) ? wrqu.data.length:IW_CUSTOM_MAX;

	DBG_871X("%s\n", __FUNCTION__);

	wireless_send_event(rtlpriv->ndev, IWEVCUSTOM, &wrqu, buff);

	if (buff) {
		rtw_mfree(buff);
	}

}

void indicate_wx_scan_complete_event(struct rtl_priv *rtlpriv)
{
	union iwreq_data wrqu;

	memset(&wrqu, 0, sizeof(union iwreq_data));

	/* DBG_871X("+rtw_indicate_wx_scan_complete_event\n"); */
	wireless_send_event(rtlpriv->ndev, SIOCGIWSCAN, &wrqu, NULL);
}


void rtw_indicate_wx_assoc_event(struct rtl_priv *rtlpriv)
{
	union iwreq_data wrqu;
	struct	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;

	memset(&wrqu, 0, sizeof(union iwreq_data));

	wrqu.ap_addr.sa_family = ARPHRD_ETHER;

	memcpy(wrqu.ap_addr.sa_data, pmlmepriv->cur_network.network.MacAddress, ETH_ALEN);
	DBG_871X("BSSID:" MAC_FMT "\n", MAC_ARG(pmlmepriv->cur_network.network.MacAddress));
	DBG_871X_LEVEL(_drv_always_, "assoc success\n");
	wireless_send_event(rtlpriv->ndev, SIOCGIWAP, &wrqu, NULL);
}

void rtw_indicate_wx_disassoc_event(struct rtl_priv *rtlpriv)
{
	union iwreq_data wrqu;

	memset(&wrqu, 0, sizeof(union iwreq_data));

	wrqu.ap_addr.sa_family = ARPHRD_ETHER;
	memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);

	DBG_871X_LEVEL(_drv_always_, "indicate disassoc\n");
	wireless_send_event(rtlpriv->ndev, SIOCGIWAP, &wrqu, NULL);
}

/*
uint	rtw_is_cckrates_included(uint8_t *rate)
{
		u32	i = 0;

		while(rate[i]!=0)
		{
			if  (  (((rate[i]) & 0x7f) == 2)	|| (((rate[i]) & 0x7f) == 4) ||
			(((rate[i]) & 0x7f) == 11)  || (((rate[i]) & 0x7f) == 22) )
			return true;
			i++;
		}

		return false;
}

uint	rtw_is_cckratesonly_included(uint8_t *rate)
{
	u32 i = 0;

	while(rate[i]!=0)
	{
			if  (  (((rate[i]) & 0x7f) != 2) && (((rate[i]) & 0x7f) != 4) &&
				(((rate[i]) & 0x7f) != 11)  && (((rate[i]) & 0x7f) != 22) )
			return false;
			i++;
	}

	return true;
}
*/

static char *translate_scan(struct rtl_priv *rtlpriv,
				struct iw_request_info *info, struct wlan_network *pnetwork,
				char *start, char *stop)
{
	struct iw_event iwe;
	u16 cap;
	u32 ht_ielen = 0, vht_ielen = 0;
	char custom[MAX_CUSTOM_LEN];
	char *p;
	u16 max_rate = 0, rate, ht_cap = false, vht_cap = false;
	u32 i = 0;
	uint8_t bw_40MHz = 0, short_GI = 0, bw_160MHz = 0, vht_highest_rate = 0;
	u16 mcs_rate = 0, vht_data_rate = 0;

	/*  AP MAC address  */
	iwe.cmd = SIOCGIWAP;
	iwe.u.ap_addr.sa_family = ARPHRD_ETHER;

	memcpy(iwe.u.ap_addr.sa_data, pnetwork->network.MacAddress, ETH_ALEN);
	start = iwe_stream_add_event(info, start, stop, &iwe, IW_EV_ADDR_LEN);

	/* Add the ESSID */
	iwe.cmd = SIOCGIWESSID;
	iwe.u.data.flags = 1;
	iwe.u.data.length = min((u16)pnetwork->network.Ssid.SsidLength, (u16)32);
	start = iwe_stream_add_point(info, start, stop, &iwe, pnetwork->network.Ssid.Ssid);

	/* parsing HT_CAP_IE */
	p = rtw_get_ie(&pnetwork->network.IEs[12], _HT_CAPABILITY_IE_, &ht_ielen, pnetwork->network.IELength-12);

	if (p && ht_ielen > 0) {
		struct rtw_ieee80211_ht_cap *pht_capie;
		ht_cap = true;
		pht_capie = (struct rtw_ieee80211_ht_cap *)(p+2);
		memcpy(&mcs_rate , pht_capie->supp_mcs_set, 2);
		bw_40MHz = (pht_capie->cap_info&IEEE80211_HT_CAP_SUP_WIDTH) ? 1:0;
		short_GI = (pht_capie->cap_info&(IEEE80211_HT_CAP_SGI_20|IEEE80211_HT_CAP_SGI_40)) ? 1:0;
	}

	/* parsing VHT_CAP_IE */
	p = rtw_get_ie(&pnetwork->network.IEs[12], EID_VHTCapability, &vht_ielen, pnetwork->network.IELength-12);
	if (p && vht_ielen > 0) {
		uint8_t	mcs_map[2];

		vht_cap = true;
		bw_160MHz = GET_VHT_CAPABILITY_ELE_CHL_WIDTH(p+2);
		if (bw_160MHz)
			short_GI = GET_VHT_CAPABILITY_ELE_SHORT_GI160M(p+2);
		else
			short_GI = GET_VHT_CAPABILITY_ELE_SHORT_GI80M(p+2);

		memcpy(mcs_map, GET_VHT_CAPABILITY_ELE_TX_MCS(p+2), 2);

		vht_highest_rate = rtw_get_vht_highest_rate(rtlpriv, mcs_map);
		vht_data_rate = rtw_vht_data_rate(CHANNEL_WIDTH_80, short_GI, vht_highest_rate);
	}

	/* Add the protocol name */
	iwe.cmd = SIOCGIWNAME;
	if ((rtw_is_cckratesonly_included((uint8_t *)&pnetwork->network.SupportedRates)) == true) {
		if (ht_cap == true)
			snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11bn");
		else
		snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11b");
	} else if ((rtw_is_cckrates_included((uint8_t *)&pnetwork->network.SupportedRates)) == true) {
		if (ht_cap == true)
			snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11bgn");
		else
			snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11bg");
	} else {
		if (pnetwork->network.Configuration.DSConfig > 14) {
			if (vht_cap == true)
				snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11AC");
			else if (ht_cap == true)
				snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11an");
			else
				snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11a");
		} else {
			if (ht_cap == true)
				snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11gn");
			else
				snprintf(iwe.u.name, IFNAMSIZ, "IEEE 802.11g");
		}
	}

	start = iwe_stream_add_event(info, start, stop, &iwe, IW_EV_CHAR_LEN);

	  /* Add mode */
	iwe.cmd = SIOCGIWMODE;
	memcpy((uint8_t *)&cap, rtw_get_capability_from_ie(pnetwork->network.IEs), 2);


	cap = le16_to_cpu(cap);

	if (cap & (WLAN_CAPABILITY_IBSS | WLAN_CAPABILITY_BSS)) {
		if (cap & WLAN_CAPABILITY_BSS)
			iwe.u.mode = IW_MODE_MASTER;
		else
			iwe.u.mode = IW_MODE_ADHOC;

		start = iwe_stream_add_event(info, start, stop, &iwe, IW_EV_UINT_LEN);
	}

	if (pnetwork->network.Configuration.DSConfig < 1 /*|| pnetwork->network.Configuration.DSConfig>14*/)
		pnetwork->network.Configuration.DSConfig = 1;

	 /* Add frequency/channel */
	iwe.cmd = SIOCGIWFREQ;
	iwe.u.freq.m = rtw_ch2freq(pnetwork->network.Configuration.DSConfig) * 100000;
	iwe.u.freq.e = 1;
	iwe.u.freq.i = pnetwork->network.Configuration.DSConfig;
	start = iwe_stream_add_event(info, start, stop, &iwe, IW_EV_FREQ_LEN);

	/* Add encryption capability */
	iwe.cmd = SIOCGIWENCODE;
	if (cap & WLAN_CAPABILITY_PRIVACY)
		iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
	else
		iwe.u.data.flags = IW_ENCODE_DISABLED;
	iwe.u.data.length = 0;
	start = iwe_stream_add_point(info, start, stop, &iwe, pnetwork->network.Ssid.Ssid);

	/*Add basic and extended rates */
	max_rate = 0;
	p = custom;
	p += snprintf(p, MAX_CUSTOM_LEN - (p - custom), " Rates (Mb/s): ");
	while (pnetwork->network.SupportedRates[i] != 0) {
		rate = pnetwork->network.SupportedRates[i] & 0x7F;
		if (rate > max_rate)
			max_rate = rate;
		p += snprintf(p, MAX_CUSTOM_LEN - (p - custom),
			      "%d%s ", rate >> 1, (rate & 1) ? ".5" : "");
		i++;
	}

	if (vht_cap == true) {
		max_rate = vht_data_rate;
	} else if (ht_cap == true) {
		if (mcs_rate&0x8000) {
			/* MCS15 */
			max_rate = (bw_40MHz) ? ((short_GI)?300:270):((short_GI)?144:130);

		} else if (mcs_rate&0x0080) {
			/* MCS7 */
			max_rate = (bw_40MHz) ? ((short_GI)?150:135):((short_GI)?72:65);
		} else {
			/*default MCS7 */
			/* DBG_871X("wx_get_scan, mcs_rate_bitmap=0x%x\n", mcs_rate); */
			max_rate = (bw_40MHz) ? ((short_GI)?150:135):((short_GI)?72:65);
		}

		max_rate = max_rate*2;	/* Mbps/2; */
	}

	iwe.cmd = SIOCGIWRATE;
	iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
	iwe.u.bitrate.value = max_rate * 500000;
	start = iwe_stream_add_event(info, start, stop, &iwe, IW_EV_PARAM_LEN);

	/* parsing WPA/WPA2 IE */
	{
		uint8_t buf[MAX_WPA_IE_LEN];
		uint8_t wpa_ie[255], rsn_ie[255];
		u16 wpa_len = 0, rsn_len = 0;
		uint8_t *p;
		int out_len = 0;
		out_len = rtw_get_sec_ie(pnetwork->network.IEs, pnetwork->network.IELength, rsn_ie, &rsn_len, wpa_ie, &wpa_len);

		if (wpa_len > 0) {
			p = buf;
			memset(buf, 0, MAX_WPA_IE_LEN);
			p += sprintf(p, "wpa_ie=");
			for (i = 0; i < wpa_len; i++) {
				p += sprintf(p, "%02x", wpa_ie[i]);
			}

			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = IWEVCUSTOM;
			iwe.u.data.length = strlen(buf);
			start = iwe_stream_add_point(info, start, stop, &iwe, buf);

			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = IWEVGENIE;
			iwe.u.data.length = wpa_len;
			start = iwe_stream_add_point(info, start, stop, &iwe, wpa_ie);
		}

		if (rsn_len > 0) {
			p = buf;
			memset(buf, 0, MAX_WPA_IE_LEN);
			p += sprintf(p, "rsn_ie=");
			for (i = 0; i < rsn_len; i++) {
				p += sprintf(p, "%02x", rsn_ie[i]);
			}
			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = IWEVCUSTOM;
			iwe.u.data.length = strlen(buf);
			start = iwe_stream_add_point(info, start, stop, &iwe, buf);

			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = IWEVGENIE;
			iwe.u.data.length = rsn_len;
			start = iwe_stream_add_point(info, start, stop, &iwe, rsn_ie);
		}
	}

	{	/* parsing WPS IE */
		uint cnt = 0, total_ielen;
		uint8_t *wpsie_ptr = NULL;
		uint wps_ielen = 0;

		uint8_t *ie_ptr = pnetwork->network.IEs + _FIXED_IE_LENGTH_;
		total_ielen = pnetwork->network.IELength - _FIXED_IE_LENGTH_;

		while (cnt < total_ielen) {
			if (rtw_is_wps_ie(&ie_ptr[cnt], &wps_ielen) && (wps_ielen > 2)) {
				wpsie_ptr = &ie_ptr[cnt];
				iwe.cmd = IWEVGENIE;
				iwe.u.data.length = (u16)wps_ielen;
				start = iwe_stream_add_point(info, start, stop, &iwe, wpsie_ptr);
			}
			cnt += ie_ptr[cnt+1] + 2; /* goto next */
		}
	}


	{
		struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
		uint8_t ss, sq;

		/* Add quality statistics */
		iwe.cmd = IWEVQUAL;
		iwe.u.qual.updated = IW_QUAL_QUAL_UPDATED | IW_QUAL_LEVEL_UPDATED | IW_QUAL_NOISE_INVALID
		;

		if (check_fwstate(pmlmepriv, _FW_LINKED) == true &&
			is_same_network(&pmlmepriv->cur_network.network, &pnetwork->network)) {
			ss = rtlpriv->recvpriv.signal_strength;
			sq = rtlpriv->recvpriv.signal_qual;
		} else {
			ss = pnetwork->network.PhyInfo.SignalStrength;
			sq = pnetwork->network.PhyInfo.SignalQuality;
		}


		iwe.u.qual.level = (uint8_t)ss;		/* % */

		iwe.u.qual.qual = (uint8_t)sq;	/* signal quality */

		iwe.u.qual.noise = 0;		/* noise level */

		/* DBG_871X("iqual=%d, ilevel=%d, inoise=%d, iupdated=%d\n", iwe.u.qual.qual, iwe.u.qual.level , iwe.u.qual.noise, iwe.u.qual.updated); */

		start = iwe_stream_add_event(info, start, stop, &iwe, IW_EV_QUAL_LEN);
	}

	return start;
}

static int wpa_set_auth_algs(struct net_device *ndev, u32 value)
{
	struct rtl_priv *rtlpriv =  rtl_priv(ndev);
	int ret = 0;

	if ((value & AUTH_ALG_SHARED_KEY) && (value & AUTH_ALG_OPEN_SYSTEM)) {
		DBG_871X("wpa_set_auth_algs, AUTH_ALG_SHARED_KEY and  AUTH_ALG_OPEN_SYSTEM [value:0x%x]\n", value);
		rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;
		rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeAutoSwitch;
		rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Auto;
	} else if (value & AUTH_ALG_SHARED_KEY) {
		DBG_871X("wpa_set_auth_algs, AUTH_ALG_SHARED_KEY  [value:0x%x]\n", value);
		rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;

		rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeShared;
		rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Shared;
	} else if (value & AUTH_ALG_OPEN_SYSTEM) {
		DBG_871X("wpa_set_auth_algs, AUTH_ALG_OPEN_SYSTEM\n");
		/* rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11EncryptionDisabled; */
		if (rtlpriv->securitypriv.ndisauthtype < Ndis802_11AuthModeWPAPSK) {
			rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeOpen;
			rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Open;
		}

	} else if (value & AUTH_ALG_LEAP) {
		DBG_871X("wpa_set_auth_algs, AUTH_ALG_LEAP\n");
	} else {
		DBG_871X("wpa_set_auth_algs, error!\n");
		ret = -EINVAL;
	}

	return ret;

}

static int wpa_set_encryption(struct net_device *ndev, struct ieee_param *param, u32 param_len)
{
	int ret = 0;
	u32 wep_key_idx, wep_key_len, wep_total_len;
	NDIS_802_11_WEP	 *pwep = NULL;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv 	*pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;

	param->u.crypt.err = 0;
	param->u.crypt.alg[IEEE_CRYPT_ALG_NAME_LEN - 1] = '\0';

	if (param_len < (u32) ((uint8_t *) param->u.crypt.key - (uint8_t *) param) + param->u.crypt.key_len) {
		ret =  -EINVAL;
		goto exit;
	}

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) {
		if (param->u.crypt.idx >= WEP_KEYS) {
			ret = -EINVAL;
			goto exit;
		}
	} else {
		ret = -EINVAL;
		goto exit;
	}

	if (strcmp(param->u.crypt.alg, "WEP") == 0) {
		DBG_871X("wpa_set_encryption, crypt.alg = WEP\n");

		rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;
		rtlpriv->securitypriv.dot11PrivacyAlgrthm = WEP40_ENCRYPTION;
		rtlpriv->securitypriv.dot118021XGrpPrivacy = WEP40_ENCRYPTION;

		wep_key_idx = param->u.crypt.idx;
		wep_key_len = param->u.crypt.key_len;

		DBG_871X("(1)wep_key_idx=%d\n", wep_key_idx);

		if (wep_key_idx > WEP_KEYS)
			return -EINVAL;

		if (wep_key_len > 0) {
			wep_key_len = wep_key_len <= 5 ? 5 : 13;
			wep_total_len = wep_key_len + FIELD_OFFSET(NDIS_802_11_WEP, KeyMaterial);
			pwep = (NDIS_802_11_WEP	 *) rtw_malloc(wep_total_len);
			if (pwep == NULL) {
				goto exit;
			}

			memset(pwep, 0, wep_total_len);

			pwep->KeyLength = wep_key_len;
			pwep->Length = wep_total_len;

			if (wep_key_len == 13) {
				rtlpriv->securitypriv.dot11PrivacyAlgrthm = WEP104_ENCRYPTION;
				rtlpriv->securitypriv.dot118021XGrpPrivacy = WEP104_ENCRYPTION;
			}
		} else {
			ret = -EINVAL;
			goto exit;
		}

		pwep->KeyIndex = wep_key_idx;
		pwep->KeyIndex |= 0x80000000;

		memcpy(pwep->KeyMaterial,  param->u.crypt.key, pwep->KeyLength);

		if (param->u.crypt.set_tx) {
			DBG_871X("wep, set_tx=1\n");

			if (rtw_set_802_11_add_wep(rtlpriv, pwep) == (uint8_t)_FAIL) {
				ret = -EOPNOTSUPP ;
			}
		} else {
			DBG_871X("wep, set_tx=0\n");

			/*
			 * don't update "psecuritypriv->dot11PrivacyAlgrthm" and
			 * "psecuritypriv->dot11PrivacyKeyIndex=keyid", but can rtw_set_key to fw/cam
			 */

			if (wep_key_idx >= WEP_KEYS) {
				ret = -EOPNOTSUPP ;
				goto exit;
			}

			memcpy(&(psecuritypriv->dot11DefKey[wep_key_idx].skey[0]), pwep->KeyMaterial, pwep->KeyLength);
			psecuritypriv->dot11DefKeylen[wep_key_idx] = pwep->KeyLength;
			rtw_set_key(rtlpriv, psecuritypriv, wep_key_idx, 0);
		}

		goto exit;
	}

	if (rtlpriv->securitypriv.dot11AuthAlgrthm == dot11AuthAlgrthm_8021X) {
		/* 802_1x */
		struct sta_info *psta, *pbcmc_sta;
		struct sta_priv *pstapriv = &rtlpriv->stapriv;

		if (check_fwstate(pmlmepriv, WIFI_STATION_STATE | WIFI_MP_STATE) == true) {
			/* sta mode */
			psta = rtw_get_stainfo(pstapriv, get_bssid(pmlmepriv));
			if (psta == NULL) {
				/* DEBUG_ERR( ("Set wpa_set_encryption: Obtain Sta_info fail \n")); */
			} else 	{
				/* Jeff: don't disable ieee8021x_blocked while clearing key */
				if (strcmp(param->u.crypt.alg, "none") != 0)
					psta->ieee8021x_blocked = false;

				if ((rtlpriv->securitypriv.ndisencryptstatus == Ndis802_11Encryption2Enabled) ||
						(rtlpriv->securitypriv.ndisencryptstatus ==  Ndis802_11Encryption3Enabled)) {
					psta->dot118021XPrivacy = rtlpriv->securitypriv.dot11PrivacyAlgrthm;
				}

				if (param->u.crypt.set_tx == 1) {
					/* pairwise key */
					memcpy(psta->dot118021x_UncstKey.skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));

					if (strcmp(param->u.crypt.alg, "TKIP") == 0) {	/*set mic key */
						/* DEBUG_ERR(("\nset key length :param->u.crypt.key_len=%d\n", param->u.crypt.key_len)); */
						memcpy(psta->dot11tkiptxmickey.skey, &(param->u.crypt.key[16]), 8);
						memcpy(psta->dot11tkiprxmickey.skey, &(param->u.crypt.key[24]), 8);

						rtlpriv->securitypriv.busetkipkey = false;
						/* _set_timer(&rtlpriv->securitypriv.tkip_timer, 50); */
					}

					/* DEBUG_ERR((" param->u.crypt.key_len=%d\n",param->u.crypt.key_len)); */
					DBG_871X(" ~~~~set sta key:unicastkey\n");

					rtw_setstakey_cmd(rtlpriv, (unsigned char *)psta, true);
				} else {
					/* group key */
					memcpy(rtlpriv->securitypriv.dot118021XGrpKey[param->u.crypt.idx].skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));
					memcpy(rtlpriv->securitypriv.dot118021XGrptxmickey[param->u.crypt.idx].skey, &(param->u.crypt.key[16]), 8);
					memcpy(rtlpriv->securitypriv.dot118021XGrprxmickey[param->u.crypt.idx].skey, &(param->u.crypt.key[24]), 8);
					rtlpriv->securitypriv.binstallGrpkey = true;
					/* DEBUG_ERR((" param->u.crypt.key_len=%d\n", param->u.crypt.key_len)); */
					DBG_871X(" ~~~~set sta key:groupkey\n");

					rtlpriv->securitypriv.dot118021XGrpKeyid = param->u.crypt.idx;

					rtw_set_key(rtlpriv, &rtlpriv->securitypriv, param->u.crypt.idx, 1);

				}
			}

			pbcmc_sta = rtw_get_bcmc_stainfo(rtlpriv);
			if (pbcmc_sta == NULL) 	{
				/* DEBUG_ERR( ("Set OID_802_11_ADD_KEY: bcmc stainfo is null \n")); */
			} else {
				/* Jeff: don't disable ieee8021x_blocked while clearing key */
				if (strcmp(param->u.crypt.alg, "none") != 0)
					pbcmc_sta->ieee8021x_blocked = false;

				if ((rtlpriv->securitypriv.ndisencryptstatus == Ndis802_11Encryption2Enabled) ||
						(rtlpriv->securitypriv.ndisencryptstatus ==  Ndis802_11Encryption3Enabled)) {
					pbcmc_sta->dot118021XPrivacy = rtlpriv->securitypriv.dot11PrivacyAlgrthm;
				}
			}
		} else if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE)) {
			/* adhoc mode */
		}
	}


exit:

	if (pwep) {
		rtw_mfree(pwep);
	}



	return ret;
}

static int rtw_set_wpa_ie(struct rtl_priv *rtlpriv, char *pie, unsigned short ielen)
{
	uint8_t *buf = NULL, *pos = NULL;
	int group_cipher = 0, pairwise_cipher = 0;
	int ret = 0;
	uint8_t null_addr[] = {0, 0, 0, 0, 0, 0};

	if ((ielen > MAX_WPA_IE_LEN) || (pie == NULL)) {
		_clr_fwstate_(&rtlpriv->mlmepriv, WIFI_UNDER_WPS);
		if (pie == NULL)
			return ret;
		else
			return -EINVAL;
	}

	if (ielen) {
		buf = rtw_zmalloc(ielen);
		if (buf == NULL) {
			ret =  -ENOMEM;
			goto exit;
		}

		memcpy(buf, pie , ielen);

		/* dump */
		{
			int i;
			DBG_871X("\n wpa_ie(length:%d):\n", ielen);
			for (i = 0; i < ielen; i = i + 8)
				DBG_871X("0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x \n", buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
		}

		pos = buf;
		if (ielen < RSN_HEADER_LEN) {
			ret  = -1;
			goto exit;
		}

		if (rtw_parse_wpa_ie(buf, ielen, &group_cipher, &pairwise_cipher, NULL) == _SUCCESS) {
			rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_8021X;
			rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeWPAPSK;
			memcpy(rtlpriv->securitypriv.supplicant_ie, &buf[0], ielen);
		}

		if (rtw_parse_wpa2_ie(buf, ielen, &group_cipher, &pairwise_cipher, NULL) == _SUCCESS) {
			rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_8021X;
			rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeWPA2PSK;
			memcpy(rtlpriv->securitypriv.supplicant_ie, &buf[0], ielen);
		}

		if (group_cipher == 0) {
			group_cipher = WPA_CIPHER_NONE;
		}

		if (pairwise_cipher == 0) {
			pairwise_cipher = WPA_CIPHER_NONE;
		}

		switch (group_cipher) {
		case WPA_CIPHER_NONE:
			rtlpriv->securitypriv.dot118021XGrpPrivacy = NO_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11EncryptionDisabled;
			break;
		case WPA_CIPHER_WEP40:
			rtlpriv->securitypriv.dot118021XGrpPrivacy = WEP40_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;
			break;
		case WPA_CIPHER_TKIP:
			rtlpriv->securitypriv.dot118021XGrpPrivacy = TKIP_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption2Enabled;
			break;
		case WPA_CIPHER_CCMP:
			rtlpriv->securitypriv.dot118021XGrpPrivacy = AESCCMP_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption3Enabled;
			break;
		case WPA_CIPHER_WEP104:
			rtlpriv->securitypriv.dot118021XGrpPrivacy = WEP104_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;
			break;
		}

		switch (pairwise_cipher) {
		case WPA_CIPHER_NONE:
			rtlpriv->securitypriv.dot11PrivacyAlgrthm = NO_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11EncryptionDisabled;
			break;
		case WPA_CIPHER_WEP40:
			rtlpriv->securitypriv.dot11PrivacyAlgrthm = WEP40_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;
			break;
		case WPA_CIPHER_TKIP:
			rtlpriv->securitypriv.dot11PrivacyAlgrthm = TKIP_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption2Enabled;
			break;
		case WPA_CIPHER_CCMP:
			rtlpriv->securitypriv.dot11PrivacyAlgrthm = AESCCMP_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption3Enabled;
			break;
		case WPA_CIPHER_WEP104:
			rtlpriv->securitypriv.dot11PrivacyAlgrthm = WEP104_ENCRYPTION;
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;
			break;
		}

		_clr_fwstate_(&rtlpriv->mlmepriv, WIFI_UNDER_WPS);
		{
			/* set wps_ie */
			u16 cnt = 0;
			uint8_t eid, wps_oui[4] = { 0x0, 0x50, 0xf2, 0x04};

			while (cnt < ielen) {
				eid = buf[cnt];

				if ((eid == _VENDOR_SPECIFIC_IE_) &&
				    (_rtw_memcmp(&buf[cnt+2], wps_oui, 4) == true)) {
					DBG_871X("SET WPS_IE\n");

					rtlpriv->securitypriv.wps_ie_len = ((buf[cnt+1]+2) < (MAX_WPA_IE_LEN << 2)) ? (buf[cnt+1] + 2) : (MAX_WPA_IE_LEN << 2);
					memcpy(rtlpriv->securitypriv.wps_ie, &buf[cnt], rtlpriv->securitypriv.wps_ie_len);
					set_fwstate(&rtlpriv->mlmepriv, WIFI_UNDER_WPS);

					cnt += buf[cnt+1]+2;

					break;
				} else {
					cnt += buf[cnt+1]+2; /* goto next */
				}
			}
		}
	}

	/* TKIP and AES disallow multicast packets until installing group key */
	if (rtlpriv->securitypriv.dot11PrivacyAlgrthm == TKIP_ENCRYPTION ||
	    rtlpriv->securitypriv.dot11PrivacyAlgrthm == RSERVED_ENCRYPTION ||
	    rtlpriv->securitypriv.dot11PrivacyAlgrthm == AESCCMP_ENCRYPTION)
		/*
		 * WPS open need to enable multicast
		 * || check_fwstate(&rtlpriv->mlmepriv, WIFI_UNDER_WPS) == true)
		 */
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_OFF_RCR_AM, null_addr);

exit:
	if (buf)
		rtw_mfree(buf);

	return ret;
}

static int rtw_wx_get_name(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	u32 ht_ielen = 0;
	char *p;
	uint8_t ht_cap = false, vht_cap = false;
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	WLAN_BSSID_EX  *pcur_bss = &pmlmepriv->cur_network.network;
	NDIS_802_11_RATES_EX *prates = NULL;

	if (check_fwstate(pmlmepriv, _FW_LINKED|WIFI_ADHOC_MASTER_STATE) == true) {
		/* parsing HT_CAP_IE */
		p = rtw_get_ie(&pcur_bss->IEs[12], _HT_CAPABILITY_IE_, &ht_ielen, pcur_bss->IELength-12);
		if (p && ht_ielen > 0) {
			ht_cap = true;
		}

		if (pmlmepriv->vhtpriv.vht_option == true)
			vht_cap = true;

		prates = &pcur_bss->SupportedRates;

		if (rtw_is_cckratesonly_included((uint8_t *)prates) == true) 	{
			if (ht_cap == true)
				snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11bn");
			else
				snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11b");
		} else if ((rtw_is_cckrates_included((uint8_t *)prates)) == true) {
			if (ht_cap == true)
				snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11bgn");
			else
				snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11bg");
		} else 	{
			if (pcur_bss->Configuration.DSConfig > 14) {
				if (vht_cap == true)
					snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11AC");
				else if (ht_cap == true)
					snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11an");
				else
					snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11a");
			} else {
				if (ht_cap == true)
					snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11gn");
				else
					snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11g");
			}
		}
	} else {
		/*
		 * prates = &rtlpriv->registrypriv.dev_network.SupportedRates;
		 * snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11g");
		 */
		snprintf(wrqu->name, IFNAMSIZ, "unassociated");
	}

	return 0;
}

static int rtw_wx_set_freq(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	return 0;
}

static int rtw_wx_get_freq(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	WLAN_BSSID_EX  *pcur_bss = &pmlmepriv->cur_network.network;

	if (check_fwstate(pmlmepriv, _FW_LINKED) == true) {
		/* wrqu->freq.m = ieee80211_wlan_frequencies[pcur_bss->Configuration.DSConfig-1] * 100000; */
		wrqu->freq.m = rtw_ch2freq(pcur_bss->Configuration.DSConfig) * 100000;
		wrqu->freq.e = 1;
		wrqu->freq.i = pcur_bss->Configuration.DSConfig;

	} else {
		wrqu->freq.m = rtw_ch2freq(rtlpriv->mlmeextpriv.cur_channel) * 100000;
		wrqu->freq.e = 1;
		wrqu->freq.i = rtlpriv->mlmeextpriv.cur_channel;
	}

	return 0;
}

static int rtw_wx_set_mode(struct net_device *ndev, struct iw_request_info *a,
			     union iwreq_data *wrqu, char *b)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	NDIS_802_11_NETWORK_INFRASTRUCTURE networkType ;
	int ret = 0;

	if (_FAIL == rtw_pwr_wakeup(rtlpriv)) {
		ret = -EPERM;
		goto exit;
	}

	if (rtlpriv->hw_init_completed == false) {
		ret = -EPERM;
		goto exit;
	}

	switch (wrqu->mode) {
	case IW_MODE_AUTO:
		networkType = Ndis802_11AutoUnknown;
		DBG_871X("set_mode = IW_MODE_AUTO\n");
		break;
	case IW_MODE_ADHOC:
		networkType = Ndis802_11IBSS;
		DBG_871X("set_mode = IW_MODE_ADHOC\n");
		break;
	case IW_MODE_MASTER:
		networkType = Ndis802_11APMode;
		DBG_871X("set_mode = IW_MODE_MASTER\n");
		/* rtw_setopmode_cmd(rtlpriv, networkType); */
		break;
	case IW_MODE_INFRA:
		networkType = Ndis802_11Infrastructure;
		DBG_871X("set_mode = IW_MODE_INFRA\n");
		break;

	default:
		ret = -EINVAL;;
		goto exit;
	}

/*
	if (Ndis802_11APMode == networkType)
	{
		rtw_setopmode_cmd(rtlpriv, networkType);
	}
	else
	{
		rtw_setopmode_cmd(rtlpriv, Ndis802_11AutoUnknown);
	}
*/

	if (rtw_set_802_11_infrastructure_mode(rtlpriv, networkType) == false) {
		ret = -EPERM;
		goto exit;

	}

	rtw_setopmode_cmd(rtlpriv, networkType);

exit:

	return ret;

}

static int rtw_wx_get_mode(struct net_device *ndev, struct iw_request_info *a,
			     union iwreq_data *wrqu, char *b)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);

	if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true) {
		wrqu->mode = IW_MODE_INFRA;
	} else
		if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true) ||
		    (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true)) {
			wrqu->mode = IW_MODE_ADHOC;
	} else if (check_fwstate(pmlmepriv, WIFI_AP_STATE) == true) {
		wrqu->mode = IW_MODE_MASTER;
	} else {
		wrqu->mode = IW_MODE_AUTO;
	}

	return 0;

}


static int rtw_wx_set_pmkid(struct net_device *ndev, struct iw_request_info *a,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	uint8_t j, blInserted = false;
	int intReturn = false;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	struct iw_pmksa *pPMK = (struct iw_pmksa *) extra;
	uint8_t strZeroMacAddress[ETH_ALEN] = { 0x00 };
	uint8_t strIssueBssid[ETH_ALEN] = { 0x00 };

	memcpy(strIssueBssid, pPMK->bssid.sa_data, ETH_ALEN);
	if (pPMK->cmd == IW_PMKSA_ADD) {
		DBG_871X("[rtw_wx_set_pmkid] IW_PMKSA_ADD!\n");
		if (_rtw_memcmp(strIssueBssid, strZeroMacAddress, ETH_ALEN) == true) {
			return intReturn;
		} else {
			intReturn = true;
		}
		blInserted = false;

		/* overwrite PMKID */
		for (j = 0 ; j < NUM_PMKID_CACHE; j++) {
			if (_rtw_memcmp(psecuritypriv->PMKIDList[j].Bssid, strIssueBssid, ETH_ALEN) == true) {
				/* BSSID is matched, the same AP => rewrite with new PMKID. */

				DBG_871X("[rtw_wx_set_pmkid] BSSID exists in the PMKList.\n");

				memcpy(psecuritypriv->PMKIDList[j].PMKID, pPMK->pmkid, IW_PMKID_LEN);
				psecuritypriv->PMKIDList[j].bUsed = true;
				psecuritypriv->PMKIDIndex = j + 1;
				blInserted = true;
				break;
			}
		}

		if (!blInserted) {
			/* Find a new entry */
			DBG_871X("[rtw_wx_set_pmkid] Use the new entry index = %d for this PMKID.\n",
				psecuritypriv->PMKIDIndex);

			memcpy(psecuritypriv->PMKIDList[psecuritypriv->PMKIDIndex].Bssid, strIssueBssid, ETH_ALEN);
			memcpy(psecuritypriv->PMKIDList[psecuritypriv->PMKIDIndex].PMKID, pPMK->pmkid, IW_PMKID_LEN);

			psecuritypriv->PMKIDList[psecuritypriv->PMKIDIndex].bUsed = true;
			psecuritypriv->PMKIDIndex++ ;
			if (psecuritypriv->PMKIDIndex == 16) {
				psecuritypriv->PMKIDIndex = 0;
			}
		}
	} else
		if (pPMK->cmd == IW_PMKSA_REMOVE) {
			DBG_871X("[rtw_wx_set_pmkid] IW_PMKSA_REMOVE!\n");
			intReturn = true;
			for (j = 0 ; j < NUM_PMKID_CACHE; j++) {
				if (_rtw_memcmp(psecuritypriv->PMKIDList[j].Bssid, strIssueBssid, ETH_ALEN) == true) {
					/*  BSSID is matched, the same AP => Remove this PMKID information and reset it. */
					memset(psecuritypriv->PMKIDList[j].Bssid, 0x00, ETH_ALEN);
					psecuritypriv->PMKIDList[j].bUsed = false;
					break;
				}
		}
	} else
		if (pPMK->cmd == IW_PMKSA_FLUSH) {
			DBG_871X("[rtw_wx_set_pmkid] IW_PMKSA_FLUSH!\n");
			memset(&psecuritypriv->PMKIDList[0], 0x00, sizeof(RT_PMKID_LIST) * NUM_PMKID_CACHE);
			psecuritypriv->PMKIDIndex = 0;
			intReturn = true;
		}

	return intReturn;
}

static int rtw_wx_get_sens(struct net_device *ndev, struct iw_request_info *info,
				union iwreq_data *wrqu, char *extra)
{
	{
		wrqu->sens.value = 0;
		wrqu->sens.fixed = 0;	/* no auto select */
		wrqu->sens.disabled = 1;
	}
	return 0;
}

static int rtw_wx_get_range(struct net_device *ndev,
				struct iw_request_info *info,
				union iwreq_data *wrqu, char *extra)
{
	struct iw_range *range = (struct iw_range *)extra;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;

	u16 val;
	int i;

	wrqu->data.length = sizeof(*range);
	memset(range, 0, sizeof(*range));

	/* Let's try to keep this struct in the same order as in
	 * linux/include/wireless.h
	 */

	/* TODO: See what values we can set, and remove the ones we can't
	 * set, or fill them with some default data.
	 */

	/* ~5 Mb/s real (802.11b) */
	range->throughput = 5 * 1000 * 1000;

	/*  TODO: Not used in 802.11b? */
	/* Minimal NWID we are able to set */
	/*	range->min_nwid; */

	/* TODO: Not used in 802.11b? */
	/* Maximal NWID we are able to set */
	/* range->max_nwid; */

	/* Old Frequency (backward compat - moved lower ) */
	/* range->old_num_channels; */
	/*range->old_num_frequency; */

	/* Filler to keep "version" at the same offset */
	/* range->old_freq[6]; */

	/* signal level threshold range */

	/* percent values between 0 and 100. */
	range->max_qual.qual = 100;
	range->max_qual.level = 100;
	range->max_qual.noise = 100;
	range->max_qual.updated = 7; /* Updated all three */


	range->avg_qual.qual = 92; /* > 8% missed beacons is 'bad' */
	/* TODO: Find real 'good' to 'bad' threshol value for RSSI */
	range->avg_qual.level = 20 + -98;
	range->avg_qual.noise = 0;
	range->avg_qual.updated = 7; /* Updated all three */

	range->num_bitrates = RATE_COUNT;

	for (i = 0; i < RATE_COUNT && i < IW_MAX_BITRATES; i++) {
		range->bitrate[i] = rtw_rates[i];
	}

	range->min_frag = MIN_FRAG_THRESHOLD;
	range->max_frag = MAX_FRAG_THRESHOLD;

	range->pm_capa = 0;

	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 16;

	for (i = 0, val = 0; i < MAX_CHANNEL_NUM; i++) {
		/* Include only legal frequencies for some countries */
		if (pmlmeext->channel_set[i].ChannelNum != 0) {
			range->freq[val].i = pmlmeext->channel_set[i].ChannelNum;
			range->freq[val].m = rtw_ch2freq(pmlmeext->channel_set[i].ChannelNum) * 100000;
			range->freq[val].e = 1;
			val++;
		}

		if (val == IW_MAX_FREQUENCIES)
			break;
	}

	range->num_channels = val;
	range->num_frequency = val;

/*
 * Commented by Albert 2009/10/13
 * The following code will proivde the security capability to network manager.
 * If the driver doesn't provide this capability to network manager,
 * the WPA/WPA2 routers can't be choosen in the network manager.
 */

/*
#define IW_SCAN_CAPA_NONE		0x00
#define IW_SCAN_CAPA_ESSID		0x01
#define IW_SCAN_CAPA_BSSID		0x02
#define IW_SCAN_CAPA_CHANNEL	0x04
#define IW_SCAN_CAPA_MODE		0x08
#define IW_SCAN_CAPA_RATE		0x10
#define IW_SCAN_CAPA_TYPE		0x20
#define IW_SCAN_CAPA_TIME		0x40
*/

#if WIRELESS_EXT > 17
	range->enc_capa = IW_ENC_CAPA_WPA|IW_ENC_CAPA_WPA2|
			  IW_ENC_CAPA_CIPHER_TKIP|IW_ENC_CAPA_CIPHER_CCMP;
#endif

#ifdef IW_SCAN_CAPA_ESSID /* WIRELESS_EXT > 21 */
	range->scan_capa = IW_SCAN_CAPA_ESSID | IW_SCAN_CAPA_TYPE | IW_SCAN_CAPA_BSSID |
					IW_SCAN_CAPA_CHANNEL | IW_SCAN_CAPA_MODE | IW_SCAN_CAPA_RATE;
#endif

	return 0;

}

/*
 * set bssid flow
 * s1. rtw_set_802_11_infrastructure_mode()
 * s2. rtw_set_802_11_authentication_mode()
 * s3. set_802_11_encryption_mode()
 * s4. rtw_set_802_11_bssid()
*/
static int rtw_wx_set_wap(struct net_device *ndev,
			 struct iw_request_info *info,
			 union iwreq_data *awrq,
			 char *extra)
{
	uint ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct sockaddr *temp = (struct sockaddr *) awrq;
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct list_head	*phead;
	uint8_t *dst_bssid, *src_bssid;
	struct __queue	*queue	= &(pmlmepriv->scanned_queue);
	struct	wlan_network	*pnetwork = NULL;
	NDIS_802_11_AUTHENTICATION_MODE	authmode;

	if (_FAIL == rtw_pwr_wakeup(rtlpriv)) {
		ret = -1;
		goto exit;
	}

	if (!rtlpriv->initialized) {
		ret = -1;
		goto exit;
	}


	if (temp->sa_family != ARPHRD_ETHER) {
		ret = -EINVAL;
		goto exit;
	}

	authmode = rtlpriv->securitypriv.ndisauthtype;
	spin_lock_bh(&queue->lock);
	phead = get_list_head(queue);
	pmlmepriv->pscanned = phead->next;

	while (1) {
		if ((rtw_end_of_queue_search(phead, pmlmepriv->pscanned)) == true) {
			break;
		}

		pnetwork = container_of(pmlmepriv->pscanned, struct wlan_network, list);

		pmlmepriv->pscanned = pmlmepriv->pscanned->next;

		dst_bssid = pnetwork->network.MacAddress;

		src_bssid = temp->sa_data;

		if ((_rtw_memcmp(dst_bssid, src_bssid, ETH_ALEN)) == true) {
			if (!rtw_set_802_11_infrastructure_mode(rtlpriv, pnetwork->network.InfrastructureMode)) {
				ret = -1;
				spin_unlock_bh(&queue->lock);
				goto exit;
			}
			break;
		}

	}
	spin_unlock_bh(&queue->lock);

	rtw_set_802_11_authentication_mode(rtlpriv, authmode);
	/* set_802_11_encryption_mode(rtlpriv, rtlpriv->securitypriv.ndisencryptstatus); */
	if (rtw_set_802_11_bssid(rtlpriv, temp->sa_data) == false) {
		ret = -1;
		goto exit;
	}

exit:

	return ret;
}

static int rtw_wx_get_wap(struct net_device *ndev,
			    struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{

	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	WLAN_BSSID_EX  *pcur_bss = &pmlmepriv->cur_network.network;

	wrqu->ap_addr.sa_family = ARPHRD_ETHER;

	memset(wrqu->ap_addr.sa_data, 0, ETH_ALEN);



	if  (((check_fwstate(pmlmepriv, _FW_LINKED)) == true) ||
	     ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) == true) ||
	     ((check_fwstate(pmlmepriv, WIFI_AP_STATE)) == true)) {
		memcpy(wrqu->ap_addr.sa_data, pcur_bss->MacAddress, ETH_ALEN);
	} else {
		memset(wrqu->ap_addr.sa_data, 0, ETH_ALEN);
	}

	return 0;

}

static int rtw_wx_set_mlme(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{

	int ret = 0;
	u16 reason;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct iw_mlme *mlme = (struct iw_mlme *) extra;


	if (mlme == NULL)
		return -1;

	DBG_871X("%s\n", __FUNCTION__);

	reason = cpu_to_le16(mlme->reason_code);

	DBG_871X("%s, cmd=%d, reason=%d\n", __FUNCTION__, mlme->cmd, reason);

	switch (mlme->cmd) {
	case IW_MLME_DEAUTH:
			if (!rtw_set_802_11_disassociate(rtlpriv))
			ret = -1;
			break;

	case IW_MLME_DISASSOC:
			if (!rtw_set_802_11_disassociate(rtlpriv))
					ret = -1;

			break;

	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

static int rtw_wx_set_scan(struct net_device *ndev, struct iw_request_info *a,
			     union iwreq_data *wrqu, char *extra)
{
	uint8_t _status = false;
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	NDIS_802_11_SSID ssid[RTW_SSID_SCAN_AMOUNT];

	if (_FAIL == rtw_pwr_wakeup(rtlpriv)) {
		ret = -1;
		goto exit;
	}

	if (rtlpriv->bDriverStopped) {
		DBG_871X("bDriverStopped=%d\n", rtlpriv->bDriverStopped);
		ret = -1;
		goto exit;
		}

	if (!rtlpriv->initialized) {
		ret = -1;
		goto exit;
	}

	if (rtlpriv->hw_init_completed == false) {
		ret = -1;
		goto exit;
	}

	/*
	 * When Busy Traffic, driver do not site survey. So driver return success.
	 * wpa_supplicant will not issue SIOCSIWSCAN cmd again after scan timeout.
	 * modify by thomas 2011-02-22.
	 */
	if (pmlmepriv->LinkDetectInfo.bBusyTraffic == true) {
		indicate_wx_scan_complete_event(rtlpriv);
		goto exit;
	}

	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY|_FW_UNDER_LINKING) == true) {
		indicate_wx_scan_complete_event(rtlpriv);
		goto exit;
	}

	/*
	 * Mareded by Albert 20101103
	 * For the DMP WiFi Display project, the driver won't to scan because
	 * the pmlmepriv->scan_interval is always equal to 3.
	 * So, the wpa_supplicant won't find out the WPS SoftAP.
	 */

/*
	if (pmlmepriv->scan_interval>10)
		pmlmepriv->scan_interval = 0;

	if (pmlmepriv->scan_interval > 0)
	{
		DBG_871X("scan done\n");
		ret = 0;
		goto exit;
	}

*/

	memset(ssid, 0, sizeof(NDIS_802_11_SSID)*RTW_SSID_SCAN_AMOUNT);

#if WIRELESS_EXT >= 17
	if (wrqu->data.length == sizeof(struct iw_scan_req)) {
		struct iw_scan_req *req = (struct iw_scan_req *)extra;

		if (wrqu->data.flags & IW_SCAN_THIS_ESSID) {
			int len = min((int)req->essid_len, IW_ESSID_MAX_SIZE);

			memcpy(ssid[0].Ssid, req->essid, len);
			ssid[0].SsidLength = len;

			DBG_871X("IW_SCAN_THIS_ESSID, ssid=%s, len=%d\n", req->essid, req->essid_len);

			spin_lock_bh(&pmlmepriv->lock);

			_status = rtw_sitesurvey_cmd(rtlpriv, ssid, 1, NULL, 0);

			spin_unlock_bh(&pmlmepriv->lock);

		} else if (req->scan_type == IW_SCAN_TYPE_PASSIVE) {
			DBG_871X("rtw_wx_set_scan, req->scan_type == IW_SCAN_TYPE_PASSIVE\n");
		}

	} else
#endif

		if (wrqu->data.length >= WEXT_CSCAN_HEADER_SIZE &&
		    _rtw_memcmp(extra, WEXT_CSCAN_HEADER, WEXT_CSCAN_HEADER_SIZE) == true) {
			int len = wrqu->data.length - WEXT_CSCAN_HEADER_SIZE;
			char *pos = extra+WEXT_CSCAN_HEADER_SIZE;
			char section;
			char sec_len;
			int ssid_index = 0;

			/* DBG_871X("%s COMBO_SCAN header is recognized\n", __FUNCTION__); */

			while (len >= 1) {
				section = *(pos++);
				len -= 1;

				switch (section) {
				case WEXT_CSCAN_SSID_SECTION:
					/* DBG_871X("WEXT_CSCAN_SSID_SECTION\n"); */
					if (len < 1) {
						len = 0;
						break;
					}

					sec_len = *(pos++);
					len -= 1;

					if (sec_len > 0 && sec_len <= len) {
						ssid[ssid_index].SsidLength = sec_len;
						memcpy(ssid[ssid_index].Ssid, pos, ssid[ssid_index].SsidLength);
						/*
						 * DBG_871X("%s COMBO_SCAN with specific ssid:%s, %d\n", __FUNCTION__
						 * 	, ssid[ssid_index].Ssid, ssid[ssid_index].SsidLength);
						 */
						ssid_index++;
					}

					pos += sec_len;
					len -= sec_len;
					break;


				case WEXT_CSCAN_CHANNEL_SECTION:
					/* DBG_871X("WEXT_CSCAN_CHANNEL_SECTION\n"); */
					pos += 1;
					len -= 1;
					break;
				case WEXT_CSCAN_ACTV_DWELL_SECTION:
					/* DBG_871X("WEXT_CSCAN_ACTV_DWELL_SECTION\n"); */
					pos += 2;
					len -= 2;
					break;
				case WEXT_CSCAN_PASV_DWELL_SECTION:
					/* DBG_871X("WEXT_CSCAN_PASV_DWELL_SECTION\n"); */
					pos += 2;
					len -= 2;
					break;
				case WEXT_CSCAN_HOME_DWELL_SECTION:
					/* DBG_871X("WEXT_CSCAN_HOME_DWELL_SECTION\n"); */
					pos += 2;
					len -= 2;
					break;
				case WEXT_CSCAN_TYPE_SECTION:
					/* DBG_871X("WEXT_CSCAN_TYPE_SECTION\n"); */
					pos += 1;
					len -= 1;
					break;

				default:
					/* DBG_871X("Unknown CSCAN section %c\n", section); */
					len = 0; 	/* stop parsing */
				}
			/* DBG_871X("len:%d\n", len); */

			}

			/* jeff: it has still some scan paramater to parse, we only do this now... */
			_status = rtw_set_802_11_bssid_list_scan(rtlpriv, ssid, RTW_SSID_SCAN_AMOUNT);

		} else {
			_status = rtw_set_802_11_bssid_list_scan(rtlpriv, NULL, 0);
		}

	if (_status == false)
		ret = -1;

exit:

	return ret;
}

static int rtw_wx_get_scan(struct net_device *ndev, struct iw_request_info *a,
			     union iwreq_data *wrqu, char *extra)
{
	struct list_head					*plist, *phead;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct __queue				*queue	= &(pmlmepriv->scanned_queue);
	struct	wlan_network	*pnetwork = NULL;
	char *ev = extra;
	char *stop = ev + wrqu->data.length;
	u32 ret = 0;
	u32 cnt = 0;
	u32 wait_for_surveydone;
	int wait_status;

	if (rtlpriv->pwrctrlpriv.brfoffbyhw && rtlpriv->bDriverStopped) {
		ret = -EINVAL;
		goto exit;
	}

	wait_for_surveydone = 100;


	wait_status = _FW_UNDER_SURVEY | _FW_UNDER_LINKING;

	while (check_fwstate(pmlmepriv, wait_status) == true) {
		msleep(30);
		cnt++;
		if (cnt > wait_for_surveydone)
			break;
	}

	spin_lock_bh(&(pmlmepriv->scanned_queue.lock));

	phead = get_list_head(queue);
	plist = phead->next;

	while (1) {
		if (rtw_end_of_queue_search(phead,plist) == true)
			break;

		if ((stop - ev) < SCAN_ITEM_SIZE) {
			ret = -E2BIG;
			break;
		}

		pnetwork = container_of(plist, struct wlan_network, list);

		/* report network only if the current channel set contains the channel to which this network belongs */
		if (rtw_ch_set_search_ch(rtlpriv->mlmeextpriv.channel_set, pnetwork->network.Configuration.DSConfig) >= 0
#ifdef CONFIG_VALIDATE_SSID
			&& true == rtw_validate_ssid(&(pnetwork->network.Ssid))
#endif
		)
		{
			ev = translate_scan(rtlpriv, a, pnetwork, ev, stop);
		}

		plist = plist->next;

	}

	spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));

	wrqu->data.length = ev-extra;
	wrqu->data.flags = 0;

exit:

	return ret ;
}

/*
 * set ssid flow
 * s1. rtw_set_802_11_infrastructure_mode()
 * s2. set_802_11_authenticaion_mode()
 * s3. set_802_11_encryption_mode()
 * s4. rtw_set_802_11_ssid()
 */
static int rtw_wx_set_essid(struct net_device *ndev,
			      struct iw_request_info *a,
			      union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct __queue *queue = &pmlmepriv->scanned_queue;
	struct list_head *phead;
	struct wlan_network *pnetwork = NULL;
	NDIS_802_11_AUTHENTICATION_MODE authmode;
	NDIS_802_11_SSID ndis_ssid;
	uint8_t *dst_ssid, *src_ssid;

	uint ret = 0, len;

	if (_FAIL == rtw_pwr_wakeup(rtlpriv)) {
		ret = -1;
		goto exit;
	}

	if (!rtlpriv->initialized) {
		ret = -1;
		goto exit;
	}

#if WIRELESS_EXT <= 20
	if ((wrqu->essid.length-1) > IW_ESSID_MAX_SIZE) {
#else
	if (wrqu->essid.length > IW_ESSID_MAX_SIZE) {
#endif
		ret = -E2BIG;
		goto exit;
	}

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE)) {
		ret = -1;
		goto exit;
	}

	authmode = rtlpriv->securitypriv.ndisauthtype;
	DBG_871X("=>%s\n", __FUNCTION__);
	if (wrqu->essid.flags && wrqu->essid.length) {
		/*
		 * Commented by Albert 20100519
		 * We got the codes in "set_info" function of iwconfig source code.
		 * 	=========================================
		 * 	wrq.u.essid.length = strlen(essid) + 1;
		 * 	if (we_kernel_version > 20)
		 * 		wrq.u.essid.length--;
		 * 	=========================================
		 * 	That means, if the WIRELESS_EXT less than or equal to 20, the correct ssid len should subtract 1.
		 */
#if WIRELESS_EXT <= 20
		len = ((wrqu->essid.length-1) < IW_ESSID_MAX_SIZE) ? (wrqu->essid.length-1) : IW_ESSID_MAX_SIZE;
#else
		len = (wrqu->essid.length < IW_ESSID_MAX_SIZE) ? wrqu->essid.length : IW_ESSID_MAX_SIZE;
#endif

		if (wrqu->essid.length != 33)
			DBG_871X("ssid=%s, len=%d\n", extra, wrqu->essid.length);

		memset(&ndis_ssid, 0, sizeof(NDIS_802_11_SSID));
		ndis_ssid.SsidLength = len;
		memcpy(ndis_ssid.Ssid, extra, len);
		src_ssid = ndis_ssid.Ssid;

		spin_lock_bh(&queue->lock);
		phead = get_list_head(queue);
		pmlmepriv->pscanned = phead->next;

		while (1) {
			if (rtw_end_of_queue_search(phead, pmlmepriv->pscanned) == true) {

				break;
			}

			pnetwork = container_of(pmlmepriv->pscanned, struct wlan_network, list);

			pmlmepriv->pscanned = pmlmepriv->pscanned->next;

			dst_ssid = pnetwork->network.Ssid.Ssid;

			if ((_rtw_memcmp(dst_ssid, src_ssid, ndis_ssid.SsidLength) == true) &&
			    (pnetwork->network.Ssid.SsidLength == ndis_ssid.SsidLength)) {

				if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true) {
					if (pnetwork->network.InfrastructureMode != pmlmepriv->cur_network.network.InfrastructureMode)
						continue;
				}

				if (rtw_set_802_11_infrastructure_mode(rtlpriv, pnetwork->network.InfrastructureMode) == false) {
					ret = -1;
					spin_unlock_bh(&queue->lock);
					goto exit;
				}

				break;
			}
		}
		spin_unlock_bh(&queue->lock);
		rtw_set_802_11_authentication_mode(rtlpriv, authmode);
		/* set_802_11_encryption_mode(rtlpriv, rtlpriv->securitypriv.ndisencryptstatus); */
		if (rtw_set_802_11_ssid(rtlpriv, &ndis_ssid) == false) {
			ret = -1;
			goto exit;
		}
	}

exit:

	DBG_871X("<=%s, ret %d\n", __FUNCTION__, ret);

	return ret;
}

static int rtw_wx_get_essid(struct net_device *ndev,
			      struct iw_request_info *a,
			      union iwreq_data *wrqu, char *extra)
{
	u32 len, ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	WLAN_BSSID_EX  *pcur_bss = &pmlmepriv->cur_network.network;

	if ((check_fwstate(pmlmepriv, _FW_LINKED) == true) ||
	   (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true)) {
		len = pcur_bss->Ssid.SsidLength;

		wrqu->essid.length = len;

		memcpy(extra, pcur_bss->Ssid.Ssid, len);

		wrqu->essid.flags = 1;
	} else {
		ret = -1;
		goto exit;
	}

exit:

	return ret;

}

static int rtw_wx_get_rate(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	u16 max_rate = 0;

	max_rate = rtw_get_cur_max_rate(rtl_priv(ndev));

	if (max_rate == 0)
		return -EPERM;

	wrqu->bitrate.fixed = 0;	/* no auto select */
	wrqu->bitrate.value = max_rate * 100000;

	return 0;
}

static int rtw_wx_set_rts(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	if (wrqu->rts.disabled)
		rtlpriv->registrypriv.rts_thresh = 2347;
	else {
		if (wrqu->rts.value < 0 ||
		    wrqu->rts.value > 2347)
			return -EINVAL;

		rtlpriv->registrypriv.rts_thresh = wrqu->rts.value;
	}

	DBG_871X("%s, rts_thresh=%d\n", __func__, rtlpriv->registrypriv.rts_thresh);

	return 0;

}

static int rtw_wx_get_rts(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	DBG_871X("%s, rts_thresh=%d\n", __func__, rtlpriv->registrypriv.rts_thresh);

	wrqu->rts.value = rtlpriv->registrypriv.rts_thresh;
	wrqu->rts.fixed = 0;	/* no auto select */
	/* wrqu->rts.disabled = (wrqu->rts.value == DEFAULT_RTS_THRESHOLD); */

	return 0;
}

static int rtw_wx_set_frag(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	if (wrqu->frag.disabled)
		rtlpriv->xmitpriv.frag_len = MAX_FRAG_THRESHOLD;
	else {
		if (wrqu->frag.value < MIN_FRAG_THRESHOLD ||
		    wrqu->frag.value > MAX_FRAG_THRESHOLD)
			return -EINVAL;

		rtlpriv->xmitpriv.frag_len = wrqu->frag.value & ~0x1;
	}

	DBG_871X("%s, frag_len=%d\n", __func__, rtlpriv->xmitpriv.frag_len);

	return 0;

}

static int rtw_wx_get_frag(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	DBG_871X("%s, frag_len=%d\n", __func__, rtlpriv->xmitpriv.frag_len);

	wrqu->frag.value = rtlpriv->xmitpriv.frag_len;
	wrqu->frag.fixed = 0;	/* no auto select */
	/* wrqu->frag.disabled = (wrqu->frag.value == DEFAULT_FRAG_THRESHOLD); */

	return 0;
}

static int rtw_wx_get_retry(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	/* struct rtl_priv *rtlpriv = rtl_priv(ndev); */


	wrqu->retry.value = 7;
	wrqu->retry.fixed = 0;	/* no auto select */
	wrqu->retry.disabled = 1;

	return 0;

}


static int rtw_wx_set_enc(struct net_device *ndev,
			    struct iw_request_info *info,
			    union iwreq_data *wrqu, char *keybuf)
{
	u32 key, ret = 0;
	u32 keyindex_provided;
	NDIS_802_11_WEP	 wep;
	NDIS_802_11_AUTHENTICATION_MODE authmode;

	struct iw_point *erq = &(wrqu->encoding);
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	DBG_871X("+rtw_wx_set_enc, flags=0x%x\n", erq->flags);

	memset(&wep, 0, sizeof(NDIS_802_11_WEP));

	key = erq->flags & IW_ENCODE_INDEX;

	if (erq->flags & IW_ENCODE_DISABLED) {
		DBG_871X("EncryptionDisabled\n");
		rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11EncryptionDisabled;
		rtlpriv->securitypriv.dot11PrivacyAlgrthm = NO_ENCRYPTION;
		rtlpriv->securitypriv.dot118021XGrpPrivacy = NO_ENCRYPTION;
		rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Open; /* open system */
		authmode = Ndis802_11AuthModeOpen;
		rtlpriv->securitypriv.ndisauthtype = authmode;

		goto exit;
	}

	if (key) {
		if (key > WEP_KEYS)
			return -EINVAL;
		key--;
		keyindex_provided = 1;
	} else {
		keyindex_provided = 0;
		key = rtlpriv->securitypriv.dot11PrivacyKeyIndex;
		DBG_871X("rtw_wx_set_enc, key=%d\n", key);
	}

	/* set authentication mode */
	if (erq->flags & IW_ENCODE_OPEN) {
		DBG_871X("rtw_wx_set_enc():IW_ENCODE_OPEN\n");
		rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;/* Ndis802_11EncryptionDisabled; */

		rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Open;

		rtlpriv->securitypriv.dot11PrivacyAlgrthm = NO_ENCRYPTION;
		rtlpriv->securitypriv.dot118021XGrpPrivacy = NO_ENCRYPTION;
		authmode = Ndis802_11AuthModeOpen;
		rtlpriv->securitypriv.ndisauthtype = authmode;
	} else if (erq->flags & IW_ENCODE_RESTRICTED) {
		DBG_871X("rtw_wx_set_enc():IW_ENCODE_RESTRICTED\n");
		rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;

		rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Shared;

		rtlpriv->securitypriv.dot11PrivacyAlgrthm = WEP40_ENCRYPTION;
		rtlpriv->securitypriv.dot118021XGrpPrivacy = WEP40_ENCRYPTION;
		authmode = Ndis802_11AuthModeShared;
		rtlpriv->securitypriv.ndisauthtype = authmode;
	} else {
		DBG_871X("rtw_wx_set_enc():erq->flags=0x%x\n", erq->flags);

		rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption1Enabled;/* Ndis802_11EncryptionDisabled; */
		rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Open; /* open system */
		rtlpriv->securitypriv.dot11PrivacyAlgrthm = NO_ENCRYPTION;
		rtlpriv->securitypriv.dot118021XGrpPrivacy = NO_ENCRYPTION;
		authmode = Ndis802_11AuthModeOpen;
		rtlpriv->securitypriv.ndisauthtype = authmode;
	}

	wep.KeyIndex = key;
	if (erq->length > 0) {
		wep.KeyLength = erq->length <= 5 ? 5 : 13;

		wep.Length = wep.KeyLength + FIELD_OFFSET(NDIS_802_11_WEP, KeyMaterial);
	} else {
		wep.KeyLength = 0 ;

		if (keyindex_provided == 1) {
			/* set key_id only, no given KeyMaterial(erq->length==0). */

			rtlpriv->securitypriv.dot11PrivacyKeyIndex = key;

			DBG_871X("(keyindex_provided == 1), keyid=%d, key_len=%d\n", key, rtlpriv->securitypriv.dot11DefKeylen[key]);

			switch (rtlpriv->securitypriv.dot11DefKeylen[key]) {
			case 5:
				rtlpriv->securitypriv.dot11PrivacyAlgrthm = WEP40_ENCRYPTION;
				break;
			case 13:
				rtlpriv->securitypriv.dot11PrivacyAlgrthm = WEP104_ENCRYPTION;
				break;
			default:
				rtlpriv->securitypriv.dot11PrivacyAlgrthm = NO_ENCRYPTION;
				break;
			}

			goto exit;

		}

	}

	wep.KeyIndex |= 0x80000000;

	memcpy(wep.KeyMaterial, keybuf, wep.KeyLength);

	if (rtw_set_802_11_add_wep(rtlpriv, &wep) == false) {
		if (ERFON == pwrpriv->rf_pwrstate)
			ret = -EOPNOTSUPP;
		goto exit;
	}

exit:



	return ret;

}

static int rtw_wx_get_enc(struct net_device *ndev,
			    struct iw_request_info *info,
			    union iwreq_data *wrqu, char *keybuf)
{
	uint key, ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct iw_point *erq = &(wrqu->encoding);
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);

	if (check_fwstate(pmlmepriv, _FW_LINKED) != true) {
		if (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) != true)  {
			erq->length = 0;
			erq->flags |= IW_ENCODE_DISABLED;
			return 0;
		}
	}


	key = erq->flags & IW_ENCODE_INDEX;

	if (key) {
		if (key > WEP_KEYS)
			return -EINVAL;
		key--;
	} else {
		key = rtlpriv->securitypriv.dot11PrivacyKeyIndex;
	}

	erq->flags = key + 1;

	/*
	 * if (rtlpriv->securitypriv.ndisauthtype == Ndis802_11AuthModeOpen) {
	 *    	erq->flags |= IW_ENCODE_OPEN;
	 * }
	 */

	switch (rtlpriv->securitypriv.ndisencryptstatus) {
	case Ndis802_11EncryptionNotSupported:
	case Ndis802_11EncryptionDisabled:
		erq->length = 0;
		erq->flags |= IW_ENCODE_DISABLED;

		break;

	case Ndis802_11Encryption1Enabled:
		erq->length = rtlpriv->securitypriv.dot11DefKeylen[key];

		if (erq->length) {
			memcpy(keybuf, rtlpriv->securitypriv.dot11DefKey[key].skey, rtlpriv->securitypriv.dot11DefKeylen[key]);

		erq->flags |= IW_ENCODE_ENABLED;

			if (rtlpriv->securitypriv.ndisauthtype == Ndis802_11AuthModeOpen) {
				erq->flags |= IW_ENCODE_OPEN;
			} else if (rtlpriv->securitypriv.ndisauthtype == Ndis802_11AuthModeShared) {
				erq->flags |= IW_ENCODE_RESTRICTED;
			}
		} else 	{
			erq->length = 0;
			erq->flags |= IW_ENCODE_DISABLED;
		}

		break;

	case Ndis802_11Encryption2Enabled:
	case Ndis802_11Encryption3Enabled:
		erq->length = 16;
		erq->flags |= (IW_ENCODE_ENABLED | IW_ENCODE_OPEN | IW_ENCODE_NOKEY);

		break;

	default:
		erq->length = 0;
		erq->flags |= IW_ENCODE_DISABLED;

		break;

	}

	return ret;

}

static int rtw_wx_get_power(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	/* struct rtl_priv *rtlpriv = rtl_priv(ndev); */

	wrqu->power.value = 0;
	wrqu->power.fixed = 0;	/* no auto select */
	wrqu->power.disabled = 1;

	return 0;

}

static int rtw_wx_set_gen_ie(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	int ret;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	ret = rtw_set_wpa_ie(rtlpriv, extra, wrqu->data.length);

	return ret;
}

static int rtw_wx_set_auth(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct iw_param *param = (struct iw_param *) &(wrqu->param);
	int ret = 0;

	switch (param->flags & IW_AUTH_INDEX) {
	case IW_AUTH_WPA_VERSION:
		break;
	case IW_AUTH_CIPHER_PAIRWISE:

		break;
	case IW_AUTH_CIPHER_GROUP:

		break;
	case IW_AUTH_KEY_MGMT:
		/*
		 *  ??? does not use these parameters
		 */
		break;

	case IW_AUTH_TKIP_COUNTERMEASURES:
		{
			if (param->value) {
				/* wpa_supplicant is enabling the tkip countermeasure. */
				rtlpriv->securitypriv.btkip_countermeasure = true;
			} else {
				/* wpa_supplicant is disabling the tkip countermeasure. */
				rtlpriv->securitypriv.btkip_countermeasure = false;
			}
			break;
		}
	case IW_AUTH_DROP_UNENCRYPTED:
		{
			/* HACK:
			 *
			 * wpa_supplicant calls set_wpa_enabled when the driver
			 * is loaded and unloaded, regardless of if WPA is being
			 * used.  No other calls are made which can be used to
			 * determine if encryption will be used or not prior to
			 * association being expected.  If encryption is not being
			 * used, drop_unencrypted is set to false, else true -- we
			 * can use this to determine if the CAP_PRIVACY_ON bit should
			 * be set.
			 */

			if (rtlpriv->securitypriv.ndisencryptstatus == Ndis802_11Encryption1Enabled) {
				break;	/* it means init value, or using wep, ndisencrypttatus = Ndis802_11Encryption1Enabled, */
					/* then it needn't reset it; */
			}

			if (param->value) {
				rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11EncryptionDisabled;
				rtlpriv->securitypriv.dot11PrivacyAlgrthm = NO_ENCRYPTION;
				rtlpriv->securitypriv.dot118021XGrpPrivacy = NO_ENCRYPTION;
				rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Open; /* open system */
				rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeOpen;
			}
			break;
		}

	case IW_AUTH_80211_AUTH_ALG:

		/*
		 *  It's the starting point of a link layer connection using wpa_supplicant
		*/
		if (check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED)) {
			LeaveAllPowerSaveMode(rtlpriv);
			rtw_disassoc_cmd(rtlpriv, 500, false);
			DBG_871X("%s...call rtw_indicate_disconnect\n ", __FUNCTION__);
			rtw_indicate_disconnect(rtlpriv);
			rtw_free_assoc_resources(rtlpriv, 1);
		}


		ret = wpa_set_auth_algs(ndev, (u32)param->value);

		break;

	case IW_AUTH_WPA_ENABLED:

		/*
		 * if (param->value)
		 * 	rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_8021X; //802.1x
		 * else
		 * 	rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_Open;//open system
		 *
		 * _disassociate(priv);
		 */

		break;

	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
		/* ieee->ieee802_1x = param->value; */
		break;

	case IW_AUTH_PRIVACY_INVOKED:
		/* ieee->privacy_invoked = param->value; */
		break;
	default:
		return -EOPNOTSUPP;

	}

	return ret;

}

static int rtw_wx_set_enc_ext(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	char *alg_name;
	u32 param_len;
	struct ieee_param *param = NULL;
	struct iw_point *pencoding = &wrqu->encoding;
	struct iw_encode_ext *pext = (struct iw_encode_ext *) extra;
	int ret = 0;

	param_len = sizeof(struct ieee_param) + pext->key_len;
	param = (struct ieee_param *)rtw_malloc(param_len);
	if (param == NULL)
		return -1;

	memset(param, 0, param_len);

	param->cmd = IEEE_CMD_SET_ENCRYPTION;
	memset(param->sta_addr, 0xff, ETH_ALEN);


	switch (pext->alg) {
	case IW_ENCODE_ALG_NONE:
		/*
		 * todo: remove key
		 * remove = 1;
		 */
		alg_name = "none";
		break;
	case IW_ENCODE_ALG_WEP:
		alg_name = "WEP";
		break;
	case IW_ENCODE_ALG_TKIP:
		alg_name = "TKIP";
		break;
	case IW_ENCODE_ALG_CCMP:
		alg_name = "CCMP";
		break;
	default:
		return -1;
	}

	strncpy((char *)param->u.crypt.alg, alg_name, IEEE_CRYPT_ALG_NAME_LEN);

	if (pext->ext_flags & IW_ENCODE_EXT_SET_TX_KEY) {
		param->u.crypt.set_tx = 1;
	}

	/* cliW: WEP does not have group key
	 * just not checking GROUP key setting
	 */
	if ((pext->alg != IW_ENCODE_ALG_WEP) &&
		(pext->ext_flags & IW_ENCODE_EXT_GROUP_KEY)) {
		param->u.crypt.set_tx = 0;
	}

	param->u.crypt.idx = (pencoding->flags&0x00FF) - 1;

	if (pext->ext_flags & IW_ENCODE_EXT_RX_SEQ_VALID) {
		memcpy(param->u.crypt.seq, pext->rx_seq, 8);
	}

	if (pext->key_len) {
		param->u.crypt.key_len = pext->key_len;
		/* memcpy(param + 1, pext + 1, pext->key_len); */
		memcpy(param->u.crypt.key, pext + 1, pext->key_len);
	}

	if (pencoding->flags & IW_ENCODE_DISABLED) {
		/*
		 * todo: remove key
		 * remove = 1;
		 */
	}

	ret =  wpa_set_encryption(ndev, param, param_len);

	if (param) {
		rtw_mfree(param);
	}

	return ret;
}


static int rtw_wx_get_nick(struct net_device *ndev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	/* struct rtl_priv *rtlpriv = rtl_priv(ndev); */
	/* struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv); */
	/* struct security_priv *psecuritypriv = &rtlpriv->securitypriv; */

	if (extra) {
		wrqu->data.length = 14;
		wrqu->data.flags = 1;
		memcpy(extra, "<WIFI@REALTEK>", 14);
	}


	/* dump debug info here */

	/*
	 * DBG_871X("auth_alg=0x%x, enc_alg=0x%x, auth_type=0x%x, enc_type=0x%x\n",
	 * 		psecuritypriv->dot11AuthAlgrthm, psecuritypriv->dot11PrivacyAlgrthm,
	 * 		psecuritypriv->ndisauthtype, psecuritypriv->ndisencryptstatus);
	 */

	/*
	 * DBG_871X("enc_alg=0x%x\n", psecuritypriv->dot11PrivacyAlgrthm);
	 * DBG_871X("auth_type=0x%x\n", psecuritypriv->ndisauthtype);
	 * DBG_871X("enc_type=0x%x\n", psecuritypriv->ndisencryptstatus);
	 */


	return 0;

}

static int rtw_wx_priv_null(struct net_device *ndev, struct iw_request_info *a,
		 union iwreq_data *wrqu, char *b)
{
	return -1;
}

static int dummy(struct net_device *ndev, struct iw_request_info *a,
		 union iwreq_data *wrqu, char *b)
{
	/* struct rtl_priv *rtlpriv = rtl_priv(ndev); */
	/* struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv); */

	/* DBG_871X("cmd_code=%x, fwstate=0x%x\n", a->cmd, get_fwstate(pmlmepriv)); */

	return -1;

}




static int rtw_cta_test_start(struct net_device *ndev,
							   struct iw_request_info *info,
							   union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	struct rtl_priv	*rtlpriv = rtl_priv(ndev);

	DBG_871X("%s %s\n", __func__, extra);

	if (!strcmp(extra, "1"))
		rtlpriv->in_cta_test = 1;
	else
		rtlpriv->in_cta_test = 0;

	if (rtlpriv->in_cta_test) {
		u32 v = rtl_read_dword(rtlpriv, REG_RCR);
		v &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);	/* | RCR_ADF */
		rtl_write_dword(rtlpriv, REG_RCR, v);
		DBG_871X("enable RCR_ADF\n");
	} else {
		u32 v = rtl_read_dword(rtlpriv, REG_RCR);
		v |= RCR_CBSSID_DATA | RCR_CBSSID_BCN ;		/* | RCR_ADF */
		rtl_write_dword(rtlpriv, REG_RCR, v);
		DBG_871X("disable RCR_ADF\n");
	}
	return ret;
}


void mac_reg_dump(struct rtl_priv *rtlpriv)
{
	int i, j = 1;
	printk("\n======= MAC REG =======\n");
	for (i = 0x0; i < 0x300; i += 4) {
		if (j % 4 == 1)
			printk("0x%02x", i);
		printk(" 0x%08x ", rtl_read_dword(rtlpriv, i));
		if ((j++) % 4 == 0)
			printk("\n");
	}
	for (i = 0x400; i < 0x800; i += 4) {
		if (j % 4 == 1)
			printk("0x%02x", i);
		printk(" 0x%08x ", rtl_read_dword(rtlpriv, i));
		if ((j++) % 4 == 0)
			printk("\n");
	}
}
void bb_reg_dump(struct rtl_priv *rtlpriv)
{
	int i, j = 1;
	printk("\n======= BB REG =======\n");
	for (i = 0x800; i < 0x1000; i += 4) {
		if (j % 4 == 1)
			printk("0x%02x", i);

		printk(" 0x%08x ", rtl_read_dword(rtlpriv, i));
		if ((j++)%4 == 0)
			printk("\n");
	}
}
void rf_reg_dump(struct rtl_priv *rtlpriv)
{
	int i, j = 1, path;
	u32 value;
	uint8_t rf_type, path_nums = 0;
	rf_type = rtlpriv->phy.rf_type;

	printk("\n======= RF REG =======\n");
	if ((RF_1T2R == rf_type) || (RF_1T1R == rf_type))
		path_nums = 1;
	else
		path_nums = 2;

	for (path = 0; path < path_nums; path++) {
		printk("\nRF_Path(%x)\n", path);
		for (i = 0; i < 0x100; i++) {
			/* value = rtl_get_rfreg(rtlpriv, path,i, bMaskDWord); */
			value = rtl_get_rfreg(rtlpriv, path, i, 0xffffffff);
			if (j % 4 == 1)
				printk("0x%02x ", i);

			printk(" 0x%08x ", value);
			if ((j++) % 4 == 0)
				printk("\n");
		}
	}
}


static int wpa_set_param(struct net_device *ndev, uint8_t name, u32 value)
{
	uint ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	switch (name) {
	case IEEE_PARAM_WPA_ENABLED:
		rtlpriv->securitypriv.dot11AuthAlgrthm = dot11AuthAlgrthm_8021X; /* 802.1x */

		/* ret = ieee80211_wpa_enable(ieee, value); */

		switch (value & 0xff) {
		case 1: /* WPA */
			rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeWPAPSK; /* WPA_PSK */
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption2Enabled;
			break;
		case 2: /* WPA2 */
			rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeWPA2PSK; /* WPA2_PSK */
			rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11Encryption3Enabled;
			break;
		}

		break;

	case IEEE_PARAM_TKIP_COUNTERMEASURES:
		/* ieee->tkip_countermeasures=value; */
		break;

	case IEEE_PARAM_DROP_UNENCRYPTED:
		{
			/* HACK:
			 *
			 * wpa_supplicant calls set_wpa_enabled when the driver
			 * is loaded and unloaded, regardless of if WPA is being
			 * used.  No other calls are made which can be used to
			 * determine if encryption will be used or not prior to
			 * association being expected.  If encryption is not being
			 * used, drop_unencrypted is set to false, else true -- we
			 * can use this to determine if the CAP_PRIVACY_ON bit should
			 * be set.
			 */

			break;

		}
	case IEEE_PARAM_PRIVACY_INVOKED:
		/* ieee->privacy_invoked=value; */
		break;

	case IEEE_PARAM_AUTH_ALGS:
		ret = wpa_set_auth_algs(ndev, value);
		break;

	case IEEE_PARAM_IEEE_802_1X:
		/* ieee->ieee802_1x=value; */
		break;

	case IEEE_PARAM_WPAX_SELECT:

		/* added for WPA2 mixed mode
		 * DBG_871X(KERN_WARNING "------------------------>wpax value = %x\n", value);
		 */
		/*
		spin_lock_irqsave(&ieee->wpax_suitlist_lock,flags);
		ieee->wpax_type_set = 1;
		ieee->wpax_type_notify = value;
		spin_unlock_irqrestore(&ieee->wpax_suitlist_lock,flags);
		*/

		break;

	default:
		ret = -EOPNOTSUPP;

		break;

	}

	return ret;

}

static int wpa_mlme(struct net_device *ndev, u32 command, u32 reason)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	switch (command) {
	case IEEE_MLME_STA_DEAUTH:
		if (!rtw_set_802_11_disassociate(rtlpriv))
			ret = -1;

		break;

	case IEEE_MLME_STA_DISASSOC:
		if (!rtw_set_802_11_disassociate(rtlpriv))
			ret = -1;

		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;

}

static int wpa_supplicant_ioctl(struct net_device *ndev, struct iw_point *p)
{
	struct ieee_param *param;
	uint ret = 0;

	/* down(&ieee->wx_sem); */

	if (p->length < sizeof(struct ieee_param) || !p->pointer) {
		ret = -EINVAL;
		goto out;
	}

	param = (struct ieee_param *)rtw_malloc(p->length);
	if (param == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(param, p->pointer, p->length)) {
		rtw_mfree(param);
		ret = -EFAULT;
		goto out;
	}

	switch (param->cmd) {
	case IEEE_CMD_SET_WPA_PARAM:
		ret = wpa_set_param(ndev, param->u.wpa_param.name, param->u.wpa_param.value);
		break;

	case IEEE_CMD_SET_WPA_IE:
		/* ret = wpa_set_wpa_ie(ndev, param, p->length); */
		ret =  rtw_set_wpa_ie(rtl_priv(ndev), (char *)param->u.wpa_ie.data, (u16)param->u.wpa_ie.len);
		break;

	case IEEE_CMD_SET_ENCRYPTION:
		ret = wpa_set_encryption(ndev, param, p->length);
		break;

	case IEEE_CMD_MLME:
		ret = wpa_mlme(ndev, param->u.mlme.command, param->u.mlme.reason_code);
		break;

	default:
		DBG_871X("Unknown WPA supplicant request: %d\n", param->cmd);
		ret = -EOPNOTSUPP;
		break;

	}

	if (ret == 0 && copy_to_user(p->pointer, param, p->length))
		ret = -EFAULT;

	rtw_mfree(param);

out:

	/* up(&ieee->wx_sem); */

	return ret;

}

#ifdef CONFIG_AP_MODE
static int rtw_set_encryption(struct net_device *ndev, struct ieee_param *param, u32 param_len)
{
	int ret = 0;
	u32 wep_key_idx, wep_key_len, wep_total_len;
	NDIS_802_11_WEP	 *pwep = NULL;
	struct sta_info *psta = NULL, *pbcmc_sta = NULL;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv 	*pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv *psecuritypriv = &(rtlpriv->securitypriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

	DBG_871X("%s\n", __FUNCTION__);

	param->u.crypt.err = 0;
	param->u.crypt.alg[IEEE_CRYPT_ALG_NAME_LEN - 1] = '\0';

	/* sizeof(struct ieee_param) = 64 bytes; */
	/* if (param_len !=  (u32) ((uint8_t *) param->u.crypt.key - (uint8_t *) param) + param->u.crypt.key_len) */
	if (param_len !=  sizeof(struct ieee_param) + param->u.crypt.key_len) {
		ret =  -EINVAL;
		goto exit;
	}

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) {
		if (param->u.crypt.idx >= WEP_KEYS) {
			ret = -EINVAL;
			goto exit;
		}
	} else {
		psta = rtw_get_stainfo(pstapriv, param->sta_addr);
		if (!psta) {
			/* ret = -EINVAL; */
			DBG_871X("rtw_set_encryption(), sta has already been removed or never been added\n");
			goto exit;
		}
	}

	if (strcmp(param->u.crypt.alg, "none") == 0 && (psta == NULL)) {
		/* todo:clear default encryption keys */

		psecuritypriv->dot11AuthAlgrthm = dot11AuthAlgrthm_Open;
		psecuritypriv->ndisencryptstatus = Ndis802_11EncryptionDisabled;
		psecuritypriv->dot11PrivacyAlgrthm = NO_ENCRYPTION;
		psecuritypriv->dot118021XGrpPrivacy = NO_ENCRYPTION;

		DBG_871X("clear default encryption keys, keyid=%d\n", param->u.crypt.idx);

		goto exit;
	}

	if (strcmp(param->u.crypt.alg, "WEP") == 0 && (psta == NULL)) {
		DBG_871X("r871x_set_encryption, crypt.alg = WEP\n");

		wep_key_idx = param->u.crypt.idx;
		wep_key_len = param->u.crypt.key_len;

		DBG_871X("r871x_set_encryption, wep_key_idx=%d, len=%d\n", wep_key_idx, wep_key_len);

		if ((wep_key_idx >= WEP_KEYS) || (wep_key_len <= 0)) {
			ret = -EINVAL;
			goto exit;
		}


		if (wep_key_len > 0) {
			wep_key_len = wep_key_len <= 5 ? 5 : 13;
			wep_total_len = wep_key_len + FIELD_OFFSET(NDIS_802_11_WEP, KeyMaterial);
			pwep = (NDIS_802_11_WEP *)rtw_malloc(wep_total_len);
			if (pwep == NULL) {
				DBG_871X(" r871x_set_encryption: pwep allocate fail !!!\n");
				goto exit;
			}

			memset(pwep, 0, wep_total_len);

			pwep->KeyLength = wep_key_len;
			pwep->Length = wep_total_len;

		}

		pwep->KeyIndex = wep_key_idx;

		memcpy(pwep->KeyMaterial,  param->u.crypt.key, pwep->KeyLength);

		if (param->u.crypt.set_tx) {
			DBG_871X("wep, set_tx=1\n");

			psecuritypriv->dot11AuthAlgrthm = dot11AuthAlgrthm_Auto;
			psecuritypriv->ndisencryptstatus = Ndis802_11Encryption1Enabled;
			psecuritypriv->dot11PrivacyAlgrthm = WEP40_ENCRYPTION;
			psecuritypriv->dot118021XGrpPrivacy = WEP40_ENCRYPTION;

			if (pwep->KeyLength == 13) {
				psecuritypriv->dot11PrivacyAlgrthm = WEP104_ENCRYPTION;
				psecuritypriv->dot118021XGrpPrivacy = WEP104_ENCRYPTION;
			}


			psecuritypriv->dot11PrivacyKeyIndex = wep_key_idx;

			memcpy(&(psecuritypriv->dot11DefKey[wep_key_idx].skey[0]), pwep->KeyMaterial, pwep->KeyLength);

			psecuritypriv->dot11DefKeylen[wep_key_idx] = pwep->KeyLength;

			rtw_ap_set_wep_key(rtlpriv, pwep->KeyMaterial, pwep->KeyLength, wep_key_idx, 1);
				} else 	{
			DBG_871X("wep, set_tx=0\n");

			/*
			 * don't update "psecuritypriv->dot11PrivacyAlgrthm" and
			 * "psecuritypriv->dot11PrivacyKeyIndex=keyid", but can rtw_set_key to cam
			 */

			memcpy(&(psecuritypriv->dot11DefKey[wep_key_idx].skey[0]), pwep->KeyMaterial, pwep->KeyLength);

			psecuritypriv->dot11DefKeylen[wep_key_idx] = pwep->KeyLength;

			rtw_ap_set_wep_key(rtlpriv, pwep->KeyMaterial, pwep->KeyLength, wep_key_idx, 0);
		}

		goto exit;

	}


	if (!psta && check_fwstate(pmlmepriv, WIFI_AP_STATE)) {
		/* group key */
		if (param->u.crypt.set_tx == 1) {
			if (strcmp(param->u.crypt.alg, "WEP") == 0) {
				DBG_871X("%s, set group_key, WEP\n", __FUNCTION__);

				memcpy(psecuritypriv->dot118021XGrpKey[param->u.crypt.idx].skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));

				psecuritypriv->dot118021XGrpPrivacy = WEP40_ENCRYPTION;
				if (param->u.crypt.key_len == 13) {
						psecuritypriv->dot118021XGrpPrivacy = WEP104_ENCRYPTION;
				}

			} else if (strcmp(param->u.crypt.alg, "TKIP") == 0) {
				DBG_871X("%s, set group_key, TKIP\n", __FUNCTION__);

				psecuritypriv->dot118021XGrpPrivacy = TKIP_ENCRYPTION;

				memcpy(psecuritypriv->dot118021XGrpKey[param->u.crypt.idx].skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));

				/*
				 * DEBUG_ERR("set key length :param->u.crypt.key_len=%d\n", param->u.crypt.key_len);
				 * set mic key
				 */
				memcpy(psecuritypriv->dot118021XGrptxmickey[param->u.crypt.idx].skey, &(param->u.crypt.key[16]), 8);
				memcpy(psecuritypriv->dot118021XGrprxmickey[param->u.crypt.idx].skey, &(param->u.crypt.key[24]), 8);

				psecuritypriv->busetkipkey = true;

			} else if (strcmp(param->u.crypt.alg, "CCMP") == 0) {
				DBG_871X("%s, set group_key, CCMP\n", __FUNCTION__);

				psecuritypriv->dot118021XGrpPrivacy = AESCCMP_ENCRYPTION;

				memcpy(psecuritypriv->dot118021XGrpKey[param->u.crypt.idx].skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));
			} else {
				DBG_871X("%s, set group_key, none\n", __FUNCTION__);

				psecuritypriv->dot118021XGrpPrivacy = NO_ENCRYPTION;
			}

			psecuritypriv->dot118021XGrpKeyid = param->u.crypt.idx;

			psecuritypriv->binstallGrpkey = true;

			psecuritypriv->dot11PrivacyAlgrthm = psecuritypriv->dot118021XGrpPrivacy;/* !!! */

			rtw_ap_set_group_key(rtlpriv, param->u.crypt.key, psecuritypriv->dot118021XGrpPrivacy, param->u.crypt.idx);

			pbcmc_sta = rtw_get_bcmc_stainfo(rtlpriv);
			if (pbcmc_sta) {
				pbcmc_sta->ieee8021x_blocked = false;
				pbcmc_sta->dot118021XPrivacy = psecuritypriv->dot118021XGrpPrivacy;/* rx will use bmc_sta's dot118021XPrivacy */
			}

		}

		goto exit;

	}

	if (psecuritypriv->dot11AuthAlgrthm == dot11AuthAlgrthm_8021X && psta) {	/* psk/802_1x */
		if (check_fwstate(pmlmepriv, WIFI_AP_STATE)) {
			if (param->u.crypt.set_tx == 1)	{
				memcpy(psta->dot118021x_UncstKey.skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));

				if (strcmp(param->u.crypt.alg, "WEP") == 0) {
					DBG_871X("%s, set pairwise key, WEP\n", __FUNCTION__);

					psta->dot118021XPrivacy = WEP40_ENCRYPTION;
					if (param->u.crypt.key_len == 13) {
						psta->dot118021XPrivacy = WEP104_ENCRYPTION;
					}
				} else if (strcmp(param->u.crypt.alg, "TKIP") == 0) {
					DBG_871X("%s, set pairwise key, TKIP\n", __FUNCTION__);

					psta->dot118021XPrivacy = TKIP_ENCRYPTION;

					/*
					 * DEBUG_ERR("set key length :param->u.crypt.key_len=%d\n", param->u.crypt.key_len);
					 * set mic key
					 */
					memcpy(psta->dot11tkiptxmickey.skey, &(param->u.crypt.key[16]), 8);
					memcpy(psta->dot11tkiprxmickey.skey, &(param->u.crypt.key[24]), 8);

					psecuritypriv->busetkipkey = true;

				} else if (strcmp(param->u.crypt.alg, "CCMP") == 0) {
					DBG_871X("%s, set pairwise key, CCMP\n", __FUNCTION__);

					psta->dot118021XPrivacy = AESCCMP_ENCRYPTION;
				} else {
					DBG_871X("%s, set pairwise key, none\n", __FUNCTION__);

					psta->dot118021XPrivacy = NO_ENCRYPTION;
				}

				rtw_ap_set_pairwise_key(rtlpriv, psta);

				psta->ieee8021x_blocked = false;

			} else {
				/* group key??? */
				if (strcmp(param->u.crypt.alg, "WEP") == 0) {
					memcpy(psecuritypriv->dot118021XGrpKey[param->u.crypt.idx].skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));

					psecuritypriv->dot118021XGrpPrivacy = WEP40_ENCRYPTION;
					if (param->u.crypt.key_len == 13) {
						psecuritypriv->dot118021XGrpPrivacy = WEP104_ENCRYPTION;
					}
				} else if (strcmp(param->u.crypt.alg, "TKIP") == 0) {
					psecuritypriv->dot118021XGrpPrivacy = TKIP_ENCRYPTION;

					memcpy(psecuritypriv->dot118021XGrpKey[param->u.crypt.idx].skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));

					/*
					 * DEBUG_ERR("set key length :param->u.crypt.key_len=%d\n", param->u.crypt.key_len);
					 * set mic key
					 */
					memcpy(psecuritypriv->dot118021XGrptxmickey[param->u.crypt.idx].skey, &(param->u.crypt.key[16]), 8);
					memcpy(psecuritypriv->dot118021XGrprxmickey[param->u.crypt.idx].skey, &(param->u.crypt.key[24]), 8);

					psecuritypriv->busetkipkey = true;

				} else if (strcmp(param->u.crypt.alg, "CCMP") == 0) {
					psecuritypriv->dot118021XGrpPrivacy = AESCCMP_ENCRYPTION;

					memcpy(psecuritypriv->dot118021XGrpKey[param->u.crypt.idx].skey,  param->u.crypt.key, (param->u.crypt.key_len > 16 ? 16 : param->u.crypt.key_len));
				} else {
					psecuritypriv->dot118021XGrpPrivacy = NO_ENCRYPTION;
				}

				psecuritypriv->dot118021XGrpKeyid = param->u.crypt.idx;

				psecuritypriv->binstallGrpkey = true;

				psecuritypriv->dot11PrivacyAlgrthm = psecuritypriv->dot118021XGrpPrivacy;	/* !!! */

				rtw_ap_set_group_key(rtlpriv, param->u.crypt.key, psecuritypriv->dot118021XGrpPrivacy, param->u.crypt.idx);

				pbcmc_sta = rtw_get_bcmc_stainfo(rtlpriv);
				if (pbcmc_sta) {
					pbcmc_sta->ieee8021x_blocked = false;
					pbcmc_sta->dot118021XPrivacy = psecuritypriv->dot118021XGrpPrivacy;	/* rx will use bmc_sta's dot118021XPrivacy */
				}

			}

		}

	}

exit:

	if (pwep) {
		rtw_mfree(pwep);
	}

	return ret;

}

static int rtw_set_beacon(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	unsigned char *pbuf = param->u.bcn_ie.buf;


	DBG_871X("%s, len=%d\n", __FUNCTION__, len);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return -EINVAL;

	memcpy(&pstapriv->max_num_sta, param->u.bcn_ie.reserved, 2);

	if ((pstapriv->max_num_sta > NUM_STA) || (pstapriv->max_num_sta <= 0))
		pstapriv->max_num_sta = NUM_STA;

	if (rtw_check_beacon_data(rtlpriv, pbuf,  (len-12-2)) == _SUCCESS)	/* 12 = param header, 2:no packed */
		ret = 0;
	else
		ret = -EINVAL;

	return ret;

}

static int rtw_hostapd_sta_flush(struct net_device *ndev)
{
	/*
	 * _irqL irqL;
	 * struct list_head	*phead, *plist;
	 */
	int ret = 0;
	/* struct sta_info *psta = NULL; */
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	/* struct sta_priv *pstapriv = &rtlpriv->stapriv; */

	DBG_871X("%s\n", __FUNCTION__);

	flush_all_cam_entry(rtlpriv);	/* clear CAM */

	ret = rtw_sta_flush(rtlpriv);

	return ret;

}

static int rtw_add_sta(struct net_device *ndev, struct ieee_param *param)
{
	int ret = 0;
	struct sta_info *psta = NULL;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

	DBG_871X("rtw_add_sta(aid=%d)=" MAC_FMT "\n", param->u.add_sta.aid, MAC_ARG(param->sta_addr));

	if (check_fwstate(pmlmepriv, (_FW_LINKED|WIFI_AP_STATE)) != true) {
		return -EINVAL;
	}

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) {
		return -EINVAL;
	}

/*
	psta = rtw_get_stainfo(pstapriv, param->sta_addr);
	if (psta)
	{
		DBG_871X("rtw_add_sta(), free has been added psta=%p\n", psta);
		spin_lock_bh(&(pstapriv->sta_hash_lock), &irqL);
		rtw_free_stainfo(rtlpriv,  psta);
		spin_unlock_bh(&(pstapriv->sta_hash_lock), &irqL);

		psta = NULL;
	}
*/
	/* psta = rtw_alloc_stainfo(pstapriv, param->sta_addr); */
	psta = rtw_get_stainfo(pstapriv, param->sta_addr);
	if (psta) {
		int flags = param->u.add_sta.flags;

		/* DBG_871X("rtw_add_sta(), init sta's variables, psta=%p\n", psta); */

		psta->aid = param->u.add_sta.aid;	/* aid=1~2007 */

		memcpy(psta->bssrateset, param->u.add_sta.tx_supp_rates, 16);


		/* check wmm cap. */
		if (WLAN_STA_WME&flags)
			psta->qos_option = 1;
		else
			psta->qos_option = 0;

		if (pmlmepriv->qospriv.qos_option == 0)
			psta->qos_option = 0;

		/* chec 802.11n ht cap. */
		if (WLAN_STA_HT&flags) {
			psta->htpriv.ht_option = true;
			psta->qos_option = 1;
			memcpy(&psta->htpriv.ht_cap, &param->u.add_sta.ht_cap, sizeof(struct rtw_ieee80211_ht_cap));
		} else {
			psta->htpriv.ht_option = false;
		}

		if (pmlmepriv->htpriv.ht_option == false)
			psta->htpriv.ht_option = false;

		update_sta_info_apmode(rtlpriv, psta);
	} else {
		ret = -ENOMEM;
	}

	return ret;

}

static int rtw_del_sta(struct net_device *ndev, struct ieee_param *param)
{
	int ret = 0;
	struct sta_info *psta = NULL;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

	DBG_871X("rtw_del_sta=" MAC_FMT "\n", MAC_ARG(param->sta_addr));

	if (check_fwstate(pmlmepriv, (_FW_LINKED|WIFI_AP_STATE)) != true) {
		return -EINVAL;
	}

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) {
		return -EINVAL;
	}

	psta = rtw_get_stainfo(pstapriv, param->sta_addr);
	if (psta) {
		uint8_t updated;

		/* DBG_871X("free psta=%p, aid=%d\n", psta, psta->aid); */

		spin_lock_bh(&pstapriv->asoc_list_lock);
		if (list_empty(&psta->asoc_list) == false) {
			list_del_init(&psta->asoc_list);
			pstapriv->asoc_list_cnt--;
			updated = ap_free_sta(rtlpriv, psta, true, WLAN_REASON_DEAUTH_LEAVING);

		}
		spin_unlock_bh(&pstapriv->asoc_list_lock);

		associated_clients_update(rtlpriv, updated);

		psta = NULL;

	} else {
		DBG_871X("rtw_del_sta(), sta has already been removed or never been added\n");

		/* ret = -1; */
	}


	return ret;

}

static int rtw_ioctl_get_sta_data(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct sta_info *psta = NULL;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct ieee_param_ex *param_ex = (struct ieee_param_ex *)param;
	struct sta_data *psta_data = (struct sta_data *)param_ex->data;

	DBG_871X("rtw_ioctl_get_sta_info, sta_addr: " MAC_FMT "\n", MAC_ARG(param_ex->sta_addr));

	if (check_fwstate(pmlmepriv, (_FW_LINKED|WIFI_AP_STATE)) != true) {
		return -EINVAL;
	}

	if (param_ex->sta_addr[0] == 0xff && param_ex->sta_addr[1] == 0xff &&
	    param_ex->sta_addr[2] == 0xff && param_ex->sta_addr[3] == 0xff &&
	    param_ex->sta_addr[4] == 0xff && param_ex->sta_addr[5] == 0xff) {
		return -EINVAL;
	}

	psta = rtw_get_stainfo(pstapriv, param_ex->sta_addr);
	if (psta) {
		psta_data->aid = (u16)psta->aid;
		psta_data->capability = psta->capability;
		psta_data->flags = psta->flags;

/*
		nonerp_set : BIT(0)
		no_short_slot_time_set : BIT(1)
		no_short_preamble_set : BIT(2)
		no_ht_gf_set : BIT(3)
		no_ht_set : BIT(4)
		ht_20mhz_set : BIT(5)
*/

		psta_data->sta_set = ((psta->nonerp_set) |
							(psta->no_short_slot_time_set << 1) |
							(psta->no_short_preamble_set << 2) |
							(psta->no_ht_gf_set << 3) |
							(psta->no_ht_set << 4) |
							(psta->ht_20mhz_set << 5));

		psta_data->tx_supp_rates_len =  psta->bssratelen;
		memcpy(psta_data->tx_supp_rates, psta->bssrateset, psta->bssratelen);
		memcpy(&psta_data->ht_cap, &psta->htpriv.ht_cap, sizeof(struct rtw_ieee80211_ht_cap));
		psta_data->rx_pkts = psta->sta_stats.rx_data_pkts;
		psta_data->rx_bytes = psta->sta_stats.rx_bytes;
		psta_data->rx_drops = psta->sta_stats.rx_drops;

		psta_data->tx_pkts = psta->sta_stats.tx_pkts;
		psta_data->tx_bytes = psta->sta_stats.tx_bytes;
		psta_data->tx_drops = psta->sta_stats.tx_drops;


	} else {
		ret = -1;
	}

	return ret;

}

static int rtw_get_sta_wpaie(struct net_device *ndev, struct ieee_param *param)
{
	int ret = 0;
	struct sta_info *psta = NULL;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

	DBG_871X("rtw_get_sta_wpaie, sta_addr: " MAC_FMT "\n", MAC_ARG(param->sta_addr));

	if (check_fwstate(pmlmepriv, (_FW_LINKED|WIFI_AP_STATE)) != true) {
		return -EINVAL;
	}

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) {
		return -EINVAL;
	}

	psta = rtw_get_stainfo(pstapriv, param->sta_addr);
	if (psta) {
		if ((psta->wpa_ie[0] == WLAN_EID_RSN) || (psta->wpa_ie[0] == WLAN_EID_GENERIC)) {
			int wpa_ie_len;
			int copy_len;

			wpa_ie_len = psta->wpa_ie[1];

			copy_len = ((wpa_ie_len+2) > sizeof(psta->wpa_ie)) ? (sizeof(psta->wpa_ie)):(wpa_ie_len+2);

			param->u.wpa_ie.len = copy_len;

			memcpy(param->u.wpa_ie.reserved, psta->wpa_ie, copy_len);
		} else {
			/* ret = -1; */
			DBG_871X("sta's wpa_ie is NONE\n");
		}
	} else {
		ret = -1;
	}

	return ret;

}

static int rtw_set_wps_beacon(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	unsigned char wps_oui[4] = { 0x0, 0x50, 0xf2, 0x04};
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	int ie_len;

	DBG_871X("%s, len=%d\n", __FUNCTION__, len);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return -EINVAL;

	ie_len = len - 12 - 2;	/* 12 = param header, 2:no packed */

	if (pmlmepriv->wps_beacon_ie) {
		rtw_mfree(pmlmepriv->wps_beacon_ie);
		pmlmepriv->wps_beacon_ie = NULL;
	}

	if (ie_len > 0) {
		pmlmepriv->wps_beacon_ie = rtw_malloc(ie_len);
		pmlmepriv->wps_beacon_ie_len = ie_len;
		if (pmlmepriv->wps_beacon_ie == NULL) {
			DBG_871X("%s()-%d: rtw_malloc() ERROR!\n", __FUNCTION__, __LINE__);
			return -EINVAL;
		}

		memcpy(pmlmepriv->wps_beacon_ie, param->u.bcn_ie.buf, ie_len);

		update_beacon(rtlpriv, _VENDOR_SPECIFIC_IE_, wps_oui, true);

		pmlmeext->bstart_bss = true;

	}

	return ret;

}

static int rtw_set_wps_probe_resp(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	int ie_len;

	DBG_871X("%s, len=%d\n", __FUNCTION__, len);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return -EINVAL;

	ie_len = len - 12 - 2;	/* 12 = param header, 2:no packed */


	if (pmlmepriv->wps_probe_resp_ie) {
		/* ULLI check usage of pmlmepriv->wps_probe_resp_ie_len */
		rtw_mfree(pmlmepriv->wps_probe_resp_ie);
		pmlmepriv->wps_probe_resp_ie = NULL;
	}

	if (ie_len > 0) {
		pmlmepriv->wps_probe_resp_ie = rtw_malloc(ie_len);
		pmlmepriv->wps_probe_resp_ie_len = ie_len;
		if (pmlmepriv->wps_probe_resp_ie == NULL) {
			DBG_871X("%s()-%d: rtw_malloc() ERROR!\n", __FUNCTION__, __LINE__);
			return -EINVAL;
		}
		memcpy(pmlmepriv->wps_probe_resp_ie, param->u.bcn_ie.buf, ie_len);
	}


	return ret;

}

static int rtw_set_wps_assoc_resp(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	int ie_len;

	DBG_871X("%s, len=%d\n", __FUNCTION__, len);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return -EINVAL;

	ie_len = len - 12 - 2;	/* 12 = param header, 2:no packed */


	if (pmlmepriv->wps_assoc_resp_ie) {
		/* ULLI check usage of pmlmepriv->wps_assoc_resp_ie_len */
		rtw_mfree(pmlmepriv->wps_assoc_resp_ie);
		pmlmepriv->wps_assoc_resp_ie = NULL;
	}

	if (ie_len > 0) {
		pmlmepriv->wps_assoc_resp_ie = rtw_malloc(ie_len);
		pmlmepriv->wps_assoc_resp_ie_len = ie_len;
		if (pmlmepriv->wps_assoc_resp_ie == NULL) {
			DBG_871X("%s()-%d: rtw_malloc() ERROR!\n", __FUNCTION__, __LINE__);
			return -EINVAL;
		}

		memcpy(pmlmepriv->wps_assoc_resp_ie, param->u.bcn_ie.buf, ie_len);
	}


	return ret;

}

static int rtw_set_hidden_ssid(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *mlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*mlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*mlmeinfo = &(mlmeext->mlmext_info);
	int ie_len;
	uint8_t *ssid_ie;
	char ssid[NDIS_802_11_LENGTH_SSID + 1];
	int ssid_len;
	uint8_t ignore_broadcast_ssid;

	if (check_fwstate(mlmepriv, WIFI_AP_STATE) != true)
		return -EPERM;

	if (param->u.bcn_ie.reserved[0] != 0xea)
		return -EINVAL;

	mlmeinfo->hidden_ssid_mode = ignore_broadcast_ssid = param->u.bcn_ie.reserved[1];

	ie_len = len - 12 - 2;	/* 12 = param header, 2:no packed */
	ssid_ie = rtw_get_ie(param->u.bcn_ie.buf,  WLAN_EID_SSID, &ssid_len, ie_len);

	if (ssid_ie && ssid_len) {
		WLAN_BSSID_EX *pbss_network = &mlmepriv->cur_network.network;
		WLAN_BSSID_EX *pbss_network_ext = &mlmeinfo->network;

		memcpy(ssid, ssid_ie+2, ssid_len);
		ssid[ssid_len > NDIS_802_11_LENGTH_SSID ? NDIS_802_11_LENGTH_SSID : ssid_len] = 0x0;

		if (0)
		DBG_871X(FUNC_ADPT_FMT" ssid:(%s,%d), from ie:(%s,%d), (%s,%d)\n", FUNC_ADPT_ARG(rtlpriv),
			ssid, ssid_len,
			pbss_network->Ssid.Ssid, pbss_network->Ssid.SsidLength,
			pbss_network_ext->Ssid.Ssid, pbss_network_ext->Ssid.SsidLength);

		memcpy(pbss_network->Ssid.Ssid, (void *)ssid, ssid_len);
		pbss_network->Ssid.SsidLength = ssid_len;
		memcpy(pbss_network_ext->Ssid.Ssid, (void *)ssid, ssid_len);
		pbss_network_ext->Ssid.SsidLength = ssid_len;

		if (0)
		DBG_871X(FUNC_ADPT_FMT" after ssid:(%s,%d), (%s,%d)\n", FUNC_ADPT_ARG(rtlpriv),
			pbss_network->Ssid.Ssid, pbss_network->Ssid.SsidLength,
			pbss_network_ext->Ssid.Ssid, pbss_network_ext->Ssid.SsidLength);
	}

	DBG_871X(FUNC_ADPT_FMT" ignore_broadcast_ssid:%d, %s,%d\n", FUNC_ADPT_ARG(rtlpriv),
		ignore_broadcast_ssid, ssid, ssid_len);

	return ret;
}

static int rtw_ioctl_acl_remove_sta(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return -EINVAL;

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) {
		return -EINVAL;
	}

	ret = rtw_acl_remove_sta(rtlpriv, param->sta_addr);

	return ret;

}

static int rtw_ioctl_acl_add_sta(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return -EINVAL;

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) {
		return -EINVAL;
	}

	ret = rtw_acl_add_sta(rtlpriv, param->sta_addr);

	return ret;

}

static int rtw_ioctl_set_macaddr_acl(struct net_device *ndev, struct ieee_param *param, int len)
{
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return -EINVAL;

	rtw_set_macaddr_acl(rtlpriv, param->u.mlme.command);

	return ret;
}

static int rtw_hostapd_ioctl(struct net_device *ndev, struct iw_point *p)
{
	struct ieee_param *param;
	int ret = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	/* DBG_871X("%s\n", __FUNCTION__); */

	/*
	* this function is expect to call in master mode, which allows no power saving
	* so, we just check hw_init_completed
	*/

	if (rtlpriv->hw_init_completed == false) {
		ret = -EPERM;
		goto out;
	}


	/* if (p->length < sizeof(struct ieee_param) || !p->pointer){ */
	if (!p->pointer) {
		ret = -EINVAL;
		goto out;
	}

	param = (struct ieee_param *)rtw_malloc(p->length);
	if (param == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(param, p->pointer, p->length)) {
		rtw_mfree(param);
		ret = -EFAULT;
		goto out;
	}

	/* DBG_871X("%s, cmd=%d\n", __FUNCTION__, param->cmd); */

	switch (param->cmd) {
	case RTL871X_HOSTAPD_FLUSH:
		ret = rtw_hostapd_sta_flush(ndev);
		break;

	case RTL871X_HOSTAPD_ADD_STA:
		ret = rtw_add_sta(ndev, param);
		break;

	case RTL871X_HOSTAPD_REMOVE_STA:
		ret = rtw_del_sta(ndev, param);
		break;

	case RTL871X_HOSTAPD_SET_BEACON:
		ret = rtw_set_beacon(ndev, param, p->length);
		break;

	case RTL871X_SET_ENCRYPTION:
		ret = rtw_set_encryption(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_GET_WPAIE_STA:
		ret = rtw_get_sta_wpaie(ndev, param);
		break;

	case RTL871X_HOSTAPD_SET_WPS_BEACON:
		ret = rtw_set_wps_beacon(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_SET_WPS_PROBE_RESP:
		ret = rtw_set_wps_probe_resp(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_SET_WPS_ASSOC_RESP:
		ret = rtw_set_wps_assoc_resp(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_SET_HIDDEN_SSID:
		ret = rtw_set_hidden_ssid(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_GET_INFO_STA:
		ret = rtw_ioctl_get_sta_data(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_SET_MACADDR_ACL:
		ret = rtw_ioctl_set_macaddr_acl(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_ACL_ADD_STA:
		ret = rtw_ioctl_acl_add_sta(ndev, param, p->length);
		break;

	case RTL871X_HOSTAPD_ACL_REMOVE_STA:
		ret = rtw_ioctl_acl_remove_sta(ndev, param, p->length);
		break;

	default:
		DBG_871X("Unknown hostapd request: %d\n", param->cmd);
		ret = -EOPNOTSUPP;
		break;

	}

	if (ret == 0 && copy_to_user(p->pointer, param, p->length))
		ret = -EFAULT;

	rtw_mfree(param);

out:

	return ret;

}
#endif

static int rtw_wx_set_priv(struct net_device *ndev,
				struct iw_request_info *info,
				union iwreq_data *awrq,
				char *extra)
{

#ifdef CONFIG_DEBUG_RTW_WX_SET_PRIV
	char *ext_dbg;
#endif

	int ret = 0;
	int len = 0;
	char *ext;

	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct iw_point *dwrq = (struct iw_point *) awrq;

	/* RT_TRACE(_module_rtl871x_ioctl_os_c, _drv_notice_, ("+rtw_wx_set_priv\n")); */
	if (dwrq->length == 0)
		return -EFAULT;

	len = dwrq->length;
	ext = rtw_vmalloc(len);
	if (!ext)
		return -ENOMEM;

	if (copy_from_user(ext, dwrq->pointer, len)) {
		rtw_vmfree(ext);
		return -EFAULT;
	}


	/*
	 * RT_TRACE(_module_rtl871x_ioctl_os_c, _drv_notice_,
	 * 	 ("rtw_wx_set_priv: %s req=%s\n",
	 * 	  ndev->name, ext));
	 */

#ifdef CONFIG_DEBUG_RTW_WX_SET_PRIV
	ext_dbg = rtw_vmalloc(len);
	if (ext_dbg) {
		rtw_vmfree(ext);
		return -ENOMEM;
	}

	memcpy(ext_dbg, ext, len);
	#endif

	/* added for wps2.0 @20110524 */
	if (dwrq->flags == 0x8766 && len > 8) {
		u32 cp_sz;
		struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
		uint8_t *probereq_wpsie = ext;
		int probereq_wpsie_len = len;
		uint8_t wps_oui[4] = { 0x0, 0x50, 0xf2, 0x04};

		if ((_VENDOR_SPECIFIC_IE_ == probereq_wpsie[0]) &&
		   (_rtw_memcmp(&probereq_wpsie[2], wps_oui, 4) == true)) {
			cp_sz = probereq_wpsie_len > MAX_WPS_IE_LEN ? MAX_WPS_IE_LEN : probereq_wpsie_len;

			/*
			 * memcpy(pmlmepriv->probereq_wpsie, probereq_wpsie, cp_sz);
			 * pmlmepriv->probereq_wpsie_len = cp_sz;
			 */
			if (pmlmepriv->wps_probe_req_ie) {
				pmlmepriv->wps_probe_req_ie_len = 0;
				rtw_mfree(pmlmepriv->wps_probe_req_ie);
				pmlmepriv->wps_probe_req_ie = NULL;
			}

			pmlmepriv->wps_probe_req_ie = rtw_malloc(cp_sz);
			if (pmlmepriv->wps_probe_req_ie == NULL) {
				printk("%s()-%d: rtw_malloc() ERROR!\n", __FUNCTION__, __LINE__);
				ret =  -EINVAL;
				goto FREE_EXT;

			}

			memcpy(pmlmepriv->wps_probe_req_ie, probereq_wpsie, cp_sz);
			pmlmepriv->wps_probe_req_ie_len = cp_sz;

		}
		goto FREE_EXT;

	}

	if (len >= WEXT_CSCAN_HEADER_SIZE &&
	    _rtw_memcmp(ext, WEXT_CSCAN_HEADER, WEXT_CSCAN_HEADER_SIZE) == true) {
		ret = rtw_wx_set_scan(ndev, info, awrq, ext);
		goto FREE_EXT;
	}



FREE_EXT:
	rtw_vmfree(ext);
	#ifdef CONFIG_DEBUG_RTW_WX_SET_PRIV
	/* ULLI check usage of len , strange above function with length c++ not in the kernel */
	rtw_vmfree(ext_dbg, len);
	#endif

	/*
	 * DBG_871X("rtw_wx_set_priv: (SIOCSIWPRIV) %s ret=%d\n",
	 * 		ndev->name, ret);
	 */

	return ret;

}

static int rtw_pm_set(struct net_device *ndev, struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	unsigned	mode = 0;
	struct rtl_priv *rtlpriv = rtl_priv(ndev);

	DBG_871X("[%s] extra = %s\n", __FUNCTION__, extra);

	if (_rtw_memcmp(extra, "lps=", 4)) {
		sscanf(extra+4, "%u", &mode);
		ret = rtw_pm_set_lps(rtlpriv, mode);
	} else if (_rtw_memcmp(extra, "ips=", 4)) {
		sscanf(extra+4, "%u", &mode);
		ret = rtw_pm_set_ips(rtlpriv, mode);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int rtw_test(
	struct net_device *ndev,
	struct iw_request_info *info,
	union iwreq_data *wrqu, char *extra)
{
	u32 len;
	uint8_t *pbuf, *pch;
	char *ptmp;
	uint8_t *delim = ",";
	struct rtl_priv *rtlpriv = rtl_priv(ndev);


	DBG_871X("+%s\n", __func__);
	len = wrqu->data.length;

	pbuf = (uint8_t *)rtw_zmalloc(len);
	if (pbuf == NULL) {
		DBG_871X("%s: no memory!\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(pbuf, wrqu->data.pointer, len)) {
		rtw_mfree(pbuf);
		DBG_871X("%s: copy from user fail!\n", __func__);
		return -EFAULT;
	}
	DBG_871X("%s: string=\"%s\"\n", __func__, pbuf);

	ptmp = (char *) pbuf;
	pch = strsep(&ptmp, delim);
	if ((pch == NULL) || (strlen(pch) == 0)) {
		rtw_mfree(pbuf);
		DBG_871X("%s: parameter error(level 1)!\n", __func__);
		return -EFAULT;
	}


	rtw_mfree(pbuf);
	return 0;
}

static iw_handler rtw_handlers[] = {
	NULL,					/* SIOCSIWCOMMIT */
	rtw_wx_get_name,		/* SIOCGIWNAME */
	dummy,					/* SIOCSIWNWID */
	dummy,					/* SIOCGIWNWID */
	rtw_wx_set_freq,		/* SIOCSIWFREQ */
	rtw_wx_get_freq,		/* SIOCGIWFREQ */
	rtw_wx_set_mode,		/* SIOCSIWMODE */
	rtw_wx_get_mode,		/* SIOCGIWMODE */
	dummy,					/* SIOCSIWSENS */
	rtw_wx_get_sens,		/* SIOCGIWSENS */
	NULL,					/* SIOCSIWRANGE */
	rtw_wx_get_range,		/* SIOCGIWRANGE */
	rtw_wx_set_priv,		/* SIOCSIWPRIV */
	NULL,					/* SIOCGIWPRIV */
	NULL,					/* SIOCSIWSTATS */
	NULL,					/* SIOCGIWSTATS */
	dummy,					/* SIOCSIWSPY */
	dummy,					/* SIOCGIWSPY */
	NULL,					/* SIOCGIWTHRSPY */
	NULL,					/* SIOCWIWTHRSPY */
	rtw_wx_set_wap,		/* SIOCSIWAP */
	rtw_wx_get_wap,		/* SIOCGIWAP */
	rtw_wx_set_mlme,		/* request MLME operation; uses struct iw_mlme */
	dummy,					/* SIOCGIWAPLIST -- depricated */
	rtw_wx_set_scan,		/* SIOCSIWSCAN */
	rtw_wx_get_scan,		/* SIOCGIWSCAN */
	rtw_wx_set_essid,		/* SIOCSIWESSID */
	rtw_wx_get_essid,		/* SIOCGIWESSID */
	dummy,					/* SIOCSIWNICKN */
	rtw_wx_get_nick,		/* SIOCGIWNICKN */
	NULL,					/* -- hole -- */
	NULL,					/* -- hole -- */
	NULL,				/* SIOCSIWRATE */
	rtw_wx_get_rate,		/* SIOCGIWRATE */
	rtw_wx_set_rts,			/* SIOCSIWRTS */
	rtw_wx_get_rts,			/* SIOCGIWRTS */
	rtw_wx_set_frag,		/* SIOCSIWFRAG */
	rtw_wx_get_frag,		/* SIOCGIWFRAG */
	dummy,					/* SIOCSIWTXPOW */
	dummy,					/* SIOCGIWTXPOW */
	dummy,					/* SIOCSIWRETRY */
	rtw_wx_get_retry,		/* SIOCGIWRETRY */
	rtw_wx_set_enc,			/* SIOCSIWENCODE */
	rtw_wx_get_enc,			/* SIOCGIWENCODE */
	dummy,					/* SIOCSIWPOWER */
	rtw_wx_get_power,		/* SIOCGIWPOWER */
	NULL,					/*---hole---*/
	NULL,					/*---hole---*/
	rtw_wx_set_gen_ie,		/* SIOCSIWGENIE */
	NULL,					/* SIOCGWGENIE */
	rtw_wx_set_auth,		/* SIOCSIWAUTH */
	NULL,					/* SIOCGIWAUTH */
	rtw_wx_set_enc_ext,		/* SIOCSIWENCODEEXT */
	NULL,					/* SIOCGIWENCODEEXT */
	rtw_wx_set_pmkid,		/* SIOCSIWPMKSA */
	NULL,					/*---hole---*/
};

static const struct iw_priv_args rtw_private_args[] = {
	{
		SIOCIWFIRSTPRIV + 0x0,
		IW_PRIV_TYPE_CHAR | 0x7FF, 0, "write"
	},
	{
		SIOCIWFIRSTPRIV + 0x1,
		IW_PRIV_TYPE_CHAR | 0x7FF,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | IFNAMSIZ, "read"
	},
	{
		SIOCIWFIRSTPRIV + 0x2, 0, 0, "driver_ext"
	},
	{
		SIOCIWFIRSTPRIV + 0x3, 0, 0, "mp_ioctl"
	},
	{
		SIOCIWFIRSTPRIV + 0x4,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "apinfo"
	},
	{
		SIOCIWFIRSTPRIV + 0x5,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "setpid"
	},
	{
		SIOCIWFIRSTPRIV + 0x6,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wps_start"
	},
/* for PLATFORM_MT53XX */
	{
		SIOCIWFIRSTPRIV + 0x7,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "get_sensitivity"
	},
	{
		SIOCIWFIRSTPRIV + 0x8,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wps_prob_req_ie"
	},
	{
		SIOCIWFIRSTPRIV + 0x9,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wps_assoc_req_ie"
	},

/* for RTK_DMP_PLATFORM */
	{
		SIOCIWFIRSTPRIV + 0xA,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "channel_plan"
	},

	{
		SIOCIWFIRSTPRIV + 0xB,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "dbg"
	},
	{
		SIOCIWFIRSTPRIV + 0xC,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0, "rfw"
	},
	{
		SIOCIWFIRSTPRIV + 0xD,
		IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | IFNAMSIZ, "rfr"
	},
	{
		SIOCIWFIRSTPRIV + 0x10,
		IW_PRIV_TYPE_CHAR | P2P_PRIVATE_IOCTL_SET_LEN, 0, "p2p_set"
	},
	{
		SIOCIWFIRSTPRIV + 0x11,
		IW_PRIV_TYPE_CHAR | 1024, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK , "p2p_get"
	},
	{
		SIOCIWFIRSTPRIV + 0x12, 0, 0, "NULL"
	},
	{
		SIOCIWFIRSTPRIV + 0x13,
		IW_PRIV_TYPE_CHAR | 64, IW_PRIV_TYPE_CHAR | 64 , "p2p_get2"
	},
	{
		SIOCIWFIRSTPRIV + 0x14,
		IW_PRIV_TYPE_CHAR  | 64, 0, "tdls"
	},
	{
		SIOCIWFIRSTPRIV + 0x15,
		IW_PRIV_TYPE_CHAR | P2P_PRIVATE_IOCTL_SET_LEN, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | P2P_PRIVATE_IOCTL_SET_LEN , "tdls_get"
	},
	{
		SIOCIWFIRSTPRIV + 0x16,
		IW_PRIV_TYPE_CHAR | 64, 0, "pm_set"
	},

	{SIOCIWFIRSTPRIV + 0x18, IW_PRIV_TYPE_CHAR | IFNAMSIZ , 0 , "rereg_nd_name"},

	{SIOCIWFIRSTPRIV + 0x1A, IW_PRIV_TYPE_CHAR | 1024, 0, "efuse_set"},
	{SIOCIWFIRSTPRIV + 0x1B, IW_PRIV_TYPE_CHAR | 128, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "efuse_get"},
	{
		SIOCIWFIRSTPRIV + 0x1D,
		IW_PRIV_TYPE_CHAR | 40, IW_PRIV_TYPE_CHAR | 0x7FF, "test"
	},


};


#if (WIRELESS_EXT >= 17)
static struct iw_statistics *rtw_get_wireless_stats(struct net_device *ndev)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct iw_statistics *piwstats = &rtlpriv->iwstats;
	int tmp_level = 0;
	int tmp_qual = 0;
	int tmp_noise = 0;

	if (check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED) != true) {
		piwstats->qual.qual = 0;
		piwstats->qual.level = 0;
		piwstats->qual.noise = 0;
		/* DBG_871X("No link  level:%d, qual:%d, noise:%d\n", tmp_level, tmp_qual, tmp_noise); */
	} else {
		tmp_level = rtlpriv->recvpriv.signal_strength;

		tmp_qual = rtlpriv->recvpriv.signal_qual;
		tmp_noise = rtlpriv->recvpriv.noise;
		/* DBG_871X("level:%d, qual:%d, noise:%d, rssi (%d)\n", tmp_level, tmp_qual, tmp_noise,rtlpriv->recvpriv.rssi); */

		piwstats->qual.level = tmp_level;
		piwstats->qual.qual = tmp_qual;
		piwstats->qual.noise = tmp_noise;
	}
	piwstats->qual.updated = IW_QUAL_ALL_UPDATED ;	/* |IW_QUAL_DBM; */

	return &rtlpriv->iwstats;
}
#endif

#ifdef CONFIG_WIRELESS_EXT
struct iw_handler_def rtw_handlers_def = {
	.standard = rtw_handlers,
	.num_standard = sizeof(rtw_handlers) / sizeof(iw_handler),
#if WIRELESS_EXT >= 17
	.get_wireless_stats = rtw_get_wireless_stats,
#endif
};
#endif

/* copy from net/wireless/wext.c start */
/* ---------------------------------------------------------------- */
/*
 * Calculate size of private arguments
 */
static const char iw_priv_type_size[] = {
	0,                              /* IW_PRIV_TYPE_NONE */
	1,                              /* IW_PRIV_TYPE_BYTE */
	1,                              /* IW_PRIV_TYPE_CHAR */
	0,                              /* Not defined */
	sizeof(__u32),                  /* IW_PRIV_TYPE_INT */
	sizeof(struct iw_freq),         /* IW_PRIV_TYPE_FLOAT */
	sizeof(struct sockaddr),        /* IW_PRIV_TYPE_ADDR */
	0,                              /* Not defined */
};

static int get_priv_size(u16 args)
{
	int num = args & IW_PRIV_SIZE_MASK;
	int type = (args & IW_PRIV_TYPE_MASK) >> 12;

	return num * iw_priv_type_size[type];
}
/* copy from net/wireless/wext.c end */

int rtw_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct iwreq *wrq = (struct iwreq *)rq;
	int ret = 0;

	switch (cmd) {
	case RTL_IOCTL_WPA_SUPPLICANT:
		ret = wpa_supplicant_ioctl(ndev, &wrq->u.data);
		break;
#ifdef CONFIG_AP_MODE
	case RTL_IOCTL_HOSTAPD:
		ret = rtw_hostapd_ioctl(ndev, &wrq->u.data);
		break;
#endif
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

