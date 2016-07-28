/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
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
#define _RTW_MLME_C_

#include <linux/etherdevice.h>
#include <drv_types.h>
#include <rtw_ap.h>
#include <rtl8812a_hal.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

static struct list_head *get_next(struct list_head	*list)
{
	return list->next;
}

extern void indicate_wx_scan_complete_event(struct rtl_priv *rtlpriv);
extern uint8_t rtw_do_join(struct rtl_priv * rtlpriv);

#ifdef CONFIG_DISABLE_MCS13TO15
extern unsigned char	MCS_rate_2R_MCS13TO15_OFF[16];
extern unsigned char	MCS_rate_2R[16];
#else
extern unsigned char	MCS_rate_2R[16];
#endif
extern unsigned char	MCS_rate_1R[16];

int _rtw_init_mlme_priv (struct rtl_priv *rtlpriv)
{
	int	i;
	uint8_t	*pbuf;
	struct wlan_network *pnetwork;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	int	res = _SUCCESS;


	/*
	 *  We don't need to memset rtlpriv->XXX to zero, because rtlpriv is allocated by rtw_zvmalloc().
	 * memset((uint8_t *)pmlmepriv, 0, sizeof(struct mlme_priv));
	 */

	pmlmepriv->nic_hdl = (uint8_t *)rtlpriv;

	pmlmepriv->pscanned = NULL;
	pmlmepriv->fw_state = 0;
	pmlmepriv->cur_network.network.InfrastructureMode = Ndis802_11AutoUnknown;
	pmlmepriv->scan_mode=SCAN_ACTIVE;// 1: active, 0: pasive. Maybe someday we should rename this varable to "active_mode" (Jeff)

	spin_lock_init(&(pmlmepriv->lock));
	_rtw_init_queue(&(pmlmepriv->free_bss_pool));
	_rtw_init_queue(&(pmlmepriv->scanned_queue));

	set_scanned_network_val(pmlmepriv, 0);

	memset(&pmlmepriv->assoc_ssid,0,sizeof(NDIS_802_11_SSID));

	pbuf = rtw_zvmalloc(MAX_BSS_CNT * (sizeof(struct wlan_network)));

	if (pbuf == NULL) {
		res=_FAIL;
		goto exit;
	}
	pmlmepriv->free_bss_buf = pbuf;

	pnetwork = (struct wlan_network *)pbuf;

	for (i = 0; i < MAX_BSS_CNT; i++) {
		INIT_LIST_HEAD(&(pnetwork->list));

		list_add_tail(&(pnetwork->list), &(pmlmepriv->free_bss_pool.list));

		pnetwork++;
	}

	/* allocate DMA-able/Non-Page memory for cmd_buf and rsp_buf */

	rtw_init_mlme_timer(rtlpriv);

exit:

	return res;
}

/* ULLI check usage of param *plen */
static void rtw_free_mlme_ie_data(uint8_t **ppie, uint32_t *plen)
{
	if (*ppie) {
		_rtw_mfree(*ppie);
		*plen = 0;
		*ppie=NULL;
	}
}

void rtw_free_mlme_priv_ie_data(struct mlme_priv *pmlmepriv)
{
#if defined (CONFIG_AP_MODE)
	rtw_buf_free(&pmlmepriv->assoc_req, &pmlmepriv->assoc_req_len);
	rtw_buf_free(&pmlmepriv->assoc_rsp, &pmlmepriv->assoc_rsp_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_beacon_ie, &pmlmepriv->wps_beacon_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_probe_req_ie, &pmlmepriv->wps_probe_req_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_probe_resp_ie, &pmlmepriv->wps_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_assoc_resp_ie, &pmlmepriv->wps_assoc_resp_ie_len);

	rtw_free_mlme_ie_data(&pmlmepriv->p2p_beacon_ie, &pmlmepriv->p2p_beacon_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_probe_req_ie, &pmlmepriv->p2p_probe_req_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_probe_resp_ie, &pmlmepriv->p2p_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_go_probe_resp_ie, &pmlmepriv->p2p_go_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_assoc_req_ie, &pmlmepriv->p2p_assoc_req_ie_len);
#endif
}

void _rtw_free_mlme_priv (struct mlme_priv *pmlmepriv)
{
	rtw_free_mlme_priv_ie_data(pmlmepriv);

	if (pmlmepriv){
		if (pmlmepriv->free_bss_buf) {
			rtw_vmfree(pmlmepriv->free_bss_buf);
		}
	}
}

int _rtw_enqueue_network(struct __queue *queue, struct wlan_network *pnetwork)
{
	if (pnetwork == NULL)
		goto exit;

	spin_lock_bh(&queue->lock);

	list_add_tail(&pnetwork->list, &queue->list);

	spin_unlock_bh(&queue->lock);

exit:

	return _SUCCESS;
}

/*
struct	wlan_network *_rtw_dequeue_network(struct __queue *queue)
{
	_irqL irqL;

	struct wlan_network *pnetwork;



	spin_lock_bh(&queue->lock, &irqL);

	if (_rtw_queue_empty(queue) == true)

		pnetwork = NULL;

	else
	{
		pnetwork = container_of(get_next(&queue->queue), struct wlan_network, list);

		list_del_init(&(pnetwork->list));
	}

	spin_unlock_bh(&queue->lock, &irqL);



	return pnetwork;
}
*/

struct	wlan_network *_rtw_alloc_network(struct	mlme_priv *pmlmepriv )	/* (struct __queue *free_queue) */
{
	struct	wlan_network *pnetwork;
	struct __queue *free_queue = &pmlmepriv->free_bss_pool;
	struct list_head* plist = NULL;

	spin_lock_bh(&free_queue->lock);

	if (list_empty(&free_queue->list)) {
		pnetwork=NULL;
		goto exit;
	}
	plist = get_next(&(free_queue->list));

	pnetwork = container_of(plist , struct wlan_network, list);

	list_del_init(&pnetwork->list);

	pnetwork->network_type = 0;
	pnetwork->fixed = false;
	pnetwork->last_scanned = jiffies;
	pnetwork->aid=0;
	pnetwork->join_res=0;

	pmlmepriv->num_of_scanned ++;

exit:
	spin_unlock_bh(&free_queue->lock);

	return pnetwork;
}

void _rtw_free_network(struct	mlme_priv *pmlmepriv ,struct wlan_network *pnetwork, uint8_t isfreeall)
{
	uint32_t delta_time;
	uint32_t lifetime = SCANQUEUE_LIFETIME;
	struct __queue *free_queue = &(pmlmepriv->free_bss_pool);

	if (pnetwork == NULL)
		goto exit;

	if (pnetwork->fixed == true)
		goto exit;

	if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)==true ) ||
		(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE)==true ) )
		lifetime = 1;

	if (!isfreeall) {
		delta_time = (uint32_t) rtw_get_passing_time_ms(pnetwork->last_scanned);
		if (delta_time < lifetime) 	/* unit:msec */
			goto exit;
	}

	spin_lock_bh(&free_queue->lock);

	list_del_init(&(pnetwork->list));
	list_add_tail(&(pnetwork->list),&(free_queue->list));

	pmlmepriv->num_of_scanned --;


	/*
	 * DBG_871X("_rtw_free_network:SSID=%s\n", pnetwork->network.Ssid.Ssid);
	 */

	spin_unlock_bh(&free_queue->lock);
exit:
	;
}

void _rtw_free_network_nolock(struct	mlme_priv *pmlmepriv, struct wlan_network *pnetwork)
{

	struct __queue *free_queue = &(pmlmepriv->free_bss_pool);

	if (pnetwork == NULL)
		goto exit;

	if (pnetwork->fixed == true)
		goto exit;

	/* spin_lock_irqsave(&free_queue->lock, &irqL); */

	list_del_init(&(pnetwork->list));
	list_add_tail(&(pnetwork->list), get_list_head(free_queue));

	pmlmepriv->num_of_scanned --;

	/*spin_unlock_irqrestore(&free_queue->lock, &irqL); */

exit:
	;
}


/*
	return the wlan_network with the matching addr

	Shall be calle under atomic context... to avoid possible racing condition...
*/
struct wlan_network *_rtw_find_network(struct __queue *scanned_queue, uint8_t *addr)
{

	/* _irqL irqL; */
	struct list_head	*phead, *plist;
	struct	wlan_network *pnetwork = NULL;
	uint8_t zero_addr[ETH_ALEN] = {0,0,0,0,0,0};

	if (_rtw_memcmp(zero_addr, addr, ETH_ALEN)){
		pnetwork=NULL;
		goto exit;
	}

	/* spin_lock_bh(&scanned_queue->lock, &irqL); */

	phead = get_list_head(scanned_queue);
	plist = get_next(phead);

	while (plist != phead) {
                pnetwork = container_of(plist, struct wlan_network ,list);

		if (_rtw_memcmp(addr, pnetwork->network.MacAddress, ETH_ALEN) == true)
                        break;

		plist = get_next(plist);
        }

	if (plist == phead)
		pnetwork = NULL;

	/* spin_unlock_bh(&scanned_queue->lock, &irqL); */

exit:

	return pnetwork;
}


void _rtw_free_network_queue(struct rtl_priv *rtlpriv, uint8_t isfreeall)
{
	struct list_head *phead, *plist;
	struct wlan_network *pnetwork;
	struct mlme_priv* pmlmepriv = &rtlpriv->mlmepriv;
	struct __queue *scanned_queue = &pmlmepriv->scanned_queue;

	spin_lock_bh(&scanned_queue->lock);

	phead = get_list_head(scanned_queue);
	plist = get_next(phead);

	while (rtw_end_of_queue_search(phead, plist) == false) {
		pnetwork = container_of(plist, struct wlan_network, list);
		plist = get_next(plist);
		_rtw_free_network(pmlmepriv,pnetwork, isfreeall);
	}

	spin_unlock_bh(&scanned_queue->lock);
}




int rtw_if_up(struct rtl_priv *rtlpriv)
{

	int res;

	if (rtlpriv->bDriverStopped || rtlpriv->bSurpriseRemoved
	   || (check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED)== false)){
		res=false;
	} else
		res=  true;

	return res;
}


void rtw_generate_random_ibss(uint8_t * pibss)
{
	uint32_t	curtime = jiffies;

	pibss[0] = 0x02;  //in ad-hoc mode BIT(1) must set to 1
	pibss[1] = 0x11;
	pibss[2] = 0x87;
	pibss[3] = (uint8_t)(curtime & 0xff) ;//p[0];
	pibss[4] = (uint8_t)((curtime>>8) & 0xff) ;//p[1];
	pibss[5] = (uint8_t)((curtime>>16) & 0xff) ;//p[2];

	return;
}

uint8_t *rtw_get_capability_from_ie(uint8_t *ie)
{
	return (ie + 8 + 2);
}


u16 rtw_get_capability(WLAN_BSSID_EX *bss)
{
	u16	val;

	memcpy((uint8_t *)&val, rtw_get_capability_from_ie(bss->IEs), 2);

	return le16_to_cpu(val);
}

uint8_t *rtw_get_timestampe_from_ie(uint8_t *ie)
{
	return (ie + 0);
}

uint8_t *rtw_get_beacon_interval_from_ie(uint8_t *ie)
{
	return (ie + 8);
}


int	rtw_init_mlme_priv (struct rtl_priv *rtlpriv)//(struct	mlme_priv *pmlmepriv)
{
	int	res;

	res = _rtw_init_mlme_priv(rtlpriv);// (pmlmepriv);

	return res;
}

void rtw_free_mlme_priv (struct mlme_priv *pmlmepriv)
{
	_rtw_free_mlme_priv (pmlmepriv);
}

static int rtw_enqueue_network(struct __queue *queue, struct wlan_network *pnetwork)
{
	int	res;

	res = _rtw_enqueue_network(queue, pnetwork);

	return res;
}

/*
static struct	wlan_network *rtw_dequeue_network(struct __queue *queue)
{
	struct wlan_network *pnetwork;

	pnetwork = _rtw_dequeue_network(queue);

	return pnetwork;
}
*/

static struct wlan_network *rtw_alloc_network(struct mlme_priv *pmlmepriv )	/* (struct __queue	*free_queue) */
{
	struct	wlan_network	*pnetwork;

	pnetwork = _rtw_alloc_network(pmlmepriv);

	return pnetwork;
}

static void rtw_free_network(struct mlme_priv *pmlmepriv,
	struct wlan_network *pnetwork, uint8_t is_freeall)/* (struct	wlan_network *pnetwork, struct __queue	*free_queue) */
{
	_rtw_free_network(pmlmepriv, pnetwork, is_freeall);
}

static void rtw_free_network_nolock(struct mlme_priv *pmlmepriv, struct wlan_network *pnetwork )
{
	/*
	 * RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("rtw_free_network==> ssid = %s \n\n" , pnetwork->network.Ssid.Ssid));
	 */
	_rtw_free_network_nolock(pmlmepriv, pnetwork);
}


void rtw_free_network_queue(struct rtl_priv *dev, uint8_t isfreeall)
{
	_rtw_free_network_queue(dev, isfreeall);
}

/*
	return the wlan_network with the matching addr

	Shall be calle under atomic context... to avoid possible racing condition...
*/
struct	wlan_network *rtw_find_network(struct __queue *scanned_queue, uint8_t *addr)
{
	struct	wlan_network *pnetwork = _rtw_find_network(scanned_queue, addr);

	return pnetwork;
}

int rtw_is_same_ibss(struct rtl_priv *rtlpriv, struct wlan_network *pnetwork)
{
	int ret=true;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;

	if ((psecuritypriv->dot11PrivacyAlgrthm != NO_ENCRYPTION )
	   && ( pnetwork->network.Privacy == 0 )) {
		ret=false;
	} else if ((psecuritypriv->dot11PrivacyAlgrthm == NO_ENCRYPTION )
		  && ( pnetwork->network.Privacy == 1 ) ) {
		ret=false;
	} else {
		ret=true;
	}

	return ret;

}

inline int is_same_ess(WLAN_BSSID_EX *a, WLAN_BSSID_EX *b);
inline int is_same_ess(WLAN_BSSID_EX *a, WLAN_BSSID_EX *b)
{
	//RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("(%s,%d)(%s,%d)\n",
	//		a->Ssid.Ssid,a->Ssid.SsidLength,b->Ssid.Ssid,b->Ssid.SsidLength));
	return (a->Ssid.SsidLength == b->Ssid.SsidLength)
		&&  _rtw_memcmp(a->Ssid.Ssid, b->Ssid.Ssid, a->Ssid.SsidLength)==true;
}

int is_same_network(WLAN_BSSID_EX *src, WLAN_BSSID_EX *dst)
{
	 u16 s_cap, d_cap;

	if (rtw_bug_check(dst, src, &s_cap, &d_cap)==false)
			return false;

	memcpy((uint8_t *)&s_cap, rtw_get_capability_from_ie(src->IEs), 2);
	memcpy((uint8_t *)&d_cap, rtw_get_capability_from_ie(dst->IEs), 2);


	s_cap = le16_to_cpu(s_cap);
	d_cap = le16_to_cpu(d_cap);

	return ((src->Ssid.SsidLength == dst->Ssid.SsidLength) &&
		//	(src->Configuration.DSConfig == dst->Configuration.DSConfig) &&
			( (_rtw_memcmp(src->MacAddress, dst->MacAddress, ETH_ALEN)) == true) &&
			( (_rtw_memcmp(src->Ssid.Ssid, dst->Ssid.Ssid, src->Ssid.SsidLength)) == true) &&
			((s_cap & WLAN_CAPABILITY_IBSS) ==
			(d_cap & WLAN_CAPABILITY_IBSS)) &&
			((s_cap & WLAN_CAPABILITY_BSS) ==
			(d_cap & WLAN_CAPABILITY_BSS)));

}

struct wlan_network *rtw_get_oldest_wlan_network(struct __queue *scanned_queue)
{
	struct list_head	*plist, *phead;

	struct	wlan_network	*pwlan = NULL;
	struct	wlan_network	*oldest = NULL;

	phead = get_list_head(scanned_queue);
	plist = get_next(phead);

	while (1) {

		if (rtw_end_of_queue_search(phead,plist)== true)
			break;

		pwlan= container_of(plist, struct wlan_network, list);

		if (pwlan->fixed!=true) {
			if (oldest == NULL ||time_after(oldest->last_scanned, pwlan->last_scanned))
				oldest = pwlan;
		}

		plist = get_next(plist);
	}

	return oldest;

}

void update_network(WLAN_BSSID_EX *dst, WLAN_BSSID_EX *src,
	struct rtl_priv * rtlpriv, bool update_ie)
{
	uint8_t ss_ori = dst->PhyInfo.SignalStrength;
	uint8_t sq_ori = dst->PhyInfo.SignalQuality;
	long rssi_ori = dst->Rssi;

	uint8_t ss_smp = src->PhyInfo.SignalStrength;
	uint8_t sq_smp = src->PhyInfo.SignalQuality;
	long rssi_smp = src->Rssi;

	uint8_t ss_final;
	uint8_t sq_final;
	long rssi_final;

	/* The rule below is 1/5 for sample value, 4/5 for history value */
	if (check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED) && is_same_network(&(rtlpriv->mlmepriv.cur_network.network), src)) {
		/* Take the recvpriv's value for the connected AP*/
		ss_final = rtlpriv->recvpriv.signal_strength;
		sq_final = rtlpriv->recvpriv.signal_qual;
		/* the rssi value here is undecorated, and will be used for antenna diversity */
		if (sq_smp != 101) /* from the right channel */
			rssi_final = (src->Rssi+dst->Rssi*4)/5;
		else
			rssi_final = rssi_ori;
	} else {
		if (sq_smp != 101) { /* from the right channel */
			ss_final = ((uint32_t)(src->PhyInfo.SignalStrength)+(uint32_t)(dst->PhyInfo.SignalStrength)*4)/5;
			sq_final = ((uint32_t)(src->PhyInfo.SignalQuality)+(uint32_t)(dst->PhyInfo.SignalQuality)*4)/5;
			rssi_final = (src->Rssi+dst->Rssi*4)/5;
		} else {
			/* bss info not receving from the right channel, use the original RX signal infos */
			ss_final = dst->PhyInfo.SignalStrength;
			sq_final = dst->PhyInfo.SignalQuality;
			rssi_final = dst->Rssi;
		}

	}

	if (update_ie)
		memcpy((uint8_t *)dst, (uint8_t *)src, get_WLAN_BSSID_EX_sz(src));

	dst->PhyInfo.SignalStrength = ss_final;
	dst->PhyInfo.SignalQuality = sq_final;
	dst->Rssi = rssi_final;
}

static void update_current_network(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork)
{
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);

	rtw_bug_check(&(pmlmepriv->cur_network.network),
		&(pmlmepriv->cur_network.network),
		&(pmlmepriv->cur_network.network),
		&(pmlmepriv->cur_network.network));

	if ((check_fwstate(pmlmepriv, _FW_LINKED)== true)
	   && (is_same_network(&(pmlmepriv->cur_network.network), pnetwork))) {
		/*
		 * RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,"Same Network\n");
		 */

		/* if (pmlmepriv->cur_network.network.IELength<= pnetwork->IELength) */
		{
			update_network(&(pmlmepriv->cur_network.network), pnetwork,rtlpriv, true);
			rtw_update_protection(rtlpriv, (pmlmepriv->cur_network.network.IEs) + sizeof (NDIS_802_11_FIXED_IEs),
									pmlmepriv->cur_network.network.IELength);
		}
	}



}


/*

Caller must hold pmlmepriv->lock first.


*/
void rtw_update_scanned_network(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *target)
{
	struct list_head	*plist, *phead;
	u32	bssid_ex_sz;
	struct mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct __queue	*queue	= &(pmlmepriv->scanned_queue);
	struct wlan_network	*pnetwork = NULL;
	struct wlan_network	*oldest = NULL;

	spin_lock_bh(&queue->lock);
	phead = get_list_head(queue);
	plist = get_next(phead);

	while (1) {
		if (rtw_end_of_queue_search(phead,plist)== true)
			break;

		pnetwork = container_of(plist, struct wlan_network, list);

		rtw_bug_check(pnetwork, pnetwork, pnetwork, pnetwork);

		if (is_same_network(&(pnetwork->network), target))
			break;

		if ((oldest == ((struct wlan_network *)0)) ||
		time_after(oldest->last_scanned, pnetwork->last_scanned))
			oldest = pnetwork;

		plist = get_next(plist);

	}


	/* If we didn't find a match, then get a new network slot to initialize
	 * with this beacon's information */
	if (rtw_end_of_queue_search(phead,plist)== true) {

		if (list_empty(&(pmlmepriv->free_bss_pool.list)) == true) {
			/* If there are no more slots, expire the oldest */
			//list_del_init(&oldest->list);
			pnetwork = oldest;

			memcpy(&(pnetwork->network), target,  get_WLAN_BSSID_EX_sz(target));
			/*
			 * pnetwork->last_scanned = jiffies;
			 * variable initialize
			 */
			pnetwork->fixed = false;
			pnetwork->last_scanned = jiffies;

			pnetwork->network_type = 0;
			pnetwork->aid=0;
			pnetwork->join_res=0;

			/* bss info not receving from the right channel */
			if (pnetwork->network.PhyInfo.SignalQuality == 101)
				pnetwork->network.PhyInfo.SignalQuality = 0;
		} else {
			/* Otherwise just pull from the free list */

			pnetwork = rtw_alloc_network(pmlmepriv); /* will update scan_time */

			if (pnetwork==NULL){
				goto exit;
			}

			bssid_ex_sz = get_WLAN_BSSID_EX_sz(target);
			target->Length = bssid_ex_sz;
			memcpy(&(pnetwork->network), target, bssid_ex_sz );

			pnetwork->last_scanned = jiffies;

			/* bss info not receving from the right channel */
			if (pnetwork->network.PhyInfo.SignalQuality == 101)
				pnetwork->network.PhyInfo.SignalQuality = 0;

			list_add_tail(&(pnetwork->list),&(queue->list));

		}
	}
	else {
		/* we have an entry and we are going to update it. But this entry may
		 * be already expired. In this case we do the same as we found a new
		 * net and call the new_net handler
		 */
		bool update_ie = true;

		pnetwork->last_scanned = jiffies;

		//target.Reserved[0]==1, means that scaned network is a bcn frame.
		if ((pnetwork->network.IELength>target->IELength) && (target->Reserved[0]==1))
			update_ie = false;

		update_network(&(pnetwork->network), target,rtlpriv, update_ie);
	}

exit:
	spin_unlock_bh(&queue->lock);
}

static void rtw_add_network(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork)
{
	struct	mlme_priv	*pmlmepriv = &(((struct rtl_priv *)rtlpriv)->mlmepriv);
	//struct __queue	*queue	= &(pmlmepriv->scanned_queue);



	//spin_lock_bh(&queue->lock, &irqL);

	update_current_network(rtlpriv, pnetwork);

	rtw_update_scanned_network(rtlpriv, pnetwork);

	//spin_unlock_bh(&queue->lock, &irqL);


}

//select the desired network based on the capability of the (i)bss.
// check items: (1) security
//			   (2) network_type
//			   (3) WMM
//			   (4) HT
//                     (5) others
static int rtw_is_desired_network(struct rtl_priv *rtlpriv, struct wlan_network *pnetwork)
{
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	uint32_t	 desired_encmode;
	uint32_t	 privacy;

	//uint8_t wps_ie[512];
	uint wps_ielen;

	int bselected = true;

	desired_encmode = psecuritypriv->ndisencryptstatus;
	privacy = pnetwork->network.Privacy;

	if (check_fwstate(pmlmepriv, WIFI_UNDER_WPS)) {
		if (rtw_get_wps_ie(pnetwork->network.IEs+_FIXED_IE_LENGTH_, pnetwork->network.IELength-_FIXED_IE_LENGTH_, NULL, &wps_ielen)!=NULL) {
			return true;
		} else {
			return false;
		}
	}

 	if ((desired_encmode != Ndis802_11EncryptionDisabled) && (privacy == 0)) {
		DBG_871X("desired_encmode: %d, privacy: %d\n", desired_encmode, privacy);
		bselected = false;
 	}

	if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true) {
		if (pnetwork->network.InfrastructureMode != pmlmepriv->cur_network.network.InfrastructureMode)
			bselected = false;
	}


	return bselected;
}

/* TODO: Perry : For Power Management */
void rtw_atimdone_event_callback(struct rtl_priv *rtlpriv , uint8_t *pbuf)
{

	return;
}


void rtw_survey_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	uint32_t	 len;
	WLAN_BSSID_EX *pnetwork;
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);


	pnetwork = (WLAN_BSSID_EX *)pbuf;

	len = get_WLAN_BSSID_EX_sz(pnetwork);
	if (len > (sizeof(WLAN_BSSID_EX))) {
		return;
	}


	spin_lock_bh(&pmlmepriv->lock);

	// update IBSS_network 's timestamp
	if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) == true) {
		//RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,"rtw_survey_event_callback : WIFI_ADHOC_MASTER_STATE \n\n");
		if (_rtw_memcmp(&(pmlmepriv->cur_network.network.MacAddress), pnetwork->MacAddress, ETH_ALEN)) {
			struct wlan_network* ibss_wlan = NULL;

			memcpy(pmlmepriv->cur_network.network.IEs, pnetwork->IEs, 8);
			spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
			ibss_wlan = rtw_find_network(&pmlmepriv->scanned_queue,  pnetwork->MacAddress);
			if (ibss_wlan) {
				memcpy(ibss_wlan->network.IEs , pnetwork->IEs, 8);
				spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
				goto exit;
			}
			spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
		}
	}

	// lock pmlmepriv->lock when you accessing network_q
	if ((check_fwstate(pmlmepriv, _FW_UNDER_LINKING)) == false) {
   	        if ( pnetwork->Ssid.Ssid[0] == 0 ) {
			pnetwork->Ssid.SsidLength = 0;
		}
		rtw_add_network(rtlpriv, pnetwork);
	}

exit:

	spin_unlock_bh(&pmlmepriv->lock);

	return;
}



void rtw_surveydone_event_callback(struct rtl_priv	*rtlpriv, uint8_t *pbuf)
{
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);

#ifdef CONFIG_MLME_EXT

	mlmeext_surveydone_event_callback(rtlpriv);

#endif



	spin_lock_bh(&pmlmepriv->lock);

	if (pmlmepriv->wps_probe_req_ie) {
		/* ULLI check usage of free_len */
		uint32_t	 free_len = pmlmepriv->wps_probe_req_ie_len;

		pmlmepriv->wps_probe_req_ie_len = 0;
		rtw_mfree(pmlmepriv->wps_probe_req_ie);
		pmlmepriv->wps_probe_req_ie = NULL;
	}

	if (check_fwstate(pmlmepriv,_FW_UNDER_SURVEY)) {
		del_timer_sync(&pmlmepriv->scan_to_timer);

		_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY);
	} else {
		;
	}

#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	rtw_set_signal_stat_timer(&rtlpriv->recvpriv);
#endif

	if (pmlmepriv->to_join == true) {
		if ((check_fwstate(pmlmepriv, WIFI_ADHOC_STATE)==true) )
		{
			if (check_fwstate(pmlmepriv, _FW_LINKED)==false)
			{
				set_fwstate(pmlmepriv, _FW_UNDER_LINKING);

		   		if (rtw_select_and_join_from_scanned_queue(pmlmepriv)==_SUCCESS)
		   		{
		       			_set_timer(&pmlmepriv->assoc_timer, MAX_JOIN_TIMEOUT );
                  		}
		   		else
		  		{
					WLAN_BSSID_EX    *pdev_network = &(rtlpriv->registrypriv.dev_network);
					uint8_t *pibss = rtlpriv->registrypriv.dev_network.MacAddress;

					//pmlmepriv->fw_state ^= _FW_UNDER_SURVEY;//because don't set assoc_timer
					_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY);

					memset(&pdev_network->Ssid, 0, sizeof(NDIS_802_11_SSID));
					memcpy(&pdev_network->Ssid, &pmlmepriv->assoc_ssid, sizeof(NDIS_802_11_SSID));

					rtw_update_registrypriv_dev_network(rtlpriv);
					rtw_generate_random_ibss(pibss);

                       			pmlmepriv->fw_state = WIFI_ADHOC_MASTER_STATE;

					if (rtw_createbss_cmd(rtlpriv)!=_SUCCESS)
					{
						;
					}

			     		pmlmepriv->to_join = false;
		   		}
		 	}
		} else 	{
			int s_ret;
			set_fwstate(pmlmepriv, _FW_UNDER_LINKING);
			pmlmepriv->to_join = false;
			if (_SUCCESS == (s_ret=rtw_select_and_join_from_scanned_queue(pmlmepriv))) {
	     		     _set_timer(&pmlmepriv->assoc_timer, MAX_JOIN_TIMEOUT);
			} else if (s_ret == 2) {//there is no need to wait for join
				_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
				rtw_indicate_connect(rtlpriv);
			} else 	{
				DBG_871X("try_to_join, but select scanning queue fail, to_roaming:%d\n", 0);
				_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
			}
		}
	}

	indicate_wx_scan_complete_event(rtlpriv);
	//DBG_871X("scan complete in %dms\n",rtw_get_passing_time_ms(pmlmepriv->scan_start_time));

	spin_unlock_bh(&pmlmepriv->lock);

	rtw_os_xmit_schedule(rtlpriv);



}

void rtw_dummy_event_callback(struct rtl_priv *rtlpriv , uint8_t *pbuf)
{

}

void rtw_fwdbg_event_callback(struct rtl_priv *rtlpriv , uint8_t *pbuf)
{

}

static void free_scanqueue(struct	mlme_priv *pmlmepriv)
{
	struct __queue *free_queue = &pmlmepriv->free_bss_pool;
	struct __queue *scan_queue = &pmlmepriv->scanned_queue;
	struct list_head	*plist, *phead, *ptemp;

	spin_lock_bh(&scan_queue->lock);
	spin_lock_bh(&free_queue->lock);

	phead = get_list_head(scan_queue);
	plist = get_next(phead);

	while (plist != phead) {
		ptemp = get_next(plist);
		list_del_init(plist);
		list_add_tail(plist, &free_queue->list);
		plist =ptemp;
		pmlmepriv->num_of_scanned --;
        }

	spin_unlock_bh(&free_queue->lock);
	spin_unlock_bh(&scan_queue->lock);

}

/*
*rtw_free_assoc_resources: the caller has to lock pmlmepriv->lock
*/
void rtw_free_assoc_resources(struct rtl_priv *rtlpriv, int lock_scanned_queue)
{
	struct wlan_network* pwlan = NULL;
     	struct	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
   	struct	sta_priv *pstapriv = &rtlpriv->stapriv;
	struct wlan_network *tgt_network = &pmlmepriv->cur_network;



	if (check_fwstate( pmlmepriv, WIFI_STATION_STATE|WIFI_AP_STATE)) {
		struct sta_info* psta;

		psta = rtw_get_stainfo(&rtlpriv->stapriv, tgt_network->network.MacAddress);
		{
			spin_lock_bh(&(pstapriv->sta_hash_lock));
			rtw_free_stainfo(rtlpriv,  psta);
		}

		spin_unlock_bh(&(pstapriv->sta_hash_lock));

	}

	if (check_fwstate( pmlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE|WIFI_AP_STATE)) {
		struct sta_info* psta;

		rtw_free_all_stainfo(rtlpriv);

		psta = rtw_get_bcmc_stainfo(rtlpriv);
		spin_lock_bh(&(pstapriv->sta_hash_lock));
		rtw_free_stainfo(rtlpriv, psta);
		spin_unlock_bh(&(pstapriv->sta_hash_lock));

		rtw_init_bcmc_stainfo(rtlpriv);
	}

	if (lock_scanned_queue)
		spin_lock_bh(&(pmlmepriv->scanned_queue.lock));

	pwlan = rtw_find_network(&pmlmepriv->scanned_queue, tgt_network->network.MacAddress);
	if (pwlan) {
		pwlan->fixed = false;
	} else {
		;
	}


	if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) && (rtlpriv->stapriv.asoc_sta_count== 1))
		/*||check_fwstate(pmlmepriv, WIFI_STATION_STATE)*/)
	{
		rtw_free_network_nolock(pmlmepriv, pwlan);
	}

	if (lock_scanned_queue)
		spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));

	pmlmepriv->key_mask = 0;

}

/*
*rtw_indicate_connect: the caller has to lock pmlmepriv->lock
*/
void rtw_indicate_connect(struct rtl_priv *rtlpriv)
{
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct xmit_priv	*pxmitpriv = &rtlpriv->xmitpriv;

	pmlmepriv->to_join = false;

	if (!check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED)) {
		set_fwstate(pmlmepriv, _FW_LINKED);
		rtw_os_indicate_connect(rtlpriv);

	}
}


/*
*rtw_indicate_disconnect: the caller has to lock pmlmepriv->lock
*/
void rtw_indicate_disconnect( struct rtl_priv *rtlpriv )
{
	struct	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv *pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX	*cur_network = &(pmlmeinfo->network);
	struct sta_info *psta;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

	_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING|WIFI_UNDER_WPS);

        //DBG_871X("clear wps when %s\n", __func__);

	if (check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED)
	   || (0 <= 0)) {
		rtw_os_indicate_disconnect(rtlpriv);

		//set ips_deny_time to avoid enter IPS before LPS leave
		rtlpriv->pwrctrlpriv.ips_deny_time = jiffies + rtw_ms_to_systime(3000);

		_clr_fwstate_(pmlmepriv, _FW_LINKED);
	}

	rtw_lps_ctrl_wk_cmd(rtlpriv, LPS_CTRL_DISCONNECT, 1);
}

inline void rtw_indicate_scan_done( struct rtl_priv *rtlpriv, bool aborted)
{
	rtw_os_indicate_scan_done(rtlpriv, aborted);
}

void rtw_scan_abort(struct rtl_priv *rtlpriv)
{
	uint32_t	 cnt=0;
	uint32_t	 start;
	struct mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);

	start = jiffies;
	pmlmeext->scan_abort = true;
	while (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY)
		&& rtw_get_passing_time_ms(start) <= 200) {

		if (rtlpriv->bDriverStopped || rtlpriv->bSurpriseRemoved)
			break;

		DBG_871X(FUNC_NDEV_FMT"fw_state=_FW_UNDER_SURVEY!\n", FUNC_NDEV_ARG(rtlpriv->ndev));
		msleep(20);
	}

	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY)) {
		if (!rtlpriv->bDriverStopped && !rtlpriv->bSurpriseRemoved)
			DBG_871X(FUNC_NDEV_FMT"waiting for scan_abort time out!\n", FUNC_NDEV_ARG(rtlpriv->ndev));
		rtw_indicate_scan_done(rtlpriv, true);
	}
	pmlmeext->scan_abort = false;
}

static struct sta_info *rtw_joinbss_update_stainfo(struct rtl_priv *rtlpriv, struct wlan_network *pnetwork)
{
	int i;
	struct sta_info *bmc_sta, *psta=NULL;
	struct recv_reorder_ctrl *preorder_ctrl;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;

	psta = rtw_get_stainfo(pstapriv, pnetwork->network.MacAddress);
	if (psta==NULL) {
		psta = rtw_alloc_stainfo(pstapriv, pnetwork->network.MacAddress);
	}

	if (psta) //update ptarget_sta
	{
		DBG_871X("%s\n", __FUNCTION__);

		psta->aid  = pnetwork->join_res;

		//psta->raid = networktype_to_raid(pmlmeext->cur_wireless_mode);
		psta->raid = rtw_hal_networktype_to_raid(rtlpriv,pmlmeext->cur_wireless_mode);


		//sta mode
		rtw_set_sta_info(rtlpriv, psta, true);

		//security related
		if (rtlpriv->securitypriv.dot11AuthAlgrthm== dot11AuthAlgrthm_8021X)
		{
			rtlpriv->securitypriv.binstallGrpkey=false;
			rtlpriv->securitypriv.busetkipkey=false;
			rtlpriv->securitypriv.bgrpkey_handshake=false;

			psta->ieee8021x_blocked=true;
			psta->dot118021XPrivacy=rtlpriv->securitypriv.dot11PrivacyAlgrthm;

			memset((uint8_t *)&psta->dot118021x_UncstKey, 0, sizeof (union Keytype));

			memset((uint8_t *)&psta->dot11tkiprxmickey, 0, sizeof (union Keytype));
			memset((uint8_t *)&psta->dot11tkiptxmickey, 0, sizeof (union Keytype));

			memset((uint8_t *)&psta->dot11txpn, 0, sizeof (union pn48));
			memset((uint8_t *)&psta->dot11rxpn, 0, sizeof (union pn48));
		}

		//	Commented by Albert 2012/07/21
		//	When doing the WPS, the wps_ie_len won't equal to 0
		//	And the Wi-Fi driver shouldn't allow the data packet to be tramsmitted.
		if ( rtlpriv->securitypriv.wps_ie_len != 0 )
		{
			psta->ieee8021x_blocked=true;
			rtlpriv->securitypriv.wps_ie_len = 0;
		}


		//for A-MPDU Rx reordering buffer control for bmc_sta & sta_info
		//if A-MPDU Rx is enabled, reseting  rx_ordering_ctrl wstart_b(indicate_seq) to default value=0xffff
		//todo: check if AP can send A-MPDU packets
		for(i=0; i < 16 ; i++)
		{
			//preorder_ctrl = &precvpriv->recvreorder_ctrl[i];
			preorder_ctrl = &psta->recvreorder_ctrl[i];
			preorder_ctrl->enable = false;
			preorder_ctrl->indicate_seq = 0xffff;
			preorder_ctrl->wend_b= 0xffff;
			preorder_ctrl->wsize_b = 64;//max_ampdu_sz;//ex. 32(kbytes) -> wsize_b=32
		}


		bmc_sta = rtw_get_bcmc_stainfo(rtlpriv);
		if (bmc_sta)
		{
			for(i=0; i < 16 ; i++)
			{
				//preorder_ctrl = &precvpriv->recvreorder_ctrl[i];
				preorder_ctrl = &bmc_sta->recvreorder_ctrl[i];
				preorder_ctrl->enable = false;
				preorder_ctrl->indicate_seq = 0xffff;
				preorder_ctrl->wend_b= 0xffff;
				preorder_ctrl->wsize_b = 64;//max_ampdu_sz;//ex. 32(kbytes) -> wsize_b=32
			}
		}


		//misc.
		update_sta_info(rtlpriv, psta);

	}

	return psta;

}

//pnetwork : returns from rtw_joinbss_event_callback
//ptarget_wlan: found from scanned_queue
static void rtw_joinbss_update_network(struct rtl_priv *rtlpriv, struct wlan_network *ptarget_wlan, struct wlan_network  *pnetwork)
{
	struct mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct wlan_network  *cur_network = &(pmlmepriv->cur_network);

	DBG_871X("%s\n", __FUNCTION__);

	// why not use ptarget_wlan??
	memcpy(&cur_network->network, &pnetwork->network, pnetwork->network.Length);
	// some IEs in pnetwork is wrong, so we should use ptarget_wlan IEs
	cur_network->network.IELength = ptarget_wlan->network.IELength;
	memcpy(&cur_network->network.IEs[0], &ptarget_wlan->network.IEs[0], MAX_IE_SZ);

	cur_network->aid = pnetwork->join_res;


#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	rtw_set_signal_stat_timer(&rtlpriv->recvpriv);
#endif
	rtlpriv->recvpriv.signal_strength = ptarget_wlan->network.PhyInfo.SignalStrength;
	rtlpriv->recvpriv.signal_qual = ptarget_wlan->network.PhyInfo.SignalQuality;
	//the ptarget_wlan->network.Rssi is raw data, we use ptarget_wlan->network.PhyInfo.SignalStrength instead (has scaled)
	rtlpriv->recvpriv.rssi = translate_percentage_to_dbm(ptarget_wlan->network.PhyInfo.SignalStrength);
#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	rtw_set_signal_stat_timer(&rtlpriv->recvpriv);
#endif

	//update fw_state //will clr _FW_UNDER_LINKING here indirectly
	switch(pnetwork->network.InfrastructureMode)
	{
		case Ndis802_11Infrastructure:

				if (pmlmepriv->fw_state&WIFI_UNDER_WPS)
					pmlmepriv->fw_state = WIFI_STATION_STATE|WIFI_UNDER_WPS;
				else
					pmlmepriv->fw_state = WIFI_STATION_STATE;

				break;
		case Ndis802_11IBSS:
				pmlmepriv->fw_state = WIFI_ADHOC_STATE;
				break;
		default:
				pmlmepriv->fw_state = WIFI_NULL_STATE;
				break;
	}

	rtw_update_protection(rtlpriv, (cur_network->network.IEs) + sizeof (NDIS_802_11_FIXED_IEs),
									(cur_network->network.IELength));

	rtw_update_ht_cap(rtlpriv, cur_network->network.IEs, cur_network->network.IELength, (uint8_t) cur_network->network.Configuration.DSConfig);
}

//Notes: the fucntion could be > passive_level (the same context as Rx tasklet)
//pnetwork : returns from rtw_joinbss_event_callback
//ptarget_wlan: found from scanned_queue
//if join_res > 0, for (fw_state==WIFI_STATION_STATE), we check if  "ptarget_sta" & "ptarget_wlan" exist.
//if join_res > 0, for (fw_state==WIFI_ADHOC_STATE), we only check if "ptarget_wlan" exist.
//if join_res > 0, update "cur_network->network" from "pnetwork->network" if (ptarget_wlan !=NULL).
//
//#define REJOIN
void rtw_joinbss_event_prehandle(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	static uint8_t retry=0;
	struct sta_info *ptarget_sta= NULL, *pcur_sta = NULL;
   	struct	sta_priv *pstapriv = &rtlpriv->stapriv;
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct wlan_network 	*pnetwork	= (struct wlan_network *)pbuf;
	struct wlan_network 	*cur_network = &(pmlmepriv->cur_network);
	struct wlan_network	*pcur_wlan = NULL, *ptarget_wlan = NULL;
	unsigned int 		the_same_macaddr = false;

	rtw_get_encrypt_decrypt_from_registrypriv(rtlpriv);

	the_same_macaddr = _rtw_memcmp(pnetwork->network.MacAddress, cur_network->network.MacAddress, ETH_ALEN);

	pnetwork->network.Length = get_WLAN_BSSID_EX_sz(&pnetwork->network);
	if (pnetwork->network.Length > sizeof(WLAN_BSSID_EX))
	{
		goto ignore_joinbss_callback;
	}

	spin_lock_bh(&pmlmepriv->lock);

	if (pnetwork->join_res > 0)
	{
		spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
		retry = 0;
		if (check_fwstate(pmlmepriv,_FW_UNDER_LINKING) )
		{
			//s1. find ptarget_wlan
			if (check_fwstate(pmlmepriv, _FW_LINKED) )
			{
				if (the_same_macaddr == true)
				{
					ptarget_wlan = rtw_find_network(&pmlmepriv->scanned_queue, cur_network->network.MacAddress);
				}
				else
				{
					pcur_wlan = rtw_find_network(&pmlmepriv->scanned_queue, cur_network->network.MacAddress);
					if (pcur_wlan)	pcur_wlan->fixed = false;

					pcur_sta = rtw_get_stainfo(pstapriv, cur_network->network.MacAddress);
					if (pcur_sta){
						spin_lock_bh(&(pstapriv->sta_hash_lock));
						rtw_free_stainfo(rtlpriv,  pcur_sta);
						spin_unlock_bh(&(pstapriv->sta_hash_lock));
					}

					ptarget_wlan = rtw_find_network(&pmlmepriv->scanned_queue, pnetwork->network.MacAddress);
					if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true){
						if (ptarget_wlan)	ptarget_wlan->fixed = true;
					}
				}

			}
			else
			{
				ptarget_wlan = rtw_find_network(&pmlmepriv->scanned_queue, pnetwork->network.MacAddress);
				if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true){
					if (ptarget_wlan)	ptarget_wlan->fixed = true;
				}
			}

			//s2. update cur_network
			if (ptarget_wlan)
			{
				rtw_joinbss_update_network(rtlpriv, ptarget_wlan, pnetwork);
			}
			else
			{
				spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
				goto ignore_joinbss_callback;
			}


			//s3. find ptarget_sta & update ptarget_sta after update cur_network only for station mode
			if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true)
			{
				ptarget_sta = rtw_joinbss_update_stainfo(rtlpriv, pnetwork);
				if (ptarget_sta==NULL)
				{
					spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
					goto ignore_joinbss_callback;
				}
			}

			//s4. indicate connect
				if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true)
				{
					rtw_indicate_connect(rtlpriv);
				}
				else
				{
					;
					//adhoc mode will rtw_indicate_connect when rtw_stassoc_event_callback
				}


			//s5. Cancle assoc_timer
			del_timer_sync(&pmlmepriv->assoc_timer);


		}
		else
		{
			spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
			goto ignore_joinbss_callback;
		}

		spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));

	}
	else if (pnetwork->join_res == -4)
	{
		rtw_reset_securitypriv(rtlpriv);
		_set_timer(&pmlmepriv->assoc_timer, 1);

		//rtw_free_assoc_resources(rtlpriv, 1);

		if ((check_fwstate(pmlmepriv, _FW_UNDER_LINKING)) == true)
		{
			_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
		}

	}
	else //if join_res < 0 (join fails), then try again
	{

		#ifdef REJOIN
		res = _FAIL;
		if (retry < 2) {
			res = rtw_select_and_join_from_scanned_queue(pmlmepriv);
		}

		 if (res == _SUCCESS)
		{
			//extend time of assoc_timer
			_set_timer(&pmlmepriv->assoc_timer, MAX_JOIN_TIMEOUT);
			retry++;
		}
		else if (res == 2)//there is no need to wait for join
		{
			_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
			rtw_indicate_connect(rtlpriv);
		}
		else
		{
			RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("Set Assoc_Timer = 1; can't find match ssid in scanned_q \n"));
		#endif

			_set_timer(&pmlmepriv->assoc_timer, 1);
			//rtw_free_assoc_resources(rtlpriv, 1);
			_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);

		#ifdef REJOIN
			retry = 0;
		}
		#endif
	}

ignore_joinbss_callback:

	spin_unlock_bh(&pmlmepriv->lock);

}

void rtw_joinbss_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	struct wlan_network 	*pnetwork	= (struct wlan_network *)pbuf;



	mlmeext_joinbss_event_callback(rtlpriv, pnetwork->join_res);

	rtw_os_xmit_schedule(rtlpriv);


}

uint8_t search_max_mac_id(struct rtl_priv *rtlpriv)
{
	uint8_t mac_id, aid;
#if (RATE_ADAPTIVE_SUPPORT==1)	//for 88E RA
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv *pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

#if defined (CONFIG_AP_MODE)
	if (check_fwstate(pmlmepriv, WIFI_AP_STATE)){

#if 1
		_irqL irqL;
		struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);

		spin_lock_bh(&rtlusb->lock, &irqL);
		for(mac_id=(NUM_STA-1); mac_id>0; mac_id--)
			if (pdvobj->macid[mac_id] == true)
				break;
		spin_unlock_bh(&rtlusb->lock, &irqL);

#else
		for (aid = (pstapriv->max_num_sta); aid > 0; aid--)
		{
			if (pstapriv->sta_aid[aid-1] != NULL)
			{
				psta = pstapriv->sta_aid[aid-1];
				break;
		}
		}
/*
		for (mac_id = (pstapriv->max_num_sta-1); mac_id >= 0; mac_id--)
		{
			if (pstapriv->sta_aid[mac_id] != NULL)
				break;
		}
*/
		mac_id = aid + 1;
#endif
	}
	else
#endif
	{//adhoc  id =  31~2
		for (mac_id = (NUM_STA-1); mac_id >= IBSS_START_MAC_ID ; mac_id--)
		{
			if (pmlmeinfo->FW_sta_info[mac_id].status == 1)
			{
				break;
			}
		}
	}
#endif

	DBG_871X("max mac_id=%d\n", mac_id);

	return mac_id;

}

//FOR AP ,AD-HOC mode
void rtw_stassoc_hw_rpt(struct rtl_priv *rtlpriv,struct sta_info *psta)
{
	u16 media_status;

	if (psta==NULL)	return;

	#if (RATE_ADAPTIVE_SUPPORT==1)	//for 88E RA
	{
		uint8_t macid = search_max_mac_id(rtlpriv);
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv,HW_VAR_TX_RPT_MAX_MACID, (uint8_t *)&macid);
	}
	#endif
	media_status = (psta->mac_id<<8)|1; //  MACID|OPMODE:1 connect
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv,HW_VAR_H2C_MEDIA_STATUS_RPT,(uint8_t *)&media_status);
}

void rtw_stassoc_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	struct sta_info *psta;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct stassoc_event	*pstassoc	= (struct stassoc_event*)pbuf;
	struct wlan_network 	*cur_network = &(pmlmepriv->cur_network);
	struct wlan_network	*ptarget_wlan = NULL;



	if (rtw_access_ctrl(rtlpriv, pstassoc->macaddr) == false)
		return;

#if defined (CONFIG_AP_MODE)
	if (check_fwstate(pmlmepriv, WIFI_AP_STATE))
	{
		psta = rtw_get_stainfo(&rtlpriv->stapriv, pstassoc->macaddr);
		if (psta)
		{
			//bss_cap_update_on_sta_join(rtlpriv, psta);
			//sta_info_update(rtlpriv, psta);
			ap_sta_info_defer_update(rtlpriv, psta);

			rtw_stassoc_hw_rpt(rtlpriv,psta);
		}

		goto exit;
	}
#endif
	//for AD-HOC mode
	psta = rtw_get_stainfo(&rtlpriv->stapriv, pstassoc->macaddr);
	if ( psta != NULL)
	{
		//the sta have been in sta_info_queue => do nothing

		goto exit; //(between drv has received this event before and  fw have not yet to set key to CAM_ENTRY)
	}

	psta = rtw_alloc_stainfo(&rtlpriv->stapriv, pstassoc->macaddr);
	if (psta == NULL) {
		goto exit;
	}

	//to do : init sta_info variable
	psta->qos_option = 0;
	psta->mac_id = (uint)pstassoc->cam_id;
	//psta->aid = (uint)pstassoc->cam_id;
	DBG_871X("%s\n",__FUNCTION__);
	//for ad-hoc mode
	rtw_set_sta_info(rtlpriv, psta, true);

	rtw_stassoc_hw_rpt(rtlpriv,psta);

	if (rtlpriv->securitypriv.dot11AuthAlgrthm==dot11AuthAlgrthm_8021X)
		psta->dot118021XPrivacy = rtlpriv->securitypriv.dot11PrivacyAlgrthm;


	psta->ieee8021x_blocked = false;

	spin_lock_bh(&pmlmepriv->lock);

	if ( (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)==true ) ||
		(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE)==true ) )
	{
		if (rtlpriv->stapriv.asoc_sta_count== 2)
		{
			spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
			ptarget_wlan = rtw_find_network(&pmlmepriv->scanned_queue, cur_network->network.MacAddress);
			if (ptarget_wlan)	ptarget_wlan->fixed = true;
			spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
			// a sta + bc/mc_stainfo (not Ibss_stainfo)
			rtw_indicate_connect(rtlpriv);
		}
	}

	spin_unlock_bh(&pmlmepriv->lock);


	mlmeext_sta_add_event_callback(rtlpriv, psta);

exit:

	;

}

void rtw_stadel_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	int mac_id = (-1);
	struct sta_info *psta;
	struct wlan_network* pwlan = NULL;
	WLAN_BSSID_EX    *pdev_network=NULL;
	uint8_t * pibss = NULL;
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct 	stadel_event *pstadel	= (struct stadel_event*)pbuf;
   	struct	sta_priv *pstapriv = &rtlpriv->stapriv;
	struct wlan_network *tgt_network = &(pmlmepriv->cur_network);



	psta = rtw_get_stainfo(&rtlpriv->stapriv, pstadel->macaddr);
	if (psta)
		mac_id = psta->mac_id;
	else
		mac_id = pstadel->mac_id;

	DBG_871X("%s(mac_id=%d)=" MAC_FMT "\n", __func__, mac_id, MAC_ARG(pstadel->macaddr));

	if (mac_id>=0){
		u16 media_status;
		media_status = (mac_id<<8)|0; //  MACID|OPMODE:0 means disconnect
		//for STA,AP,ADHOC mode, report disconnect stauts to FW
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_H2C_MEDIA_STATUS_RPT, (uint8_t *)&media_status);
	}

        if (check_fwstate(pmlmepriv, WIFI_AP_STATE))
        {
		return;
        }


	mlmeext_sta_del_event_callback(rtlpriv);

	spin_lock_bh(&pmlmepriv->lock);

	if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) )
	{
		rtw_free_uc_swdec_pending_queue(rtlpriv);

		rtw_free_assoc_resources(rtlpriv, 1);
		rtw_indicate_disconnect(rtlpriv);
		spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
		// remove the network entry in scanned_queue
		pwlan = rtw_find_network(&pmlmepriv->scanned_queue, tgt_network->network.MacAddress);
		if (pwlan) {
			pwlan->fixed = false;
			rtw_free_network_nolock(pmlmepriv, pwlan);
		}
		spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
	}

	if ( check_fwstate(pmlmepriv,WIFI_ADHOC_MASTER_STATE) ||
	      check_fwstate(pmlmepriv,WIFI_ADHOC_STATE))
	{

		spin_lock_bh(&(pstapriv->sta_hash_lock));
		rtw_free_stainfo(rtlpriv,  psta);
		spin_unlock_bh(&(pstapriv->sta_hash_lock));

		if (rtlpriv->stapriv.asoc_sta_count== 1) //a sta + bc/mc_stainfo (not Ibss_stainfo)
		{
			//rtw_indicate_disconnect(rtlpriv);//removed@20091105
			spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
			//free old ibss network
			//pwlan = rtw_find_network(&pmlmepriv->scanned_queue, pstadel->macaddr);
			pwlan = rtw_find_network(&pmlmepriv->scanned_queue, tgt_network->network.MacAddress);
			if (pwlan)
			{
				pwlan->fixed = false;
				rtw_free_network_nolock(pmlmepriv, pwlan);
			}
			spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
			//re-create ibss
			pdev_network = &(rtlpriv->registrypriv.dev_network);
			pibss = rtlpriv->registrypriv.dev_network.MacAddress;

			memcpy(pdev_network, &tgt_network->network, get_WLAN_BSSID_EX_sz(&tgt_network->network));

			memset(&pdev_network->Ssid, 0, sizeof(NDIS_802_11_SSID));
			memcpy(&pdev_network->Ssid, &pmlmepriv->assoc_ssid, sizeof(NDIS_802_11_SSID));

			rtw_update_registrypriv_dev_network(rtlpriv);

			rtw_generate_random_ibss(pibss);

			if (check_fwstate(pmlmepriv,WIFI_ADHOC_STATE))
			{
				set_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
				_clr_fwstate_(pmlmepriv, WIFI_ADHOC_STATE);
			}

			if (rtw_createbss_cmd(rtlpriv)!=_SUCCESS)
			{
				;

			}


		}

	}

	spin_unlock_bh(&pmlmepriv->lock);



}


void rtw_cpwm_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{





}

/*
* _rtw_join_timeout_handler - Timeout/faliure handler for CMD JoinBss
* @rtlpriv: pointer to struct rtl_priv structure
*/
void _rtw_join_timeout_handler (struct rtl_priv *rtlpriv)
{
	struct	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;




	DBG_871X("%s, fw_state=%x\n", __FUNCTION__, get_fwstate(pmlmepriv));

	if (rtlpriv->bDriverStopped ||rtlpriv->bSurpriseRemoved)
		return;


	spin_lock_bh(&pmlmepriv->lock);

	{
		rtw_indicate_disconnect(rtlpriv);
		free_scanqueue(pmlmepriv);//???

 	}

	spin_unlock_bh(&pmlmepriv->lock);



}

/*
* rtw_scan_timeout_handler - Timeout/Faliure handler for CMD SiteSurvey
* @rtlpriv: pointer to struct rtl_priv structure
*/
void rtw_scan_timeout_handler (struct rtl_priv *rtlpriv)
{
	struct	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;

	DBG_871X(FUNC_ADPT_FMT" fw_state=%x\n", FUNC_ADPT_ARG(rtlpriv), get_fwstate(pmlmepriv));

	spin_lock_bh(&pmlmepriv->lock);

	_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY);

	spin_unlock_bh(&pmlmepriv->lock);

	rtw_indicate_scan_done(rtlpriv, true);

}

void rtw_dynamic_check_timer_handlder(struct rtl_priv *rtlpriv)
{
#ifdef CONFIG_AP_MODE
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
#endif //CONFIG_AP_MODE
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;

	if (!rtlpriv)
		return;

	if (rtlpriv->hw_init_completed == false)
		return;

	if ((rtlpriv->bDriverStopped == true)||(rtlpriv->bSurpriseRemoved== true))
		return;


	if (rtlpriv->net_closed == true)
	{
		return;
	}

	rtw_dynamic_chk_wk_cmd(rtlpriv);

#ifdef CONFIG_AP_MODE
	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)
	{
		expire_timeout_chk(rtlpriv);
	}
#endif
}

#if defined(IEEE80211_SCAN_RESULT_EXPIRE)
#define RTW_SCAN_RESULT_EXPIRE IEEE80211_SCAN_RESULT_EXPIRE/HZ*1000 -1000 //3000 -1000
#else
#define RTW_SCAN_RESULT_EXPIRE 2000
#endif

/*
* Select a new join candidate from the original @param candidate and @param competitor
* @return true: candidate is updated
* @return false: candidate is not updated
*/
static int rtw_check_join_candidate(struct mlme_priv *pmlmepriv
	, struct wlan_network **candidate, struct wlan_network *competitor)
{
	int updated = false;
	struct rtl_priv *rtlpriv = container_of(pmlmepriv, struct rtl_priv, mlmepriv);


	//check bssid, if needed
	if (pmlmepriv->assoc_by_bssid==true) {
		if (_rtw_memcmp(competitor->network.MacAddress, pmlmepriv->assoc_bssid, ETH_ALEN) ==false)
			goto exit;
	}

	//check ssid, if needed
	if (pmlmepriv->assoc_ssid.Ssid && pmlmepriv->assoc_ssid.SsidLength) {
		if (	competitor->network.Ssid.SsidLength != pmlmepriv->assoc_ssid.SsidLength
			|| _rtw_memcmp(competitor->network.Ssid.Ssid, pmlmepriv->assoc_ssid.Ssid, pmlmepriv->assoc_ssid.SsidLength) == false
		)
			goto exit;
	}

	if (rtw_is_desired_network(rtlpriv, competitor)  == false)
		goto exit;


	if (*candidate == NULL ||(*candidate)->network.Rssi<competitor->network.Rssi )
	{
		*candidate = competitor;
		updated = true;
	}

	if (updated){
		DBG_871X("[by_bssid:%u][assoc_ssid:%s]"
			"new candidate: %s("MAC_FMT") rssi:%d\n",
			pmlmepriv->assoc_by_bssid,
			pmlmepriv->assoc_ssid.Ssid,
			(*candidate)->network.Ssid.Ssid,
			MAC_ARG((*candidate)->network.MacAddress),
			(int)(*candidate)->network.Rssi
		);
	}

exit:
	return updated;
}

/*
Calling context:
The caller of the sub-routine will be in critical section...

The caller must hold the following spinlock

pmlmepriv->lock


*/

int rtw_select_and_join_from_scanned_queue(struct mlme_priv *pmlmepriv )
{
	int ret;
	struct list_head	*phead;
	struct rtl_priv *rtlpriv;
	struct __queue	*queue	= &(pmlmepriv->scanned_queue);
	struct	wlan_network	*pnetwork = NULL;
	struct	wlan_network	*candidate = NULL;
	uint8_t 		bSupportAntDiv = false;



	spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
	phead = get_list_head(queue);
	rtlpriv = (struct rtl_priv *)pmlmepriv->nic_hdl;

	pmlmepriv->pscanned = get_next( phead );

	while (!rtw_end_of_queue_search(phead, pmlmepriv->pscanned)) {

		pnetwork = container_of(pmlmepriv->pscanned, struct wlan_network, list);
		if (pnetwork==NULL){
			ret = _FAIL;
			goto exit;
		}

		pmlmepriv->pscanned = get_next(pmlmepriv->pscanned);

		#if 0
		DBG_871X("MacAddress:"MAC_FMT" ssid:%s\n", MAC_ARG(pnetwork->network.MacAddress), pnetwork->network.Ssid.Ssid);
		#endif

		rtw_check_join_candidate(pmlmepriv, &candidate, pnetwork);

 	}

	if (candidate == NULL) {
		DBG_871X("%s: return _FAIL(candidate == NULL)\n", __FUNCTION__);
		ret = _FAIL;
		goto exit;
	} else {
		DBG_871X("%s: candidate: %s("MAC_FMT", ch:%u)\n", __FUNCTION__,
			candidate->network.Ssid.Ssid, MAC_ARG(candidate->network.MacAddress),
			candidate->network.Configuration.DSConfig);
	}


	// check for situation of  _FW_LINKED
	if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
	{
		DBG_871X("%s: _FW_LINKED while ask_for_joinbss!!!\n", __FUNCTION__);

		#if 0 // for WPA/WPA2 authentication, wpa_supplicant will expect authentication from AP, it is needed to reconnect AP...
		if (is_same_network(&pmlmepriv->cur_network.network, &candidate->network))
		{
			DBG_871X("%s: _FW_LINKED and is same network, it needn't join again\n", __FUNCTION__);

			rtw_indicate_connect(rtlpriv);//rtw_indicate_connect again

			ret = 2;
			goto exit;
		}
		else
		#endif
		{
			rtw_disassoc_cmd(rtlpriv, 0, true);
			rtw_indicate_disconnect(rtlpriv);
			rtw_free_assoc_resources(rtlpriv, 0);
		}
	}

	set_fwstate(pmlmepriv, _FW_UNDER_LINKING);
	ret = rtw_joinbss_cmd(rtlpriv, candidate);

exit:
	spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));



	return ret;
}


int rtw_set_auth(struct rtl_priv * rtlpriv,struct security_priv *psecuritypriv)
{
	struct	cmd_obj* pcmd;
	struct 	setauth_parm *psetauthparm;
	struct	cmd_priv	*pcmdpriv=&(rtlpriv->cmdpriv);
	int		res=_SUCCESS;



	pcmd = (struct	cmd_obj*)rtw_zmalloc(sizeof(struct	cmd_obj));
	if (pcmd==NULL){
		res= _FAIL;  //try again
		goto exit;
	}

	psetauthparm=(struct setauth_parm*)rtw_zmalloc(sizeof(struct setauth_parm));
	if (psetauthparm==NULL){
		rtw_mfree(pcmd);
		res= _FAIL;
		goto exit;
	}

	memset(psetauthparm, 0, sizeof(struct setauth_parm));
	psetauthparm->mode=(unsigned char)psecuritypriv->dot11AuthAlgrthm;

	pcmd->cmdcode = _SetAuth_CMD_;
	pcmd->parmbuf = (unsigned char *)psetauthparm;
	pcmd->cmdsz =  (sizeof(struct setauth_parm));
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;


	INIT_LIST_HEAD(&pcmd->list);

	res = rtw_enqueue_cmd(pcmdpriv, pcmd);

exit:



	return res;

}


int rtw_set_key(struct rtl_priv * rtlpriv,struct security_priv *psecuritypriv,int keyid, uint8_t set_tx)
{
	uint8_t	keylen;
	struct cmd_obj		*pcmd;
	struct setkey_parm	*psetkeyparm;
	struct cmd_priv		*pcmdpriv = &(rtlpriv->cmdpriv);
	struct mlme_priv		*pmlmepriv = &(rtlpriv->mlmepriv);
	int	res=_SUCCESS;



	pcmd = (struct	cmd_obj*)rtw_zmalloc(sizeof(struct	cmd_obj));
	if (pcmd==NULL){
		res= _FAIL;  //try again
		goto exit;
	}
	psetkeyparm=(struct setkey_parm*)rtw_zmalloc(sizeof(struct setkey_parm));
	if (psetkeyparm==NULL){
		rtw_mfree(pcmd);
		res= _FAIL;
		goto exit;
	}

	memset(psetkeyparm, 0, sizeof(struct setkey_parm));

	if (psecuritypriv->dot11AuthAlgrthm ==dot11AuthAlgrthm_8021X){
		psetkeyparm->algorithm=(unsigned char)psecuritypriv->dot118021XGrpPrivacy;
	}
	else{
		psetkeyparm->algorithm=(uint8_t)psecuritypriv->dot11PrivacyAlgrthm;

	}
	psetkeyparm->keyid = (uint8_t)keyid;//0~3
	psetkeyparm->set_tx = set_tx;
	if (is_wep_enc(psetkeyparm->algorithm))
		pmlmepriv->key_mask |= BIT(psetkeyparm->keyid);

#ifdef CONFIG_AUTOSUSPEND
	if ( true  == rtlpriv->pwrctrlpriv.bInternalAutoSuspend)
	{
		rtlpriv->pwrctrlpriv.wepkeymask = pmlmepriv->key_mask;
		DBG_871X("....AutoSuspend pwrctrlpriv.wepkeymask(%x)\n",rtlpriv->pwrctrlpriv.wepkeymask);
	}
#endif
	DBG_871X("==> rtw_set_key algorithm(%x),keyid(%x),key_mask(%x)\n",psetkeyparm->algorithm,psetkeyparm->keyid,pmlmepriv->key_mask);

	switch(psetkeyparm->algorithm){

		case WEP40_ENCRYPTION:
			keylen=5;
			memcpy(&(psetkeyparm->key[0]), &(psecuritypriv->dot11DefKey[keyid].skey[0]), keylen);
			break;
		case WEP104_ENCRYPTION:
			keylen=13;
			memcpy(&(psetkeyparm->key[0]), &(psecuritypriv->dot11DefKey[keyid].skey[0]), keylen);
			break;
		case TKIP_ENCRYPTION:
			keylen=16;
			memcpy(&psetkeyparm->key, &psecuritypriv->dot118021XGrpKey[keyid], keylen);
			psetkeyparm->grpkey=1;
			break;
		case AESCCMP_ENCRYPTION:
			keylen=16;
			memcpy(&psetkeyparm->key, &psecuritypriv->dot118021XGrpKey[keyid], keylen);
			psetkeyparm->grpkey=1;
			break;
		default:
			res= _FAIL;
			goto exit;
	}


	pcmd->cmdcode = _SetKey_CMD_;
	pcmd->parmbuf = (uint8_t *)psetkeyparm;
	pcmd->cmdsz =  (sizeof(struct setkey_parm));
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;


	INIT_LIST_HEAD(&pcmd->list);

	//sema_init(&(pcmd->cmd_sem), 0);

	res = rtw_enqueue_cmd(pcmdpriv, pcmd);

exit:

	return res;

}


//adjust IEs for rtw_joinbss_cmd in WMM
int rtw_restruct_wmm_ie(struct rtl_priv *rtlpriv, uint8_t *in_ie, uint8_t *out_ie, uint in_len, uint initial_out_len)
{
	unsigned	int ielength=0;
	unsigned int i, j;

	i = 12; //after the fixed IE
	while(i<in_len)
	{
		ielength = initial_out_len;

		if (in_ie[i] == 0xDD && in_ie[i+2] == 0x00 && in_ie[i+3] == 0x50  && in_ie[i+4] == 0xF2 && in_ie[i+5] == 0x02 && i+5 < in_len) //WMM element ID and OUI
		{

			//Append WMM IE to the last index of out_ie
			/*
			for(j=i; j< i+(in_ie[i+1]+2); j++)
			{
				out_ie[ielength] = in_ie[j];
				ielength++;
			}
			out_ie[initial_out_len+8] = 0x00; //force the QoS Info Field to be zero
	                */

                        for ( j = i; j < i + 9; j++ )
                        {
                            out_ie[ ielength] = in_ie[ j ];
                            ielength++;
                        }
                        out_ie[ initial_out_len + 1 ] = 0x07;
                        out_ie[ initial_out_len + 6 ] = 0x00;
                        out_ie[ initial_out_len + 8 ] = 0x00;

			break;
		}

		i+=(in_ie[i+1]+2); // to the next IE element
	}

	return ielength;

}


//
// Ported from 8185: IsInPreAuthKeyList(). (Renamed from SecIsInPreAuthKeyList(), 2006-10-13.)
// Added by Annie, 2006-05-07.
//
// Search by BSSID,
// Return Value:
//		-1 		:if there is no pre-auth key in the  table
//		>=0		:if there is pre-auth key, and   return the entry id
//
//

static int SecIsInPMKIDList(struct rtl_priv *rtlpriv, uint8_t *bssid)
{
	struct security_priv *psecuritypriv=&rtlpriv->securitypriv;
	int i=0;

	do
	{
		if ( ( psecuritypriv->PMKIDList[i].bUsed ) &&
                    (  _rtw_memcmp( psecuritypriv->PMKIDList[i].Bssid, bssid, ETH_ALEN ) == true ) )
		{
			break;
		}
		else
		{
			i++;
			//continue;
		}

	}while(i<NUM_PMKID_CACHE);

	if ( i == NUM_PMKID_CACHE )
	{
		i = -1;// Could not find.
	}
	else
	{
		// There is one Pre-Authentication Key for the specific BSSID.
	}

	return (i);

}

//
// Check the RSN IE length
// If the RSN IE length <= 20, the RSN IE didn't include the PMKID information
// 0-11th element in the array are the fixed IE
// 12th element in the array is the IE
// 13th element in the array is the IE length
//

static int rtw_append_pmkid(struct rtl_priv *rtlpriv,int iEntry, uint8_t *ie, uint ie_len)
{
	struct security_priv *psecuritypriv=&rtlpriv->securitypriv;

	if (ie[13]<=20){
		// The RSN IE didn't include the PMK ID, append the PMK information
			ie[ie_len]=1;
			ie_len++;
			ie[ie_len]=0;	//PMKID count = 0x0100
			ie_len++;
			memcpy(	&ie[ie_len], &psecuritypriv->PMKIDList[iEntry].PMKID, 16);

			ie_len+=16;
			ie[13]+=18;//PMKID length = 2+16

	}
	return (ie_len);

}
int rtw_restruct_sec_ie(struct rtl_priv *rtlpriv,uint8_t *in_ie, uint8_t *out_ie, uint in_len)
{
	uint8_t authmode, securitytype, match;
	uint8_t sec_ie[255], uncst_oui[4], bkup_ie[255];
	uint8_t wpa_oui[4]={0x0, 0x50, 0xf2, 0x01};
	uint 	ielength, cnt, remove_cnt;
	int iEntry;

	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv *psecuritypriv=&rtlpriv->securitypriv;
	uint 	ndisauthmode=psecuritypriv->ndisauthtype;
	uint ndissecuritytype = psecuritypriv->ndisencryptstatus;



	//copy fixed ie only
	memcpy(out_ie, in_ie,12);
	ielength=12;
	if ((ndisauthmode==Ndis802_11AuthModeWPA)||(ndisauthmode==Ndis802_11AuthModeWPAPSK))
			authmode=_WPA_IE_ID_;
	if ((ndisauthmode==Ndis802_11AuthModeWPA2)||(ndisauthmode==Ndis802_11AuthModeWPA2PSK))
			authmode=_WPA2_IE_ID_;

	if (check_fwstate(pmlmepriv, WIFI_UNDER_WPS))
	{
		memcpy(out_ie+ielength, psecuritypriv->wps_ie, psecuritypriv->wps_ie_len);

		ielength += psecuritypriv->wps_ie_len;
	}
	else if ((authmode==_WPA_IE_ID_)||(authmode==_WPA2_IE_ID_))
	{
		//copy RSN or SSN
		memcpy(&out_ie[ielength], &psecuritypriv->supplicant_ie[0], psecuritypriv->supplicant_ie[1]+2);
		ielength+=psecuritypriv->supplicant_ie[1]+2;
		rtw_report_sec_ie(rtlpriv, authmode, psecuritypriv->supplicant_ie);
	}

	iEntry = SecIsInPMKIDList(rtlpriv, pmlmepriv->assoc_bssid);
	if (iEntry<0)
	{
		return ielength;
	}
	else
	{
		if (authmode == _WPA2_IE_ID_)
		{
			ielength=rtw_append_pmkid(rtlpriv, iEntry, out_ie, ielength);
		}
	}



	return ielength;
}

void rtw_init_registrypriv_dev_network(	struct rtl_priv * rtlpriv)
{
	struct registry_priv* pregistrypriv = &rtlpriv->registrypriv;
	WLAN_BSSID_EX    *pdev_network = &pregistrypriv->dev_network;



	memcpy(pdev_network->MacAddress, rtlpriv->mac80211.mac_addr, ETH_ALEN);

	memcpy(&pdev_network->Ssid, &pregistrypriv->ssid, sizeof(NDIS_802_11_SSID));

	pdev_network->Configuration.Length=sizeof(NDIS_802_11_CONFIGURATION);
	pdev_network->Configuration.BeaconPeriod = 100;
	pdev_network->Configuration.FHConfig.Length = 0;
	pdev_network->Configuration.FHConfig.HopPattern = 0;
	pdev_network->Configuration.FHConfig.HopSet = 0;
	pdev_network->Configuration.FHConfig.DwellTime = 0;




}

void rtw_update_registrypriv_dev_network(struct rtl_priv * rtlpriv)
{
	int sz=0;
	struct registry_priv* pregistrypriv = &rtlpriv->registrypriv;
	WLAN_BSSID_EX    *pdev_network = &pregistrypriv->dev_network;
	struct	security_priv*	psecuritypriv = &rtlpriv->securitypriv;
	struct	wlan_network	*cur_network = &rtlpriv->mlmepriv.cur_network;
	//struct	xmit_priv	*pxmitpriv = &rtlpriv->xmitpriv;



	pdev_network->Privacy = (psecuritypriv->dot11PrivacyAlgrthm > 0 ? 1 : 0) ; // adhoc no 802.1x

	pdev_network->Rssi = 0;

	switch(WIRELESS_MODE_MAX) {
	case WIRELESS_11B:
		pdev_network->NetworkTypeInUse = (Ndis802_11DS);
		break;
	case WIRELESS_11G:
	case WIRELESS_11BG:
	case WIRELESS_11_24N:
	case WIRELESS_11G_24N:
	case WIRELESS_11BG_24N:
		pdev_network->NetworkTypeInUse = (Ndis802_11OFDM24);
		break;
	case WIRELESS_11A:
	case WIRELESS_11A_5N:
		pdev_network->NetworkTypeInUse = (Ndis802_11OFDM5);
		break;
	case WIRELESS_11ABGN:
		if (pregistrypriv->channel > 14)
			pdev_network->NetworkTypeInUse = (Ndis802_11OFDM5);
		else
			pdev_network->NetworkTypeInUse = (Ndis802_11OFDM24);
		break;
	default :
		// TODO
		break;
	}

	pdev_network->Configuration.DSConfig = (pregistrypriv->channel);

	if (cur_network->network.InfrastructureMode == Ndis802_11IBSS)
		pdev_network->Configuration.ATIMWindow = (0);

	pdev_network->InfrastructureMode = (cur_network->network.InfrastructureMode);

	// 1. Supported rates
	// 2. IE

	//rtw_set_supported_rate(pdev_network->SupportedRates, pregistrypriv->wireless_mode) ; // will be called in rtw_generate_ie
	sz = rtw_generate_ie(pregistrypriv);

	pdev_network->IELength = sz;

	pdev_network->Length = get_WLAN_BSSID_EX_sz((WLAN_BSSID_EX  *)pdev_network);

	//notes: translate IELength & Length after assign the Length to cmdsz in createbss_cmd();
	//pdev_network->IELength = cpu_to_le32(sz);



}

void rtw_get_encrypt_decrypt_from_registrypriv(struct rtl_priv * rtlpriv)
{





}

//the fucntion is at passive_level
void rtw_joinbss_reset(struct rtl_priv *rtlpriv)
{
	uint8_t	threshold;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;

	struct ht_priv		*phtpriv = &pmlmepriv->htpriv;

	//todo: if you want to do something io/reg/hw setting before join_bss, please add code here

	pmlmepriv->num_FortyMHzIntolerant = 0;

	pmlmepriv->num_sta_no_ht = 0;

	phtpriv->ampdu_enable = false;//reset to disabled

	// TH=1 => means that invalidate usb rx aggregation
	// TH=0 => means that validate usb rx aggregation, use init value.
	if (phtpriv->ht_option)
	{
		threshold = 0;
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_RXDMA_AGG_PG_TH, (uint8_t *)(&threshold));
	}
	else
	{
		threshold = 1;
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_RXDMA_AGG_PG_TH, (uint8_t *)(&threshold));
	}
}


//the fucntion is >= passive_level
unsigned int rtw_restructure_ht_ie(struct rtl_priv *rtlpriv, uint8_t *in_ie, uint8_t *out_ie, uint in_len, uint *pout_len)
{
	uint32_t	 ielen, out_len;
	HT_CAP_AMPDU_FACTOR max_rx_ampdu_factor;
	unsigned char *p, *pframe;
	struct rtw_ieee80211_ht_cap ht_capie;
	unsigned char WMM_IE[] = {0x00, 0x50, 0xf2, 0x02, 0x00, 0x01, 0x00};
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct qos_priv   	*pqospriv= &pmlmepriv->qospriv;
	struct ht_priv		*phtpriv = &pmlmepriv->htpriv;


	phtpriv->ht_option = false;

	p = rtw_get_ie(in_ie+12, _HT_CAPABILITY_IE_, &ielen, in_len-12);

	if (p && ielen>0)
	{
		if (pqospriv->qos_option == 0)
		{
			out_len = *pout_len;
			pframe = rtw_set_ie(out_ie+out_len, _VENDOR_SPECIFIC_IE_,
								_WMM_IE_Length_, WMM_IE, pout_len);

			pqospriv->qos_option = 1;
		}

		out_len = *pout_len;

		memset(&ht_capie, 0, sizeof(struct rtw_ieee80211_ht_cap));

		ht_capie.cap_info = IEEE80211_HT_CAP_SUP_WIDTH |IEEE80211_HT_CAP_SGI_20 |
							IEEE80211_HT_CAP_SGI_40 | IEEE80211_HT_CAP_TX_STBC |
							IEEE80211_HT_CAP_DSSSCCK40;


		/*
		AMPDU_para [1:0]:Max AMPDU Len => 0:8k , 1:16k, 2:32k, 3:64k
		AMPDU_para [4:2]:Min MPDU Start Spacing
		*/

		max_rx_ampdu_factor = MAX_AMPDU_FACTOR_64K;
		ht_capie.ampdu_params_info = (max_rx_ampdu_factor&0x03);

		if (rtlpriv->securitypriv.dot11PrivacyAlgrthm == AESCCMP_ENCRYPTION )
			ht_capie.ampdu_params_info |= (IEEE80211_HT_CAP_AMPDU_DENSITY&(0x07<<2));
		else
			ht_capie.ampdu_params_info |= (IEEE80211_HT_CAP_AMPDU_DENSITY&0x00);


		pframe = rtw_set_ie(out_ie+out_len, _HT_CAPABILITY_IE_,
							sizeof(struct rtw_ieee80211_ht_cap), (unsigned char*)&ht_capie, pout_len);


		//memcpy(out_ie+out_len, p, ielen+2);//gtest
		//*pout_len = *pout_len + (ielen+2);


		phtpriv->ht_option = true;

		p = rtw_get_ie(in_ie+12, _HT_ADD_INFO_IE_, &ielen, in_len-12);
		if (p && (ielen==sizeof(struct ieee80211_ht_addt_info)))
		{
			out_len = *pout_len;
			pframe = rtw_set_ie(out_ie+out_len, _HT_ADD_INFO_IE_, ielen, p+2 , pout_len);
		}

	}

	return (phtpriv->ht_option);

}

//the fucntion is > passive_level (in critical_section)
void rtw_update_ht_cap(struct rtl_priv *rtlpriv, uint8_t *pie, uint ie_len, uint8_t channel)
{
	uint8_t *p, max_ampdu_sz;
	int len;
	//struct sta_info *bmc_sta, *psta;
	struct rtw_ieee80211_ht_cap *pht_capie;
	struct ieee80211_ht_addt_info *pht_addtinfo;
	//struct recv_reorder_ctrl *preorder_ctrl;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct ht_priv		*phtpriv = &pmlmepriv->htpriv;
	//struct recv_priv *precvpriv = &rtlpriv->recvpriv;
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	//struct wlan_network *pcur_network = &(pmlmepriv->cur_network);;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t cbw40_enable=0;


	if (!phtpriv->ht_option)
		return;

	if ((!pmlmeinfo->HT_info_enable) || (!pmlmeinfo->HT_caps_enable))
		return;

	DBG_871X("+rtw_update_ht_cap()\n");

	//maybe needs check if ap supports rx ampdu.
	if ((phtpriv->ampdu_enable==false))
		phtpriv->ampdu_enable = true;

	//check Max Rx A-MPDU Size
	len = 0;
	p = rtw_get_ie(pie+sizeof (NDIS_802_11_FIXED_IEs), _HT_CAPABILITY_IE_, &len, ie_len-sizeof (NDIS_802_11_FIXED_IEs));
	if (p && len>0)
	{
		pht_capie = (struct rtw_ieee80211_ht_cap *)(p+2);
		max_ampdu_sz = (pht_capie->ampdu_params_info & IEEE80211_HT_CAP_AMPDU_FACTOR);
		max_ampdu_sz = 1 << (max_ampdu_sz+3); // max_ampdu_sz (kbytes);

		//DBG_871X("rtw_update_ht_cap(): max_ampdu_sz=%d\n", max_ampdu_sz);
		phtpriv->rx_ampdu_maxlen = max_ampdu_sz;

	}


	len=0;
	p = rtw_get_ie(pie+sizeof (NDIS_802_11_FIXED_IEs), _HT_ADD_INFO_IE_, &len, ie_len-sizeof (NDIS_802_11_FIXED_IEs));
	if (p && len>0)
	{
		pht_addtinfo = (struct ieee80211_ht_addt_info *)(p+2);
		//todo:
	}

	if (channel > 14) {
		if ((0x21 & 0xf0) > 0)
			cbw40_enable = 1;
	} else {
		if ((0x21 & 0x0f) > 0)
			cbw40_enable = 1;
	}

	//update cur_bwmode & cur_ch_offset
	if ((cbw40_enable) &&
		(pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info & BIT(1)) &&
		(pmlmeinfo->HT_info.infos[0] & BIT(2)))
	{
		int i;
		uint8_t	rf_type;

		rf_type = rtlpriv->phy.rf_type;

		//update the MCS rates
		for (i = 0; i < 16; i++)
		{
			if ((rf_type == RF_1T1R) || (rf_type == RF_1T2R))
			{
				pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate[i] &= MCS_rate_1R[i];
			}
			else
			{
				#ifdef CONFIG_DISABLE_MCS13TO15
				if (pmlmeext->cur_bwmode == CHANNEL_WIDTH_40 && pregistrypriv->wifi_spec != 1 )
				{
					pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate[i] &= MCS_rate_2R_MCS13TO15_OFF[i];
				}
				else
					pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate[i] &= MCS_rate_2R[i];
				#else
				pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate[i] &= MCS_rate_2R[i];
				#endif //CONFIG_DISABLE_MCS13TO15
			}
		}
		//switch to the 40M Hz mode accoring to the AP
		pmlmeext->cur_bwmode = CHANNEL_WIDTH_40;
		switch ((pmlmeinfo->HT_info.infos[0] & 0x3))
		{
			case EXTCHNL_OFFSET_UPPER:
				pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_LOWER;
				break;

			case EXTCHNL_OFFSET_LOWER:
				pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_UPPER;
				break;

			default:
				pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
				break;
		}
	}

	//
	// Config SM Power Save setting
	//
	pmlmeinfo->SM_PS = (pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info & 0x0C) >> 2;
	if (pmlmeinfo->SM_PS == WLAN_HT_CAP_SM_PS_STATIC)
	{
		/*uint8_t i;
		//update the MCS rates
		for (i = 0; i < 16; i++)
		{
			pmlmeinfo->HT_caps.HT_cap_element.MCS_rate[i] &= MCS_rate_1R[i];
		}*/
		DBG_871X("%s(): WLAN_HT_CAP_SM_PS_STATIC\n",__FUNCTION__);
	}

	//
	// Config current HT Protection mode.
	//
	pmlmeinfo->HT_protection = pmlmeinfo->HT_info.infos[1] & 0x3;
}

void rtw_issue_addbareq_cmd(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	uint8_t issued;
	int priority;
	struct sta_info *psta=NULL;
	struct ht_priv	*phtpriv;
	struct tx_pkt_attrib *pattrib =&pxmitframe->tx_attrib;
	int32_t bmcst = is_multicast_ether_addr(pattrib->ra);

	//if (bmcst || (rtlpriv->mlmepriv.LinkDetectInfo.bTxBusyTraffic == false))
	if (bmcst || (rtlpriv->mlmepriv.LinkDetectInfo.NumTxOkInPeriod<100))
		return;

	priority = pattrib->tx_priority;

	psta = rtw_get_stainfo(&rtlpriv->stapriv, pattrib->ra);
	if (pattrib->psta != psta)
	{
		DBG_871X("%s, pattrib->psta(%p) != psta(%p)\n", __func__, pattrib->psta, psta);
		return;
	}

	if (psta==NULL)
	{
		DBG_871X("%s, psta==NUL\n", __func__);
		return;
	}

	if (!(psta->state &_FW_LINKED))
	{
		DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
		return;
	}


	phtpriv = &psta->htpriv;

	if ((phtpriv->ht_option==true) && (phtpriv->ampdu_enable==true))
	{
		issued = (phtpriv->agg_enable_bitmap>>priority)&0x1;
		issued |= (phtpriv->candidate_tid_bitmap>>priority)&0x1;

		if (0==issued)
		{
			DBG_871X("rtw_issue_addbareq_cmd, p=%d\n", priority);
			psta->htpriv.candidate_tid_bitmap |= BIT((uint8_t)priority);
			rtw_addbareq_cmd(rtlpriv,(uint8_t) priority, pattrib->ra);
		}
	}

}

int rtw_linked_check(struct rtl_priv *rtlpriv)
{
	if (	(check_fwstate(&rtlpriv->mlmepriv, WIFI_AP_STATE) == true) ||
			(check_fwstate(&rtlpriv->mlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE) == true))
	{
		if (rtlpriv->stapriv.asoc_sta_count > 2)
			return true;
	}
	else
	{	//Station mode
		if (check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED)== true)
			return true;
	}
	return false;
}

