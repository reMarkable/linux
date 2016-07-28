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
#define _RTW_CMD_C_

#include <../rtl8821au/def.h>
#include <drv_types.h>

#include <rtw_ap.h>

#include <../usb.h>
#include <../wifi.h>
#include <../cam.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

/*
Caller and the rtw_cmd_thread can protect cmd_q by spin_lock.
No irqsave is necessary.
*/

static int _rtw_init_cmd_priv (struct	cmd_priv *pcmdpriv)
{
	int res=_SUCCESS;



	sema_init(&(pcmdpriv->cmd_queue_sema), 0);
	//sema_init(&(pcmdpriv->cmd_done_sema), 0);
	sema_init(&(pcmdpriv->terminate_cmdthread_sema), 0);


	_rtw_init_queue(&(pcmdpriv->cmd_queue));

	//allocate DMA-able/Non-Page memory for cmd_buf and rsp_buf

	pcmdpriv->cmd_seq = 1;

	pcmdpriv->cmd_allocated_buf = rtw_zmalloc(MAX_CMDSZ + CMDBUFF_ALIGN_SZ);

	if (pcmdpriv->cmd_allocated_buf == NULL){
		res= _FAIL;
		goto exit;
	}

	pcmdpriv->cmd_buf = pcmdpriv->cmd_allocated_buf  +  CMDBUFF_ALIGN_SZ - ( (SIZE_PTR)(pcmdpriv->cmd_allocated_buf) & (CMDBUFF_ALIGN_SZ-1));

	pcmdpriv->rsp_allocated_buf = rtw_zmalloc(MAX_RSPSZ + 4);

	if (pcmdpriv->rsp_allocated_buf == NULL){
		res= _FAIL;
		goto exit;
	}

	pcmdpriv->rsp_buf = pcmdpriv->rsp_allocated_buf  +  4 - ( (SIZE_PTR)(pcmdpriv->rsp_allocated_buf) & 3);

	pcmdpriv->cmd_issued_cnt = pcmdpriv->cmd_done_cnt = pcmdpriv->rsp_cnt = 0;

exit:



	return res;

}

static int _rtw_init_evt_priv(struct evt_priv *pevtpriv)
{
	int res=_SUCCESS;


	//allocate DMA-able/Non-Page memory for cmd_buf and rsp_buf
	atomic_set(&pevtpriv->event_seq, 0);
	pevtpriv->evt_done_cnt = 0;

	return res;
}

static void _rtw_free_evt_priv (struct	evt_priv *pevtpriv)
{
}

static void _rtw_free_cmd_priv (struct	cmd_priv *pcmdpriv)
{


	if(pcmdpriv){
		if (pcmdpriv->cmd_allocated_buf)
			rtw_mfree(pcmdpriv->cmd_allocated_buf);

		if (pcmdpriv->rsp_allocated_buf)
			rtw_mfree(pcmdpriv->rsp_allocated_buf);
	}

}

/*
Calling Context:

rtw_enqueue_cmd can only be called between kernel thread,
since only spin_lock is used.

ISR/Call-Back functions can't call this sub-function.

*/

static int _rtw_enqueue_cmd(struct __queue *queue, struct cmd_obj *obj)
{
	unsigned long flags;



	if (obj == NULL)
		goto exit;

	//spin_lock_bh(&queue->lock, &irqL);
	spin_lock_irqsave(&queue->lock, flags);

	list_add_tail(&obj->list, &queue->list);

	//spin_unlock_bh(&queue->lock, &irqL);
	spin_unlock_irqrestore(&queue->lock, flags);

exit:



	return _SUCCESS;
}

static inline struct list_head *get_next(struct list_head	*list)
{
	return list->next;
}

static struct	cmd_obj	*_rtw_dequeue_cmd(struct __queue *queue)
{
	unsigned long flags;
	struct cmd_obj *obj;



	//spin_lock_bh(&(queue->lock), &irqL);
	spin_lock_irqsave(&queue->lock, flags);
	if (list_empty(&(queue->list)))
		obj = NULL;
	else
	{
		obj = container_of(get_next(&(queue->list)), struct cmd_obj, list);
		list_del_init(&obj->list);
	}

	//spin_unlock_bh(&(queue->lock), &irqL);
	spin_unlock_irqrestore(&queue->lock, flags);



	return obj;
}

uint32_t	rtw_init_cmd_priv(struct cmd_priv *pcmdpriv)
{
	uint32_t	res;

	res = _rtw_init_cmd_priv (pcmdpriv);

	return res;
}

uint32_t	rtw_init_evt_priv (struct	evt_priv *pevtpriv)
{
	int	res;

	res = _rtw_init_evt_priv(pevtpriv);

	return res;
}

void rtw_free_evt_priv (struct	evt_priv *pevtpriv)
{

	_rtw_free_evt_priv(pevtpriv);

}

void rtw_free_cmd_priv (struct	cmd_priv *pcmdpriv)
{

	_rtw_free_cmd_priv(pcmdpriv);

}

int rtw_cmd_filter(struct cmd_priv *pcmdpriv, struct cmd_obj *cmd_obj);
int rtw_cmd_filter(struct cmd_priv *pcmdpriv, struct cmd_obj *cmd_obj)
{
	uint8_t bAllow = false; //set to true to allow enqueuing cmd when hw_init_completed is false

	if( (pcmdpriv->rtlpriv->hw_init_completed ==false && bAllow == false)
		|| pcmdpriv->cmdthd_running== false	//com_thread not running
	)
	{
		//DBG_871X("%s:%s: drop cmdcode:%u, hw_init_completed:%u, cmdthd_running:%u\n", caller_func, __FUNCTION__,
		//	cmd_obj->cmdcode,
		//	pcmdpriv->rtlpriv->hw_init_completed,
		//	pcmdpriv->cmdthd_running
		//);

		return _FAIL;
	}
	return _SUCCESS;
}



uint32_t	 rtw_enqueue_cmd(struct cmd_priv *pcmdpriv, struct cmd_obj *cmd_obj)
{
	int res = _FAIL;
	struct rtl_priv *rtlpriv = pcmdpriv->rtlpriv;



	if (cmd_obj == NULL) {
		goto exit;
	}

	cmd_obj->rtlpriv = rtlpriv;

	if( _FAIL == (res=rtw_cmd_filter(pcmdpriv, cmd_obj)) ) {
		rtw_free_cmd_obj(cmd_obj);
		goto exit;
	}

	res = _rtw_enqueue_cmd(&pcmdpriv->cmd_queue, cmd_obj);

	if(res == _SUCCESS)
		up(&pcmdpriv->cmd_queue_sema);

exit:



	return res;
}

struct	cmd_obj	*rtw_dequeue_cmd(struct cmd_priv *pcmdpriv)
{
	struct cmd_obj *cmd_obj;



	cmd_obj = _rtw_dequeue_cmd(&pcmdpriv->cmd_queue);


	return cmd_obj;
}

void rtw_cmd_clr_isr(struct	cmd_priv *pcmdpriv)
{

	pcmdpriv->cmd_done_cnt++;
	//up(&(pcmdpriv->cmd_done_sema));

}

void rtw_free_cmd_obj(struct cmd_obj *pcmd)
{


	if((pcmd->cmdcode!=_JoinBss_CMD_) &&(pcmd->cmdcode!= _CreateBss_CMD_))
	{
		//free parmbuf in cmd_obj
		/* ULLI check usage of pcmd->cmdsz */
		rtw_mfree(pcmd->parmbuf);
	}

	if(pcmd->rsp!=NULL)
	{
		if(pcmd->rspsz!= 0)
		{
			//free rsp in cmd_obj
			/* ULLI check usage of pcmd->rspsz */
			rtw_mfree(pcmd->rsp);
		}
	}

	//free cmd_obj
	rtw_mfree(pcmd);


}

int rtw_cmd_thread(void *context)
{
	uint8_t ret;
	struct cmd_obj *pcmd;
	uint8_t *pcmdbuf, *prspbuf;
	uint8_t (*cmd_hdl)(struct rtl_priv *rtlpriv, uint8_t * pbuf);
	void (*pcmd_callback)(struct rtl_priv *dev, struct cmd_obj *pcmd);
	struct rtl_priv *rtlpriv = (struct rtl_priv *) context;
	struct cmd_priv *pcmdpriv = &(rtlpriv->cmdpriv);



	thread_enter("RTW_CMD_THREAD");

	pcmdbuf = pcmdpriv->cmd_buf;
	prspbuf = pcmdpriv->rsp_buf;

	pcmdpriv->cmdthd_running=true;
	up(&pcmdpriv->terminate_cmdthread_sema);

	while(1)
	{
		if (down_interruptible(&pcmdpriv->cmd_queue_sema))
			break;

		if ((rtlpriv->bDriverStopped == true)||(rtlpriv->bSurpriseRemoved == true))
		{
			DBG_871X("%s: DriverStopped(%d) SurpriseRemoved(%d) break at line %d\n",
				__FUNCTION__, rtlpriv->bDriverStopped, rtlpriv->bSurpriseRemoved, __LINE__);
			break;
		}

_next:
		if ((rtlpriv->bDriverStopped == true)||(rtlpriv->bSurpriseRemoved== true))
		{
			DBG_871X("%s: DriverStopped(%d) SurpriseRemoved(%d) break at line %d\n",
				__FUNCTION__, rtlpriv->bDriverStopped, rtlpriv->bSurpriseRemoved, __LINE__);
			break;
		}

		if(!(pcmd = rtw_dequeue_cmd(pcmdpriv))) {
			continue;
		}

		if( _FAIL == rtw_cmd_filter(pcmdpriv, pcmd) )
		{
			pcmd->res = H2C_DROPPED;
			goto post_process;
		}

		pcmdpriv->cmd_issued_cnt++;

		pcmd->cmdsz = _RND4((pcmd->cmdsz));//_RND4

		memcpy(pcmdbuf, pcmd->parmbuf, pcmd->cmdsz);

		if(pcmd->cmdcode <= (sizeof(wlancmds) /sizeof(struct cmd_hdl)))
		{
			cmd_hdl = wlancmds[pcmd->cmdcode].h2cfuns;

			if (cmd_hdl)
			{
				ret = cmd_hdl(pcmd->rtlpriv, pcmdbuf);
				pcmd->res = ret;
			}

			pcmdpriv->cmd_seq++;
		}
		else
		{
			pcmd->res = H2C_PARAMETERS_ERROR;
		}

		cmd_hdl = NULL;

post_process:

		//call callback function for post-processed
		if(pcmd->cmdcode <= (sizeof(rtw_cmd_callback) /sizeof(struct _cmd_callback)))
		{
			pcmd_callback = rtw_cmd_callback[pcmd->cmdcode].callback;
			if(pcmd_callback == NULL)
			{
				rtw_free_cmd_obj(pcmd);
			}
			else
			{
				//todo: !!! fill rsp_buf to pcmd->rsp if (pcmd->rsp!=NULL)
				pcmd_callback(pcmd->rtlpriv, pcmd);//need conider that free cmd_obj in rtw_cmd_callback
			}
		}
		else
		{
			rtw_free_cmd_obj(pcmd);
		}

		flush_signals_thread();

		goto _next;

	}
	pcmdpriv->cmdthd_running=false;


	// free all cmd_obj resources
	do{
		pcmd = rtw_dequeue_cmd(pcmdpriv);
		if(pcmd==NULL){
			break;
		}

		//DBG_871X("%s: leaving... drop cmdcode:%u\n", __FUNCTION__, pcmd->cmdcode);

		rtw_free_cmd_obj(pcmd);
	}while(1);

	up(&pcmdpriv->terminate_cmdthread_sema);



	 complete_and_exit(NULL, 0);

}

/*
rtw_sitesurvey_cmd(~)
	### NOTE:#### (!!!!)
	MUST TAKE CARE THAT BEFORE CALLING THIS FUNC, YOU SHOULD HAVE LOCKED pmlmepriv->lock
*/
uint8_t rtw_sitesurvey_cmd(struct rtl_priv  *rtlpriv, NDIS_802_11_SSID *ssid, int ssid_num,
	struct rtw_ieee80211_channel *ch, int ch_num)
{
	uint8_t res = _FAIL;
	struct cmd_obj		*ph2c;
	struct sitesurvey_parm	*psurveyPara;
	struct cmd_priv 	*pcmdpriv = &rtlpriv->cmdpriv;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;



	if(check_fwstate(pmlmepriv, _FW_LINKED) == true){
		rtw_lps_ctrl_wk_cmd(rtlpriv, LPS_CTRL_SCAN, 1);
	}

	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if (ph2c == NULL)
		return _FAIL;

	psurveyPara = (struct sitesurvey_parm*)rtw_zmalloc(sizeof(struct sitesurvey_parm));
	if (psurveyPara == NULL) {
		rtw_mfree(ph2c);
		return _FAIL;
	}

	rtw_free_network_queue(rtlpriv, false);

	init_h2fwcmd_w_parm_no_rsp(ph2c, psurveyPara, GEN_CMD_CODE(_SiteSurvey));

	/* psurveyPara->bsslimit = 48; */
	psurveyPara->scan_mode = pmlmepriv->scan_mode;

	/* prepare ssid list */
	if (ssid) {
		int i;
		for (i=0; i<ssid_num && i< RTW_SSID_SCAN_AMOUNT; i++) {
			if (ssid[i].SsidLength) {
				memcpy(&psurveyPara->ssid[i], &ssid[i], sizeof(NDIS_802_11_SSID));
				psurveyPara->ssid_num++;
				if (0)
				DBG_871X(FUNC_ADPT_FMT" ssid:(%s, %d)\n", FUNC_ADPT_ARG(rtlpriv),
					psurveyPara->ssid[i].Ssid, psurveyPara->ssid[i].SsidLength);
			}
		}
	}

	/* prepare channel list */
	if (ch) {
		int i;
		for (i=0; i<ch_num && i< RTW_CHANNEL_SCAN_AMOUNT; i++) {
			if (ch[i].hw_value && !(ch[i].flags & RTW_IEEE80211_CHAN_DISABLED)) {
				memcpy(&psurveyPara->ch[i], &ch[i], sizeof(struct rtw_ieee80211_channel));
				psurveyPara->ch_num++;
				if (0)
				DBG_871X(FUNC_ADPT_FMT" ch:%u\n", FUNC_ADPT_ARG(rtlpriv),
					psurveyPara->ch[i].hw_value);
			}
		}
	}

	set_fwstate(pmlmepriv, _FW_UNDER_SURVEY);

	res = rtw_enqueue_cmd(pcmdpriv, ph2c);

	if(res == _SUCCESS) {

		pmlmepriv->scan_start_time = jiffies;

			_set_timer(&pmlmepriv->scan_to_timer, SCANNING_TIMEOUT);
		pmlmepriv->scan_interval = SCAN_INTERVAL;// 30*2 sec = 60sec
	} else {
		_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY);
	}



	return res;
}

void rtw_getbbrfreg_cmdrsp_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd)
{


	//rtw_free_cmd_obj(pcmd);
	/* ULLI check usage of pcmd->cmdsz */
	rtw_mfree(pcmd->parmbuf);
	rtw_mfree(pcmd);


}

void rtw_readtssi_cmdrsp_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd)
{

	/* ULLI check usage of pcmd->cmdsz */
	rtw_mfree(pcmd->parmbuf);
	rtw_mfree(pcmd);



}

uint8_t rtw_createbss_cmd(struct rtl_priv  *rtlpriv)
{
	struct cmd_obj*			pcmd;
	struct cmd_priv 			*pcmdpriv=&rtlpriv->cmdpriv;
	struct mlme_priv			*pmlmepriv = &rtlpriv->mlmepriv;
	WLAN_BSSID_EX		*pdev_network = &rtlpriv->registrypriv.dev_network;
	uint8_t	res=_SUCCESS;

	if (pmlmepriv->assoc_ssid.SsidLength == 0){
		;
	} else {
		;
	}

	pcmd = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(pcmd==NULL){
		res= _FAIL;
		goto exit;
	}

	INIT_LIST_HEAD(&pcmd->list);
	pcmd->cmdcode = _CreateBss_CMD_;
	pcmd->parmbuf = (unsigned char *)pdev_network;
	pcmd->cmdsz = get_WLAN_BSSID_EX_sz((WLAN_BSSID_EX*)pdev_network);
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;

	pdev_network->Length = pcmd->cmdsz;

	res = rtw_enqueue_cmd(pcmdpriv, pcmd);

exit:



	return res;
}

uint8_t rtw_createbss_cmd_ex(struct rtl_priv  *rtlpriv, unsigned char *pbss, unsigned int sz)
{
	struct cmd_obj*	pcmd;
	struct cmd_priv 	*pcmdpriv=&rtlpriv->cmdpriv;
	uint8_t	res=_SUCCESS;



	pcmd = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(pcmd==NULL){
		res= _FAIL;
		goto exit;
	}

	INIT_LIST_HEAD(&pcmd->list);
	pcmd->cmdcode = GEN_CMD_CODE(_CreateBss);
	pcmd->parmbuf = pbss;
	pcmd->cmdsz =  sz;
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;

	res = rtw_enqueue_cmd(pcmdpriv, pcmd);

exit:



	return res;
}

uint8_t rtw_joinbss_cmd(struct rtl_priv  *rtlpriv, struct wlan_network* pnetwork)
{
	uint8_t	*auth, res = _SUCCESS;
	uint	t_len = 0;
	WLAN_BSSID_EX		*psecnetwork;
	struct cmd_obj		*pcmd;
	struct cmd_priv		*pcmdpriv=&rtlpriv->cmdpriv;
	struct mlme_priv		*pmlmepriv = &rtlpriv->mlmepriv;
	struct qos_priv		*pqospriv= &pmlmepriv->qospriv;
	struct security_priv	*psecuritypriv=&rtlpriv->securitypriv;
	struct registry_priv	*pregistrypriv = &rtlpriv->registrypriv;
	struct ht_priv			*phtpriv = &pmlmepriv->htpriv;
	struct vht_priv		*pvhtpriv = &pmlmepriv->vhtpriv;
	NDIS_802_11_NETWORK_INFRASTRUCTURE ndis_network_mode = pnetwork->network.InfrastructureMode;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	if (pmlmepriv->assoc_ssid.SsidLength == 0){
		;
	} else {
		;
	}

	pcmd = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(pcmd==NULL){
		res=_FAIL;
		goto exit;
	}
	/* // for IEs is pointer
	t_len = sizeof (u32) + sizeof (NDIS_802_11_MAC_ADDRESS) + 2 +
			sizeof (NDIS_802_11_SSID) + sizeof (u32) +
			sizeof (NDIS_802_11_RSSI) + sizeof (NDIS_802_11_NETWORK_TYPE) +
			sizeof (NDIS_802_11_CONFIGURATION) +
			sizeof (NDIS_802_11_NETWORK_INFRASTRUCTURE) +
			sizeof (NDIS_802_11_RATES_EX)+ sizeof(WLAN_PHY_INFO)+ sizeof (u32) + MAX_IE_SZ;
	*/
	//for IEs is fix buf size
	t_len = sizeof(WLAN_BSSID_EX);


	//for hidden ap to set fw_state here
	if (check_fwstate(pmlmepriv, WIFI_STATION_STATE|WIFI_ADHOC_STATE) != true)
	{
		switch(ndis_network_mode)
		{
			case Ndis802_11IBSS:
				set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
				break;

			case Ndis802_11Infrastructure:
				set_fwstate(pmlmepriv, WIFI_STATION_STATE);
				break;

			case Ndis802_11APMode:
			case Ndis802_11AutoUnknown:
			case Ndis802_11InfrastructureMax:
				break;

		}
	}

	psecnetwork=(WLAN_BSSID_EX *)&psecuritypriv->sec_bss;
	if(psecnetwork==NULL)
	{
		if(pcmd !=NULL)
			rtw_mfree(pcmd);

		res=_FAIL;

		goto exit;
	}

	memset(psecnetwork, 0, t_len);

	memcpy(psecnetwork, &pnetwork->network, get_WLAN_BSSID_EX_sz(&pnetwork->network));

	auth=&psecuritypriv->authenticator_ie[0];
	psecuritypriv->authenticator_ie[0]=(unsigned char)psecnetwork->IELength;

	if((psecnetwork->IELength-12) < (256-1)) {
		memcpy(&psecuritypriv->authenticator_ie[1], &psecnetwork->IEs[12], psecnetwork->IELength-12);
	} else {
		memcpy(&psecuritypriv->authenticator_ie[1], &psecnetwork->IEs[12], (256-1));
	}

	psecnetwork->IELength = 0;
	// Added by Albert 2009/02/18
	// If the the driver wants to use the bssid to create the connection.
	// If not,  we have to copy the connecting AP's MAC address to it so that
	// the driver just has the bssid information for PMKIDList searching.

	if ( pmlmepriv->assoc_by_bssid == false )
	{
		memcpy( &pmlmepriv->assoc_bssid[ 0 ], &pnetwork->network.MacAddress[ 0 ], ETH_ALEN );
	}

	psecnetwork->IELength = rtw_restruct_sec_ie(rtlpriv, &pnetwork->network.IEs[0], &psecnetwork->IEs[0], pnetwork->network.IELength);


	pqospriv->qos_option = 0;

	if(pregistrypriv->wmm_enable)
	{
		uint32_t	 tmp_len;

		tmp_len = rtw_restruct_wmm_ie(rtlpriv, &pnetwork->network.IEs[0], &psecnetwork->IEs[0], pnetwork->network.IELength, psecnetwork->IELength);

		if (psecnetwork->IELength != tmp_len)
		{
			psecnetwork->IELength = tmp_len;
			pqospriv->qos_option = 1; //There is WMM IE in this corresp. beacon
		}
		else
		{
			pqospriv->qos_option = 0;//There is no WMM IE in this corresp. beacon
		}
	}

	phtpriv->ht_option = false;
	{
		//	Added by Albert 2010/06/23
		//	For the WEP mode, we will use the bg mode to do the connection to avoid some IOT issue.
		//	Especially for Realtek 8192u SoftAP.
		if (	( rtlpriv->securitypriv.dot11PrivacyAlgrthm != WEP40_ENCRYPTION ) &&
			( rtlpriv->securitypriv.dot11PrivacyAlgrthm != WEP104_ENCRYPTION ) &&
			( rtlpriv->securitypriv.dot11PrivacyAlgrthm != TKIP_ENCRYPTION ))
		{
			//rtw_restructure_ht_ie
			rtw_restructure_ht_ie(rtlpriv, &pnetwork->network.IEs[0], &psecnetwork->IEs[0],
									pnetwork->network.IELength, &psecnetwork->IELength);
		}
	}


	pvhtpriv->vht_option = false;
	if (phtpriv->ht_option) {
		rtw_restructure_vht_ie(rtlpriv, &pnetwork->network.IEs[0], &psecnetwork->IEs[0],
								pnetwork->network.IELength, &psecnetwork->IELength);
	}

	pmlmeinfo->assoc_AP_vendor = check_assoc_AP(pnetwork->network.IEs, pnetwork->network.IELength);

	#if 0
	psecuritypriv->supplicant_ie[0]=(uint8_t)psecnetwork->IELength;

	if(psecnetwork->IELength < (256-1))
	{
		memcpy(&psecuritypriv->supplicant_ie[1], &psecnetwork->IEs[0], psecnetwork->IELength);
	}
	else
	{
		memcpy(&psecuritypriv->supplicant_ie[1], &psecnetwork->IEs[0], (256-1));
	}
	#endif

	pcmd->cmdsz = get_WLAN_BSSID_EX_sz(psecnetwork);//get cmdsz before endian conversion

	INIT_LIST_HEAD(&pcmd->list);
	pcmd->cmdcode = _JoinBss_CMD_;//GEN_CMD_CODE(_JoinBss)
	pcmd->parmbuf = (unsigned char *)psecnetwork;
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;

	res = rtw_enqueue_cmd(pcmdpriv, pcmd);

exit:



	return res;
}

uint8_t rtw_disassoc_cmd(struct rtl_priv*rtlpriv, uint32_t	 deauth_timeout_ms, bool enqueue) /* for sta_mode */
{
	struct cmd_obj *cmdobj = NULL;
	struct disconnect_parm *param = NULL;
	struct cmd_priv *cmdpriv = &rtlpriv->cmdpriv;
	uint8_t res = _SUCCESS;



	/* prepare cmd parameter */
	param = (struct disconnect_parm *)rtw_zmalloc(sizeof(*param));
	if (param == NULL) {
		res = _FAIL;
		goto exit;
	}
	param->deauth_timeout_ms = deauth_timeout_ms;

	if (enqueue) {
		/* need enqueue, prepare cmd_obj and enqueue */
		cmdobj = (struct cmd_obj *)rtw_zmalloc(sizeof(*cmdobj));
		if (cmdobj == NULL) {
			res = _FAIL;
			rtw_mfree(param);
			goto exit;
		}
		init_h2fwcmd_w_parm_no_rsp(cmdobj, param, _DisConnect_CMD_);
		res = rtw_enqueue_cmd(cmdpriv, cmdobj);
	} else {
		/* no need to enqueue, do the cmd hdl directly and free cmd parameter */
		if (H2C_SUCCESS != disconnect_hdl(rtlpriv, (uint8_t *)param))
			res = _FAIL;
		rtw_mfree(param);
	}

exit:



	return res;
}

uint8_t rtw_setopmode_cmd(struct rtl_priv  *rtlpriv, NDIS_802_11_NETWORK_INFRASTRUCTURE networktype)
{
	struct	cmd_obj*	ph2c;
	struct	setopmode_parm* psetop;

	struct	cmd_priv   *pcmdpriv= &rtlpriv->cmdpriv;
	uint8_t	res=_SUCCESS;



	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(ph2c==NULL){
		res= false;
		goto exit;
	}
	psetop = (struct setopmode_parm*)rtw_zmalloc(sizeof(struct setopmode_parm));

	if(psetop==NULL){
		rtw_mfree(ph2c);
		res=false;
		goto exit;
	}

	init_h2fwcmd_w_parm_no_rsp(ph2c, psetop, _SetOpMode_CMD_);
	psetop->mode = (uint8_t)networktype;

	res = rtw_enqueue_cmd(pcmdpriv, ph2c);

exit:



	return res;
}

uint8_t rtw_setstakey_cmd(struct rtl_priv *rtlpriv, uint8_t *psta, uint8_t unicast_key)
{
	struct cmd_obj*			ph2c;
	struct set_stakey_parm	*psetstakey_para;
	struct cmd_priv 			*pcmdpriv=&rtlpriv->cmdpriv;
	struct set_stakey_rsp		*psetstakey_rsp = NULL;

	struct mlme_priv			*pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv 		*psecuritypriv = &rtlpriv->securitypriv;
	struct sta_info* 			sta = (struct sta_info* )psta;
	uint8_t	res=_SUCCESS;



	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if ( ph2c == NULL){
		res= _FAIL;
		goto exit;
	}

	psetstakey_para = (struct set_stakey_parm*)rtw_zmalloc(sizeof(struct set_stakey_parm));
	if(psetstakey_para==NULL){
		rtw_mfree(ph2c);
		res=_FAIL;
		goto exit;
	}

	psetstakey_rsp = (struct set_stakey_rsp*)rtw_zmalloc(sizeof(struct set_stakey_rsp));
	if(psetstakey_rsp == NULL){
		rtw_mfree(ph2c);
		rtw_mfree(psetstakey_para);
		res=_FAIL;
		goto exit;
	}

	init_h2fwcmd_w_parm_no_rsp(ph2c, psetstakey_para, _SetStaKey_CMD_);
	ph2c->rsp = (uint8_t *) psetstakey_rsp;
	ph2c->rspsz = sizeof(struct set_stakey_rsp);

	memcpy(psetstakey_para->addr, sta->hwaddr,ETH_ALEN);

	if(check_fwstate(pmlmepriv, WIFI_STATION_STATE)){
		psetstakey_para->algorithm =(unsigned char) psecuritypriv->dot11PrivacyAlgrthm;
	}else{
		GET_ENCRY_ALGO(psecuritypriv, sta, psetstakey_para->algorithm, false);
	}

	if (unicast_key == true) {
			memcpy(&psetstakey_para->key, &sta->dot118021x_UncstKey, 16);
        } else {
		memcpy(&psetstakey_para->key, &psecuritypriv->dot118021XGrpKey[psecuritypriv->dot118021XGrpKeyid].skey, 16);
        }

	//jeff: set this becasue at least sw key is ready
	rtlpriv->securitypriv.busetkipkey=true;

	res = rtw_enqueue_cmd(pcmdpriv, ph2c);

exit:



	return res;
}

uint8_t rtw_clearstakey_cmd(struct rtl_priv *rtlpriv, uint8_t *psta, uint8_t entry, uint8_t enqueue)
{
	struct cmd_obj*			ph2c;
	struct set_stakey_parm	*psetstakey_para;
	struct cmd_priv 			*pcmdpriv=&rtlpriv->cmdpriv;
	struct set_stakey_rsp		*psetstakey_rsp = NULL;
	struct mlme_priv			*pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv 		*psecuritypriv = &rtlpriv->securitypriv;
	struct sta_info* 			sta = (struct sta_info* )psta;
	uint8_t	res=_SUCCESS;



	if(!enqueue)
	{
		clear_cam_entry(rtlpriv, entry);
	}
	else
	{
		ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
		if ( ph2c == NULL){
			res= _FAIL;
			goto exit;
		}

		psetstakey_para = (struct set_stakey_parm*)rtw_zmalloc(sizeof(struct set_stakey_parm));
		if(psetstakey_para==NULL){
			rtw_mfree(ph2c);
			res=_FAIL;
			goto exit;
		}

		psetstakey_rsp = (struct set_stakey_rsp*)rtw_zmalloc(sizeof(struct set_stakey_rsp));
		if(psetstakey_rsp == NULL){
			rtw_mfree(ph2c);
			rtw_mfree(psetstakey_para);
			res=_FAIL;
			goto exit;
		}

		init_h2fwcmd_w_parm_no_rsp(ph2c, psetstakey_para, _SetStaKey_CMD_);
		ph2c->rsp = (uint8_t *) psetstakey_rsp;
		ph2c->rspsz = sizeof(struct set_stakey_rsp);

		memcpy(psetstakey_para->addr, sta->hwaddr, ETH_ALEN);

		psetstakey_para->algorithm = NO_ENCRYPTION;

		psetstakey_para->id = entry;

		res = rtw_enqueue_cmd(pcmdpriv, ph2c);

	}

exit:



	return res;
}

uint8_t rtw_addbareq_cmd(struct rtl_priv*rtlpriv, uint8_t tid, uint8_t *addr)
{
	struct cmd_priv		*pcmdpriv = &rtlpriv->cmdpriv;
	struct cmd_obj*		ph2c;
	struct addBaReq_parm	*paddbareq_parm;

	uint8_t	res=_SUCCESS;



	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(ph2c==NULL){
		res= _FAIL;
		goto exit;
	}

	paddbareq_parm = (struct addBaReq_parm*)rtw_zmalloc(sizeof(struct addBaReq_parm));
	if(paddbareq_parm==NULL){
		rtw_mfree(ph2c);
		res= _FAIL;
		goto exit;
	}

	paddbareq_parm->tid = tid;
	memcpy(paddbareq_parm->addr, addr, ETH_ALEN);

	init_h2fwcmd_w_parm_no_rsp(ph2c, paddbareq_parm, GEN_CMD_CODE(_AddBAReq));

	//DBG_871X("rtw_addbareq_cmd, tid=%d\n", tid);

	//rtw_enqueue_cmd(pcmdpriv, ph2c);
	res = rtw_enqueue_cmd(pcmdpriv, ph2c);

exit:



	return res;
}

uint8_t rtw_dynamic_chk_wk_cmd(struct rtl_priv*rtlpriv)
{
	struct cmd_obj*		ph2c;
	struct drvextra_cmd_parm  *pdrvextra_cmd_parm;
	struct cmd_priv	*pcmdpriv=&rtlpriv->cmdpriv;
	uint8_t	res=_SUCCESS;



	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(ph2c==NULL){
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm = (struct drvextra_cmd_parm*)rtw_zmalloc(sizeof(struct drvextra_cmd_parm));
	if(pdrvextra_cmd_parm==NULL){
		rtw_mfree(ph2c);
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm->ec_id = DYNAMIC_CHK_WK_CID;
	pdrvextra_cmd_parm->type_size = 0;
	pdrvextra_cmd_parm->pbuf = (uint8_t *)rtlpriv;

	init_h2fwcmd_w_parm_no_rsp(ph2c, pdrvextra_cmd_parm, GEN_CMD_CODE(_Set_Drv_Extra));


	//rtw_enqueue_cmd(pcmdpriv, ph2c);
	res = rtw_enqueue_cmd(pcmdpriv, ph2c);

exit:



	return res;

}

uint8_t rtw_set_ch_cmd(struct rtl_priv*rtlpriv, uint8_t ch, uint8_t bw, uint8_t ch_offset, uint8_t enqueue)
{
	struct cmd_obj *pcmdobj;
	struct set_ch_parm *set_ch_parm;
	struct cmd_priv *pcmdpriv = &rtlpriv->cmdpriv;

	uint8_t res=_SUCCESS;



	DBG_871X(FUNC_NDEV_FMT" ch:%u, bw:%u, ch_offset:%u\n",
		FUNC_NDEV_ARG(rtlpriv->ndev), ch, bw, ch_offset);

	/* check input parameter */

	/* prepare cmd parameter */
	set_ch_parm = (struct set_ch_parm *)rtw_zmalloc(sizeof(*set_ch_parm));
	if (set_ch_parm == NULL) {
		res= _FAIL;
		goto exit;
	}
	set_ch_parm->ch = ch;
	set_ch_parm->bw = bw;
	set_ch_parm->ch_offset = ch_offset;

	if (enqueue) {
		/* need enqueue, prepare cmd_obj and enqueue */
		pcmdobj = (struct cmd_obj*)rtw_zmalloc(sizeof(struct	cmd_obj));
		if(pcmdobj == NULL){
			rtw_mfree(set_ch_parm);
			res=_FAIL;
			goto exit;
		}

		init_h2fwcmd_w_parm_no_rsp(pcmdobj, set_ch_parm, GEN_CMD_CODE(_SetChannel));
		res = rtw_enqueue_cmd(pcmdpriv, pcmdobj);
	} else {
		/* no need to enqueue, do the cmd hdl directly and free cmd parameter */
		if( H2C_SUCCESS !=set_ch_hdl(rtlpriv, (uint8_t *)set_ch_parm) )
			res = _FAIL;

		rtw_mfree(set_ch_parm);
	}

	/* do something based on res... */

exit:

	DBG_871X(FUNC_NDEV_FMT" res:%u\n", FUNC_NDEV_ARG(rtlpriv->ndev), res);



	return res;
}

uint8_t rtw_set_csa_cmd(struct rtl_priv*rtlpriv, uint8_t new_ch_no)
{
	struct	cmd_obj*	pcmdobj;
	struct	SetChannelSwitch_param*setChannelSwitch_param;
	struct 	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct	cmd_priv   *pcmdpriv = &rtlpriv->cmdpriv;

	uint8_t	res=_SUCCESS;



	pcmdobj = (struct	cmd_obj*)rtw_zmalloc(sizeof(struct	cmd_obj));
	if(pcmdobj == NULL){
		res=_FAIL;
		goto exit;
	}

	setChannelSwitch_param = (struct SetChannelSwitch_param *)rtw_zmalloc(sizeof(struct	SetChannelSwitch_param));
	if(setChannelSwitch_param == NULL) {
		rtw_mfree(pcmdobj);
		res= _FAIL;
		goto exit;
	}

	setChannelSwitch_param->new_ch_no=new_ch_no;

	init_h2fwcmd_w_parm_no_rsp(pcmdobj, setChannelSwitch_param, GEN_CMD_CODE(_SetChannelSwitch));
	res = rtw_enqueue_cmd(pcmdpriv, pcmdobj);

exit:



	return res;
}

static void traffic_status_watchdog(struct rtl_priv *rtlpriv)
{
	uint8_t	bEnterPS;
	u16	BusyThreshold = 100;
	uint8_t	bBusyTraffic = false, bTxBusyTraffic = false, bRxBusyTraffic = false;
	uint8_t	bHigherBusyTraffic = false, bHigherBusyRxTraffic = false, bHigherBusyTxTraffic = false;
	struct mlme_priv		*pmlmepriv = &(rtlpriv->mlmepriv);

	//
	// Determine if our traffic is busy now
	//
	if((check_fwstate(pmlmepriv, _FW_LINKED)== true)
		/*&& !MgntInitAdapterInProgress(pMgntInfo)*/)
	{

		// if we raise bBusyTraffic in last watchdog, using lower threshold.
		if (pmlmepriv->LinkDetectInfo.bBusyTraffic)
			BusyThreshold = 75;
		if( pmlmepriv->LinkDetectInfo.NumRxOkInPeriod > BusyThreshold ||
			pmlmepriv->LinkDetectInfo.NumTxOkInPeriod > BusyThreshold )
		{
			bBusyTraffic = true;

			if (pmlmepriv->LinkDetectInfo.NumRxOkInPeriod > pmlmepriv->LinkDetectInfo.NumTxOkInPeriod)
				bRxBusyTraffic = true;
			else
				bTxBusyTraffic = true;
		}

		// Higher Tx/Rx data.
		if( pmlmepriv->LinkDetectInfo.NumRxOkInPeriod > 4000 ||
			pmlmepriv->LinkDetectInfo.NumTxOkInPeriod > 4000 )
		{
			bHigherBusyTraffic = true;

			if (pmlmepriv->LinkDetectInfo.NumRxOkInPeriod > pmlmepriv->LinkDetectInfo.NumTxOkInPeriod)
				bHigherBusyRxTraffic = true;
			else
				bHigherBusyTxTraffic = true;
		}

		{
		// check traffic for  powersaving.
		if( ((pmlmepriv->LinkDetectInfo.NumRxUnicastOkInPeriod + pmlmepriv->LinkDetectInfo.NumTxOkInPeriod) > 8 ) ||
			(pmlmepriv->LinkDetectInfo.NumRxUnicastOkInPeriod > 2) )
		{
			//DBG_871X("Tx = %d, Rx = %d \n",pmlmepriv->LinkDetectInfo.NumTxOkInPeriod,pmlmepriv->LinkDetectInfo.NumRxUnicastOkInPeriod);
			bEnterPS= false;
		}
		else
		{
			bEnterPS= true;
		}

		// LeisurePS only work in infra mode.
		if(bEnterPS)
		{
			LPS_Enter(rtlpriv);
		}
		else
		{
			LPS_Leave(rtlpriv);
		}
		}
	}
	else
	{
		LPS_Leave(rtlpriv);
	}

	pmlmepriv->LinkDetectInfo.NumRxOkInPeriod = 0;
	pmlmepriv->LinkDetectInfo.NumTxOkInPeriod = 0;
	pmlmepriv->LinkDetectInfo.NumRxUnicastOkInPeriod = 0;
	pmlmepriv->LinkDetectInfo.bBusyTraffic = bBusyTraffic;
	pmlmepriv->LinkDetectInfo.bTxBusyTraffic = bTxBusyTraffic;
	pmlmepriv->LinkDetectInfo.bRxBusyTraffic = bRxBusyTraffic;
	pmlmepriv->LinkDetectInfo.bHigherBusyTraffic = bHigherBusyTraffic;
	pmlmepriv->LinkDetectInfo.bHigherBusyRxTraffic = bHigherBusyRxTraffic;
	pmlmepriv->LinkDetectInfo.bHigherBusyTxTraffic = bHigherBusyTxTraffic;
}

void dynamic_chk_wk_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf, int sz);
void dynamic_chk_wk_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf, int sz)
{
	struct mlme_priv *pmlmepriv;

	rtlpriv = (struct rtl_priv *)pbuf;
	pmlmepriv = &(rtlpriv->mlmepriv);

	//if(check_fwstate(pmlmepriv, _FW_UNDER_LINKING|_FW_UNDER_SURVEY)==false)
	{
		linked_status_chk(rtlpriv);
		traffic_status_watchdog(rtlpriv);
	}

	rtlpriv->cfg->ops->dm_watchdog(rtlpriv);

	//check_hw_pbc(rtlpriv, pdrvextra_cmd->pbuf, pdrvextra_cmd->type_size);

}

void lps_ctrl_wk_hdl(struct rtl_priv *rtlpriv, uint8_t lps_ctrl_type);
void lps_ctrl_wk_hdl(struct rtl_priv *rtlpriv, uint8_t lps_ctrl_type)
{
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	uint8_t	mstatus;



	if((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true)
		|| (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true))
	{
		return;
	}

	switch(lps_ctrl_type)
	{
		case LPS_CTRL_SCAN:
			//DBG_871X("LPS_CTRL_SCAN \n");
			{
				if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
				{ //connect
					LPS_Leave(rtlpriv);
				}
			}
			break;
		case LPS_CTRL_JOINBSS:
			//DBG_871X("LPS_CTRL_JOINBSS \n");
			LPS_Leave(rtlpriv);
			break;
		case LPS_CTRL_CONNECT:
			//DBG_871X("LPS_CTRL_CONNECT \n");
			mstatus = 1;//connect
			// Reset LPS Setting
			rtlpriv->pwrctrlpriv.LpsIdleCount = 0;
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_H2C_FW_JOINBSSRPT, (uint8_t *)(&mstatus));
			break;
		case LPS_CTRL_DISCONNECT:
			//DBG_871X("LPS_CTRL_DISCONNECT \n");
			mstatus = 0;//disconnect
			{
				LPS_Leave(rtlpriv);
			}
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_H2C_FW_JOINBSSRPT, (uint8_t *)(&mstatus));
			break;
		case LPS_CTRL_SPECIAL_PACKET:
			//DBG_871X("LPS_CTRL_SPECIAL_PACKET \n");
			pwrpriv->DelayLPSLastTimeStamp = jiffies;
			{
				LPS_Leave(rtlpriv);
			}
			break;
		case LPS_CTRL_LEAVE:
			//DBG_871X("LPS_CTRL_LEAVE \n");
			{
				LPS_Leave(rtlpriv);
			}
			break;

		default:
			break;
	}


}

uint8_t rtw_lps_ctrl_wk_cmd(struct rtl_priv*rtlpriv, uint8_t lps_ctrl_type, uint8_t enqueue)
{
	struct cmd_obj	*ph2c;
	struct drvextra_cmd_parm	*pdrvextra_cmd_parm;
	struct cmd_priv	*pcmdpriv = &rtlpriv->cmdpriv;
	//struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;
	uint8_t	res = _SUCCESS;



	//if(!pwrctrlpriv->bLeisurePs)
	//	return res;

	if(enqueue)
	{
		ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
		if(ph2c==NULL){
			res= _FAIL;
			goto exit;
		}

		pdrvextra_cmd_parm = (struct drvextra_cmd_parm*)rtw_zmalloc(sizeof(struct drvextra_cmd_parm));
		if(pdrvextra_cmd_parm==NULL){
			rtw_mfree(ph2c);
			res= _FAIL;
			goto exit;
		}

		pdrvextra_cmd_parm->ec_id = LPS_CTRL_WK_CID;
		pdrvextra_cmd_parm->type_size = lps_ctrl_type;
		pdrvextra_cmd_parm->pbuf = NULL;

		init_h2fwcmd_w_parm_no_rsp(ph2c, pdrvextra_cmd_parm, GEN_CMD_CODE(_Set_Drv_Extra));

		res = rtw_enqueue_cmd(pcmdpriv, ph2c);
	}
	else
	{
		lps_ctrl_wk_hdl(rtlpriv, lps_ctrl_type);
	}

exit:



	return res;

}


#if (RATE_ADAPTIVE_SUPPORT==1)
void rpt_timer_setting_wk_hdl(struct rtl_priv *rtlpriv, u16 minRptTime)
{
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_RPT_TIMER_SETTING, (uint8_t *)(&minRptTime));
}

uint8_t rtw_rpt_timer_cfg_cmd(struct rtl_priv*rtlpriv, u16 minRptTime)
{
	struct cmd_obj		*ph2c;
	struct drvextra_cmd_parm	*pdrvextra_cmd_parm;
	struct cmd_priv	*pcmdpriv = &rtlpriv->cmdpriv;

	uint8_t	res = _SUCCESS;


	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(ph2c==NULL){
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm = (struct drvextra_cmd_parm*)rtw_zmalloc(sizeof(struct drvextra_cmd_parm));
	if(pdrvextra_cmd_parm==NULL){
		rtw_mfree(ph2c);
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm->ec_id = RTP_TIMER_CFG_WK_CID;
	pdrvextra_cmd_parm->type_size = minRptTime;
	pdrvextra_cmd_parm->pbuf = NULL;
	init_h2fwcmd_w_parm_no_rsp(ph2c, pdrvextra_cmd_parm, GEN_CMD_CODE(_Set_Drv_Extra));
	res = rtw_enqueue_cmd(pcmdpriv, ph2c);
exit:



	return res;

}

#endif


void power_saving_wk_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf, int sz);
void power_saving_wk_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf, int sz)
{
	 rtw_ps_processor(rtlpriv);
}

uint8_t rtw_ps_cmd(struct rtl_priv*rtlpriv)
{
	struct cmd_obj		*ppscmd;
	struct drvextra_cmd_parm	*pdrvextra_cmd_parm;
	struct cmd_priv	*pcmdpriv = &rtlpriv->cmdpriv;

	uint8_t	res = _SUCCESS;



	ppscmd = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(ppscmd==NULL){
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm = (struct drvextra_cmd_parm*)rtw_zmalloc(sizeof(struct drvextra_cmd_parm));
	if(pdrvextra_cmd_parm==NULL){
		rtw_mfree(ppscmd);
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm->ec_id = POWER_SAVING_CTRL_WK_CID;
	pdrvextra_cmd_parm->pbuf = NULL;
	init_h2fwcmd_w_parm_no_rsp(ppscmd, pdrvextra_cmd_parm, GEN_CMD_CODE(_Set_Drv_Extra));

	res = rtw_enqueue_cmd(pcmdpriv, ppscmd);

exit:



	return res;

}

#ifdef CONFIG_AP_MODE

static void rtw_chk_hi_queue_hdl(struct rtl_priv *rtlpriv)
{
	int cnt=0;
	struct sta_info *psta_bmc;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

	psta_bmc = rtw_get_bcmc_stainfo(rtlpriv);
	if(!psta_bmc)
		return;

	if(psta_bmc->sleepq_len==0)
	{
		uint8_t val = 0;

		//while((rtw_read32(rtlpriv, 0x414)&0x00ffff00)!=0)
		//while((rtw_read32(rtlpriv, 0x414)&0x0000ff00)!=0)

		rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_CHK_HI_QUEUE_EMPTY, &val);

		while(false == val)
		{
			msleep(100);

			cnt++;

			if(cnt>10)
				break;

			rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_CHK_HI_QUEUE_EMPTY, &val);
		}

		if(cnt<=10)
		{
			pstapriv->tim_bitmap &= ~BIT(0);
			pstapriv->sta_dz_bitmap &= ~BIT(0);

			update_beacon(rtlpriv, _TIM_IE_, NULL, false);
		}
		else //re check again
		{
			rtw_chk_hi_queue_cmd(rtlpriv);
		}

	}

}

uint8_t rtw_chk_hi_queue_cmd(struct rtl_priv*rtlpriv)
{
	struct cmd_obj	*ph2c;
	struct drvextra_cmd_parm	*pdrvextra_cmd_parm;
	struct cmd_priv	*pcmdpriv = &rtlpriv->cmdpriv;
	uint8_t	res = _SUCCESS;

	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if(ph2c==NULL){
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm = (struct drvextra_cmd_parm*)rtw_zmalloc(sizeof(struct drvextra_cmd_parm));
	if(pdrvextra_cmd_parm==NULL){
		rtw_mfree(ph2c);
		res= _FAIL;
		goto exit;
	}

	pdrvextra_cmd_parm->ec_id = CHECK_HIQ_WK_CID;
	pdrvextra_cmd_parm->type_size = 0;
	pdrvextra_cmd_parm->pbuf = NULL;

	init_h2fwcmd_w_parm_no_rsp(ph2c, pdrvextra_cmd_parm, GEN_CMD_CODE(_Set_Drv_Extra));

	res = rtw_enqueue_cmd(pcmdpriv, ph2c);

exit:

	return res;

}
#endif

uint8_t rtw_drvextra_cmd_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf)
{
	struct drvextra_cmd_parm *pdrvextra_cmd;

	if(!pbuf)
		return H2C_PARAMETERS_ERROR;

	pdrvextra_cmd = (struct drvextra_cmd_parm*)pbuf;

	switch(pdrvextra_cmd->ec_id)
	{
		case DYNAMIC_CHK_WK_CID:
			dynamic_chk_wk_hdl(rtlpriv, pdrvextra_cmd->pbuf, pdrvextra_cmd->type_size);
			break;
		case POWER_SAVING_CTRL_WK_CID:
			power_saving_wk_hdl(rtlpriv, pdrvextra_cmd->pbuf, pdrvextra_cmd->type_size);
			break;
		case LPS_CTRL_WK_CID:
			lps_ctrl_wk_hdl(rtlpriv, (uint8_t)pdrvextra_cmd->type_size);
			break;
#if (RATE_ADAPTIVE_SUPPORT==1)
		case RTP_TIMER_CFG_WK_CID:
			rpt_timer_setting_wk_hdl(rtlpriv, pdrvextra_cmd->type_size);
			break;
#endif
#ifdef CONFIG_AP_MODE
		case CHECK_HIQ_WK_CID:
			rtw_chk_hi_queue_hdl(rtlpriv);
			break;
#endif //CONFIG_AP_MODE
		default:
			break;
	}

	if (pdrvextra_cmd->pbuf && pdrvextra_cmd->type_size>0)
	{
		/* ULLI check usage of pdrvextra_cmd->type_size */
		rtw_mfree(pdrvextra_cmd->pbuf);
	}

	return H2C_SUCCESS;
}

void rtw_survey_cmd_callback(struct rtl_priv*	rtlpriv ,  struct cmd_obj *pcmd)
{
	struct 	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;



	if(pcmd->res == H2C_DROPPED)
	{
		//TODO: cancel timer and do timeout handler directly...
		//need to make timeout handlerOS independent
		_set_timer(&pmlmepriv->scan_to_timer, 1);
	}
	else if (pcmd->res != H2C_SUCCESS) {
		_set_timer(&pmlmepriv->scan_to_timer, 1);
	}

	// free cmd
	rtw_free_cmd_obj(pcmd);


}
void rtw_disassoc_cmd_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd)
{
	struct 	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;



	if (pcmd->res != H2C_SUCCESS)
	{
		spin_lock_bh(&pmlmepriv->lock);
		set_fwstate(pmlmepriv, _FW_LINKED);
		spin_unlock_bh(&pmlmepriv->lock);

		goto exit;
	}

	// free cmd
	rtw_free_cmd_obj(pcmd);

exit:
	;

}


void rtw_joinbss_cmd_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd)
{
	struct 	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;



	if(pcmd->res == H2C_DROPPED)
	{
		//TODO: cancel timer and do timeout handler directly...
		//need to make timeout handlerOS independent
		_set_timer(&pmlmepriv->assoc_timer, 1);
	}
	else if(pcmd->res != H2C_SUCCESS)
	{
		_set_timer(&pmlmepriv->assoc_timer, 1);
	}

	rtw_free_cmd_obj(pcmd);


}

void rtw_createbss_cmd_callback(struct rtl_priv *rtlpriv, struct cmd_obj *pcmd)
{
	struct sta_info *psta = NULL;
	struct wlan_network *pwlan = NULL;
	struct 	mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	WLAN_BSSID_EX *pnetwork = (WLAN_BSSID_EX *)pcmd->parmbuf;
	struct wlan_network *tgt_network = &(pmlmepriv->cur_network);



	if((pcmd->res != H2C_SUCCESS))
	{
		_set_timer(&pmlmepriv->assoc_timer, 1 );
	}

	del_timer_sync(&pmlmepriv->assoc_timer);

#ifdef CONFIG_FW_MLMLE
       //endian_convert
	pnetwork->Length = le32_to_cpu(pnetwork->Length);
  	pnetwork->Ssid.SsidLength = le32_to_cpu(pnetwork->Ssid.SsidLength);
	pnetwork->Privacy =le32_to_cpu(pnetwork->Privacy);
	pnetwork->Rssi = le32_to_cpu(pnetwork->Rssi);
	pnetwork->NetworkTypeInUse =le32_to_cpu(pnetwork->NetworkTypeInUse);
	pnetwork->Configuration.ATIMWindow = le32_to_cpu(pnetwork->Configuration.ATIMWindow);
	//pnetwork->Configuration.BeaconPeriod = le32_to_cpu(pnetwork->Configuration.BeaconPeriod);
	pnetwork->Configuration.DSConfig =le32_to_cpu(pnetwork->Configuration.DSConfig);
	pnetwork->Configuration.FHConfig.DwellTime=le32_to_cpu(pnetwork->Configuration.FHConfig.DwellTime);
	pnetwork->Configuration.FHConfig.HopPattern=le32_to_cpu(pnetwork->Configuration.FHConfig.HopPattern);
	pnetwork->Configuration.FHConfig.HopSet=le32_to_cpu(pnetwork->Configuration.FHConfig.HopSet);
	pnetwork->Configuration.FHConfig.Length=le32_to_cpu(pnetwork->Configuration.FHConfig.Length);
	pnetwork->Configuration.Length = le32_to_cpu(pnetwork->Configuration.Length);
	pnetwork->InfrastructureMode = le32_to_cpu(pnetwork->InfrastructureMode);
	pnetwork->IELength = le32_to_cpu(pnetwork->IELength);
#endif

	spin_lock_bh(&pmlmepriv->lock);


	if(check_fwstate(pmlmepriv, WIFI_AP_STATE) )
	{
		psta = rtw_get_stainfo(&rtlpriv->stapriv, pnetwork->MacAddress);
		if(!psta)
		{
		psta = rtw_alloc_stainfo(&rtlpriv->stapriv, pnetwork->MacAddress);
		if (psta == NULL)
		{
			goto createbss_cmd_fail ;
		}
		}

		rtw_indicate_connect( rtlpriv);
	}
	else
	{
		pwlan = _rtw_alloc_network(pmlmepriv);
		spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
		if ( pwlan == NULL)
		{
			pwlan = rtw_get_oldest_wlan_network(&pmlmepriv->scanned_queue);
			if( pwlan == NULL)
			{
				spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
				goto createbss_cmd_fail;
			}
			pwlan->last_scanned = jiffies;
		}
		else
		{
			list_add_tail(&(pwlan->list), &pmlmepriv->scanned_queue.list);
		}

		pnetwork->Length = get_WLAN_BSSID_EX_sz(pnetwork);
		memcpy(&(pwlan->network), pnetwork, pnetwork->Length);
		//pwlan->fixed = true;

		//list_add_tail(&(pwlan->list), &pmlmepriv->scanned_queue.queue);

		// copy pdev_network information to 	pmlmepriv->cur_network
		memcpy(&tgt_network->network, pnetwork, (get_WLAN_BSSID_EX_sz(pnetwork)));

		// reset DSConfig
		//tgt_network->network.Configuration.DSConfig = (uint32_t)rtw_ch2freq(pnetwork->Configuration.DSConfig);

		_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);

		spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
		// we will set _FW_LINKED when there is one more sat to join us (rtw_stassoc_event_callback)

	}

createbss_cmd_fail:

	spin_unlock_bh(&pmlmepriv->lock);

	rtw_free_cmd_obj(pcmd);



}



void rtw_setstaKey_cmdrsp_callback(struct rtl_priv*	rtlpriv ,  struct cmd_obj *pcmd)
{

	struct sta_priv * pstapriv = &rtlpriv->stapriv;
	struct set_stakey_rsp* psetstakey_rsp = (struct set_stakey_rsp*) (pcmd->rsp);
	struct sta_info*	psta = rtw_get_stainfo(pstapriv, psetstakey_rsp->addr);



	if(psta==NULL)
	{
		goto exit;
	}

	//psta->aid = psta->mac_id = psetstakey_rsp->keyid; //CAM_ID(CAM_ENTRY)

exit:

	rtw_free_cmd_obj(pcmd);



}
void rtw_setassocsta_cmdrsp_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd)
{
	struct sta_priv * pstapriv = &rtlpriv->stapriv;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct set_assocsta_parm* passocsta_parm = (struct set_assocsta_parm*)(pcmd->parmbuf);
	struct set_assocsta_rsp* passocsta_rsp = (struct set_assocsta_rsp*) (pcmd->rsp);
	struct sta_info*	psta = rtw_get_stainfo(pstapriv, passocsta_parm->addr);



	if(psta==NULL)
	{
		goto exit;
	}

	psta->aid = psta->mac_id = passocsta_rsp->cam_id;

	spin_lock_bh(&pmlmepriv->lock);

	if ((check_fwstate(pmlmepriv, WIFI_MP_STATE) == true) && (check_fwstate(pmlmepriv, _FW_UNDER_LINKING) == true))
		_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);

	set_fwstate(pmlmepriv, _FW_LINKED);
	spin_unlock_bh(&pmlmepriv->lock);

exit:
	rtw_free_cmd_obj(pcmd);


}

void rtw_getrttbl_cmd_cmdrsp_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd);
void rtw_getrttbl_cmd_cmdrsp_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd)
{


	rtw_free_cmd_obj(pcmd);



}

