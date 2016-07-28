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
#define _RTW_IOCTL_SET_C_

#include <drv_types.h>
#include <rtw_ap.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

#define _drv_always_		1
#undef DBG_871X_LEVEL
static inline void DBG_871X_LEVEL(const int level, const char *fmt, ...)
{
}

extern void indicate_wx_scan_complete_event(struct rtl_priv *rtlpriv);

#define IS_MAC_ADDRESS_BROADCAST(addr) \
( \
	( (addr[0] == 0xff) && (addr[1] == 0xff) && \
		(addr[2] == 0xff) && (addr[3] == 0xff) && \
		(addr[4] == 0xff) && (addr[5] == 0xff) )  ? true : false \
)

uint8_t rtw_validate_ssid(NDIS_802_11_SSID *ssid)
{
	uint8_t	 i;
	uint8_t	ret=true;



	if (ssid->SsidLength > 32) {
		ret= false;
		goto exit;
	}

	for(i = 0; i < ssid->SsidLength; i++)
	{
		//wifi, printable ascii code must be supported
		if(!( (ssid->Ssid[i] >= 0x20) && (ssid->Ssid[i] <= 0x7e) )){
			ret= false;
			break;
		}
	}

exit:



	return ret;
}

static inline struct list_head *get_next(struct list_head	*list)
{
	return list->next;
}

uint8_t rtw_do_join(struct rtl_priv * rtlpriv);
uint8_t rtw_do_join(struct rtl_priv * rtlpriv)
{
	struct list_head	*plist, *phead;
	uint8_t * pibss = NULL;
	struct	mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	struct __queue	*queue	= &(pmlmepriv->scanned_queue);
	uint8_t ret=_SUCCESS;



	spin_lock_bh(&(pmlmepriv->scanned_queue.lock));
	phead = get_list_head(queue);
	plist = get_next(phead);

	pmlmepriv->cur_network.join_res = -2;

	set_fwstate(pmlmepriv, _FW_UNDER_LINKING);

	pmlmepriv->pscanned = plist;

	pmlmepriv->to_join = true;

	if(list_empty(&queue->list)) {
		spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
		_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);

		//when set_ssid/set_bssid for rtw_do_join(), but scanning queue is empty
		//we try to issue sitesurvey firstly

		if (pmlmepriv->LinkDetectInfo.bBusyTraffic ==false)
		{
			// submit site_survey_cmd
			if(_SUCCESS!=(ret=rtw_sitesurvey_cmd(rtlpriv, &pmlmepriv->assoc_ssid, 1, NULL, 0)) ) {
				pmlmepriv->to_join = false;
			}
		}
		else
		{
			pmlmepriv->to_join = false;
			ret = _FAIL;
		}

		goto exit;
	}
	else
	{
		int select_ret;
		spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));
		if((select_ret=rtw_select_and_join_from_scanned_queue(pmlmepriv))==_SUCCESS)
		{
			pmlmepriv->to_join = false;
			_set_timer(&pmlmepriv->assoc_timer, MAX_JOIN_TIMEOUT);
		}
		else
		{
			if(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE)==true)
			{
				// submit createbss_cmd to change to a ADHOC_MASTER

 				//pmlmepriv->lock has been acquired by caller...
				WLAN_BSSID_EX    *pdev_network = &(rtlpriv->registrypriv.dev_network);

				pmlmepriv->fw_state = WIFI_ADHOC_MASTER_STATE;

				pibss = rtlpriv->registrypriv.dev_network.MacAddress;

				memset(&pdev_network->Ssid, 0, sizeof(NDIS_802_11_SSID));
				memcpy(&pdev_network->Ssid, &pmlmepriv->assoc_ssid, sizeof(NDIS_802_11_SSID));

				rtw_update_registrypriv_dev_network(rtlpriv);

				rtw_generate_random_ibss(pibss);

				if(rtw_createbss_cmd(rtlpriv)!=_SUCCESS)
				{
					ret =  false;
					goto exit;
				}

			     	pmlmepriv->to_join = false;

			}
			else
			{
				// can't associate ; reset under-linking
				_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);

#if 0
				if((check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true))
				{
					if(_rtw_memcmp(pmlmepriv->cur_network.network.Ssid.Ssid, pmlmepriv->assoc_ssid.Ssid, pmlmepriv->assoc_ssid.SsidLength))
					{
						// for funk to do roaming
						// funk will reconnect, but funk will not sitesurvey before reconnect
						if(pmlmepriv->sitesurveyctrl.traffic_busy==false)
							rtw_sitesurvey_cmd(rtlpriv, &pmlmepriv->assoc_ssid, 1, NULL, 0);
					}

				}
#endif

				//when set_ssid/set_bssid for rtw_do_join(), but there are no desired bss in scanning queue
				//we try to issue sitesurvey firstly
				if(pmlmepriv->LinkDetectInfo.bBusyTraffic==false)
				{
					//DBG_871X("rtw_do_join() when   no desired bss in scanning queue \n");
					if( _SUCCESS!=(ret=rtw_sitesurvey_cmd(rtlpriv, &pmlmepriv->assoc_ssid, 1, NULL, 0)) ){
						pmlmepriv->to_join = false;
					}
				}
				else
				{
					ret = _FAIL;
					pmlmepriv->to_join = false;
				}
			}

		}

	}

exit:



	return ret;
}


uint8_t rtw_set_802_11_bssid(struct rtl_priv* rtlpriv, uint8_t *bssid)
{
	uint8_t status=_SUCCESS;

	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;



	DBG_871X_LEVEL(_drv_always_, "set bssid:%pM\n", bssid);

	if ((bssid[0]==0x00 && bssid[1]==0x00 && bssid[2]==0x00 && bssid[3]==0x00 && bssid[4]==0x00 &&bssid[5]==0x00) ||
	    (bssid[0]==0xFF && bssid[1]==0xFF && bssid[2]==0xFF && bssid[3]==0xFF && bssid[4]==0xFF &&bssid[5]==0xFF))
	{
		status = _FAIL;
		goto exit;
	}

	spin_lock_bh(&pmlmepriv->lock);


	DBG_871X("Set BSSID under fw_state=0x%08x\n", get_fwstate(pmlmepriv));
	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) == true) {
		goto handle_tkip_countermeasure;
	} else if (check_fwstate(pmlmepriv, _FW_UNDER_LINKING) == true) {
		goto release_mlme_lock;
	}

	if (check_fwstate(pmlmepriv, _FW_LINKED|WIFI_ADHOC_MASTER_STATE) == true)
	{

		if (_rtw_memcmp(&pmlmepriv->cur_network.network.MacAddress, bssid, ETH_ALEN) == true)
		{
			if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == false)
				goto release_mlme_lock;//it means driver is in WIFI_ADHOC_MASTER_STATE, we needn't create bss again.
		} else {
			rtw_disassoc_cmd(rtlpriv, 0, true);

			if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
				rtw_indicate_disconnect(rtlpriv);

			rtw_free_assoc_resources(rtlpriv, 1);

			if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true)) {
				_clr_fwstate_(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
				set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
			}
		}
	}

handle_tkip_countermeasure:
	//should we add something here...?
	if((status=rtw_handle_tkip_countermeasure(rtlpriv)) == _FAIL)
		goto release_mlme_lock;

	memcpy(&pmlmepriv->assoc_bssid, bssid, ETH_ALEN);
	pmlmepriv->assoc_by_bssid=true;

	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) == true) {
		pmlmepriv->to_join = true;
	}
	else {
		status = rtw_do_join(rtlpriv);
	}

release_mlme_lock:
	spin_unlock_bh(&pmlmepriv->lock);

exit:



	return status;
}

uint8_t rtw_set_802_11_ssid(struct rtl_priv* rtlpriv, NDIS_802_11_SSID *ssid)
{
	uint8_t status = _SUCCESS;
	uint32_t	 cur_time = 0;

	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct wlan_network *pnetwork = &pmlmepriv->cur_network;



	DBG_871X_LEVEL(_drv_always_, "set ssid [%s] fw_state=0x%08x\n",
		       	ssid->Ssid, get_fwstate(pmlmepriv));

	if(rtlpriv->hw_init_completed==false){
		status = _FAIL;
		goto exit;
	}

	spin_lock_bh(&pmlmepriv->lock);

	DBG_871X("Set SSID under fw_state=0x%08x\n", get_fwstate(pmlmepriv));
	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) == true) {
		goto handle_tkip_countermeasure;
	} else if (check_fwstate(pmlmepriv, _FW_UNDER_LINKING) == true) {
		goto release_mlme_lock;
	}

	if (check_fwstate(pmlmepriv, _FW_LINKED|WIFI_ADHOC_MASTER_STATE) == true)
	{

		if ((pmlmepriv->assoc_ssid.SsidLength == ssid->SsidLength) &&
		    (_rtw_memcmp(&pmlmepriv->assoc_ssid.Ssid, ssid->Ssid, ssid->SsidLength) == true))
		{
			if((check_fwstate(pmlmepriv, WIFI_STATION_STATE) == false))
			{
				if(rtw_is_same_ibss(rtlpriv, pnetwork) == false)
				{
					//if in WIFI_ADHOC_MASTER_STATE | WIFI_ADHOC_STATE, create bss or rejoin again
					rtw_disassoc_cmd(rtlpriv, 0, true);

					if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
						rtw_indicate_disconnect(rtlpriv);

					rtw_free_assoc_resources(rtlpriv, 1);

					if (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true) {
						_clr_fwstate_(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
						set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
					}
				}
				else
				{
					goto release_mlme_lock;//it means driver is in WIFI_ADHOC_MASTER_STATE, we needn't create bss again.
				}
			}
			else {
				rtw_lps_ctrl_wk_cmd(rtlpriv, LPS_CTRL_JOINBSS, 1);
			}
		}
		else
		{
			rtw_disassoc_cmd(rtlpriv, 0, true);

			if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
				rtw_indicate_disconnect(rtlpriv);

			rtw_free_assoc_resources(rtlpriv, 1);

			if (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true) {
				_clr_fwstate_(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
				set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
			}
		}
	}

handle_tkip_countermeasure:

	if((status=rtw_handle_tkip_countermeasure(rtlpriv)) == _FAIL)
		goto release_mlme_lock;

	#ifdef CONFIG_VALIDATE_SSID
	if (rtw_validate_ssid(ssid) == false) {
		status = _FAIL;
		goto release_mlme_lock;
	}
	#endif

	memcpy(&pmlmepriv->assoc_ssid, ssid, sizeof(NDIS_802_11_SSID));
	pmlmepriv->assoc_by_bssid=false;

	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) == true) {
		pmlmepriv->to_join = true;
	}
	else {
		status = rtw_do_join(rtlpriv);
	}

release_mlme_lock:
	spin_unlock_bh(&pmlmepriv->lock);

exit:



	return status;

}

uint8_t rtw_set_802_11_infrastructure_mode(struct rtl_priv* rtlpriv,
	NDIS_802_11_NETWORK_INFRASTRUCTURE networktype)
{
	struct	mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct	wlan_network	*cur_network = &pmlmepriv->cur_network;
	NDIS_802_11_NETWORK_INFRASTRUCTURE* pold_state = &(cur_network->network.InfrastructureMode);



	if(*pold_state != networktype)
	{
		spin_lock_bh(&pmlmepriv->lock);

		//DBG_871X("change mode, old_mode=%d, new_mode=%d, fw_state=0x%x\n", *pold_state, networktype, get_fwstate(pmlmepriv));

		if(*pold_state==Ndis802_11APMode)
		{
			//change to other mode from Ndis802_11APMode
			cur_network->join_res = -1;

			stop_ap_mode(rtlpriv);
		}

		if((check_fwstate(pmlmepriv, _FW_LINKED)== true) ||(*pold_state==Ndis802_11IBSS))
			rtw_disassoc_cmd(rtlpriv, 0, true);

		if((check_fwstate(pmlmepriv, _FW_LINKED)== true) ||
			(check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)== true) )
			rtw_free_assoc_resources(rtlpriv, 1);

		if((*pold_state == Ndis802_11Infrastructure) ||(*pold_state == Ndis802_11IBSS))
	       {
			if(check_fwstate(pmlmepriv, _FW_LINKED) == true)
			{
				rtw_indicate_disconnect(rtlpriv); //will clr Linked_state; before this function, we must have chked whether  issue dis-assoc_cmd or not
			}
	       }

		*pold_state = networktype;

		_clr_fwstate_(pmlmepriv, ~WIFI_NULL_STATE);

		switch(networktype)
		{
			case Ndis802_11IBSS:
				set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
				break;

			case Ndis802_11Infrastructure:
				set_fwstate(pmlmepriv, WIFI_STATION_STATE);
				break;

			case Ndis802_11APMode:
				set_fwstate(pmlmepriv, WIFI_AP_STATE);
				start_ap_mode(rtlpriv);
				//rtw_indicate_connect(rtlpriv);

				break;

			case Ndis802_11AutoUnknown:
			case Ndis802_11InfrastructureMax:
				break;
		}

		//SecClearAllKeys(rtlpriv);

		//RT_TRACE(COMP_OID_SET, DBG_LOUD, ("set_infrastructure: fw_state:%x after changing mode\n",
		//									get_fwstate(pmlmepriv) ));

		spin_unlock_bh(&pmlmepriv->lock);
	}



	return true;
}


uint8_t rtw_set_802_11_disassociate(struct rtl_priv *rtlpriv)
{
	struct mlme_priv * pmlmepriv = &rtlpriv->mlmepriv;



	spin_lock_bh(&pmlmepriv->lock);

	if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
	{
		rtw_disassoc_cmd(rtlpriv, 0, true);
		rtw_indicate_disconnect(rtlpriv);
		rtw_free_assoc_resources(rtlpriv, 1);
		rtw_pwr_wakeup(rtlpriv);
	}

	spin_unlock_bh(&pmlmepriv->lock);



	return true;
}

uint8_t rtw_set_802_11_bssid_list_scan(struct rtl_priv* rtlpriv, NDIS_802_11_SSID *pssid, int ssid_max_num)
{
	struct	mlme_priv		*pmlmepriv= &rtlpriv->mlmepriv;
	uint8_t	res=true;



	if (rtlpriv == NULL) {
		res=false;
		goto exit;
	}
	if (rtlpriv->hw_init_completed==false){
		res = false;
		goto exit;
	}

	if ((check_fwstate(pmlmepriv, _FW_UNDER_SURVEY|_FW_UNDER_LINKING) == true) ||
		(pmlmepriv->LinkDetectInfo.bBusyTraffic == true))
	{
		// Scan or linking is in progress, do nothing.
		res = true;

		if(check_fwstate(pmlmepriv, (_FW_UNDER_SURVEY|_FW_UNDER_LINKING))== true){
			;
		} else {
			;
		}
	} else {
		spin_lock_bh(&pmlmepriv->lock);

		res = rtw_sitesurvey_cmd(rtlpriv, pssid, ssid_max_num, NULL, 0);

		spin_unlock_bh(&pmlmepriv->lock);
	}
exit:



	return res;
}

uint8_t rtw_set_802_11_authentication_mode(struct rtl_priv* rtlpriv, NDIS_802_11_AUTHENTICATION_MODE authmode)
{
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	int res;
	uint8_t ret;



	psecuritypriv->ndisauthtype=authmode;

	if(psecuritypriv->ndisauthtype>3)
		psecuritypriv->dot11AuthAlgrthm=dot11AuthAlgrthm_8021X;

	res=rtw_set_auth(rtlpriv,psecuritypriv);

	if(res==_SUCCESS)
		ret=true;
	else
		ret=false;



	return ret;
}

uint8_t rtw_set_802_11_add_wep(struct rtl_priv* rtlpriv, NDIS_802_11_WEP *wep){

	uint8_t		bdefaultkey;
	uint8_t		btransmitkey;
	int		keyid,res;
	struct security_priv* psecuritypriv=&(rtlpriv->securitypriv);
	uint8_t		ret=_SUCCESS;



	bdefaultkey=(wep->KeyIndex & 0x40000000) > 0 ? false : true;   //for ???
	btransmitkey= (wep->KeyIndex & 0x80000000) > 0 ? true  : false;	//for ???
	keyid=wep->KeyIndex & 0x3fffffff;

	if(keyid>4)
	{
		ret=false;
		goto exit;
	}

	switch(wep->KeyLength)
	{
		case 5:
			psecuritypriv->dot11PrivacyAlgrthm=WEP40_ENCRYPTION;
			break;
		case 13:
			psecuritypriv->dot11PrivacyAlgrthm=WEP104_ENCRYPTION;
			break;
		default:
			psecuritypriv->dot11PrivacyAlgrthm=NO_ENCRYPTION;
			break;
	}

	memcpy(&(psecuritypriv->dot11DefKey[keyid].skey[0]),&(wep->KeyMaterial),wep->KeyLength);

	psecuritypriv->dot11DefKeylen[keyid]=wep->KeyLength;

	psecuritypriv->dot11PrivacyKeyIndex=keyid;

	res=rtw_set_key(rtlpriv,psecuritypriv, keyid, 1);

	if(res==_FAIL)
		ret= false;
exit:



	return ret;

}

uint8_t rtw_set_802_11_remove_wep(struct rtl_priv* rtlpriv, uint32_t	 keyindex){

	uint8_t ret=_SUCCESS;



	if (keyindex >= 0x80000000 || rtlpriv == NULL){

		ret=false;
		goto exit;

	}
	else
	{
		int res;
		struct security_priv* psecuritypriv=&(rtlpriv->securitypriv);
		if( keyindex < 4 ){

			memset(&psecuritypriv->dot11DefKey[keyindex], 0, 16);

			res=rtw_set_key(rtlpriv,psecuritypriv,keyindex, 0);

			psecuritypriv->dot11DefKeylen[keyindex]=0;

			if(res==_FAIL)
				ret=_FAIL;

		}
		else
		{
			ret=_FAIL;
		}

	}

exit:



	return ret;

}

uint8_t rtw_set_802_11_add_key(struct rtl_priv* rtlpriv, NDIS_802_11_KEY *key){

	uint	encryptionalgo;
	uint8_t * pbssid;
	struct sta_info *stainfo;
	uint8_t	bgroup = false;
	uint8_t	bgrouptkey = false;//can be remove later
	uint8_t	ret=_SUCCESS;



	if (((key->KeyIndex & 0x80000000) == 0) && ((key->KeyIndex & 0x40000000) > 0)){

		// It is invalid to clear bit 31 and set bit 30. If the miniport driver encounters this combination,
		// it must fail the request and return NDIS_STATUS_INVALID_DATA.
		ret= _FAIL;
		goto exit;
	}

	if(key->KeyIndex & 0x40000000)
	{
		// Pairwise key

		pbssid=get_bssid(&rtlpriv->mlmepriv);
		stainfo=rtw_get_stainfo(&rtlpriv->stapriv, pbssid);

		if((stainfo!=NULL)&&(rtlpriv->securitypriv.dot11AuthAlgrthm==dot11AuthAlgrthm_8021X)){
			encryptionalgo=stainfo->dot118021XPrivacy;
		}
		else{
			encryptionalgo=rtlpriv->securitypriv.dot11PrivacyAlgrthm;
		}

		if((stainfo!=NULL)){
			;
		}

		if(key->KeyIndex & 0x000000FF){
			// The key index is specified in the lower 8 bits by values of zero to 255.
			// The key index should be set to zero for a Pairwise key, and the driver should fail with
			// NDIS_STATUS_INVALID_DATA if the lower 8 bits is not zero
			ret= _FAIL;
			goto exit;
		}

		// check BSSID
		if (IS_MAC_ADDRESS_BROADCAST(key->BSSID) == true){
			ret= false;
			goto exit;
		}

		// Check key length for TKIP.
		//if(encryptionAlgorithm == RT_ENC_TKIP_ENCRYPTION && key->KeyLength != 32)
		if((encryptionalgo== TKIP_ENCRYPTION)&& (key->KeyLength != 32)){
			ret=_FAIL;
			goto exit;

		}

		// Check key length for AES.
		if((encryptionalgo== AESCCMP_ENCRYPTION)&& (key->KeyLength != 16)) {
			// For our supplicant, EAPPkt9x.vxd, cannot differentiate TKIP and AES case.
			if(key->KeyLength == 32) {
				key->KeyLength = 16;
			} else {
				ret= _FAIL;
				goto exit;
			}
		}

		// Check key length for WEP. For NDTEST, 2005.01.27, by rcnjko.
		if(	(encryptionalgo== WEP40_ENCRYPTION|| encryptionalgo== WEP104_ENCRYPTION) && (key->KeyLength != 5 || key->KeyLength != 13)) {
			ret=_FAIL;
			goto exit;
		}

		bgroup = false;

	}
	else
	{

		// when add wep key through add key and didn't assigned encryption type before
		if((rtlpriv->securitypriv.ndisauthtype<=3)&&(rtlpriv->securitypriv.dot118021XGrpPrivacy==0))
		{
			switch(key->KeyLength)
			{
				case 5:
					rtlpriv->securitypriv.dot11PrivacyAlgrthm=WEP40_ENCRYPTION;
					break;
				case 13:
					rtlpriv->securitypriv.dot11PrivacyAlgrthm=WEP104_ENCRYPTION;
					break;
				default:
					rtlpriv->securitypriv.dot11PrivacyAlgrthm=NO_ENCRYPTION;
					break;
			}

			encryptionalgo=rtlpriv->securitypriv.dot11PrivacyAlgrthm;
		}
		else
		{
			encryptionalgo=rtlpriv->securitypriv.dot118021XGrpPrivacy;

		}

		if((check_fwstate(&rtlpriv->mlmepriv, WIFI_ADHOC_STATE)==true) && (IS_MAC_ADDRESS_BROADCAST(key->BSSID) == false)) {
			ret= _FAIL;
			goto exit;
		}

		// Check key length for TKIP
		if((encryptionalgo== TKIP_ENCRYPTION) && (key->KeyLength != 32)) {
			ret= _FAIL;
			goto exit;

		} else if(encryptionalgo== AESCCMP_ENCRYPTION && (key->KeyLength != 16 && key->KeyLength != 32) ) {

			// Check key length for AES
			// For NDTEST, we allow keylen=32 in this case. 2005.01.27, by rcnjko.
			ret= _FAIL;
			goto exit;
		}

		// Change the key length for EAPPkt9x.vxd. Added by Annie, 2005-11-03.
		if((encryptionalgo==  AESCCMP_ENCRYPTION) && (key->KeyLength == 32) ) {
			key->KeyLength = 16;
		}

		if(key->KeyIndex & 0x8000000) {//error ??? 0x8000_0000
			bgrouptkey = true;
		}

		if((check_fwstate(&rtlpriv->mlmepriv, WIFI_ADHOC_STATE)==true)&&(check_fwstate(&rtlpriv->mlmepriv, _FW_LINKED)==true))
		{
			bgrouptkey = true;
		}

		bgroup = true;
	}

	// If WEP encryption algorithm, just call rtw_set_802_11_add_wep().
	if((rtlpriv->securitypriv.dot11AuthAlgrthm !=dot11AuthAlgrthm_8021X)&&(encryptionalgo== WEP40_ENCRYPTION  || encryptionalgo== WEP104_ENCRYPTION))
	{
		uint8_t ret;
		uint32_t	 keyindex;
		uint32_t	 len = FIELD_OFFSET(NDIS_802_11_KEY, KeyMaterial) + key->KeyLength;
		NDIS_802_11_WEP *wep = &rtlpriv->securitypriv.ndiswep;

		wep->Length = len;
		keyindex = key->KeyIndex&0x7fffffff;
		wep->KeyIndex = keyindex ;
		wep->KeyLength = key->KeyLength;

		memcpy(wep->KeyMaterial, key->KeyMaterial, key->KeyLength);
		memcpy(&(rtlpriv->securitypriv.dot11DefKey[keyindex].skey[0]), key->KeyMaterial, key->KeyLength);

		rtlpriv->securitypriv.dot11DefKeylen[keyindex]=key->KeyLength;
		rtlpriv->securitypriv.dot11PrivacyKeyIndex=keyindex;

		ret = rtw_set_802_11_add_wep(rtlpriv, wep);

		goto exit;

	}

	if(key->KeyIndex & 0x20000000){
		// SetRSC
		if(bgroup == true)
		{
			NDIS_802_11_KEY_RSC keysrc=key->KeyRSC & 0x00FFFFFFFFFFFFULL;
			memcpy(&rtlpriv->securitypriv.dot11Grprxpn, &keysrc, 8);
		}
		else
		{
			NDIS_802_11_KEY_RSC keysrc=key->KeyRSC & 0x00FFFFFFFFFFFFULL;
			memcpy(&rtlpriv->securitypriv.dot11Grptxpn, &keysrc, 8);
		}

	}

	// Indicate this key idx is used for TX
	// Save the key in KeyMaterial
	if(bgroup == true) // Group transmit key
	{
		int res;

		if(bgrouptkey == true)
		{
			rtlpriv->securitypriv.dot118021XGrpKeyid=(uint8_t)key->KeyIndex;
		}

		if((key->KeyIndex&0x3) == 0){
			ret = _FAIL;
			goto exit;
		}

		memset(&rtlpriv->securitypriv.dot118021XGrpKey[(uint8_t)((key->KeyIndex) & 0x03)], 0, 16);
		memset(&rtlpriv->securitypriv.dot118021XGrptxmickey[(uint8_t)((key->KeyIndex) & 0x03)], 0, 16);
		memset(&rtlpriv->securitypriv.dot118021XGrprxmickey[(uint8_t)((key->KeyIndex) & 0x03)], 0, 16);

		if((key->KeyIndex & 0x10000000))
		{
			memcpy(&rtlpriv->securitypriv.dot118021XGrptxmickey[(uint8_t)((key->KeyIndex) & 0x03)], key->KeyMaterial + 16, 8);
			memcpy(&rtlpriv->securitypriv.dot118021XGrprxmickey[(uint8_t)((key->KeyIndex) & 0x03)], key->KeyMaterial + 24, 8);
		}
		else
		{
			memcpy(&rtlpriv->securitypriv.dot118021XGrptxmickey[(uint8_t)((key->KeyIndex) & 0x03)], key->KeyMaterial + 24, 8);
			memcpy(&rtlpriv->securitypriv.dot118021XGrprxmickey[(uint8_t)((key->KeyIndex) & 0x03)], key->KeyMaterial + 16, 8);
		}

		//set group key by index
		memcpy(&rtlpriv->securitypriv.dot118021XGrpKey[(uint8_t)((key->KeyIndex) & 0x03)], key->KeyMaterial, key->KeyLength);

		key->KeyIndex=key->KeyIndex & 0x03;

		rtlpriv->securitypriv.binstallGrpkey=true;

		rtlpriv->securitypriv.bcheck_grpkey=false;

		res=rtw_set_key(rtlpriv,&rtlpriv->securitypriv, key->KeyIndex, 1);

		if(res==_FAIL)
			ret= _FAIL;

		goto exit;

	}
	else // Pairwise Key
	{
		uint8_t res;

		pbssid=get_bssid(&rtlpriv->mlmepriv);
		stainfo=rtw_get_stainfo(&rtlpriv->stapriv , pbssid );

		if(stainfo!=NULL)
		{
			memset( &stainfo->dot118021x_UncstKey, 0, 16);// clear keybuffer

			memcpy(&stainfo->dot118021x_UncstKey, key->KeyMaterial, 16);

			if(encryptionalgo== TKIP_ENCRYPTION)
			{
				rtlpriv->securitypriv.busetkipkey=false;

				//_set_timer(&rtlpriv->securitypriv.tkip_timer, 50);

				// if TKIP, save the Receive/Transmit MIC key in KeyMaterial[128-255]
				if((key->KeyIndex & 0x10000000)){
					memcpy(&stainfo->dot11tkiptxmickey, key->KeyMaterial + 16, 8);
					memcpy(&stainfo->dot11tkiprxmickey, key->KeyMaterial + 24, 8);

				} else {
					memcpy(&stainfo->dot11tkiptxmickey, key->KeyMaterial + 24, 8);
					memcpy(&stainfo->dot11tkiprxmickey, key->KeyMaterial + 16, 8);

				}

			}
			else if(encryptionalgo == AESCCMP_ENCRYPTION)
			{

			}


			//Set key to CAM through H2C command
			if(bgrouptkey)//never go to here
			{
				res=rtw_setstakey_cmd(rtlpriv, (unsigned char *)stainfo, false);
			}
			else{
				res=rtw_setstakey_cmd(rtlpriv, (unsigned char *)stainfo, true);
			}

			if(res ==false)
				ret= _FAIL;

		}

	}

exit:



	return ret;
}

uint8_t rtw_set_802_11_remove_key(struct rtl_priv*	rtlpriv, NDIS_802_11_REMOVE_KEY *key){

	uint				encryptionalgo;
	uint8_t * pbssid;
	struct sta_info *stainfo;
	uint8_t	bgroup = (key->KeyIndex & 0x4000000) > 0 ? false: true;
	uint8_t	keyIndex = (uint8_t)key->KeyIndex & 0x03;
	uint8_t	ret=_SUCCESS;



	if ((key->KeyIndex & 0xbffffffc) > 0) {
		ret=_FAIL;
		goto exit;
	}

	if (bgroup == true) {
		encryptionalgo= rtlpriv->securitypriv.dot118021XGrpPrivacy;
		// clear group key by index
		//NdisZeroMemory(rtlpriv->MgntInfo.SecurityInfo.KeyBuf[keyIndex], MAX_WEP_KEY_LEN);
		//rtlpriv->MgntInfo.SecurityInfo.KeyLen[keyIndex] = 0;

		memset(&rtlpriv->securitypriv.dot118021XGrpKey[keyIndex], 0, 16);

		//! \todo Send a H2C Command to Firmware for removing this Key in CAM Entry.

	} else {

		pbssid=get_bssid(&rtlpriv->mlmepriv);
		stainfo=rtw_get_stainfo(&rtlpriv->stapriv , pbssid );
		if(stainfo !=NULL){
			encryptionalgo=stainfo->dot118021XPrivacy;

		// clear key by BSSID
		memset(&stainfo->dot118021x_UncstKey, 0, 16);

		//! \todo Send a H2C Command to Firmware for disable this Key in CAM Entry.

		}
		else{
			ret= _FAIL;
			goto exit;
		}
	}

exit:



	return true;

}

/*
* rtw_get_cur_max_rate -
* @rtlpriv: pointer to struct rtl_priv structure
*
* Return 0 or 100Kbps
*/
u16 rtw_get_cur_max_rate(struct rtl_priv *rtlpriv)
{
	int	i = 0;
	uint8_t	*p;
	u16	rate = 0, max_rate = 0;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	WLAN_BSSID_EX  *pcur_bss = &pmlmepriv->cur_network.network;
	struct rtw_ieee80211_ht_cap *pht_capie;
	uint8_t	rf_type = 0;
	uint8_t	bw_40MHz=0, short_GI_20=0, short_GI_40=0, cbw40_enable=0;
	u16	mcs_rate=0;
	uint32_t	ht_ielen = 0;
	struct vht_priv	*pvhtpriv = &pmlmepriv->vhtpriv;

	if((check_fwstate(pmlmepriv, _FW_LINKED) != true)
		&& (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) != true))
		return 0;

	if (IsSupportedTxHT(pmlmeext->cur_wireless_mode)) {
		p = rtw_get_ie(&pcur_bss->IEs[12], _HT_CAPABILITY_IE_, &ht_ielen, pcur_bss->IELength-12);
		if(p && ht_ielen>0)
		{
			pht_capie = (struct rtw_ieee80211_ht_cap *)(p+2);

			memcpy(&mcs_rate , pht_capie->supp_mcs_set, 2);

			//bw_40MHz = (pht_capie->cap_info&IEEE80211_HT_CAP_SUP_WIDTH) ? 1:0;
			//cur_bwmod is updated by beacon, pmlmeinfo is updated by association response
			bw_40MHz = (pmlmeext->cur_bwmode && (HT_INFO_HT_PARAM_REC_TRANS_CHNL_WIDTH & pmlmeinfo->HT_info.infos[0])) ? 1:0;

			//short_GI = (pht_capie->cap_info&(IEEE80211_HT_CAP_SGI_20|IEEE80211_HT_CAP_SGI_40)) ? 1:0;
			short_GI_20 = (pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info&IEEE80211_HT_CAP_SGI_20) ? 1:0;
			short_GI_40 = (pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info&IEEE80211_HT_CAP_SGI_40) ? 1:0;

			rf_type = rtlpriv->phy.rf_type;

			if (pmlmeext->cur_channel > 14) {
				if ((0x21 & 0xf0) > 0)
					cbw40_enable = 1;
			} else {
				if ((0x21 & 0x0f) > 0)
					cbw40_enable = 1;
			}

			max_rate = rtw_mcs_rate(
				rf_type,
				bw_40MHz & cbw40_enable,
				short_GI_20,
				short_GI_40,
				pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate
			);
		}
	}
	else if (IsSupportedVHT(pmlmeext->cur_wireless_mode)) {
		max_rate = ((rtw_vht_data_rate(pvhtpriv->vht_bwmode, pvhtpriv->sgi, pvhtpriv->vht_highest_rate) + 1) >> 1) * 10;
	}
	else
	{
		while( (pcur_bss->SupportedRates[i]!=0) && (pcur_bss->SupportedRates[i]!=0xFF))
		{
			rate = pcur_bss->SupportedRates[i]&0x7F;
			if(rate>max_rate)
				max_rate = rate;
			i++;
		}

		max_rate = max_rate*10/2;
	}

	return max_rate;
}

/*
* rtw_set_scan_mode -
* @rtlpriv: pointer to struct rtl_priv structure
* @scan_mode:
*
* Return _SUCCESS or _FAIL
*/
int rtw_set_scan_mode(struct rtl_priv *rtlpriv, RT_SCAN_TYPE scan_mode)
{
	if(scan_mode != SCAN_ACTIVE && scan_mode != SCAN_PASSIVE)
		return _FAIL;

	rtlpriv->mlmepriv.scan_mode = scan_mode;

	return _SUCCESS;
}
