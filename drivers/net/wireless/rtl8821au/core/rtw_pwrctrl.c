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
#define _RTW_PWRCTRL_C_

#include <drv_types.h>
#include <../debug.h>

// Should not include hal dependent herader here, it will remove later. Lucas@20130123

/* ULLI : rtw_ prefix for not distrub rtlwifi */

void rtw_ips_enter(struct rtl_priv *rtlpriv)
{
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;

	down(&pwrpriv->lock);

	pwrpriv->bips_processing = true;

	/* syn ips_mode with request */
	pwrpriv->ips_mode = pwrpriv->ips_mode_req;

	pwrpriv->ips_enter_cnts++;
	RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "==>ips_enter cnts:%d\n",pwrpriv->ips_enter_cnts);
	if (ERFOFF == pwrpriv->change_rfpwrstate) {
		pwrpriv->bpower_saving = true;
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "nolinked power save enter\n");

		rtw_ips_pwr_down(rtlpriv);
		pwrpriv->rf_pwrstate = ERFOFF;
	}
	pwrpriv->bips_processing = false;

	up(&pwrpriv->lock);

}

/* ULLI : rtw_ prefix for not distrub rtlwifi */

int rtw_ips_leave(struct rtl_priv * rtlpriv)
{
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	struct security_priv* psecuritypriv=&(rtlpriv->securitypriv);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	int result = _SUCCESS;
	int keyid;

	down(&pwrpriv->lock);

	if ((pwrpriv->rf_pwrstate == ERFOFF) &&(!pwrpriv->bips_processing)) {
		pwrpriv->bips_processing = true;
		pwrpriv->change_rfpwrstate = ERFON;
		pwrpriv->ips_leave_cnts++;
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "==>ips_leave cnts:%d\n",pwrpriv->ips_leave_cnts);

		if ((result = rtw_ips_pwr_up(rtlpriv)) == _SUCCESS) {
			pwrpriv->rf_pwrstate = ERFON;
		}
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "nolinked power save leave\n");

		if ((WEP40_ENCRYPTION == psecuritypriv->dot11PrivacyAlgrthm)
		   ||(WEP104_ENCRYPTION == psecuritypriv->dot11PrivacyAlgrthm)) {
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "==>%s,channel(%d),processing(%x)\n",__FUNCTION__,rtlpriv->mlmeextpriv.cur_channel,pwrpriv->bips_processing);
			set_channel_bwmode(rtlpriv, rtlpriv->mlmeextpriv.cur_channel, HAL_PRIME_CHNL_OFFSET_DONT_CARE, CHANNEL_WIDTH_20);
			for (keyid = 0; keyid < 4; keyid++) {
				if (pmlmepriv->key_mask & BIT(keyid)) {
					if (keyid == psecuritypriv->dot11PrivacyKeyIndex)
						result=rtw_set_key(rtlpriv,psecuritypriv, keyid, 1);
					else
						result=rtw_set_key(rtlpriv,psecuritypriv, keyid, 0);
				}
			}
		}

		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "==> ips_leave.....LED(0x%08x)...\n",rtl_read_dword(rtlpriv,0x4c));
		pwrpriv->bips_processing = false;

		pwrpriv->bpower_saving = false;
	}

	up(&pwrpriv->lock);

	return result;
}

#ifdef CONFIG_AUTOSUSPEND
extern void autosuspend_enter(struct rtl_priv* rtlpriv);
extern int autoresume_enter(struct rtl_priv* rtlpriv);
#endif

bool rtw_pwr_unassociated_idle(struct rtl_priv *rtlpriv)
{
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct xmit_priv *pxmit_priv = &rtlpriv->xmitpriv;

	bool ret = false;

	if (rtlpriv->pwrctrlpriv.ips_deny_time >= jiffies) {
		//DBG_871X("%s ips_deny_time\n", __func__);
		goto exit;
	}

	if (check_fwstate(pmlmepriv, WIFI_ASOC_STATE|WIFI_SITE_MONITOR)
		|| check_fwstate(pmlmepriv, WIFI_UNDER_LINKING|WIFI_UNDER_WPS)
		|| check_fwstate(pmlmepriv, WIFI_AP_STATE)
		|| check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE|WIFI_ADHOC_STATE)
	) {
		goto exit;
	}

	if (pxmit_priv->free_xmitbuf_cnt != NR_XMITBUFF ||
		pxmit_priv->free_xmit_extbuf_cnt != NR_XMIT_EXTBUFF) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "There are some pkts to transmit\n");
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "free_xmitbuf_cnt: %d, free_xmit_extbuf_cnt: %d\n",
			pxmit_priv->free_xmitbuf_cnt, pxmit_priv->free_xmit_extbuf_cnt);
		goto exit;
	}

	ret = true;

exit:
	return ret;
}

void rtw_ps_processor(struct rtl_priv *rtlpriv)
{
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);

	pwrpriv->ps_processing = true;

	if (pwrpriv->ips_mode_req == IPS_NONE
	)
		goto exit;

	if (rtw_pwr_unassociated_idle(rtlpriv) == false)
		goto exit;

	if ((pwrpriv->rf_pwrstate == ERFON)
	   && ((pwrpriv->pwr_state_check_cnts%4)==0)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "==>%s .fw_state(%x)\n",__FUNCTION__,get_fwstate(pmlmepriv));
		pwrpriv->change_rfpwrstate = ERFOFF;
#ifdef CONFIG_AUTOSUSPEND
		if (rtlpriv->registrypriv.usbss_enable) {
			if(pwrpriv->bHWPwrPindetect)
				pwrpriv->bkeepfwalive = true;

			if(rtlpriv->net_closed == true)
				pwrpriv->ps_flag = true;

			rtlpriv->bCardDisableWOHSM = true;
			autosuspend_enter(rtlpriv);
		} else if(pwrpriv->bHWPwrPindetect) {
		}
		else
#endif
		{

			rtw_ips_enter(rtlpriv);
		}
	}
exit:
	rtw_set_pwr_state_check_timer(&rtlpriv->pwrctrlpriv);
	pwrpriv->ps_processing = false;
	return;
}


static void pwr_state_check_handler(RTW_TIMER_HDL_ARGS)
{
	struct rtl_priv *rtlpriv = (struct rtl_priv *)FunctionContext;
	rtw_ps_cmd(rtlpriv);
}


/*
 *
 * Parameters
 *	rtlpriv
 *	pslv			power state level, only could be PS_STATE_S0 ~ PS_STATE_S4
 *
 */
void rtw_set_rpwm(struct rtl_priv *rtlpriv, uint8_t pslv)
{
	uint8_t	rpwm;
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;



	pslv = PS_STATE(pslv);

	if (true == pwrpriv->btcoex_rfon) {
		if (pslv < PS_STATE_S4)
			pslv = PS_STATE_S3;
	}

#ifdef CONFIG_LPS_RPWM_TIMER
	if (pwrpriv->brpwmtimeout == true) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s: RPWM timeout, force to set RPWM(0x%02X) again!\n", __FUNCTION__, pslv);
	} else
#endif // CONFIG_LPS_RPWM_TIMER
	{
	if ( (pwrpriv->rpwm == pslv)
		)
	{
		return;
	}
	}

	if ((rtlpriv->bSurpriseRemoved == true)
	   || (rtlpriv->hw_init_completed == false)) {

		pwrpriv->cpwm = PS_STATE_S4;
		return;
	}

	if (rtlpriv->bDriverStopped == true) {

		if (pslv < PS_STATE_S2) {
			return;
		}
	}

	rpwm = pslv | pwrpriv->tog;

	pwrpriv->rpwm = pslv;

#ifdef CONFIG_LPS_RPWM_TIMER
	if (rpwm & PS_ACK)
		_set_timer(&pwrpriv->pwr_rpwm_timer, LPS_RPWM_WAIT_MS);
#endif // CONFIG_LPS_RPWM_TIMER

	pwrpriv->tog += 0x80;

	{
		pwrpriv->cpwm = pslv;
	}


}

static uint8_t PS_RDY_CHECK(struct rtl_priv *rtlpriv)
{
	uint32_t	 curr_time, delta_time;
	struct pwrctrl_priv	*pwrpriv = &rtlpriv->pwrctrlpriv;
	struct mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);

	curr_time = jiffies;

	delta_time = curr_time -pwrpriv->DelayLPSLastTimeStamp;

	if(delta_time < LPS_DELAY_TIME) {
		return false;
	}

	if ((check_fwstate(pmlmepriv, _FW_LINKED) == false) ||
		(check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) == true) ||
		(check_fwstate(pmlmepriv, WIFI_AP_STATE) == true) ||
		(check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true) ||
		(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true) )
		return false;
	if (true == pwrpriv->bInSuspend )
		return false;
	if ((rtlpriv->securitypriv.dot11AuthAlgrthm == dot11AuthAlgrthm_8021X)
	  && (rtlpriv->securitypriv.binstallGrpkey == false)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "Group handshake still in progress !!!\n");
		return false;
	}

	return true;
}

void rtw_set_ps_mode(struct rtl_priv *rtlpriv, uint8_t ps_mode, uint8_t smart_ps, uint8_t bcn_ant_mode)
{
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;



	if (ps_mode > PM_Card_Disable) {
		return;
	}

	if (pwrpriv->pwr_mode == ps_mode) {
		if (PS_MODE_ACTIVE == ps_mode)
			return;

		if ((pwrpriv->smart_ps == smart_ps)
		   && (pwrpriv->bcn_ant_mode == bcn_ant_mode)) {
			return;
		}
	}

	//if(pwrpriv->pwr_mode == PS_MODE_ACTIVE)
	if(ps_mode == PS_MODE_ACTIVE)
	{
		{
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "rtw_set_ps_mode: Leave 802.11 power save\n");


			pwrpriv->pwr_mode = ps_mode;
			rtw_set_rpwm(rtlpriv, PS_STATE_S4);
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_H2C_FW_PWRMODE, (uint8_t *)(&ps_mode));
			pwrpriv->fw_current_inpsmode = false;
		}
	} else 	{
		if (PS_RDY_CHECK(rtlpriv)) {
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s: Enter 802.11 power save\n", __FUNCTION__);


			pwrpriv->fw_current_inpsmode = true;
			pwrpriv->pwr_mode = ps_mode;
			pwrpriv->smart_ps = smart_ps;
			pwrpriv->bcn_ant_mode = bcn_ant_mode;
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_H2C_FW_PWRMODE, (uint8_t *)(&ps_mode));

			rtw_set_rpwm(rtlpriv, PS_STATE_S2);
		}
	}


}

/*
 * Return:
 *	0:	Leave OK
 *	-1:	Timeout
 *	-2:	Other error
 */
int32_t LPS_RF_ON_check(struct rtl_priv *rtlpriv, uint32_t	 delay_ms)
{
	uint32_t	 start_time;
	uint8_t bAwake = false;
	int32_t err = 0;


	start_time = jiffies;
	while (1) {
		rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_FWLPS_RF_ON, &bAwake);
		if (true == bAwake)
			break;

		if (true == rtlpriv->bSurpriseRemoved) {
			err = -2;
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s: device surprise removed!!\n", __FUNCTION__);
			break;
		}

		if (rtw_get_passing_time_ms(start_time) > delay_ms) {
			err = -1;
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s: Wait for FW LPS leave more than %u ms!!!\n", __FUNCTION__, delay_ms);
			break;
		}
		rtw_usleep_os(100);
	}

	return err;
}

//
//	Description:
//		Enter the leisure power save mode.
//
void LPS_Enter(struct rtl_priv *rtlpriv)
{
	struct pwrctrl_priv	*pwrpriv = &rtlpriv->pwrctrlpriv;
	struct mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);



//	DBG_871X("+LeisurePSEnter\n");


	if (PS_RDY_CHECK(rtlpriv) == false)
		return;

	if (pwrpriv->bLeisurePs) {
		// Idle for a while if we connect to AP a while ago.
		if(pwrpriv->LpsIdleCount >= 2) { //  4 Sec
			if(pwrpriv->pwr_mode == PS_MODE_ACTIVE) {
				pwrpriv->bpower_saving = true;
				rtw_set_ps_mode(rtlpriv, pwrpriv->power_mgnt, rtlpriv->registrypriv.smart_ps, 0);
			}
		} else
			pwrpriv->LpsIdleCount++;
	}

//	DBG_871X("-LeisurePSEnter\n");


}

//
//	Description:
//		Leave the leisure power save mode.
//
void LPS_Leave(struct rtl_priv *rtlpriv)
{
#define LPS_LEAVE_TIMEOUT_MS 100

	struct pwrctrl_priv	*pwrpriv = &rtlpriv->pwrctrlpriv;
	uint32_t	 start_time;
	uint8_t bAwake = false;




//	DBG_871X("+LeisurePSLeave\n");

	if (pwrpriv->bLeisurePs) {
		if(pwrpriv->pwr_mode != PS_MODE_ACTIVE) {
			rtw_set_ps_mode(rtlpriv, PS_MODE_ACTIVE, 0, 0);

			if(pwrpriv->pwr_mode == PS_MODE_ACTIVE)
				LPS_RF_ON_check(rtlpriv, LPS_LEAVE_TIMEOUT_MS);
		}
	}

	pwrpriv->bpower_saving = false;

//	DBG_871X("-LeisurePSLeave\n");


}

//
// Description: Leave all power save mode: LPS, FwLPS, IPS if needed.
// Move code to function by tynli. 2010.03.26.
//
void LeaveAllPowerSaveMode(struct rtl_priv *rtlpriv)
{
	struct mlme_priv	*pmlmepriv = &(rtlpriv->mlmepriv);
	uint8_t	enqueue = 0;



	//DBG_871X("%s.....\n",__FUNCTION__);
	if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
	{ //connect
		rtw_lps_ctrl_wk_cmd(rtlpriv, LPS_CTRL_LEAVE, enqueue);

	} else {
		if(rtlpriv->pwrctrlpriv.rf_pwrstate== ERFOFF) {
#ifdef CONFIG_AUTOSUSPEND
			if(rtlpriv->registrypriv.usbss_enable) 	{
				usb_disable_autosuspend(rtl_usbdev(rtlpriv)->pusbdev);
			} else
#endif
			{
			}
		}
	}


}

void rtw_init_pwrctrl_priv(struct rtl_priv *rtlpriv)
{
	struct rtl_hal_cfg *cfg = rtlpriv->cfg;
	struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;



	sema_init(&pwrctrlpriv->lock, 1);
	pwrctrlpriv->rf_pwrstate = ERFON;
	pwrctrlpriv->ips_enter_cnts=0;
	pwrctrlpriv->ips_leave_cnts=0;
	pwrctrlpriv->bips_processing = false;

	pwrctrlpriv->ips_mode = cfg->mod_params->inactiveps;
	pwrctrlpriv->ips_mode_req = cfg->mod_params->inactiveps;

	pwrctrlpriv->pwr_state_check_interval = RTW_PWR_STATE_CHK_INTERVAL;
	pwrctrlpriv->pwr_state_check_cnts = 0;
	pwrctrlpriv->bInternalAutoSuspend = false;
	pwrctrlpriv->bInSuspend = false;

	pwrctrlpriv->LpsIdleCount = 0;
	//pwrctrlpriv->FWCtrlPSMode =rtlpriv->registrypriv.power_mgnt;// PS_MODE_MIN;
	pwrctrlpriv->power_mgnt =rtlpriv->registrypriv.power_mgnt;// PS_MODE_MIN;
	pwrctrlpriv->bLeisurePs = (PS_MODE_ACTIVE != pwrctrlpriv->power_mgnt)?true:false;

	pwrctrlpriv->fw_current_inpsmode = false;

	pwrctrlpriv->rpwm = 0;
	pwrctrlpriv->cpwm = PS_STATE_S4;

	pwrctrlpriv->pwr_mode = PS_MODE_ACTIVE;
	pwrctrlpriv->smart_ps = rtlpriv->registrypriv.smart_ps;
	pwrctrlpriv->bcn_ant_mode = 0;

	pwrctrlpriv->tog = 0x80;

	pwrctrlpriv->btcoex_rfon = false;

	_init_timer(&pwrctrlpriv->pwr_state_check_timer, rtlpriv->ndev, 
		    pwr_state_check_handler, rtlpriv);



}

/*
* rtw_pwr_wakeup - Wake the NIC up from: 1)IPS. 2)USB autosuspend
* @rtlpriv: pointer to struct rtl_priv structure
* @ips_deffer_ms: the ms wiil prevent from falling into IPS after wakeup
* Return _SUCCESS or _FAIL
*/

int _rtw_pwr_wakeup(struct rtl_priv *rtlpriv, uint32_t	 ips_deffer_ms, const char *caller)
{
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	int ret = _SUCCESS;
	uint32_t	 start = jiffies;


	if (pwrpriv->ips_deny_time < jiffies + rtw_ms_to_systime(ips_deffer_ms))
		pwrpriv->ips_deny_time = jiffies + rtw_ms_to_systime(ips_deffer_ms);


	if (pwrpriv->ps_processing) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s wait ps_processing...\n", __func__);
		while (pwrpriv->ps_processing && rtw_get_passing_time_ms(start) <= 3000)
			msleep(10);
		if (pwrpriv->ps_processing)
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s wait ps_processing timeout\n", __func__);
		else
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s wait ps_processing done\n", __func__);
	}

	if (pwrpriv->bInternalAutoSuspend == false && pwrpriv->bInSuspend) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s wait bInSuspend...\n", __func__);
		while (pwrpriv->bInSuspend
			&& ((rtw_get_passing_time_ms(start) <= 3000)
				|| (rtw_get_passing_time_ms(start) <= 500))
		) {
			msleep(10);
		}

		if (pwrpriv->bInSuspend)
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s wait bInSuspend timeout\n", __func__);
		else
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s wait bInSuspend done\n", __func__);
	}

	//System suspend is not allowed to wakeup
	if ((pwrpriv->bInternalAutoSuspend == false) && (true == pwrpriv->bInSuspend )){
		ret = _FAIL;
		goto exit;
	}

	//block???
	if ((pwrpriv->bInternalAutoSuspend == true)  && (rtlpriv->net_closed == true)) {
		ret = _FAIL;
		goto exit;
	}

	//I think this should be check in IPS, LPS, autosuspend functions...
	if (check_fwstate(pmlmepriv, _FW_LINKED) == true) {
		ret = _SUCCESS;
		goto exit;
	}

	if (ERFOFF == pwrpriv->rf_pwrstate ) {
#ifdef CONFIG_AUTOSUSPEND
		if (pwrpriv->brfoffbyhw==true) {
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "hw still in ERFOFF state ...........\n");
			ret = _FAIL;
			goto exit;
		} else if (rtlpriv->registrypriv.usbss_enable) {
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s call autoresume_enter....\n",__FUNCTION__);
			if (_FAIL ==  autoresume_enter(rtlpriv)) {
				RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "======> autoresume fail.............\n");
				ret = _FAIL;
				goto exit;
			}
		} else
#endif
		{
			RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s call ips_leave....\n",__FUNCTION__);
			if (_FAIL ==  rtw_ips_leave(rtlpriv)) {
				RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "======> ips_leave fail.............\n");
				ret = _FAIL;
				goto exit;
			}
		}
	}

	//TODO: the following checking need to be merged...
	if (rtlpriv->bDriverStopped
		|| !rtlpriv->initialized
		|| !rtlpriv->hw_init_completed
	){
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s: bDriverStopped=%d, initialized=%d, hw_init_completed=%u\n"
			, caller
		   	, rtlpriv->bDriverStopped
		   	, rtlpriv->initialized
		   	, rtlpriv->hw_init_completed);
		ret= false;
		goto exit;
	}

exit:
	if (pwrpriv->ips_deny_time < jiffies + rtw_ms_to_systime(ips_deffer_ms))
		pwrpriv->ips_deny_time = jiffies + rtw_ms_to_systime(ips_deffer_ms);
	return ret;

}

int rtw_pm_set_lps(struct rtl_priv *rtlpriv, uint8_t mode)
{
	int	ret = 0;
	struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;

	if (mode < PS_MODE_NUM) {
		if (pwrctrlpriv->power_mgnt !=mode) {
			if (PS_MODE_ACTIVE == mode) {
				LeaveAllPowerSaveMode(rtlpriv);
			} else {
				pwrctrlpriv->LpsIdleCount = 2;
			}
			pwrctrlpriv->power_mgnt = mode;
			pwrctrlpriv->bLeisurePs = (PS_MODE_ACTIVE != pwrctrlpriv->power_mgnt)?true:false;
		}
	} else {
		ret = -EINVAL;
	}

	return ret;
}

int rtw_pm_set_ips(struct rtl_priv *rtlpriv, uint8_t mode)
{
	struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;

	if (mode == IPS_NORMAL) {
		pwrctrlpriv->ips_mode_req = mode;
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s %s\n", __FUNCTION__, "IPS_NORMAL");
		return 0;
	} else if ( mode ==IPS_NONE) {
		pwrctrlpriv->ips_mode_req = mode;
		RT_TRACE(rtlpriv, COMP_POWER, DBG_DMESG, "%s %s\n", __FUNCTION__, "IPS_NONE");
		if ((rtlpriv->bSurpriseRemoved ==0)&&(_FAIL == rtw_pwr_wakeup(rtlpriv)) )
			return -EFAULT;
	} else {
		return -EINVAL;
	}
	return 0;
}


