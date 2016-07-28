#include <drv_types.h>
#include <../usb.h>
#include <../wifi.h>
#include <../debug.h>
#include <../cam.h>
#include <../rtl8821au/reg.h>

void rtw_cam_reset_sec_info(struct rtl_priv *rtlpriv)
{
	rtlpriv->sec.use_defaultkey = false;
	rtlpriv->sec.pairwise_enc_algorithm = NO_ENCRYPTION;
	rtlpriv->sec.group_enc_algorithm = NO_ENCRYPTION;
	memset(rtlpriv->sec.key_buf, 0, KEY_BUF_SIZE * MAX_KEY_LEN);
	memset(rtlpriv->sec.key_len, 0, KEY_BUF_SIZE);
	rtlpriv->sec.pairwise_key = NULL;
}


void rtw_cam_reset_all_entry(struct rtl_priv *rtlpriv)
{
	u32 ul_command;

	ul_command = BIT(31) | BIT(30);
	rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[RWCAM], ul_command);
}

void CAM_empty_entry(struct rtl_priv *rtlpriv, uint8_t ucIndex)
{
	uint8_t i;
	uint32_t	ulCommand = 0;
	uint32_t	ulContent = 0;
	uint32_t	ulEncAlgo = CAM_AES;

	for (i = 0; i < CAM_CONTENT_COUNT; i++) {
		/* filled id in CAM config 2 byte */
		if (i == 0) {
			ulContent |= (ucIndex & 0x03) | ((u16)(ulEncAlgo)<<2);
			/* ulContent |= CAM_VALID; */
		} else 	{
			ulContent = 0;
		}
		/*  polling bit, and No Write enable, and address */
		ulCommand = CAM_CONTENT_COUNT*ucIndex+i;
		ulCommand = ulCommand | CAM_POLLINIG | CAM_WRITE;
		/* write content 0 is equall to mark invalid */
		rtl_write_dword(rtlpriv, WCAMI, ulContent);  /* delay_ms(40); */
		rtl_write_dword(rtlpriv, RWCAM, ulCommand);  /* delay_ms(40); */
	}
}

static void write_cam(struct rtl_priv *rtlpriv, uint8_t entry, u16 ctrl, uint8_t *mac, uint8_t *key)
{
	unsigned int	i, val, addr;
	//unsigned int    cmd;
	int j;
	uint32_t	cam_val[2];

	addr = entry << 3;

	for (j = 5; j >= 0; j--)
	{
		switch (j)
		{
			case 0:
				val = (ctrl | (mac[0] << 16) | (mac[1] << 24) );
				break;

			case 1:
				val = (mac[2] | ( mac[3] << 8) | (mac[4] << 16) | (mac[5] << 24));
				break;

			default:
				i = (j - 2) << 2;
				val = (key[i] | (key[i+1] << 8) | (key[i+2] << 16) | (key[i+3] << 24));
				break;

		}

		cam_val[0] = val;
		cam_val[1] = addr + (unsigned int)j;

		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_CAM_WRITE, (uint8_t *)cam_val);

		//rtw_write32(rtlpriv, WCAMI, val);

		//cmd = CAM_POLLINIG | CAM_WRITE | (addr + j);
		//rtw_write32(rtlpriv, RWCAM, cmd);

		//DBG_871X("%s=> cam write: %x, %x\n",__FUNCTION__, cmd, val);

	}

}

void clear_cam_entry(struct rtl_priv *rtlpriv, uint8_t entry)
{

	unsigned char null_sta[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	unsigned char null_key[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00};

	write_cam(rtlpriv, entry, 0, null_sta, null_key);
}

static void rtl_cam_program_entry(struct rtl_priv *rtlpriv, u32 entry_no,
			   u8 *mac_addr, u8 *key_cont_128, u16 us_config)
{
	u32 target_command;
	u32 target_content = 0;
	u8 entry_i;

	RT_PRINT_DATA(rtlpriv, COMP_SEC, DBG_DMESG, "Key content :",
		      key_cont_128, 16);

	for (entry_i = 0; entry_i < CAM_CONTENT_COUNT; entry_i++) {
		target_command = entry_i + CAM_CONTENT_COUNT * entry_no;
		target_command = target_command | BIT(31) | BIT(16);

		if (entry_i == 0) {
			target_content = (u32) (*(mac_addr + 0)) << 16 |
			    (u32) (*(mac_addr + 1)) << 24 | (u32) us_config;

			rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[WCAMI],
					target_content);
			rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[RWCAM],
					target_command);

			RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
				 "WRITE %x: %x\n",
				 rtlpriv->cfg->maps[WCAMI], target_content);
			RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
				 "The Key ID is %d\n", entry_no);
			RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
				 "WRITE %x: %x\n",
				 rtlpriv->cfg->maps[RWCAM], target_command);

		} else if (entry_i == 1) {

			target_content = (u32) (*(mac_addr + 5)) << 24 |
			    (u32) (*(mac_addr + 4)) << 16 |
			    (u32) (*(mac_addr + 3)) << 8 |
			    (u32) (*(mac_addr + 2));

			rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[WCAMI],
					target_content);
			rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[RWCAM],
					target_command);

			RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
				 "WRITE A4: %x\n", target_content);
			RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
				 "WRITE A0: %x\n", target_command);

		} else {

			target_content =
			    (u32) (*(key_cont_128 + (entry_i * 4 - 8) + 3)) <<
			    24 | (u32) (*(key_cont_128 + (entry_i * 4 - 8) + 2))
			    << 16 |
			    (u32) (*(key_cont_128 + (entry_i * 4 - 8) + 1)) << 8
			    | (u32) (*(key_cont_128 + (entry_i * 4 - 8) + 0));

			rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[WCAMI],
					target_content);
			rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[RWCAM],
					target_command);
			udelay(100);

			RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
				 "WRITE A4: %x\n", target_content);
			RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
				 "WRITE A0: %x\n", target_command);
		}
	}

	RT_TRACE(rtlpriv, COMP_SEC, DBG_LOUD,
		 "after set key, usconfig:%x\n", us_config);
}

u8 rtw_cam_add_one_entry(struct rtl_priv *rtlpriv, u8 *mac_addr,
			 u32 ul_key_id, u32 ul_entry_idx, u32 ul_enc_alg,
			 u32 ul_default_key, u8 *key_content)
{
	u32 us_config;

	RT_TRACE(rtlpriv, COMP_SEC, DBG_DMESG,
		 "EntryNo:%x, ulKeyId=%x, ulEncAlg=%x, ulUseDK=%x MacAddr %pM\n",
		 ul_entry_idx, ul_key_id, ul_enc_alg,
		 ul_default_key, mac_addr);

	if (ul_key_id == TOTAL_CAM_ENTRY) {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_WARNING,
			 "ulKeyId exceed!\n");
		return 0;
	}

	if (ul_default_key == 1)
		us_config = CFG_VALID | ((u16) (ul_enc_alg) << 2);
	else
		us_config = CFG_VALID | ((ul_enc_alg) << 2) | ul_key_id;


	rtl_cam_program_entry(rtlpriv, ul_entry_idx, mac_addr,
			      (u8 *)key_content, us_config);

	RT_TRACE(rtlpriv, COMP_SEC, DBG_DMESG, "end\n");
	
	return 1;
}
