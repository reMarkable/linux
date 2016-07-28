#ifndef __RTW_CAM_H__
#define __RTW_CAM_H__

#define	CAM_CONTENT_COUNT		8
#define	CFG_VALID			BIT(15)
#define	CAM_WRITE			BIT(16)
#define	CAM_POLLINIG			BIT(31)

void rtw_cam_reset_all_entry(struct rtl_priv *rtlpriv);
// void write_cam(struct rtl_priv *rtlpriv, uint8_t entry, u16 ctrl, uint8_t *mac, uint8_t *key);
void clear_cam_entry(struct rtl_priv *rtlpriv, uint8_t entry);

void CAM_empty_entry(struct rtl_priv *rtlpriv, uint8_t ucIndex);

u8 rtw_cam_add_one_entry(struct rtl_priv *rtlpriv, u8 *mac_addr,
			 u32 ul_key_id, u32 ul_entry_idx, u32 ul_enc_alg,
			 u32 ul_default_key, u8 *key_content);

#endif
