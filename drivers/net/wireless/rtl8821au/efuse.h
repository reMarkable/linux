#ifndef __RTL_EFUSE_H_
#define __RTL_EFUSE_H_

#define EFUSE_INIT_MAP			0
#define EFUSE_MODIFY_MAP		1
#define	EFUSE_ERROE_HANDLE		1

#define	PG_STATE_HEADER 		0x01
#define	PG_STATE_WORD_0		0x02
#define	PG_STATE_WORD_1		0x04
#define	PG_STATE_WORD_2		0x08
#define	PG_STATE_WORD_3		0x10
#define	PG_STATE_DATA			0x20

#define	PG_SWBYTE_H			0x01
#define	PG_SWBYTE_L			0x02

#define	PGPKT_DATA_SIZE		8

#define		EFUSE_MAX_MAP_LEN		256
#define		EFUSE_MAX_HW_SIZE		512
#define		EFUSE_MAX_SECTION_BASE	16

#define EXT_HEADER(header) ((header & 0x1F ) == 0x0F)
#define ALL_WORDS_DISABLED(wde)	((wde & 0x0F) == 0x0F)
#define GET_HDR_OFFSET_2_0(header) ( (header & 0xE0) >> 5)

#define		EFUSE_REPEAT_THRESHOLD_			3

//=============================================
//	The following is for BT Efuse definition
//=============================================
#define		EFUSE_BT_MAX_MAP_LEN		1024
#define		EFUSE_MAX_BANK			4
#define		EFUSE_MAX_BT_BANK		(EFUSE_MAX_BANK-1)
//=============================================
/*--------------------------Define Parameters-------------------------------*/
#define		EFUSE_MAX_WORD_UNIT			4

/*------------------------------Define structure----------------------------*/
struct pgpkt_struct {
	u8 offset;
	u8 word_en;
	u8 data[8];
};

/*------------------------------Define structure----------------------------*/


/*------------------------Export global variable----------------------------*/

uint8_t	rtw_efuse_map_read(struct rtl_priv *rtlpriv, u16 addr, u16 cnts, uint8_t *data);

void	ReadEFuseByte(struct rtl_priv *rtlpriv, u16 _offset, uint8_t *pbuf) ;

uint8_t	EFUSE_Read1Byte(struct rtl_priv *rtlpriv, u16 Address);
void	EFUSE_ShadowRead(struct rtl_priv *rtlpriv, uint8_t Type, u16 Offset, u32 *Value);

uint8_t	efuse_OneByteRead(struct rtl_priv *rtlpriv, u16 addr, uint8_t *data);
uint8_t	efuse_OneByteWrite(struct rtl_priv *rtlpriv, u16 addr, uint8_t data);
void	rtw_efuse_shadow_map_update(struct rtl_priv *rtlpriv);

enum {
	VOLTAGE_V25	= 0x03,
	LDOE25_SHIFT	= 28 ,
	};

#endif
