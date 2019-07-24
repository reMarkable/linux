/*
 * MAX77818 Driver Core
 *
 * Copyright (C) 2014 Maxim Integrated
 * TaiEup Kim <clark.kim@maximintegrated.com>
 *
 * Copyright and License statement to be determined with Customer.
 * GNU Public License version 2 requires software code to be
 * publically open source if the code is to be statically linked with
 * the Linux kernel binary object.
 */

#ifndef __MAX77818_MFD_H__
#define __MAX77818_MFD_H__

/* MAX77818 Top Devices */
#define MAX77818_NAME                      	"max77818"

/* MAX77818 PMIC Devices */
#define MAX77818_REGULATOR_NAME				MAX77818_NAME"-regulator"
#define MAX77818_CHARGER_NAME				MAX77818_NAME"-charger"
#define MAX77818_FUELGAUGE_NAME             MAX77818_NAME"-fuelgauge"

/* Interrupt register & mask bit */
#define REG_INTSRC			0x22
#define REG_INTSRCMASK     	0x23
#define BIT_CHGR_INT        BIT (0)
#define BIT_FG_INT			BIT (1)
#define BIT_SYS_INT			BIT (2)

#define REG_SYSINTSRC      	0x24
#define REG_SYSINTMASK		0x26
#define BIT_SYSUVLO_INT		BIT (0)
#define BIT_SYSOVLO_INT		BIT (1)
#define BIT_TSHDN_INT		BIT (2)
#define BIT_TM_INT			BIT (7)

#define REG_CHARGER_INT			0xB0
#define REG_CHARGER_INT_MASK	0xB1

#define BIT_CHG_BYP_I		BIT (0)
#define BIT_CHG_BATP_I		BIT (2)
#define BIT_CHG_BAT_I		BIT (3)
#define BIT_CHG_CHG_I		BIT (4)
#define BIT_CHG_WCIN_I		BIT (5)
#define BIT_CHG_CHGIN_I		BIT (6)
#define BIT_CHG_AICL_I		BIT (7)

#define __sync_set(flag, lock) mutex_lock(&lock);flag = true;mutex_unlock(&lock)
#define __sync_clear(flag, lock) mutex_lock(&lock);flag = false;mutex_unlock(&lock)
#define __sync_get(flag, lock) ({ int value; mutex_lock(&lock);value = flag; mutex_unlock(&lock); value;})

/* Chip Interrupts */
enum {
	MAX77818_CHGR_INT = 0,
	MAX77818_FG_INT,
	MAX77818_SYS_INT,

	MAX77818_SYS_IRQ_START,
	MAX77818_SYS_IRQ_UVLO = MAX77818_SYS_IRQ_START,
	MAX77818_SYS_IRQ_OVLO,
	MAX77818_SYS_IRQ_TSHDN,
	MAX77818_SYS_IRQ_TM,

	MAX77818_CHG_IRQ_START,
	MAX77818_CHG_IRQ_BYP_I = MAX77818_CHG_IRQ_START,
	MAX77818_CHG_IRQ_BATP_I,
	MAX77818_CHG_IRQ_BAT_I,
	MAX77818_CHG_IRQ_CHG_I,
	MAX77818_CHG_IRQ_WCIN_I,
	MAX77818_CHG_IRQ_CHGIN_I,
	MAX77818_CHG_IRQ_AICL_I, 

    MAX77818_NUM_OF_INTS,
};

enum{
	SYS_IRQ_UVLO = 0,
	SYS_IRQ_OVLO,
	SYS_IRQ_TSHDN,
	SYS_IRQ_TM,

	CHG_IRQ_BYP_I = 0,
	CHG_IRQ_BATP_I,
	CHG_IRQ_BAT_I,
	CHG_IRQ_CHG_I,
	CHG_IRQ_WCIN_I,
	CHG_IRQ_CHGIN_I,
	CHG_IRQ_AICL_I,	

	FG_IRQ_ALERT = 0,
};



/*******************************************************************************
 * Useful Macros
 ******************************************************************************/

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
                                      ((_x) & 0x04 ? 2 : 3)) :\
                       ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
                                      ((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
        ((_x) ? __CONST_FFS(_x) : 0)

#undef  BIT_RSVD
#define BIT_RSVD  0

#undef  BITS
#define BITS(_end, _start) \
        ((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_word, _mask, _shift) \
        (((_word) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_word, _bit) \
        __BITS_GET(_word, _bit, FFS(_bit))

#undef  __BITS_SET
#define __BITS_SET(_word, _mask, _shift, _val) \
        (((_word) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef  BITS_SET
#define BITS_SET(_word, _bit, _val) \
        __BITS_SET(_word, _bit, FFS(_bit), _val)

#undef  BITS_MATCH
#define BITS_MATCH(_word, _bit) \
        (((_word) & (_bit)) == (_bit))

/*******************************************************************************
 * Sub Modules Support
 ******************************************************************************/
enum {
    MAX77818_DEV_REGULATOR = 0,
    MAX77818_DEV_CHARGER,
    MAX77818_DEV_FUELGAUGE,
    /***/
    MAX77818_DEV_NUM_OF_DEVICES,
};

 
struct max77818_dev {
	void						*pdata;
	struct mutex				lock;
	struct device				*dev;
	
	int 						irq;
	int 						irq_gpio;

	struct regmap_irq_chip_data	*irqc_intsrc;
	struct regmap_irq_chip_data	*irqc_sys;
	struct regmap_irq_chip_data	*irqc_chg;

	struct i2c_client			*pmic;	/* 0xCC , CLOGIC/SAFELDOS */
	struct i2c_client			*chg;	/* 0xD2, CHARGER */
	struct i2c_client			*fuel;	/* 0x6C, FUEL GAUGE */
	
	struct regmap				*regmap_pmic;	/* CLOGIC/SAFELDOS */
	struct regmap				*regmap_chg;	/* CHARGER */
	struct regmap				*regmap_fuel;	/* FUEL GAUGE */
};

/*******************************************************************************
 * Platform Data
 ******************************************************************************/

struct max77818_pmic_platform_data {
    int irq; /* system interrupt number for PMIC */
};


/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max77818_read (struct regmap *regmap, u8 addr, u8 *val);
int max77818_write (struct regmap *regmap, u8 addr, u8 val);
int max77818_fg_read (struct regmap *regmap, u8 addr, u16 *val);
int max77818_fg_write (struct regmap *regmap, u8 addr, u16 val);
int max77818_bulk_read (struct regmap *regmap, u8 addr, u8 *dst, u16 len);
int max77818_bulk_write (struct regmap *regmap, u8 addr, const u8 *src, u16 len);

/*******************************************************************************
 * Interrupt
 ******************************************************************************/
extern int max77818_irq_init(struct max77818_dev *max77818);
extern void max77818_irq_exit(struct max77818_dev *max77818);
extern int max77818_irq_resume(struct max77818_dev *max77818);

int max77818_map_irq(struct max77818_dev *max77818, int irq);

/*******************************************************************************
 * Debugging Stuff
 ******************************************************************************/

#undef  log_fmt
#define log_fmt(format) \
        DRIVER_NAME ": " format
#undef  log_err
#define log_err(format, ...) \
        printk(KERN_ERR log_fmt(format), ##__VA_ARGS__)
#undef  log_warn
#define log_warn(format, ...) \
        printk(KERN_WARNING log_fmt(format), ##__VA_ARGS__)
#undef  log_info
#define log_info(format, ...) \
        if (likely(log_level >= 0)) {\
            printk(KERN_INFO log_fmt(format), ##__VA_ARGS__);\
        }
#undef  log_dbg
#define log_dbg(format, ...) \
        if (likely(log_level >= 1)) {\
            printk(KERN_DEFAULT log_fmt(format), ##__VA_ARGS__);\
        }
#undef  log_vdbg
#define log_vdbg(format, ...) \
        if (likely(log_level >= 2)) {\
            printk(KERN_DEFAULT log_fmt(format), ##__VA_ARGS__);\
        }
#endif /* !__MAX77818_MFD_H__ */

