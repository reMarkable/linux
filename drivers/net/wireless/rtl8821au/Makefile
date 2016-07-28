EXTRA_CFLAGS += -Wno-unused-label
EXTRA_CFLAGS += -Wno-unused
EXTRA_CFLAGS += -Wno-uninitialized
EXTRA_CFLAGS += -I$(src)/include

RTLWIFI_FILES :=	cam.o \
			debug.o \
			efuse.o \
			usb.o

RTL8821AU_FILES	:=	rtl8821au/dm.o \
			rtl8821au/fw.o \
			rtl8821au/hw.o \
			rtl8821au/led.o \
			rtl8821au/phy.o \
			rtl8821au/rf.o \
			rtl8821au/quirks.o \
			rtl8821au/sw.o \
			rtl8821au/table.o \
			rtl8821au/trx.o

OS_FILES :=		os_dep/osdep_service.o \
			os_dep/os_intfs.o \
			os_dep/usb_ops_linux.o \
			os_dep/ioctl_linux.o \
			os_dep/xmit_linux.o \
			os_dep/mlme_linux.o \
			os_dep/recv_linux.o \
			os_dep/usb_halinit.o 

HAL_FILES 	:=	hal/hal_intf.o \
			hal/hal_com.o \
			hal/odm.o \
			hal/HalPwrSeqCmd.o \
			hal/Hal8812PwrSeq.o \
			hal/Hal8821APwrSeq.o\
			hal/rtl8812a_hal_init.o \
			hal/rtl8812a_phycfg.o \
			hal/rtl8812a_dm.o \
			hal/rtl8812a_cmd.o \
			hal/rtl8812au_recv.o

CORE_FILES :=		core/rtw_cmd.o \
			core/rtw_security.o \
			core/rtw_ioctl_set.o \
			core/rtw_ieee80211.o \
			core/rtw_mlme.o \
			core/rtw_mlme_ext.o \
			core/rtw_wlan_util.o \
			core/rtw_vht.o \
			core/rtw_pwrctrl.o \
			core/rtw_rf.o \
			core/rtw_recv.o \
			core/rtw_sta_mgt.o \
			core/rtw_ap.o \
			core/rtw_xmit.o	\

rtl8821au-objs := 	$(OS_FILES) \
			$(CORE_FILES) \
			$(RTLWIFI_FILES) \
			$(HAL_FILES) \
			$(RTL8821AU_FILES)


obj-$(CONFIG_RTL8821AU) += rtl8821au.o

