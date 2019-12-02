
cdns_mhdp_drmcore-y := cdns-mhdp-common.o cdns-mhdp-hdmi.o

cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_HDMI) += cdns-hdmi-core.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_DP) += cdns-dp-core.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_AUDIO) += cdns-mhdp-audio.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_HDMI_CEC) += cdns-mhdp-cec.o

obj-$(CONFIG_DRM_CDNS_MHDP)		+= cdns_mhdp_drmcore.o
