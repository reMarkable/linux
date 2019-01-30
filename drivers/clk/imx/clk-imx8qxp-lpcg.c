// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 *	Dong Aisheng <aisheng.dong@nxp.com>
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "clk-scu.h"
#include "clk-imx8qxp-lpcg.h"

#include <dt-bindings/clock/imx8-clock.h>

/*
 * struct imx8qxp_lpcg_data - Description of one LPCG clock
 * @id: clock ID
 * @name: clock name
 * @parent: parent clock name
 * @flags: common clock flags
 * @offset: offset of this LPCG clock
 * @bit_idx: bit index of this LPCG clock
 * @hw_gate: whether supports HW autogate
 *
 * This structure describes one LPCG clock
 */
struct imx8qxp_lpcg_data {
	int id;
	char *name;
	char *parent;
	unsigned long flags;
	u32 offset;
	u8 bit_idx;
	bool hw_gate;
};

/*
 * struct imx8qxp_ss_lpcg - Description of one subsystem LPCG clocks
 * @lpcg: LPCG clocks array of one subsystem
 * @num_lpcg: the number of LPCG clocks
 * @num_max: the maximum number of LPCG clocks
 *
 * This structure describes each subsystem LPCG clocks information
 * which then will be used to create respective LPCGs clocks
 */
struct imx8qxp_ss_lpcg {
	const struct imx8qxp_lpcg_data *lpcg;
	u8 num_lpcg;
	u8 num_max;
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_cm40[] = {
	{ IMX_CM40_LPCG_I2C_IPG_CLK, "cm40_lpcg_i2c_ipg_clk", "cm40_ipg_clk_root", 0, CM40_I2C_LPCG, 16, 0, },
	{ IMX_CM40_LPCG_I2C_CLK, "cm40_lpcg_i2c_clk", "cm40_i2c_div", 0, CM40_I2C_LPCG, 0, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_cm40 = {
	.lpcg = imx8qxp_lpcg_cm40,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_cm40),
	.num_max = IMX_CM40_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_adma[] = {
	{ IMX_ADMA_LPCG_SPI0_IPG_CLK, "spi0_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPSPI_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SPI0_CLK, "spi0_lpcg_clk", "spi0_clk", 0, ADMA_LPSPI_0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SPI2_IPG_CLK, "spi2_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPSPI_2_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SPI2_CLK, "spi2_lpcg_clk", "spi2_clk", 0, ADMA_LPSPI_2_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_UART0_IPG_CLK, "uart0_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPUART_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_UART0_BAUD_CLK, "uart0_lpcg_baud_clk", "uart0_clk", 0, ADMA_LPUART_0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_UART1_IPG_CLK, "uart1_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPUART_1_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_UART1_BAUD_CLK, "uart1_lpcg_baud_clk", "uart1_clk", 0, ADMA_LPUART_1_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_UART2_IPG_CLK, "uart2_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPUART_2_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_UART2_BAUD_CLK, "uart2_lpcg_baud_clk", "uart2_clk", 0, ADMA_LPUART_2_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_UART3_IPG_CLK, "uart3_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPUART_3_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_UART3_BAUD_CLK, "uart3_lpcg_baud_clk", "uart3_clk", 0, ADMA_LPUART_3_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_I2C0_IPG_CLK, "i2c0_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPI2C_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_I2C0_CLK, "i2c0_lpcg_clk", "i2c0_clk", 0, ADMA_LPI2C_0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_I2C1_IPG_CLK, "i2c1_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPI2C_1_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_I2C1_CLK, "i2c1_lpcg_clk", "i2c1_clk", 0, ADMA_LPI2C_1_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_I2C2_IPG_CLK, "i2c2_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPI2C_2_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_I2C2_CLK, "i2c2_lpcg_clk", "i2c2_clk", 0, ADMA_LPI2C_2_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_I2C3_IPG_CLK, "i2c3_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_LPI2C_3_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_I2C3_CLK, "i2c3_lpcg_clk", "i2c3_clk", 0, ADMA_LPI2C_3_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_CAN0_IPG_CHI_CLK, "can0_lpcg_chi_clk", "dma_ipg_clk_root", 0, ADMA_FLEXCAN_0_LPCG, 20, 0, },
	{ IMX_ADMA_LPCG_CAN0_IPG_CLK, "can0_lpcg_ipg_clk", "can0_lpcg_chi_clk", 0, ADMA_FLEXCAN_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_CAN0_IPG_PE_CLK, "can0_lpcg_pe_clk", "can0_clk", 0, ADMA_FLEXCAN_0_LPCG, 0, 0, },

	{ IMX_ADMA_LPCG_DSP_CORE_CLK, "dsp_lpcg_core_clk", "dma_ipg_clk_root", 0, ADMA_HIFI_LPCG, 28, 0, },
	{ IMX_ADMA_LPCG_DSP_IPG_CLK, "dsp_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_HIFI_LPCG, 20, 0, },
	{ IMX_ADMA_LPCG_DSP_ADB_CLK, "dsp_lpcg_adb_clk", "dma_ipg_clk_root", 0, ADMA_HIFI_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_OCRAM_IPG_CLK, "ocram_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_OCRAM_LPCG, 16, 0, },

	{ IMX_ADMA_LPCG_AUD_PLL_DIV_CLK0_CLK, "aud_pll_div_clk0_lpcg_clk", "audio_pll_div_clk0_clk", 0, ADMA_PLL_CLK0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_AUD_PLL_DIV_CLK1_CLK, "aud_pll_div_clk1_lpcg_clk", "audio_pll_div_clk1_clk", 0, ADMA_PLL_CLK1_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_AUD_REC_CLK0_CLK, "aud_rec_clk0_lpcg_clk", "audio_rec_clk0_clk", 0, ADMA_REC_CLK0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_AUD_REC_CLK1_CLK, "aud_rec_clk1_lpcg_clk", "audio_rec_clk1_clk", 0, ADMA_REC_CLK1_LPCG, 0, 0, },

	{ IMX_ADMA_LPCG_AMIX_IPG_CLK, "amix_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_AMIX_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_ESAI_0_IPG_CLK, "esai0_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_ESAI_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_ESAI_0_EXTAL_CLK, "esai0_lpcg_extal_clk", "acm_esai0_mclk_sel", 0, ADMA_ESAI_0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SAI_0_IPG_CLK, "sai0_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_SAI_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SAI_0_MCLK, "sai0_lpcg_mclk", "acm_sai0_mclk_sel", 0, ADMA_SAI_0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SAI_1_IPG_CLK, "sai1_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_SAI_1_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SAI_1_MCLK, "sai1_lpcg_mclk", "acm_sai1_mclk_sel", 0, ADMA_SAI_1_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SAI_2_IPG_CLK, "sai2_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_SAI_2_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SAI_3_MCLK, "sai2_lpcg_mclk", "acm_sai2_mclk_sel", 0, ADMA_SAI_2_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SAI_3_IPG_CLK, "sai3_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_SAI_3_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SAI_3_MCLK, "sai3_lpcg_mclk", "acm_sai3_mclk_sel", 0, ADMA_SAI_3_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SAI_4_IPG_CLK, "sai4_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_SAI_4_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SAI_4_MCLK, "sai4_lpcg_mclk", "acm_sai4_mclk_sel", 0, ADMA_SAI_4_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SAI_5_IPG_CLK, "sai5_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_SAI_5_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SAI_5_MCLK, "sai5_lpcg_mclk", "acm_sai5_mclk_sel", 0, ADMA_SAI_5_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_MQS_IPG_CLK, "mqs_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_MQS_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_MQS_MCLK, "mqs_lpcg_mclk", "acm_mqs_mclk_sel", 0, ADMA_MQS_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_GPT5_IPG_CLK, "gpt5_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_GPT_5_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_GPT5_CLKIN, "gpt5_lpcg_clkin", "dma_ipg_clk_root", 0, ADMA_GPT_5_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_GPT6_IPG_CLK, "gpt6_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_GPT_6_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_GPT6_CLKIN, "gpt6_lpcg_clkin", "dma_ipg_clk_root", 0, ADMA_GPT_6_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_GPT7_IPG_CLK, "gpt7_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_GPT_7_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_GPT7_CLKIN, "gpt7_lpcg_clkin", "dma_ipg_clk_root", 0, ADMA_GPT_7_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_GPT8_IPG_CLK, "gpt8_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_GPT_8_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_GPT8_CLKIN, "gpt8_lpcg_clkin", "dma_ipg_clk_root", 0, ADMA_GPT_8_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_GPT9_IPG_CLK, "gpt9_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_GPT_9_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_GPT9_CLKIN, "gpt9_lpcg_clkin", "dma_ipg_clk_root", 0, ADMA_GPT_9_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_GPT10_IPG_CLK, "gpt10_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_GPT_10_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_GPT10_CLKIN, "gpt10_lpcg_clkin", "dma_ipg_clk_root", 0, ADMA_GPT_10_LPCG, 0, 0, },

	{ IMX_ADMA_LPCG_MCLKOUT0, "mclkout0_lpcg", "acm_mclkout0_sel", 0, ADMA_MCLKOUT0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_MCLKOUT1, "mclkout1_lpcg", "acm_mclkout1_sel", 0, ADMA_MCLKOUT1_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_SPDIF_0_GCLKW, "spdif0_lpcg_gclkw", "dma_ipg_clk_root", 0, ADMA_SPDIF_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_SPDIF_0_TX_CLK, "spdif0_lpcg_tx_clk", "acm_spdif0_mclk_sel", 0, ADMA_SPDIF_0_LPCG, 0, 0, },
	{ IMX_ADMA_LPCG_ASRC_0_IPG_CLK, "asrc0_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_ASRC_0_LPCG, 16, 0, },
	{ IMX_ADMA_LPCG_ASRC_1_IPG_CLK, "asrc1_lpcg_ipg_clk", "dma_ipg_clk_root", 0, ADMA_ASRC_1_LPCG, 16, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_adma = {
	.lpcg = imx8qxp_lpcg_adma,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_adma),
	.num_max = IMX_ADMA_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_conn[] = {
	{ IMX_CONN_LPCG_SDHC0_PER_CLK, "sdhc0_lpcg_per_clk", "sdhc0_clk", 0, CONN_USDHC_0_LPCG, 0, 0, },
	{ IMX_CONN_LPCG_SDHC0_IPG_CLK, "sdhc0_lpcg_ipg_clk", "conn_ipg_clk_root", 0, CONN_USDHC_0_LPCG, 16, 0, },
	{ IMX_CONN_LPCG_SDHC0_HCLK, "sdhc0_lpcg_ahb_clk", "conn_axi_clk_root", 0, CONN_USDHC_0_LPCG, 20, 0, },
	{ IMX_CONN_LPCG_SDHC1_PER_CLK, "sdhc1_lpcg_per_clk", "sdhc1_clk", 0, CONN_USDHC_1_LPCG, 0, 0, },
	{ IMX_CONN_LPCG_SDHC1_IPG_CLK, "sdhc1_lpcg_ipg_clk", "conn_ipg_clk_root", 0, CONN_USDHC_1_LPCG, 16, 0, },
	{ IMX_CONN_LPCG_SDHC1_HCLK, "sdhc1_lpcg_ahb_clk", "conn_axi_clk_root", 0, CONN_USDHC_1_LPCG, 20, 0, },
	{ IMX_CONN_LPCG_SDHC2_PER_CLK, "sdhc2_lpcg_per_clk", "sdhc2_clk", 0, CONN_USDHC_2_LPCG, 0, 0, },
	{ IMX_CONN_LPCG_SDHC2_IPG_CLK, "sdhc2_lpcg_ipg_clk", "conn_ipg_clk_root", 0, CONN_USDHC_2_LPCG, 16, 0, },
	{ IMX_CONN_LPCG_SDHC2_HCLK, "sdhc2_lpcg_ahb_clk", "conn_axi_clk_root", 0, CONN_USDHC_2_LPCG, 20, 0, },
	{ IMX_CONN_LPCG_ENET0_ROOT_CLK, "enet0_ipg_root_clk", "enet0_clk", 0, CONN_ENET_0_LPCG, 0, 0, },
	{ IMX_CONN_LPCG_ENET0_TX_CLK, "enet0_tx_clk", "enet0_clk", 0, CONN_ENET_0_LPCG, 4, 0, },
	{ IMX_CONN_LPCG_ENET0_AHB_CLK, "enet0_ahb_clk", "conn_axi_clk_root", 0, CONN_ENET_0_LPCG, 8, 0, },
	{ IMX_CONN_LPCG_ENET0_IPG_S_CLK, "enet0_ipg_s_clk", "conn_ipg_clk_root", 0, CONN_ENET_0_LPCG, 20, 0, },
	{ IMX_CONN_LPCG_ENET0_IPG_CLK, "enet0_ipg_clk", "enet0_ipg_s_clk", 0, CONN_ENET_0_LPCG, 16, 0, },
	{ IMX_CONN_LPCG_ENET1_ROOT_CLK, "enet1_ipg_root_clk", "enet1_clk", 0, CONN_ENET_1_LPCG, 0, 0, },
	{ IMX_CONN_LPCG_ENET1_TX_CLK, "enet1_tx_clk", "enet1_clk", 0, CONN_ENET_1_LPCG, 4, 0, },
	{ IMX_CONN_LPCG_ENET1_AHB_CLK, "enet1_ahb_clk", "conn_axi_clk_root", 0, CONN_ENET_1_LPCG, 8, 0, },
	{ IMX_CONN_LPCG_ENET1_IPG_S_CLK, "enet1_ipg_s_clk", "conn_ipg_clk_root", 0, CONN_ENET_1_LPCG, 20, 0, },
	{ IMX_CONN_LPCG_ENET1_IPG_CLK, "enet1_ipg_clk", "enet0_ipg_s_clk", 0, CONN_ENET_1_LPCG, 16, 0, },
	{ IMX_CONN_LPCG_USB2_PHY_IPG_CLK, "usboh3_phy_ipg_clk", "conn_ipg_clk_root", 0, CONN_USB_2_LPCG, 28, 0, },
	{ IMX_CONN_LPCG_USB2_AHB_CLK, "usboh3_ahb_clk", "conn_ahb_clk_root", 0, CONN_USB_2_LPCG, 24, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_conn = {
	.lpcg = imx8qxp_lpcg_conn,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_conn),
	.num_max = IMX_CONN_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_lsio[] = {
	{ IMX_LSIO_LPCG_PWM0_IPG_CLK, "pwm0_lpcg_ipg_clk", "pwm0_clk", 0, LSIO_PWM_0_LPCG, 0, 0, },
	{ IMX_LSIO_LPCG_PWM0_IPG_HF_CLK, "pwm0_lpcg_ipg_hf_clk", "pwm0_clk", 0, LSIO_PWM_0_LPCG, 4, 0, },
	{ IMX_LSIO_LPCG_PWM0_IPG_S_CLK, "pwm0_lpcg_ipg_s_clk", "pwm0_clk", 0, LSIO_PWM_0_LPCG, 16, 0, },
	{ IMX_LSIO_LPCG_PWM0_IPG_SLV_CLK, "pwm0_lpcg_ipg_slv_clk", "lsio_bus_clk_root", 0, LSIO_PWM_0_LPCG, 20, 0, },
	{ IMX_LSIO_LPCG_PWM0_IPG_MSTR_CLK, "pwm0_lpcg_ipg_mstr_clk", "pwm0_clk", 0, LSIO_PWM_0_LPCG, 24, 0, },
	{ IMX_LSIO_LPCG_PWM1_IPG_CLK, "pwm1_lpcg_ipg_clk", "pwm1_clk", 0, LSIO_PWM_1_LPCG, 0, 0, },
	{ IMX_LSIO_LPCG_PWM1_IPG_HF_CLK, "pwm1_lpcg_ipg_hf_clk", "pwm1_clk", 0, LSIO_PWM_1_LPCG, 4, 0, },
	{ IMX_LSIO_LPCG_PWM1_IPG_S_CLK, "pwm1_lpcg_ipg_s_clk", "pwm1_clk", 0, LSIO_PWM_1_LPCG, 16, 0, },
	{ IMX_LSIO_LPCG_PWM1_IPG_SLV_CLK, "pwm1_lpcg_ipg_slv_clk", "lsio_bus_clk_root", 0, LSIO_PWM_1_LPCG, 20, 0, },
	{ IMX_LSIO_LPCG_PWM1_IPG_MSTR_CLK, "pwm1_lpcg_ipg_mstr_clk", "pwm1_clk", 0, LSIO_PWM_1_LPCG, 24, 0, },
	{ IMX_LSIO_LPCG_PWM2_IPG_CLK, "pwm2_lpcg_ipg_clk", "pwm2_clk", 0, LSIO_PWM_2_LPCG, 0, 0, },
	{ IMX_LSIO_LPCG_PWM2_IPG_HF_CLK, "pwm2_lpcg_ipg_hf_clk", "pwm2_clk", 0, LSIO_PWM_2_LPCG, 4, 0, },
	{ IMX_LSIO_LPCG_PWM2_IPG_S_CLK, "pwm2_lpcg_ipg_s_clk", "pwm2_clk", 0, LSIO_PWM_2_LPCG, 16, 0, },
	{ IMX_LSIO_LPCG_PWM2_IPG_SLV_CLK, "pwm2_lpcg_ipg_slv_clk", "lsio_bus_clk_root", 0, LSIO_PWM_2_LPCG, 20, 0, },
	{ IMX_LSIO_LPCG_PWM2_IPG_MSTR_CLK, "pwm2_lpcg_ipg_mstr_clk", "pwm2_clk", 0, LSIO_PWM_2_LPCG, 24, 0, },
	{ IMX_LSIO_LPCG_PWM3_IPG_CLK, "pwm3_lpcg_ipg_clk", "pwm3_clk", 0, LSIO_PWM_3_LPCG, 0, 0, },
	{ IMX_LSIO_LPCG_PWM3_IPG_HF_CLK, "pwm3_lpcg_ipg_hf_clk", "pwm3_clk", 0, LSIO_PWM_3_LPCG, 4, 0, },
	{ IMX_LSIO_LPCG_PWM3_IPG_S_CLK, "pwm3_lpcg_ipg_s_clk", "pwm3_clk", 0, LSIO_PWM_3_LPCG, 16, 0, },
	{ IMX_LSIO_LPCG_PWM3_IPG_SLV_CLK, "pwm3_lpcg_ipg_slv_clk", "lsio_bus_clk_root", 0, LSIO_PWM_3_LPCG, 20, 0, },
	{ IMX_LSIO_LPCG_PWM3_IPG_MSTR_CLK, "pwm3_lpcg_ipg_mstr_clk", "pwm3_clk", 0, LSIO_PWM_3_LPCG, 24, 0, },
	{ IMX_LSIO_LPCG_PWM4_IPG_CLK, "pwm4_lpcg_ipg_clk", "pwm4_clk", 0, LSIO_PWM_4_LPCG, 0, 0, },
	{ IMX_LSIO_LPCG_PWM4_IPG_HF_CLK, "pwm4_lpcg_ipg_hf_clk", "pwm4_clk", 0, LSIO_PWM_4_LPCG, 4, 0, },
	{ IMX_LSIO_LPCG_PWM4_IPG_S_CLK, "pwm4_lpcg_ipg_s_clk", "pwm4_clk", 0, LSIO_PWM_4_LPCG, 16, 0, },
	{ IMX_LSIO_LPCG_PWM4_IPG_SLV_CLK, "pwm4_lpcg_ipg_slv_clk", "lsio_bus_clk_root", 0, LSIO_PWM_4_LPCG, 20, 0, },
	{ IMX_LSIO_LPCG_PWM4_IPG_MSTR_CLK, "pwm4_lpcg_ipg_mstr_clk", "pwm4_clk", 0, LSIO_PWM_4_LPCG, 24, 0, },
	{ IMX_LSIO_LPCG_PWM5_IPG_CLK, "pwm5_lpcg_ipg_clk", "pwm5_clk", 0, LSIO_PWM_5_LPCG, 0, 0, },
	{ IMX_LSIO_LPCG_PWM5_IPG_HF_CLK, "pwm5_lpcg_ipg_hf_clk", "pwm5_clk", 0, LSIO_PWM_5_LPCG, 4, 0, },
	{ IMX_LSIO_LPCG_PWM5_IPG_S_CLK, "pwm5_lpcg_ipg_s_clk", "pwm5_clk", 0, LSIO_PWM_5_LPCG, 16, 0, },
	{ IMX_LSIO_LPCG_PWM5_IPG_SLV_CLK, "pwm5_lpcg_ipg_slv_clk", "lsio_bus_clk_root", 0, LSIO_PWM_5_LPCG, 20, 0, },
	{ IMX_LSIO_LPCG_PWM5_IPG_MSTR_CLK, "pwm5_lpcg_ipg_mstr_clk", "pwm5_clk", 0, LSIO_PWM_5_LPCG, 24, 0, },
	{ IMX_LSIO_LPCG_PWM6_IPG_CLK, "pwm6_lpcg_ipg_clk", "pwm6_clk", 0, LSIO_PWM_6_LPCG, 0, 0, },
	{ IMX_LSIO_LPCG_PWM6_IPG_HF_CLK, "pwm6_lpcg_ipg_hf_clk", "pwm6_clk", 0, LSIO_PWM_6_LPCG, 4, 0, },
	{ IMX_LSIO_LPCG_PWM6_IPG_S_CLK, "pwm6_lpcg_ipg_s_clk", "pwm6_clk", 0, LSIO_PWM_6_LPCG, 16, 0, },
	{ IMX_LSIO_LPCG_PWM6_IPG_SLV_CLK, "pwm6_lpcg_ipg_slv_clk", "lsio_bus_clk_root", 0, LSIO_PWM_6_LPCG, 20, 0, },
	{ IMX_LSIO_LPCG_PWM6_IPG_MSTR_CLK, "pwm6_lpcg_ipg_mstr_clk", "pwm6_clk", 0, LSIO_PWM_6_LPCG, 24, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_lsio = {
	.lpcg = imx8qxp_lpcg_lsio,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_lsio),
	.num_max = IMX_LSIO_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_hsio[] = {
	{ IMX_HSIO_LPCG_PCIEB_MSTR_AXI_CLK, "hsio_pcie_mstr_axi_clk", "hsio_axi_clk_root", 0, HSIO_PCIEB_LPCG, 16, 0, },
	{ IMX_HSIO_LPCG_PCIEB_SLV_AXI_CLK, "hsio_pcie_slv_axi_clk", "hsio_axi_clk_root", 0, HSIO_PCIEB_LPCG, 20, 0, },
	{ IMX_HSIO_LPCG_PCIEB_DBI_AXI_CLK, "hsio_pcie_dbi_axi_clk", "hsio_axi_clk_root", 0, HSIO_PCIEB_LPCG, 24, 0, },
	{ IMX_HSIO_LPCG_PHYX1_PCLK, "hsio_pcie_phyx1_pclk", "dummy", 0, HSIO_PHYX1_LPCG, 0, 0, },
	{ IMX_HSIO_LPCG_PHYX1_APB_CLK, "hsio_pcie_phyx1_apb_clk", "hsio_per_clk_root", 0, HSIO_PHYX1_LPCG, 16, 0, },
	{ IMX_HSIO_LPCG_PHYX1_PER_CLK, "hsio_phyx1_per_clk", "hsio_per_clk_root", 0, HSIO_CRR_1_LPCG, 16, 0, },
	{ IMX_HSIO_LPCG_PCIEB_PER_CLK, "hsio_pcieb_per_clk", "hsio_per_clk_root", 0, HSIO_CRR_3_LPCG, 16, 0, },
	{ IMX_HSIO_LPCG_MISC_PER_CLK, "hsio_misc_per_clk", "hsio_per_clk_root", 0, HSIO_CRR_5_LPCG, 16, 0, },
	{ IMX_HSIO_LPCG_GPIO_PER_CLK, "hsio_gpio_per_clk", "hsio_per_clk_root", 0, HSIO_GPIO_LPCG, 16, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_hsio = {
	.lpcg = imx8qxp_lpcg_hsio,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_hsio),
	.num_max = IMX_HSIO_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_img[] = {
	{ IMX_IMG_LPCG_PDMA0_CLK, "img_lpcg_pdma0_clk", "img_pxl_clk_root", 0, IMG_PDMA0_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_PDMA1_CLK, "img_lpcg_pdma1_clk", "img_pxl_clk_root", 0, IMG_PDMA1_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_PDMA2_CLK, "img_lpcg_pdma2_clk", "img_pxl_clk_root", 0, IMG_PDMA2_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_PDMA3_CLK, "img_lpcg_pdma3_clk", "img_pxl_clk_root", 0, IMG_PDMA3_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_PDMA4_CLK, "img_lpcg_pdma4_clk", "img_pxl_clk_root", 0, IMG_PDMA4_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_PDMA5_CLK, "img_lpcg_pdma5_clk", "img_pxl_clk_root", 0, IMG_PDMA5_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_PDMA6_CLK, "img_lpcg_pdma6_clk", "img_pxl_clk_root", 0, IMG_PDMA6_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_PDMA7_CLK, "img_lpcg_pdma7_clk", "img_pxl_clk_root", 0, IMG_PDMA7_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_CSI0_PXL_LINK_CLK, "img_lpcg_csi0_pxl_link_clk", "img_pxl_clk_root", 0, IMG_MIPI_CSI0_LPCG, 0, 0, },
	{ IMX_IMG_LPCG_CSI1_PXL_LINK_CLK, "img_lpcg_csi1_pxl_link_clk", "img_pxl_clk_root", 0, IMG_MIPI_CSI1_LPCG, 0, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_img = {
	.lpcg = imx8qxp_lpcg_img,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_img),
	.num_max = IMX_IMG_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_csi0[] = {
	{ IMX_CSI_LPCG_CSI0_CORE_CLK, "csi_lpcg_csi0_core_clk", "mipi_csi0_core_clk", 0, CSI_CSI0_CORE_LPCG, 16, 0, },
	{ IMX_CSI_LPCG_CSI0_ESC_CLK, "csi_lpcg_csi0_esc_clk", "mipi_csi0_esc_clk", 0, CSI_CSI0_ESC_LPCG, 16, 0, },
	{ IMX_CSI_LPCG_CSI0_I2C0_CLK, "csi_lpcg_csi0_i2c0_clk", "mipi_csi0_i2c0_clk", 0, CSI_CSI0_I2C0_LPCG, 0, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_csi0 = {
	.lpcg = imx8qxp_lpcg_csi0,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_csi0),
	.num_max = IMX_CSI_LPCG_CSI0_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_csi1[] = {
	{ IMX_CSI_LPCG_CSI1_CORE_CLK, "csi_lpcg_csi1_core_clk", "mipi_csi1_core_clk", 0, CSI_CSI1_CORE_LPCG, 16, 0, },
	{ IMX_CSI_LPCG_CSI1_ESC_CLK, "csi_lpcg_csi1_esc_clk", "mipi_csi1_esc_clk", 0, CSI_CSI1_ESC_LPCG, 16, 0, },
	{ IMX_CSI_LPCG_CSI1_I2C0_CLK, "csi_lpcg_csi1_i2c0_clk", "mipi_csi1_i2c0_clk", 0, CSI_CSI1_I2C0_LPCG, 0, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_csi1 = {
	.lpcg = imx8qxp_lpcg_csi1,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_csi1),
	.num_max = IMX_CSI_LPCG_CSI1_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_dc[] = {
	{ IMX_DC0_LPCG_PRG0_RTRAM_CLK, "dc0_lpcg_prg0_rtram_clk", "dc_axi_ext_clk_root", 0, 0x20, 0, 0, },
	{ IMX_DC0_LPCG_PRG0_APB_CLK, "dc0_lpcg_prg0_apb_clk", "dc_cfg_clk_root", 0, 0x20, 16, 0, },
	{ IMX_DC0_LPCG_PRG1_RTRAM_CLK, "dc0_lpcg_prg1_rtram_clk", "dc_axi_ext_clk_root", 0, 0x24, 0, 0, },
	{ IMX_DC0_LPCG_PRG1_APB_CLK, "dc0_lpcg_prg1_apb_clk", "dc_cfg_clk_root", 0, 0x24, 16, 0, },
	{ IMX_DC0_LPCG_PRG2_RTRAM_CLK, "dc0_lpcg_prg2_rtram_clk", "dc_axi_ext_clk_root", 0, 0x28, 0, 0, },
	{ IMX_DC0_LPCG_PRG2_APB_CLK, "dc0_lpcg_prg2_apb_clk", "dc_cfg_clk_root", 0, 0x28, 16, 0, },
	{ IMX_DC0_LPCG_PRG3_RTRAM_CLK, "dc0_lpcg_prg3_rtram_clk", "dc_axi_ext_clk_root", 0, 0x34, 0, 0, },
	{ IMX_DC0_LPCG_PRG3_APB_CLK, "dc0_lpcg_prg3_apb_clk", "dc_cfg_clk_root", 0, 0x34, 16, 0, },
	{ IMX_DC0_LPCG_PRG4_RTRAM_CLK, "dc0_lpcg_prg4_rtram_clk", "dc_axi_ext_clk_root", 0, 0x38, 0, 0, },
	{ IMX_DC0_LPCG_PRG4_APB_CLK, "dc0_lpcg_prg4_apb_clk", "dc_cfg_clk_root", 0, 0x38, 16, 0, },
	{ IMX_DC0_LPCG_PRG5_RTRAM_CLK, "dc0_lpcg_prg5_rtram_clk", "dc_axi_ext_clk_root", 0, 0x3c, 0, 0, },
	{ IMX_DC0_LPCG_PRG5_APB_CLK, "dc0_lpcg_prg5_apb_clk", "dc_cfg_clk_root", 0, 0x3c, 16, 0, },
	{ IMX_DC0_LPCG_PRG6_RTRAM_CLK, "dc0_lpcg_prg6_rtram_clk", "dc_axi_ext_clk_root", 0, 0x40, 0, 0, },
	{ IMX_DC0_LPCG_PRG6_APB_CLK, "dc0_lpcg_prg6_apb_clk", "dc_cfg_clk_root", 0, 0x40, 16, 0, },
	{ IMX_DC0_LPCG_PRG7_RTRAM_CLK, "dc0_lpcg_prg7_rtram_clk", "dc_axi_ext_clk_root", 0, 0x44, 0, 0, },
	{ IMX_DC0_LPCG_PRG7_APB_CLK, "dc0_lpcg_prg7_apb_clk", "dc_cfg_clk_root", 0, 0x44, 16, 0, },
	{ IMX_DC0_LPCG_PRG8_RTRAM_CLK, "dc0_lpcg_prg8_rtram_clk", "dc_axi_ext_clk_root", 0, 0x48, 0, 0, },
	{ IMX_DC0_LPCG_PRG8_APB_CLK, "dc0_lpcg_prg8_apb_clk", "dc_cfg_clk_root", 0, 0x48, 16, 0, },

	{ IMX_DC0_LPCG_DPR0_APB_CLK, "dc0_lpcg_dpr0_apb_clk", "dc_cfg_clk_root", 0, 0x18, 16, 0, },
	{ IMX_DC0_LPCG_DPR0_B_CLK, "dc0_lpcg_drp0_b_clk", "dc_axi_ext_clk_root", 0, 0x18, 20, 0, },
	{ IMX_DC0_LPCG_RTRAM0_CLK, "dc0_lpcg_rtram0_clk", "dc_axi_int_clk_root", 0, 0x1c, 0, 0, },
	{ IMX_DC0_LPCG_DPR1_APB_CLK, "dc0_lpcg_dpr1_apb_clk", "dc_cfg_clk_root", 0, 0x2c, 16, 0, },
	{ IMX_DC0_LPCG_DPR1_B_CLK, "dc0_lpcg_drp1_b_clk", "dc_axi_ext_clk_root", 0, 0x2c, 20, 0, },
	{ IMX_DC0_LPCG_RTRAM1_CLK, "dc0_lpcg_rtram1_clk", "dc_axi_int_clk_root", 0, 0x30, 0, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_dc = {
	.lpcg = imx8qxp_lpcg_dc,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_dc),
	.num_max = IMX_DC0_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_mipi0[] = {
	{ IMX_MIPI0_LPCG_I2C0_CLK, "mipi0_lpcg_i2c0_clk", "mipi0_i2c0_clk", 0, 0x14, 0, 0, },
	{ IMX_MIPI0_LPCG_I2C1_CLK, "mipi0_lpcg_i2c1_clk", "mipi0_i2c1_clk", 0, 0x14, 0, 0, }, //FIXME: same LPCG offset as I2C0
	{ IMX_MIPI0_LPCG_I2C0_IPG_S_CLK, "mipi0_lpcg_i2c0_ipg_s", "mipi_ipg_clk_root", 0, 0x10, 0, 0, },
	{ IMX_MIPI0_LPCG_I2C0_IPG_CLK, "mipi0_lpcg_i2c0_ipg_clk", "mipi0_lpcg_i2c0_ipg_s", 0, 0, 0, 0, },
	{ IMX_MIPI0_LPCG_I2C1_IPG_S_CLK, "mipi0_lpcg_i2c1_ipg_s", "mipi_ipg_clk_root", 0, 0x14, 0, 0, },
	{ IMX_MIPI0_LPCG_I2C1_IPG_CLK, "mipi0_lpcg_i2c1_ipg_clk", "mipi0_lpcg_i2c1_ipg_s", 0, 0, 0, 0, }, //FIXME: same LPCG offset as I2C0
	{ IMX_MIPI0_LPCG_PWM_IPG_S_CLK, "mipi0_lpcg_pwm_ipg_s", "mipi_ipg_clk_root", 0, 0xc, 16, 0, },
	{ IMX_MIPI0_LPCG_PWM_IPG_CLK, "mipi0_lpcg_pwm_ipg_cl", "mipi0_lpcg_pwm_ipg_s", 0, 0xc, 16, 0, }, //FIXME: same LPCG offset as IPG_S
	{ IMX_MIPI0_LPCG_PWM_32K_CLK, "mipi0_lpcg_pwm_32K_clk", "xtal_32KHz", 0, 0xc, 0, 0, },
	{ IMX_MIPI0_LPCG_PWM_CLK, "mipi0_lpcg_pwm_clk", "mipi0_pwm_clk", 0, 0xc, 0, 0, }, //FIXME: same LPCG offset as 32K_CLK
	{ IMX_MIPI0_LPCG_GPIO_IPG_CLK, "mipi0_lpcg_gpio_ipg_clk", "mipi_ipg_clk_root", 0, 0x8, 0, 0, },
	{ IMX_MIPI0_LPCG_LIS_IPG_CLK, "mipi0_lis_ipg_clk", "mipi_ipg_clk_root", 0, 0, 16, 0, },
};

static const struct imx8qxp_ss_lpcg imx8qxp_ss_mipi0 = {
	.lpcg = imx8qxp_lpcg_mipi0,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_mipi0),
	.num_max = IMX_MIPI0_LPCG_CLK_END,
};

static const struct imx8qxp_lpcg_data imx8qxp_lpcg_mipi1[] = {
	{ IMX_MIPI1_LPCG_I2C0_CLK, "mipi1_lpcg_i2c0_clk", "mipi1_i2c0_clk", 0, 0x14, 0, 0, },
	{ IMX_MIPI1_LPCG_I2C1_CLK, "mipi1_lpcg_i2c1_clk", "mipi1_i2c1_clk", 0, 0x14, 0, 0, }, //FIXME: same LPCG offset as I2C0
	{ IMX_MIPI1_LPCG_I2C0_IPG_S_CLK, "mipi1_lpcg_i2c0_ipg_s", "mipi_ipg_clk_root", 0, 0x10, 0, 0, },
	{ IMX_MIPI1_LPCG_I2C0_IPG_CLK, "mipi1_lpcg_i2c0_ipg_clk", "mipi1_lpcg_i2c0_ipg_s", 0, 0, 0, 0, },
	{ IMX_MIPI1_LPCG_I2C1_IPG_S_CLK, "mipi1_lpcg_i2c1_ipg_s", "mipi_ipg_clk_root", 0, 0x14, 0, 0, },
	{ IMX_MIPI1_LPCG_I2C1_IPG_CLK, "mipi1_lpcg_i2c1_ipg_clk", "mipi1_lpcg_i2c1_ipg_s", 0, 0, 0, 0, }, //FIXME: same LPCG offset as I2C0
	{ IMX_MIPI1_LPCG_PWM_IPG_S_CLK, "mipi1_lpcg_pwm_ipg_s", "mipi_ipg_clk_root", 0, 0xc, 16, 0, },
	{ IMX_MIPI1_LPCG_PWM_IPG_CLK, "mipi1_lpcg_pwm_ipg_clk", "mipi1_lpcg_pwm_ipg_s", 0, 0xc, 16, 0, }, //FIXME: same LPCG offset as IPG_S
	{ IMX_MIPI1_LPCG_PWM_32K_CLK, "mipi1_lpcg_pwm_32K_clk", "xtal_32KHz", 0, 0xc, 0, 0, },
	{ IMX_MIPI1_LPCG_PWM_CLK, "mipi1_lpcg_pwm_clk", "mipi1_pwm_clk", 0, 0xc, 0, 0, }, //FIXME: same LPCG offset as 32K_CLK
	{ IMX_MIPI1_LPCG_GPIO_IPG_CLK, "mipi1_lpcg_gpio_ipg_clk", "mipi_ipg_clk_root", 0, 0x8, 0, 0, },
	{ IMX_MIPI1_LPCG_LIS_IPG_CLK, "mipi1_lis_ipg_clk", "mipi_ipg_clk_root", 0, 0, 16, 0, },
};


static const struct imx8qxp_ss_lpcg imx8qxp_ss_mipi1 = {
	.lpcg = imx8qxp_lpcg_mipi1,
	.num_lpcg = ARRAY_SIZE(imx8qxp_lpcg_mipi1),
	.num_max = IMX_MIPI1_LPCG_CLK_END,
};

static int imx8qxp_lpcg_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct clk_hw_onecell_data *clk_data;
	const struct imx8qxp_ss_lpcg *ss_lpcg;
	const struct imx8qxp_lpcg_data *lpcg;
	struct resource *res;
	struct clk_hw **clks;
	void __iomem *base;
	int i;

	ss_lpcg = of_device_get_match_data(dev);
	if (!ss_lpcg)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

	clk_data = devm_kzalloc(&pdev->dev, struct_size(clk_data, hws,
				ss_lpcg->num_max), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = ss_lpcg->num_max;
	clks = clk_data->hws;

	for (i = 0; i < ss_lpcg->num_lpcg; i++) {
		lpcg = ss_lpcg->lpcg + i;
		clks[lpcg->id] = imx_clk_lpcg_scu(lpcg->name, lpcg->parent,
						  lpcg->flags, base + lpcg->offset,
						  lpcg->bit_idx, lpcg->hw_gate);
	}

	for (i = 0; i < clk_data->num; i++) {
		if (IS_ERR(clks[i]))
			pr_warn("i.MX clk %u: register failed with %ld\n",
				i, PTR_ERR(clks[i]));
	}

	return of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_data);
}

static const struct of_device_id imx8qxp_lpcg_match[] = {
	{ .compatible = "fsl,imx8qxp-lpcg-cm40", &imx8qxp_ss_cm40, },
	{ .compatible = "fsl,imx8qxp-lpcg-adma", &imx8qxp_ss_adma, },
	{ .compatible = "fsl,imx8qxp-lpcg-conn", &imx8qxp_ss_conn, },
	{ .compatible = "fsl,imx8qxp-lpcg-lsio", &imx8qxp_ss_lsio, },
	{ .compatible = "fsl,imx8qxp-lpcg-hsio", &imx8qxp_ss_hsio, },
	{ .compatible = "fsl,imx8qxp-lpcg-img",  &imx8qxp_ss_img, },
	{ .compatible = "fsl,imx8qxp-lpcg-csi0",  &imx8qxp_ss_csi0, },
	{ .compatible = "fsl,imx8qxp-lpcg-csi1",  &imx8qxp_ss_csi1, },
	{ .compatible = "fsl,imx8qxp-lpcg-dc",   &imx8qxp_ss_dc, },
	{ .compatible = "fsl,imx8qxp-lpcg-mipi0",  &imx8qxp_ss_mipi0, },
	{ .compatible = "fsl,imx8qxp-lpcg-mipi1",  &imx8qxp_ss_mipi1, },
	{ /* sentinel */ }
};

static struct platform_driver imx8qxp_lpcg_clk_driver = {
	.driver = {
		.name = "imx8qxp-lpcg-clk",
		.of_match_table = imx8qxp_lpcg_match,
		.suppress_bind_attrs = true,
	},
	.probe = imx8qxp_lpcg_clk_probe,
};

builtin_platform_driver(imx8qxp_lpcg_clk_driver);
