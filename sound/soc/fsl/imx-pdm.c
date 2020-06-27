/*
 * Copyright 2017-2020 NXP.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/soc.h>

#include "fsl_sai.h"

#define IMX_PDM_FORMATS (SNDRV_PCM_FMTBIT_DSD_U8 | \
			 SNDRV_PCM_FMTBIT_DSD_U16_LE | \
			 SNDRV_PCM_FMTBIT_DSD_U32_LE)

struct imx_pdm_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	unsigned int decimation;
	unsigned long mclk_11k;
	unsigned long mclk_8k;
	bool fixed_mclk;
	int osr_id;
};

static const struct imx_pdm_mic_fs_mul {
	unsigned int min;
	unsigned int max;
	unsigned int mul;
} fs_mul[] = {
	{ .min =  8000, .max = 11025, .mul =  8 }, /* low power */
	{ .min = 16000, .max = 64000, .mul = 16 }, /* performance */
};

/* Ratio based on default Audio PLLs
 * Audio PLL1 = 393216000 Hz
 * Audio PLL2 = 361267200 Hz
 */
static const struct imx_pdm_mic_mclk_fixed {
	unsigned long mclk_11k;
	unsigned long mclk_8k;
	unsigned int ratio;
} mclk_fixed[] = {
	{ .mclk_11k = 11289600, .mclk_8k = 12288000, .ratio = 32 },
	{ .mclk_11k = 15052800, .mclk_8k = 16384000, .ratio = 24 },
	{ .mclk_11k = 22579200, .mclk_8k = 24576000, .ratio = 16 },
	{ .mclk_11k = 45158400, .mclk_8k = 49152000, .ratio =  8 },
};

static const unsigned int imx_pdm_mic_rates[] = {
	8000,  11025, 16000, 22050,
	32000, 44100, 48000, 64000,
};

static const struct imx_pdm_mic_osr_map {
	int id;
	unsigned int osr;
} osr_map[] = {
	{ .id = 0, .osr =  48 }, /* 4x12 */
	{ .id = 1, .osr =  64 }, /* 4x16 */
	{ .id = 2, .osr =  96 }, /* 4x24 */
	{ .id = 3, .osr = 128 }, /* 4x32 */
	{ .id = 4, .osr = 192 }, /* 4x48 */
};

static int imx_pdm_mic_get_osr_id(int decimation)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(osr_map); i++) {
		if (osr_map[i].osr == decimation)
			return osr_map[i].id;
	}

	return -EINVAL;
}

static unsigned int imx_pdm_mic_get_osr_rate(int osr_id)
{
	int i;

	for (i = 0; ARRAY_SIZE(osr_map); i++) {
		if (osr_map[i].id == osr_id)
			return osr_map[i].osr;
	}

	return -EINVAL;
}

static const char *const osr_rate_text[] = {
	"OSR_4x12",
	"OSR_4x16",
	"OSR_4x24",
	"OSR_4x32",
	"OSR_4x48"
};

static int osr_rate_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_pdm_data *data = snd_soc_card_get_drvdata(card);

	ucontrol->value.enumerated.item[0] = data->osr_id;

	return 0;
}

static int osr_rate_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_pdm_data *data = snd_soc_card_get_drvdata(card);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	int osr = snd_soc_enum_item_to_val(e, item[0]);

	data->decimation = imx_pdm_mic_get_osr_rate(osr);
	data->osr_id = osr;

	return 0;
}

static const struct soc_enum osr_rate_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(osr_rate_text), osr_rate_text);

const struct snd_kcontrol_new imx_pdm_mic_snd_ctrls[] = {
	SOC_ENUM_EXT("over sampling ratio", osr_rate_enum,
		     osr_rate_get, osr_rate_set),
};

static struct snd_pcm_hw_constraint_list imx_pdm_mic_rate_constrains = {
	.count = ARRAY_SIZE(imx_pdm_mic_rates),
	.list = imx_pdm_mic_rates,
};
static unsigned int imx_pdm_mic_channels[] = { 1, 2, 4, 6, 8 };
static struct snd_pcm_hw_constraint_list imx_pdm_mic_channels_constrains = {
	.count = ARRAY_SIZE(imx_pdm_mic_channels),
	.list = imx_pdm_mic_channels,
};

static unsigned long imx_pdm_mic_mclk_freq(unsigned int decimation,
		unsigned int rate)
{
	int i;

	/* Find appropriate mclk freq */
	for (i = 0; i < ARRAY_SIZE(fs_mul); i++) {
		if (rate >= fs_mul[i].min && rate <= fs_mul[i].max)
			return (rate * decimation * fs_mul[i].mul);
	}

	return 0;
}

static int imx_pdm_mic_get_mclk_fixed(struct imx_pdm_data *data,
		unsigned int ratio)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mclk_fixed); i++) {
		if (mclk_fixed[i].ratio == ratio) {
			data->mclk_11k = mclk_fixed[i].mclk_11k;
			data->mclk_8k = mclk_fixed[i].mclk_8k;
			return 0;
		}
	}

	return -EINVAL;
}

static int imx_pdm_mic_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_card *card = rtd->card;
	int ret;

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
			&imx_pdm_mic_rate_constrains);
	if (ret) {
		dev_err(card->dev,
			"fail to set pcm hw rate constrains: %d\n", ret);
		return ret;
	}

	ret = snd_pcm_hw_constraint_list(runtime, 0,
		SNDRV_PCM_HW_PARAM_CHANNELS, &imx_pdm_mic_channels_constrains);
	if (ret) {
		dev_err(card->dev,
			"fail to set pcm hw channels constrains: %d\n", ret);
		return ret;
	}

	ret = snd_pcm_hw_constraint_mask64(runtime,
			SNDRV_PCM_HW_PARAM_FORMAT, IMX_PDM_FORMATS);
	if (ret) {
		dev_err(card->dev,
			"fail to set pcm hw format constrains: %d\n", ret);
		return ret;
	}

	return 0;
}

static int imx_pdm_mic_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct imx_pdm_data *data = snd_soc_card_get_drvdata(card);
	unsigned int sample_rate = params_rate(params);
	unsigned long mclk_freq;
	int ret;

	/* set cpu dai format configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_PDM |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret) {
		dev_err(card->dev, "fail to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	/* Set bitclk ratio */
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, data->decimation);
	if (ret) {
		dev_err(card->dev, "fail to set cpu sysclk: %d\n", ret);
		return ret;
	}

	if (data->fixed_mclk) {
		mclk_freq = (do_div(sample_rate, 8000) ?
			data->mclk_11k : data->mclk_8k);
	} else {
		mclk_freq = imx_pdm_mic_mclk_freq(data->decimation,
			sample_rate);
	}
	/* set mclk freq */
	ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1,
			mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(card->dev, "fail to set cpu mclk1 rate: %lu\n",
			mclk_freq);
		return ret;
	}

	dev_dbg(card->dev, "mclk: %lu, bclk ratio: %u\n",
			mclk_freq, data->decimation);

	return 0;
}

static const struct snd_soc_ops imx_pdm_mic_ops = {
	.startup = imx_pdm_mic_startup,
	.hw_params = imx_pdm_mic_hw_params,
};

static int imx_pdm_mic_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct platform_device *cpu_pdev;
	struct imx_pdm_data *data;
	struct snd_soc_dai_link_component *dlc;
	unsigned long sai_mclk, sai_pll8k;
	struct fsl_sai *sai;
	unsigned int ratio;
	int ret;

	dlc = devm_kzalloc(&pdev->dev, 3 * sizeof(*dlc), GFP_KERNEL);
	if (!dlc)
		return -ENOMEM;

	cpu_np = of_parse_phandle(np, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "fail to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = of_property_read_u32(np, "decimation", &data->decimation);
	if (ret < 0) {
		ret = -EINVAL;
		goto fail;
	}

	data->osr_id = imx_pdm_mic_get_osr_id(data->decimation);
	if (data->osr_id < 0) {
		ret = -EINVAL;
		goto fail;
	}

	if (of_find_property(np, "fixed-mclk", NULL))
		data->fixed_mclk = true;

	if (data->fixed_mclk) {
		sai = dev_get_drvdata(&cpu_pdev->dev);
		/* Get SAI clock settings */
		sai_mclk = clk_get_rate(sai->mclk_clk[FSL_SAI_CLK_MAST1]);
		sai_pll8k = clk_get_rate(sai->pll8k_clk);
		ratio = sai_pll8k / sai_mclk;

		ret = imx_pdm_mic_get_mclk_fixed(data, ratio);
		if (ret) {
			dev_err(&pdev->dev, "fail to set fixed mclk: %d\n", ret);
			return ret;
		}

		dev_dbg(&pdev->dev, "sai_pll8k: %lu, sai_mclk: %lu, ratio: %u\n",
			sai_pll8k, sai_mclk, ratio);
	}

	data->dai.cpus = &dlc[0];
	data->dai.num_cpus = 1;
	data->dai.platforms = &dlc[1];
	data->dai.num_platforms = 1;
	data->dai.codecs = &dlc[2];
	data->dai.num_codecs = 1;

	data->dai.name = "pdm hifi";
	data->dai.stream_name = "pdm hifi";
	data->dai.codecs->dai_name = "snd-soc-dummy-dai";
	data->dai.codecs->name = "snd-soc-dummy";
	data->dai.cpus->dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platforms->of_node = cpu_np;
	data->dai.capture_only = 1;
	data->dai.ops = &imx_pdm_mic_ops;

	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;

	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret) {
		dev_err(&pdev->dev, "fail to find card model name\n");
		goto fail;
	}

	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.controls = imx_pdm_mic_snd_ctrls;
	data->card.num_controls = ARRAY_SIZE(imx_pdm_mic_snd_ctrls);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd soc register card failed: %d\n", ret);
		goto fail;
	}

	ret = 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);

	return ret;
}

static int imx_pdm_mic_remove(struct platform_device *pdev)
{
	struct imx_pdm_data *data = platform_get_drvdata(pdev);
	/* unregister card */
	snd_soc_unregister_card(&data->card);
	return 0;
}

static const struct of_device_id imx_pdm_mic_dt_ids[] = {
	{ .compatible = "fsl,imx-pdm-mic", },
	{ /* sentinel*/ }
};
MODULE_DEVICE_TABLE(of, imx_pdm_mic_dt_ids);

static struct platform_driver imx_pdm_mic_driver = {
	.driver = {
		.name = "imx-pdm-mic",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_pdm_mic_dt_ids,
	},
	.probe = imx_pdm_mic_probe,
	.remove = imx_pdm_mic_remove,
};
module_platform_driver(imx_pdm_mic_driver);

MODULE_DESCRIPTION("NXP i.MX PDM mic ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-pdm-mic");
