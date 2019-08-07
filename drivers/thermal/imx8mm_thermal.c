// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device_cooling.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#include "thermal_core.h"

#define TER			0x0	/* TMU enable */
#define TRITSR			0x20	/* TMU immediate temp */

#define TER_EN			BIT(31)
#define TRITSR_VAL_MASK		0xff

#define TEMP_LOW_LIMIT		10
#define IMX_TEMP_PASSIVE_COOL_DELTA 10000

struct imx8mm_tmu {
	struct thermal_zone_device *tzd;
	void __iomem *base;
	struct clk *clk;
	struct thermal_cooling_device *cdev;
	int temp_passive;
	int temp_critical;
};

/* The driver support 1 passive trip point and 1 critical trip point */
enum imx_thermal_trip {
	IMX_TRIP_PASSIVE,
	IMX_TRIP_CRITICAL,
	IMX_TRIP_NUM,
};

static int tmu_get_temp(void *data, int *temp)
{
	struct imx8mm_tmu *tmu = data;
	u32 val;

	/* the temp sensor need about 1ms to finish the measurement */
	usleep_range(1000, 2000);

	val = readl_relaxed(tmu->base + TRITSR) & TRITSR_VAL_MASK;
	if (val < TEMP_LOW_LIMIT)
		return -EAGAIN;

	*temp = val * 1000;

	return 0;
}

static int tmu_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	int trip_temp;
	struct imx8mm_tmu *tmu = p;

	if (!tmu->tzd)
		return 0;

	trip_temp = (trip == IMX_TRIP_PASSIVE) ? tmu->temp_passive : tmu->temp_critical;

	if (tmu->tzd->temperature >= (trip_temp - IMX_TEMP_PASSIVE_COOL_DELTA))
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

static int tmu_set_trip_temp(void *p, int trip, int temp)
{
	struct imx8mm_tmu *tmu = p;

	if (trip == IMX_TRIP_CRITICAL)
		tmu->temp_critical = temp;

	if (trip == IMX_TRIP_PASSIVE)
		tmu->temp_passive = temp;

	return 0;
}

static struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
	.get_trend = tmu_get_trend,
	.set_trip_temp = tmu_set_trip_temp,
};

static int imx8mm_tmu_probe(struct platform_device *pdev)
{
	const struct thermal_trip *trips;
	struct imx8mm_tmu *tmu;
	u32 val;
	int ret;

	tmu = devm_kzalloc(&pdev->dev, sizeof(struct imx8mm_tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);

	tmu->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(tmu->clk)) {
		ret = PTR_ERR(tmu->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"failed to get tmu clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(tmu->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable tmu clock: %d\n", ret);
		return ret;
	}

	tmu->tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0,
							tmu, &tmu_tz_ops);
	if (IS_ERR(tmu->tzd)) {
		dev_err(&pdev->dev,
			"failed to register thermal zone sensor: %d\n", ret);
		return PTR_ERR(tmu->tzd);
	}

	platform_set_drvdata(pdev, tmu);

	tmu->cdev = devfreq_cooling_register();
	if (IS_ERR(tmu->cdev)) {
		ret = PTR_ERR(tmu->cdev);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to register devfreq cooling device %d\n", ret);
		return ret;
	}

	ret = thermal_zone_bind_cooling_device(tmu->tzd,
		IMX_TRIP_PASSIVE,
		tmu->cdev,
		THERMAL_NO_LIMIT,
		THERMAL_NO_LIMIT,
		THERMAL_WEIGHT_DEFAULT);
	if (ret) {
		dev_err(&pdev->dev,
			"binding zone %s with cdev %s failed:%d\n",
			tmu->tzd->type, tmu->cdev->type, ret);
		devfreq_cooling_unregister(tmu->cdev);
		return ret;
	}

	trips = of_thermal_get_trip_points(tmu->tzd);

	/* get the thermal trip temp */
	tmu->temp_passive = trips[0].temperature;
	tmu->temp_critical = trips[1].temperature;

	/* enable the monitor */
	val = readl_relaxed(tmu->base + TER);
	val |= TER_EN;
	writel_relaxed(val, tmu->base + TER);

	return 0;
}

static int imx8mm_tmu_remove(struct platform_device *pdev)
{
	struct imx8mm_tmu *tmu = platform_get_drvdata(pdev);
	u32 val;

	/* disable TMU */
	val = readl_relaxed(tmu->base + TER);
	val &= ~TER_EN;
	writel_relaxed(val, tmu->base + TER);

	clk_disable_unprepare(tmu->clk);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id imx8mm_tmu_table[] = {
	{ .compatible = "fsl,imx8mm-tmu", },
	{ },
};

static struct platform_driver imx8mm_tmu = {
	.driver = {
		.name	= "i.mx8mm_thermal",
		.of_match_table = imx8mm_tmu_table,
	},
	.probe = imx8mm_tmu_probe,
	.remove = imx8mm_tmu_remove,
};
module_platform_driver(imx8mm_tmu);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("i.MX8MM Thermal Monitor Unit driver");
MODULE_LICENSE("GPL v2");
