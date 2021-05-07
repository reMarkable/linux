/*
 * Copyright (c) 2014 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <defs.h>
#include "debug.h"
#include "core.h"
#include "common.h"
#include "of.h"

static const char * const bcm43456c5_ccode_map[] = {
	"AE-AE-6", "AG-AG-2", "AI-AI-1", "AL-AL-2", "AS-AS-12", "AT-AT-4",
	"AU-AU-6", "AW-AW-2", "AZ-AZ-2",
	"BA-BA-2", "BD-BD-1", "BE-BE-4", "BG-BG-4", "BH-BH-4", "BM-BM-12",
	"BN-BN-4", "BR-BR-4", "BS-BS-2", "BY-BY-3",
	"CA-CA-2", "CH-CH-4", "CN-CN-38", "CO-CO-17", "CR-CR-17", "CY-CY-4",
	"CZ-CZ-4",
	"DE-DE-7", "DK-DK-4",
	"EC-EC-21", "EE-EE-4", "EG-EG-13", "ES-ES-4", "ET-ET-2",
	"FI-FI-4", "FR-FR-5",
	"GB-GB-6", "GD-GD-2", "GF-GF-2", "GP-GP-2", "GR-GR-4", "GT-GT-1",
	"GU-GU-30",
	"HK-HK-2", "HR-HR-4", "HU-HU-4",
	"ID-ID-1", "IE-IE-5", "IL-IL-14", "IN-IN-3", "IS-IS-4", "IT-IT-4",
	"JO-JO-3", "JP-JP-58",
	"KH-KH-2", "KR-KR-96", "KW-KW-5", "KY-KY-3",
	"LA-LA-2", "LB-LB-5", "LI-LI-4", "LK-LK-1", "LS-LS-2", "LT-LT-4",
	"LU-LU-3", "LV-LV-4",
	"MA-MA-2", "MC-MC-1", "MD-MD-2", "ME-ME-2", "MK-MK-2", "MN-MN-1",
	"MQ-MQ-2", "MR-MR-2", "MT-MT-4", "MU-MU-2", "MV-MV-3", "MW-MW-1",
	"MX-MX-44", "MY-MY-3",
	"NI-NI-2", "NL-NL-4", "NO-NO-4", "NZ-NZ-4",
	"OM-OM-4",
	"PA-PA-17", "PE-PE-20", "PH-PH-5", "PL-PL-4", "PR-PR-38", "PT-PT-4",
	"PY-PY-2",
	"Q2-Q2-993",
	"RE-RE-2", "RO-RO-4", "RS-RS-2", "RU-RU-13",
	"SE-SE-4", "SI-SI-4", "SK-SK-4", "SV-SV-25",
	"TH-TH-5", "TN-TN-1", "TR-TR-7", "TT-TT-3", "TW-TW-65",
	"UA-UA-8", "US-US-988",
	"VA-VA-2", "VE-VE-3", "VG-VG-2", "VN-VN-4",
	"XZ-XZ-11",
	"YT-YT-2",
	"ZA-ZA-6",
};

static int brcmf_of_get_country_codes(struct device *dev,
				      struct brcmf_mp_device *settings,
				      u32 chip, u32 chiprev)
{
	struct device_node *np = dev->of_node;
	const char * const *ccode_map = NULL;
	struct brcmfmac_pd_cc_entry *cce;
	struct brcmfmac_pd_cc *cc;
	int count;
	int i;

	count = of_property_count_strings(np, "brcm,ccode-map");

	/* Use hard-coded map table over DT one for BCM4345/9 */
	if (chip == 0x4345 && chiprev == 0x9) {
		count = ARRAY_SIZE(bcm43456c5_ccode_map);
		ccode_map = bcm43456c5_ccode_map;
	}

	if (count < 0) {
		/* The property is optional, so return success if it doesn't
		 * exist. Otherwise propagate the error code.
		 */
		return (count == -EINVAL) ? 0 : count;
	}

	cc = devm_kzalloc(dev, sizeof(*cc) + count * sizeof(*cce), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;

	cc->table_size = count;

	for (i = 0; i < count; i++) {
		const char *map;

		cce = &cc->table[i];

		/* Try hard-coded map table first, and DT otherwise. */
		if (ccode_map) {
			map = ccode_map[i];
		} else {
			if (of_property_read_string_index(np, "brcm,ccode-map",
							  i, &map))
				continue;
		}

		/* String format e.g. US-Q2-86 */
		if (sscanf(map, "%2c-%2c-%d", cce->iso3166, cce->cc,
			   &cce->rev) != 3)
			brcmf_err("failed to read country map %s\n", map);
		else
			brcmf_dbg(INFO, "%s-%s-%d\n", cce->iso3166, cce->cc,
				  cce->rev);
	}

	settings->country_codes = cc;

	return 0;
}

void brcmf_of_probe(struct device *dev, enum brcmf_bus_type bus_type,
		    struct brcmf_mp_device *settings, u32 chip, u32 chiprev)
{
	struct brcmfmac_sdio_pd *sdio = &settings->bus.sdio;
	struct device_node *np = dev->of_node;
	int irq;
	int err;
	u32 irqf;
	u32 val32;
	u16 val16;

	if (!np || !of_device_is_compatible(np, "brcm,bcm4329-fmac"))
		return;

	err = brcmf_of_get_country_codes(dev, settings, chip, chiprev);
	if (err)
		brcmf_err("failed to get OF country code map (err=%d)\n", err);

	if (bus_type != BRCMF_BUSTYPE_SDIO)
		return;

	if (of_property_read_u32(np, "brcm,drive-strength", &val32) == 0)
		sdio->drive_strength = val32;

	sdio->broken_sg_support = of_property_read_bool(np,
			"brcm,broken_sg_support");
	if (of_property_read_u16(np, "brcm,sd_head_align", &val16) == 0)
		sdio->sd_head_align = val16;
	if (of_property_read_u16(np, "brcm,sd_sgentry_align", &val16) == 0)
		sdio->sd_sgentry_align = val16;

	/* make sure there are interrupts defined in the node */
	if (!of_find_property(np, "interrupts", NULL))
		return;

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		brcmf_err("interrupt could not be mapped\n");
		return;
	}
	irqf = irqd_get_trigger_type(irq_get_irq_data(irq));

	sdio->oob_irq_supported = true;
	sdio->oob_irq_nr = irq;
	sdio->oob_irq_flags = irqf;
}
