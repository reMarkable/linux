// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#ifdef CONFIG_OF
void brcmf_of_probe(struct device *dev, enum brcmf_bus_type bus_type,
		    struct brcmf_mp_device *settings, u32 chip, u32 chiprev);
#else
static void brcmf_of_probe(struct device *dev, enum brcmf_bus_type bus_type,
			   struct brcmf_mp_device *settings,
			   u32 chip, u32 chiprev)
{
}
#endif /* CONFIG_OF */
