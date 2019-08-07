/*
 * linux/arch/arm/mach-m2s/mmc.c
 *
 * (C) Copyright 2019 CSIRO
 * Commonwealth Scientific and Industrial Research Organisation
 * Mike Pilawa <Mike.Pilawa@csiro.au>
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mmc/sdhci-iwave.h>

#include <mach/mmc.h>

#if defined(CONFIG_MMC_SDHCI_IWAVE)

/*
 * SDHCI IWAVE resources
 */
#define MMC_IWAVE_START	0x30000000
#define MMC_IWAVE_END	0x300000FF
#define MMC_IWAVE_IRQ	34

/*
 * MMC platform device resources
 */
static struct resource	mmc_iwave_resources[] = {
	{
		.start	= MMC_IWAVE_START,
		.end	= MMC_IWAVE_END,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MMC_IWAVE_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * MMC platform device data
 */
static struct sdhci_iwave_platdata	mmc_iwave_platdata = {
	.hw_name	= "m2s-mmc",
};

/*
 * MMC platform device instance
 */
static struct platform_device	mmc_iwave_dev = {
	.name			= "sdhci-iwave",
	.id				= 0,
	.num_resources	= ARRAY_SIZE(mmc_iwave_resources),
	.resource		= mmc_iwave_resources,
};
#endif /* CONFIG_MMC_SDHCI_IWAVE */

void __init m2s_mmc_init(void)
{
#if defined(CONFIG_MMC_SDHCI_IWAVE)
	mmc_iwave_dev.dev.platform_data = &mmc_iwave_platdata;
	platform_device_register(&mmc_iwave_dev);
#endif
}
