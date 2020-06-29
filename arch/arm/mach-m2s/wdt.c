/*
 * linux/arch/arm/mach-m2s/wdt.c
 *
 * (C) Copyright 2020 CSIRO
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

#include <mach/wdt.h>

#if defined(CONFIG_M2S_MSS_WDT)

static struct platform_device m2s_wdt_device = {
	.name		= "m2s_wdt",
	.id		= -1,
	.num_resources	= 0,
};

#endif /* CONFIG_M2S_MSS_WDT */

void __init m2s_init_wdt(void)
{
#if defined(CONFIG_M2S_MSS_WDT)
	platform_device_register(&m2s_wdt_device);
#endif
}
