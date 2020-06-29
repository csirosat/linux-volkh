/*
 * linux/arch/arm/mach-m2s/m2s_platform.c
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/module.h>

#include <asm/mach-types.h>
#include <asm/hardware/nvic.h>

#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/platform.h>
#include <mach/hardware.h>
#include <mach/iomux.h>
#include <mach/timer.h>
#include <mach/uart.h>
#include <mach/spi.h>
#include <mach/eth.h>
#include <mach/i2c.h>
#include <mach/usb.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/can.h>
#include <mach/wdt.h>
#include <mach/m2s.h>

/*
 * M2S System Reference clock.
 * Global defined in arch/arm/mach-m2s/clock.c
 */
extern unsigned int m2s_clock_sysref;

/*
 * Define a particular platform (board)
 */
static int m2s_platform = PLATFORM_G4M_VB;

/*
 * Interface to get the platform
 */
int m2s_platform_get(void)
{
	return m2s_platform;
}
EXPORT_SYMBOL(m2s_platform_get);

/*
 * Interface to get the SmartFusion2 device
 */
int m2s_device_get(void)
{
	int r;

	switch (m2s_platform) {
	case PLATFORM_M2S_FG484_SOM:
	case PLATFORM_M2S_VOLKH:
		r = DEVICE_M2S_060;
		break;
	case PLATFORM_M2S_SOM:
	case PLATFORM_G4M_VB:
	case PLATFORM_SF2_DEV_KIT:
	default:
		r = DEVICE_M2S_050;
		break;
	}
	return r;
}
EXPORT_SYMBOL(m2s_device_get);

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init m2s_platform_parse(char *plat)
{
	char *sysref;
	unsigned int sysref_val;

	/*
	 * Find sysref in m2s_platform from cmdline, if it exists.
	 */
	if ((sysref = strchr(plat, ':')) != NULL)
		*(sysref++) = '\0';

	if (!strcmp(plat, "g4m-vb"))
		m2s_platform = PLATFORM_G4M_VB;
	else if (!strcmp(plat, "m2s-som"))
		m2s_platform = PLATFORM_M2S_SOM;
	else if (!strcmp(plat, "sf2-dev-kit"))
		m2s_platform = PLATFORM_SF2_DEV_KIT;
	else if (!strcmp(plat, "m2s-fg484-som"))
		m2s_platform = PLATFORM_M2S_FG484_SOM;
	else if (!strcmp(plat, "m2s-volkh")) {
		m2s_platform = PLATFORM_M2S_VOLKH;
		m2s_clock_sysref = M2S_SYSREF_VOLKH;
	}

	/*
	 * Decode sysref in m2s_platform from cmdline, if it exists.
	 */
	if (sysref && strlen(sysref)) {
		sysref_val = simple_strtoul(sysref, NULL, 10);
		if (sysref_val)
			m2s_clock_sysref = sysref_val;
	}

//	printk("m2s_clock_sysref=%u\n", m2s_clock_sysref);

	return 1;
}
__setup("m2s_platform=", m2s_platform_parse);

/*
 * Forward declarations.
 */
static void __init m2s_map_io(void);
static void __init m2s_init_irq(void);
static void __init m2s_init(void);

/*
 * Data structure for the timer system.
 */
static struct sys_timer m2s_timer = {
	.init		= m2s_timer_init,
};

/*
 * M2S plaform machine description.
 */
MACHINE_START(M2S, "Microsemi M2S")
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging (CONFIG_DEBUG_LL=y).
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.phys_io	= MSS_UART1_BASE,
	.io_pg_offst	= (IO_ADDRESS(MSS_UART1_BASE) >> 18) & 0xfffc,
	.map_io		= m2s_map_io,
	.init_irq	= m2s_init_irq,
	.timer		= &m2s_timer,
	.init_machine	= m2s_init,
MACHINE_END

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init m2s_map_io(void)
{
}

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init m2s_init_irq(void)
{
	/*
	 * Initialize NVIC. All interrupts are masked initially.
	 */
	nvic_init();
}

#define M2S_RESET_SOURCE_POWERUP	0
#define M2S_RESET_SOURCE_CNTRLR_MSS	1
#define M2S_RESET_SOURCE_CNTRLR_M3	2
#define M2S_RESET_SOURCE_SOFT		3
#define M2S_RESET_SOURCE_LOCKUP		4
#define M2S_RESET_SOURCE_WATCHDOG	5
#define M2S_RESET_SOURCE_USER_MSS	6
#define M2S_RESET_SOURCE_USER_M3	7

static char * m2s_reset_source_lut[] =
{
	[M2S_RESET_SOURCE_POWERUP]	= "Power-up",
	[M2S_RESET_SOURCE_CNTRLR_MSS]	= "MSS controller",
	[M2S_RESET_SOURCE_CNTRLR_M3]	= "M3 controller",
	[M2S_RESET_SOURCE_SOFT]		= "Soft",
	[M2S_RESET_SOURCE_LOCKUP]	= "M3 lockup",
	[M2S_RESET_SOURCE_WATCHDOG]	= "Watchdog",
	[M2S_RESET_SOURCE_USER_MSS]	= "MSS user",
	[M2S_RESET_SOURCE_USER_M3]	= "M3 user",
};

/*
 * Determine the last reset source/cause.
 * Clear/re-arm the register, and print the register value & decoded string.
 */
static void __init m2s_reset_source(void)
{
	unsigned int reset_source = M2S_SYSREG->reset_source_cr;
	char * reset_source_str = "UNKNOWN";
	int i;

	M2S_SYSREG->reset_source_cr = 0;

	for (i = 0; i < sizeof(m2s_reset_source_lut)/sizeof(char *); i++)
		if (reset_source == 1<<i) {
			reset_source_str = m2s_reset_source_lut[i];
			break;
		}

	printk(KERN_INFO "M2S Reset Cause: 0x%x = %s reset\n", reset_source, reset_source_str);
}

/*
 * M2S platform initialization.
 */
static void __init m2s_init(void)
{
	/*
	 * Print last reset source/cause
	 */
	m2s_reset_source();

	/*
	 * Configure the IOMUXes of SmartFusion2
	 */
	m2s_iomux_init();

#if defined(CONFIG_SERIAL_8250)
	/*
	 * Configure the UART devices
	 */
	m2s_uart_init();
#endif

#if defined(CONFIG_SPI_M2S)
	/*
	 * Configure the SPI master interfaces (and possibly,
	 * SPI slave devices).
	 */
	m2s_spi_init();
#endif

#if defined(CONFIG_M2S_ETH)
	m2s_eth_init();
#endif

#if defined(CONFIG_I2C_A2F)
	/*
	 * Configure the I2C controllers (and possible I2C devices).
	 */
	m2s_i2c_init();
#endif

#if defined(CONFIG_GPIOLIB)
	/*
	 * Register M2S GPIO lines
	 */
	m2s_gpio_init();
#endif

#if defined(CONFIG_M2S_MSS_USB)
	m2s_usb_init();
#endif

#if defined(CONFIG_MMC_SDHCI_IWAVE)
	m2s_mmc_init();
#endif

#if defined(CONFIG_CAN_M2S)
	m2s_can_init();
#endif

#if defined(CONFIG_M2S_WATCHDOG) || defined(CONFIG_M2S_WATCHDOG_MODULE)
	m2s_init_wdt();
#endif
}
