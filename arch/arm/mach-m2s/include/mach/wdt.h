/*
 * arch/arm/mach-m2s/include/mach/wdt.h
 *
 * Copyright (C) 2020 CSIRO
 * Commonwealth Scientific and Industrial Research Organisation
 * Mike Pilawa <Mike.Pilawa@csiro.au>
 *
 * Based on SmartFusion2 Microcontroller Subsystem
 * 	User Guide UG0331 Revision 15.0
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MACH_M2S_WDT_H_
#define _MACH_M2S_WDT_H_

/*
 * M2S Watchdog Timer Control/Status registers.
 * Many of the below registers are read-only.
 * The values are set either in the SYSREG block,
 * or they are fixed by flash bits via the FPGA FW flow.
 */
struct m2s_wdt_reg {
	unsigned int wdogvalue;		/* RO : Watchdog current Value */
	unsigned int wdogload;		/* RO : Watchdog reLoad value */
	unsigned int wdogmvrp;		/* RO : Watchdog Maximum Value Refresh Permitted */
	unsigned int wdogrefresh;	/* WO : Watchdog Refresh register */
	unsigned int wdogenable;	/* RO : Watchdog Enable(d) register */
	unsigned int wdogcontrol;	/* RW : Watchdog Control register */
	unsigned int wdogstatus;	/* RO : Watchdog Status register */
	unsigned int wdogris;		/* W1C: Watchdog Raw Interrupt Status register */
};

#define M2S_WDT_REG_BASE	0x40005000
#define M2S_WDT_REG		((volatile struct m2s_wdt_reg*)(M2S_WDT_REG_BASE))

#define M2S_WDT_REFRESH_VAL	0xAC15DE42

#define M2S_WDT_ENABLED		(1 << 0)

#define M2S_WDT_TIMEOUTINTEN	(1 << 0)
#define M2S_WDT_WAKEUPINTEN	(1 << 1)
#define M2S_WDT_MODE		(1 << 2)

#define M2S_WDT_REFRESHSTATUS	(1 << 0)

#define M2S_WDT_TIMEOUTRS	(1 << 0)
#define M2S_WDT_WAKEUPRS	(1 << 1)

/*
 * Bit definitions for the M2S SYSREG WDOG_CR register.
 */
#define M2S_WDOG_CR_ENABLE	(1 << 0)
#define M2S_WDOG_CR_MODE	(1 << 1)

/*
 * Function for initialization (platform device registration) of the WDT.
 */
extern void __init m2s_init_wdt(void);

#endif
