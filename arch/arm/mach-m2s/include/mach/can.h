/*
 * (C) Copyright 2016
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

#ifndef _MACH_M2S_CAN_H_
#define _MACH_M2S_CAN_H_

#include <linux/init.h>

typedef struct m2s_can_platform_data {
	uint32_t freq_apb;	
};
void __init m2s_can_init(void);

#endif /* _MACH_M2S_CAN_H_ */
