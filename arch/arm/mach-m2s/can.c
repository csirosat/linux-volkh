// based on linux/arch/arm/mach-stm32/can.c
// 

#include <linux/init.h>
#include <linux/platform_device.h> 
#include <mach/m2s.h> 
#include <mach/clock.h>
#include <mach/platform.h>
#include <mach/can.h>

#if defined(CONFIG_M2S_MSS_CAN)

#define MSS_CAN_BASE        0x40015000
#define MSS_CAN_RGSZ        (0x20)
#define MSS_CAN_IRQ         16 // from m2sxxx.h

// CAN SYSREG 
#define CAN_RST_CLR         (1 << 13) // from m2sxxx.h

struct m2s_can_chip {
    u32 can_cfg;
    u32 can_irq;
    spinlock_t lock;
    spinlock_t irq_lock;
    unsigned irq_base;	
};

/*
 * CAN1 platform device resources
 */ 
 
static struct resource      can_resources[] = {
    {
        .start  = MSS_CAN_BASE,
        .end    = MSS_CAN_BASE + 0xFFF,
        .flags  = IORESOURCE_MEM,
    },
    {
        .start  = MSS_CAN_IRQ,
        .flags  = IORESOURCE_IRQ,
    }
};

static struct m2s_can_platform_data     m2s_can_device_data = {
    .freq_apb  = 50000000,
};

/*
 * CAN platform device instance
 * static structure used to tell kernel about existence of platform device 
 */ 
static struct platform_device can_device = {
    .name           = "m2s_can",
    .id             = 0,
    .resource       = can_resources, 
    .num_resources  = ARRAY_SIZE(can_resources)
 };
 
 
#endif /* CONFIG_M2S_MSS_CAN*/


void __init m2s_can_init(void)
{
#if defined (CONFIG_M2S_MSS_CAN)
    m2s_can_device_data.freq_apb = m2s_clock_get(CLCK_PCLK1);
    platform_set_drvdata(&can_device, &m2s_can_device_data);
    // Make platform device known to system 
    // Once made known to system, driver's probe() function will be called 
    platform_device_register(&can_device);
#endif
}
