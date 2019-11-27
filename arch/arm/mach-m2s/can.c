// based on linux/arch/arm/mach-stm32/can.c
// 

#include <linux/init.h>
#include <linux/platform_device.h> 

//#include <linux/interrupt.h>
//#include <linux/irq.h>

#include <mach/m2s.h> // where sysreg register map is defined 
#include <mach/clock.h>
#include <mach/platform.h>
#include <mach/can.h>



#if defined(CONFIG_CAN_M2S)

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
 
// static struct irq_chip m2s_irq_chip = {
	// .name		= "CAN",
	// .enable		= m2s_irq_enable,
	// .disable	= m2s_irq_disable,
    // .set_type   = m2s_irq_set_type,
	// .ack		= m2s_irq_ack,
// };


 
#endif /* CONFIG_CAN_M2S*/




// static void m2s_irq_enable(unsigned irq)
// {
    // struct m2s_can_chip *m2s_chip = get_irq_chip_data(irq);
    // struct irq_desc *desc = irq_to_desc(irq);
    // unsigned long flags, can_cfg;

    // spin_lock_irqsave(&m2s_chip->irq_lock, flags);
    // desc->chip->unmask(irq);
    // desc->status &= ~IRQ_MASKED;

    // spin_unlock_irqrestore(&m2s_chip->irq_lock, flags);
// } 

// static void m2s_irq_disable(unsigned irq)
// {
    // struct m2s_can_chip *m2s_chip = get_irq_chip_data(irq);
    // struct irq_desc *desc = irq_to_desc(irq);
    // unsigned long flags, can_cfg;

    // spin_lock_irq_save(&m2s_chip->irq_lock, flags);
    // desc->chip->mask(irq);
    // desc->status |= IRQ_MASKED;

    // spin_unlock_irqrestore(&m2s_chip->irq_lock, flags);   
// }

// static void m2s_irq_ack(unsigned irq)
// {
    // struct m2s_can_chip *m2s_chip = get_irq_chip_data(irq);
    

// }




void __init m2s_can_init(void)
{
    //unsigned int IRQn;
#if defined (CONFIG_CAN_M2S)
    //irq = MSS_CAN_IRQ;
    //nvic_unmask_irq(irq);
    // IRQn = MSS_CAN_IRQ;
    
    // m2s_irq_chip.mask = get_irq_chip(IRQn)->mask;
    
    // m2s_irq_chip.unmask = get_irq_chip(IRQn)->unmask;
    
    // set_irq_chip(IRQn, &m2s_irq_chip);
    // set_irq_handler(IRQn, handle_level_irq);
    // set_irq_flags(IRQn, IRQF_VALID);
    //set_irq_chip_data(IRQn, &);
    /* Check clocks, and register platform device */
    // get apb1 clk for CAN 

    m2s_can_device_data.freq_apb = m2s_clock_get(CLCK_PCLK1);
    platform_set_drvdata(&can_device, &m2s_can_device_data);
    // Make platform device known to system 
    // Once made known to system, driver's probe() function will be called 
    platform_device_register(&can_device);
#endif
}
