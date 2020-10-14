/*
 * m2s_can.c
 * Used sample.c as a template for development 
 * 
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/netdevice.h>
#include <asm/hardware/nvic.h>

#include <linux/can.h>
#include <linux/can/dev.h>

#include <linux/err.h>
#include <linux/io.h>

#include <mach/can.h>
#include <mach/clock.h>

// for dev only 
// #define NAPI_ENABLED 1
#define NAPI_ENABLED 0

/*
 * Driver name
 */
#define DRV_NAME		"m2s_can"

/*
 * Device major number
 */
static uint dev_major = 1;

/*
 * User can change the major number
 */
module_param(dev_major, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(dev_major, "Device driver major number");


/*
 * Debug control - define to enable additional debug prints
 */
// #define CAN_DEBUG
#ifdef CAN_DEBUG
# define CDBG(x...)		printk(x)
#else
# define CDBG(x...)		do { } while (0)
#endif

/*
 * Timeouts, msec
 */
#define M2S_CAN_INIT_TOUT	100	/* Initialization */
#define M2S_CAN_RX_ACK_TOUT	10	/* RX mail-box ack */


#define CAN_RX_MAILBOX      32
#define CAN_TX_MAILBOX      32

/*
 * NAPI rx quota
 */
#define M2S_NAPI_WEIGHT	    CAN_RX_MAILBOX //64 // temporary value based on stm32_can.c

#define CAN_SET_SJW(_sjw)           ((_sjw) << 2)
#define CAN_SET_TSEG2(_tseg2)       ((_tseg2) << 5)
#define CAN_SET_TSEG1(_tseg1)       ((_tseg1) << 8)
#define CAN_SET_BITRATE(_bitrate)   ((_bitrate) << 16)

//#define M2S_CAN_SET_ID(_id)         (_id) & 0x1FFFFFFF   
#define M2S_CAN_SET_TXRQ            (1 << 0)
#define M2S_CAN_AUTO_RST            (1 << 4)
#define M2S_CAN_SWAP_END            (1 << 13)
#define M2S_CAN_ECR_MODE            (1 << 14) // Error capture mode - bit pos of last captured CAN error 
#define M2S_CAN_SET_RTR             (1 << 21)
#define M2S_CAN_ARB_FIXED           (1 << 12)
#define M2S_CAN_CFG_DEFAULT         (M2S_CAN_ECR_MODE | M2S_CAN_SWAP_END | M2S_CAN_ARB_FIXED | M2S_CAN_AUTO_RST)

// Matching definition in include/linux/can.h
// #define CAN_EFF_FLAG        0x80000000 // Extended Frame Format is set in the MSB
// #define CAN_RTR_FLAG        0x40000000 // Remote transmission request

// 80%
#define CAN_SPEED_50M_100K   CAN_SET_BITRATE(24)|CAN_SET_TSEG1(15)|CAN_SET_TSEG2(3)

#define DISABLE             0
#define ENABLE              1

/* CAN Controller SYSREG */
#define M2S_SYS_SOFT_RST_CR_CAN     (1 << 13)
#define USERCONFIG1         0x178

/* M2S CAN Command Register */
#define M2S_CAN_CMD_REV     0xFF << 16
#define M2S_CAN_IDE         (1 << 20)
#define M2S_CAN_LENDIAN     (1 << 13)
#define M2S_CAN_CMD_SRAM    (1 << 3)
#define M2S_CAN_CMD_LPBK    (1 << 2)
#define M2S_CAN_CMD_LISTEN  (1 << 1)    
#define M2S_CAN_CMD_RUNSTOP (1 << 0)

/* M2S Mailbox Register Field Macros */
#define M2S_CAN_SET_DLC(_nof_bytes) ((_nof_bytes & 0xF) << 16)
#define M2S_CAN_GET_DLC(_rxb)		((_rxb >> 16) & 0xF)
#define M2S_CAN_SET_EXT(_val)       (_val | M2S_CAN_IDE)
#define M2S_CAN_SET_STD(_val)       (_val & ~M2S_CAN_IDE)
#define M2S_CAN_SET_ID(_id)         (((_id) & 0x1FFFFFFF) << 3)
#define M2S_CAN_SET_IDE(_id)         (((_id) & 0x1FFFFFFF) << 3)
#define M2S_CAN_SET_IDS(_id)         (((_id) & 0x7FF) << 21)
#define M2S_CAN_GET_ID(_reg)        ((_reg >> 3) & 0x1FFFFFFF)
#define M2S_CAN_GET_IDE(_reg)        ((_reg >> 3) & 0x1FFFFFFF)
#define M2S_CAN_GET_IDS(_reg)        ((_reg >> 21) & 0x7FF)
/* Tx Mailbox Command/Control Fields */ 
#define M2S_CAN_TXINTEN     (1 << 2)
#define M2S_CAN_TXABRT      (1 << 1)
#define M2S_CAN_TXRQ        (1 << 0)
#define M2S_CAN_TX_WPNH     (1 << 23)
#define M2S_CAN_TX_WPNL     (1 << 3)
/* Rx Mailbox Command/Control Fields */ 
#define M2S_CAN_RX_WPNH     (1 << 23)
#define M2S_CAN_RX_RTR      (1 << 21)
#define M2S_CAN_RX_WPNL     (1 << 7)
#define M2S_CAN_RX_LF       (1 << 6)
#define M2S_CAN_RXINT_EN    (1 << 5)
#define M2S_CAN_RTR_EN      (1 << 4)
#define M2S_CAN_RXBUF_EN    (1 << 3)
#define M2S_CAN_RX_MSGAV    (1 << 0)
#define M2S_CAN_RX_ENABLE   (M2S_CAN_RX_WPNH | M2S_CAN_RX_WPNL | M2S_CAN_RXINT_EN | M2S_CAN_RXBUF_EN | M2S_CAN_RX_LF)

// Interrupt Status/Enable Fields
#define M2S_INT_SSTFAIL     (1 << 15)
#define M2S_INT_STUCK       (1 << 14)
#define M2S_INT_RTR         (1 << 13)    
#define M2S_INT_RXMSG       (1 << 12)
#define M2S_INT_TXMSG       (1 << 11)
#define M2S_INT_RXLOST      (1 << 10)
#define M2S_INT_BUSOFF      (1 << 9)
#define M2S_INT_CRCERR      (1 << 8)
#define M2S_INT_FORMERR     (1 << 7)
#define M2S_INT_ACKERR      (1 << 6)
#define M2S_INT_STUFFERR    (1 << 5)
#define M2S_INT_BITERR      (1 << 4)
#define M2S_INT_OVRLOAD     (1 << 3)
#define M2S_INT_ARBLOSS     (1 << 2)
#define M2S_INT_GLOBAL      (1 << 0) // only applies to enable register
#define M2S_INT_ERRS        (M2S_INT_BUSOFF | M2S_INT_CRCERR | M2S_INT_FORMERR | M2S_INT_ACKERR | M2S_INT_STUFFERR | M2S_INT_BITERR | M2S_INT_OVRLOAD | M2S_INT_ARBLOSS)


// Register access macros 
#define M2S_CAN_REG(d)      ((volatile struct m2s_can_cfg_regs *)d->reg)

/******************************************************************************
 * Local variables and function prototypes
 ******************************************************************************/
 //--------------------------define M2S CAN Device Descriptor
/*
 * M2S CAN Device descriptor 
 */
struct m2s_can_dev {
    
    struct  can_priv            can; // MUST be first member, defined in dev.h
    void    __iomem             *reg; /* CAN registers */
    struct  net_device          *ndev;
    struct  device              *dev;
    struct  napi_struct         napi;
    spinlock_t mbx_lock; // Mailbox registers need protection
    u32			rx_next;
};
 
/*
 * CAN hardware-dependent bit-timing constant
 * Used for calculating and checking bit-timing parameters in dev.c
 */
 static struct can_bittiming_const m2s_bittiming_const = {
    .tseg1_min  = 3,    /* Time segement 1 = prop_seg + phase_seg1 */
    .tseg1_max  = 16,
    .tseg2_min  = 2,    /* Time segement 2 = phase_seg2 */
    .tseg2_max  = 8,
    .sjw_max    = 4,    /* Synchronisation jump width */ // Bosch: SJW shall be prog between 1 and min(4, PHASE_SEG1)
    .brp_min    = 1,    /* Bit-rate prescaler */
    .brp_max    = 32768,
    .brp_inc    = 1,
 };
 
 
 
/* CAN TX MSG OBJECT */
struct can_tx_msg_object {
    union 
    {
        u32 L;     /* 32 bit flag */
        struct 
        {
            u32 TXREQ:1;      /* [0] TxReq */
            u32 TXABORT:1;    /* [1] TxAbort */
            u32 TXINTEBL:1;   /* [2] TxIntEbl */
            u32 WPNL:1;       /* [3] Write protect not for bit 2 */
            u32 NA0:12;       /* [4..15] Message Valid Bit, 0 == Not valid. */
            u32 DLC:4;        /* [16..19] Data Length Code (number of bytes in a CAN message) */
            u32 IDE:1;        /* [20] Extended Identifier Bit 1 == 29 bit identifier, 0 == 11 bit identifier. */
            u32 RTR:1;        /* [21] Remote Bit  0 == regular message */
            u32 NA1:1;        /* [22] Reserved */
            u32 WPNH:1;       /* [23] Write protect not for bits 21..16 */
            u32 NA2:8;        /* [24..31] */
        };
    } TXB; /* TX_MSGn_CTRL_CMD */
    
    /* CAN Message ID */
    union
    {
        u32 L;
        struct
        {
            u32 N_ID:3;
            u32 REAL_ID:29;
        };
    }  ID;           
    /* CAN Message Data organized as two 32 bit words or 8 data bytes */
    union
    {
        struct
        {
            u32 DATAHIGH;
            u32 DATALOW;
        };
        u8 DATA[8];                 
    };
};

struct can_rx_msg_object
{
    /* CAN Message flags and smaller values organized as one single 32 bit word 
       or a number of bit fields. */
    union
    {
        u32 L;                 /* 32 bit flag */

        /* Tx Flags structure. */
        struct
        {
            u32 MSGAV:1;         /* [0] MSGAV */
            u32 RTRREPLYPEND:1;  /* [1] RTRREPLY_PENDING */
            u32 RTRABORT:1;      /* [2] RTRABORT */
            u32 BUFFEREBL:1;     /* [3] BUFFERENABLE */
            u32 RTRREPLY:1;      /* [4] RTRREPLY */
            u32 RXINTEBL:1;      /* [5] RXINTEBL */
            u32 LINKFLAG:1;      /* [6] LINKFLAG */
            u32 WPNL:1;          /* [7] Write protect not for bit 6..3 */
            u32 NA0:8;           /* [8..15] Message Valid Bit, 0 == Not valid. */
            u32 DLC:4;           /* [16..19] Data Length Code (number of bytes in a CAN message) */
            u32 IDE:1;           /* [20] Extended Identifier Bit 1 == 29 bit identifier, 0 == 11 bit identifier. */
            u32 RTR:1;           /* [21] Remote Bit  0 == regular message */
            u32 NA1:1;           /* [22] Reserved */
            u32 WPNH:1;          /* [23] Write protect not for bits 21..16 */
            u32 NA2:8;           /* [24..31] */
        };
    } RXB; /* RX_MSGn_CTRL_CMD */
    
    /* CAN Message ID.  */
    union
    {
        u32 L;
        struct
        {
            u32 N_ID:3;
            u32 REAL_ID:29;
        };
    } ID; 
             
    /* CAN Message Data organized as two 32 bit words or 8 data bytes */
    union
    {
        struct
        {
            u32 DATAHIGH;
            u32 DATALOW;
        };
        u8 DATA[8];
    };
    
    /* CAN Message Filter: Acceptance mask register */
    union
    {
        u32 L;
        struct
        {
            u32 N_A:1;
            u32 RTR:1;
            u32 IDE:1;
            u32 ID:29;
       };
    } AMR;

    /* CAN Message Filter: Acceptance code register */
    union
    {
        u32 L;
        struct
        {
            u32 N_A:1;
            u32 RTR:1;
            u32 IDE:1;
            u32 ID:29;
        };
    } ACR;
    
    u32 AMR_D;
    u32 ACR_D;
};




/*
 * MSS CAN Controller Register Map, accessed via M2S_CAN_CFG macro
 */
 
struct m2s_can_cfg_regs {
    u32                         status;         /* 0x000 Interrupt status register */
    u32                         int_en;         /* 0x004 Interrupt enable register */
    u32							rx_buf_status;	/* 0x008 Receive messageAv buffer status */
    u32                         tx_buf_status;  /* 0x00C Transmit message buffer status */  
    u32                         err_status;     /* 0x010 CAN error status indicator register */
    u32                         cmd;            /* 0x014 CAN operating mode */
    u32                         cfg;            /* 0x018 CAN configuration register  */
    u32                         ecr;            /* 0x01C Error capture register */
    struct can_tx_msg_object    tx_msg[CAN_TX_MAILBOX];      /* 0x020-0x21C Transmit message0 buffer control and command register */
    struct can_rx_msg_object    rx_msg[CAN_RX_MAILBOX];      /* 0x220-0x61C */
    
    
};


/******************************************************************************
 * Functions local to this module
******************************************************************************/
/*
 * Dump CAN registers
 */
static void m2s_can_dump(struct m2s_can_dev *prv)
{
    u32 val;
    
    printk(KERN_INFO "CAN registers dump:\n");
    // Release CAN controller from reset 
    val = readl(&M2S_SYSREG->soft_reset_cr);
    printk(KERN_INFO "value in soft_reset_cr before write=0x%08x\n", val);    
    writel(val &= ~M2S_SYS_SOFT_RST_CR_CAN, &M2S_SYSREG->soft_reset_cr);    
    printk(KERN_INFO "value in soft_reset_cr after write=0x%08x\n", readl(&M2S_SYSREG->soft_reset_cr));
    
    // read registers
    printk(KERN_INFO "INT_STATUS=%08x; INT_EN=%08x; TX_BS=%08x; ERR_STATS=%08x; CMD=%08x; CFG=%08x\n",
    readl(&M2S_CAN_REG(prv)->status), readl(&M2S_CAN_REG(prv)->int_en), readl(&M2S_CAN_REG(prv)->tx_buf_status),
    readl(&M2S_CAN_REG(prv)->err_status), readl(&M2S_CAN_REG(prv)->cmd), readl(&M2S_CAN_REG(prv)->cfg));
    
    printk(KERN_INFO "Register read/write test..\n");
    printk(KERN_INFO "Read from sys_reg->m3_cr 0x%08x\n",
        readl(M2S_SYSREG->m3_cr));

        
    // put CAN controller back in reset
    writel(val |= M2S_SYS_SOFT_RST_CR_CAN, &M2S_SYSREG->soft_reset_cr);       
    printk(KERN_INFO "value in soft_reset_cr after write=0x%08x\n", readl(&M2S_SYSREG->soft_reset_cr));    
}

/*
* Base on MSS_CAN_init()
*/ 
static int m2s_chip_init(struct m2s_can_dev *prv)
{   
    u32 val;
    int i, rv;
    
    rv = 0;
    
    // Initialise device structure already done in platform device instantiation

    CDBG(KERN_ALERT "%s: Enter driver %s \n", __func__, DRV_NAME);

    // Release CAN controller from reset 
    val = readl(&M2S_SYSREG->soft_reset_cr);
    writel(val &= ~M2S_SYS_SOFT_RST_CR_CAN, &M2S_SYSREG->soft_reset_cr);
    
    // Clear pending IRQs
    
    // Initialise RX mailbox
    for (i = 0; i < CAN_RX_MAILBOX; i++)
    {
        // set ID, data high data low, AMR, ACR, AMRD ACRD and RXB flags to 0
        writel(0, &M2S_CAN_REG(prv)->rx_msg[i].ID.L);
        // acceptance filter bits set to "don't care" 
        writel(0xFFFFFFFF, &M2S_CAN_REG(prv)->rx_msg[i].AMR.L);
        writel(0xFFFFFFFF, &M2S_CAN_REG(prv)->rx_msg[i].AMR_D);
        writel(0, &M2S_CAN_REG(prv)->rx_msg[i].ACR.L);
        writel(0, &M2S_CAN_REG(prv)->rx_msg[i].ACR_D);
        writel(0, &M2S_CAN_REG(prv)->rx_msg[i].RXB.L);
        writel(0, &M2S_CAN_REG(prv)->rx_msg[i].DATAHIGH);
        writel(0, &M2S_CAN_REG(prv)->rx_msg[i].DATALOW);
    }
    
    // Configure CAN controller before controller is started 
    // Controller has to be configured prior to its use, including
    // - CAN data synchronization
    // - message buffer arbitration 

    // Set arbitration, endianness and 100 kHz bit rate in CAN_CONFIG register 
    writel(M2S_CAN_CFG_DEFAULT | CAN_SPEED_50M_100K, &M2S_CAN_REG(prv)->cfg);

    // check CAN mode 
    val = readl(&M2S_CAN_REG(prv)->cmd);
    
    // output CAN revision_ctrl register for major, minor and rev number
    printk(KERN_ALERT "%s: driver %s ver major %d minor %d rev %d, mode 0x%x\n",
		       __func__, DRV_NAME, (val & 0xF0000000) >> 28, (val & 0x0F000000) >> 24, (val & 0x00FF0000) >> 16, val);
    
    // Disable interrupts
    // writel(DISABLE, &M2S_CAN_REG(prv)->int_en);
    
    return rv;
}
 
static int m2s_chip_start(struct net_device *ndev)
{
    struct m2s_can_dev *prv = netdev_priv(ndev);
    int val, rv, i;
      
    CDBG(KERN_ALERT "%s: Enter driver %s \n", __func__, DRV_NAME);
    //m2s_can_dump(prv);
    rv = 1;
    // clear all pending interrupts
    writel(DISABLE, &M2S_CAN_REG(prv)->status);
    
    // Enable CAN device
    val = readl(&M2S_CAN_REG(prv)->cmd);
    // enable loopback (just for debug)
    // val |= M2S_CAN_CMD_LPBK;
    // val |= M2S_CAN_CMD_LISTEN;
    
    writel(val |= M2S_CAN_CMD_RUNSTOP,&M2S_CAN_REG(prv)->cmd);
    CDBG(KERN_ALERT "%s M2S_CAN_REG_cmd = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->cmd));
    CDBG(KERN_ALERT "%s M2S_CAN_REG_cfg = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->cfg));
    
    // Enable receive interrupts for each mailbox  
    for (i = 0; i < CAN_RX_MAILBOX-1; i++) {
        writel(M2S_CAN_RX_ENABLE, &M2S_CAN_REG(prv)->rx_msg[i].RXB.L);
    }    
    // The last buffer of an array MAY not have it's link flag set 
    writel(M2S_CAN_RX_ENABLE & ~M2S_CAN_RX_LF, &M2S_CAN_REG(prv)->rx_msg[i].RXB.L);
    
    // Enable interrupt types
    val = M2S_INT_RXMSG | M2S_INT_TXMSG | M2S_INT_RXLOST | M2S_INT_BUSOFF | M2S_INT_GLOBAL;
    // val = M2S_INT_TXMSG | M2S_INT_RXLOST | M2S_INT_BUSOFF | M2S_INT_GLOBAL;
    writel(val, &M2S_CAN_REG(prv)->int_en);
    CDBG(KERN_ALERT "%s: Exit driver %s \n", __func__, DRV_NAME);    
    
    return rv;
}

static int m2s_chip_stop(struct net_device *ndev)
{
	struct m2s_can_dev *prv = netdev_priv(ndev);    
    int val;
    
    // Disable CAN device
    val = readl(&M2S_CAN_REG(prv)->cmd);
    writel(val |= M2S_CAN_CMD_RUNSTOP, &M2S_CAN_REG(prv)->cmd);
 
    return 0;
}


// for non NAPI polling method
static int m2s_process_rx(struct net_device *ndev, int mailbox)
{
    struct net_device_stats *stats = &ndev->stats;
    struct m2s_can_dev *prv = netdev_priv(ndev);
    struct can_frame *cf;
    struct sk_buff *skb;
    u32 val, id;
    int high, low, numbytes;
    int done = 0;
    
    // allocate a generic buff which marks the buffer as a CAN frame
    // and lets "cf" point to the can_frame struct
    // skb = alloc_can_skb(ndev, &cf); 
    skb = dev_alloc_skb(sizeof(struct can_frame));
    CDBG(KERN_INFO "%s: allocated %d bytes for can_frame socket buffer", __func__, sizeof(struct can_frame));
    
    if (!skb) {
        stats->rx_dropped++;
        printk(KERN_INFO "%s: alloc_can_skb() failed!\n", __func__);
        return 0;
    }
    
    cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

    // assign data length code to can frame
    val = readl(&M2S_CAN_REG(prv)->rx_msg[mailbox].RXB.L);
    numbytes = M2S_CAN_GET_DLC(val);
    cf->can_dlc = get_can_dlc(numbytes);
    
    // assigned identifier
    id = readl(&M2S_CAN_REG(prv)->rx_msg[mailbox].ID.L);
    CDBG(KERN_INFO "%s: rx_msg[%d].ID=0x%08X\n", __func__, mailbox, id);  
    
    // check for extended identifier bit
    val = readl(&M2S_CAN_REG(prv)->rx_msg[mailbox].RXB.L);
    if (val & M2S_CAN_IDE) 
	{
		cf->can_id = M2S_CAN_GET_IDE(id);
        cf->can_id |= CAN_EFF_FLAG;    
        CDBG(KERN_INFO "%s: EXTENDED ID = 0x%03X", __func__, M2S_CAN_GET_IDE(id));
    } else {
		cf->can_id = M2S_CAN_GET_IDS(id);
        CDBG(KERN_INFO "%s: STANDARD ID = 0x%03X", __func__, M2S_CAN_GET_IDS(id));
	}
	
    if (numbytes > 0) 
    {
        low = readl(&M2S_CAN_REG(prv)->rx_msg[mailbox].DATALOW); // bytes 7:4
        high = readl(&M2S_CAN_REG(prv)->rx_msg[mailbox].DATAHIGH); // bytes 3:0


        *(u32 *)(cf->data + 0) = high;
        *(u32 *)(cf->data + 4) = low;    

        CDBG(KERN_INFO "%s: rxb[%d] ID=0x%08x datahigh=0x%08x datalow=0x%08x", __func__, mailbox, cf->can_id, high, low);
        CDBG(KERN_INFO "%s: numbytes=%d can_dlc=%d\n",__func__, numbytes, cf->can_dlc);
        
    }
    
    /*
     * Push packet to the upper layer
     */
    stats->rx_packets++;
    stats->rx_bytes += cf->can_dlc;    
    
    skb->dev = ndev;
    skb->protocol = htons(ETH_P_CAN);
    skb->pkt_type = PACKET_BROADCAST;
    skb->ip_summed =  CHECKSUM_UNNECESSARY;
    
    // pass the buffer up to the protocol layers
    if (netif_rx(skb) == NET_RX_SUCCESS) {
        CDBG(KERN_INFO "%s: packet received successfully", __func__);
    }
    done++;

    // clear the data registers for this mailbox
    writel(0, &M2S_CAN_REG(prv)->rx_msg[mailbox].DATALOW);
    writel(0, &M2S_CAN_REG(prv)->rx_msg[mailbox].DATAHIGH);
    
    /*
     * Ack mail-box
     */
    writel(val | M2S_CAN_RX_MSGAV, &M2S_CAN_REG(prv)->rx_msg[mailbox].RXB.L);
    
    // clear the data registers for this mailbox

    
    return done;
}


static irqreturn_t m2s_irq(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct net_device_stats *stats = &ndev->stats;
	struct m2s_can_dev *prv = netdev_priv(ndev);    
    int mb, val, int_status, txb, rx_buf_status, tx_buf_status;
    
    // Read INT_STATUS register
    int_status = readl(&M2S_CAN_REG(prv)->status);
    
    // Clear interrupts ASAP
    writel(0xFFFF, &M2S_CAN_REG(prv)->status);   
    
    // Check status of receive mailboxes
    rx_buf_status = readl(&M2S_CAN_REG(prv)->rx_buf_status);
    // printk(KERN_INFO "%s: RX BUF STATUS 0x%08x\n", __func__, rx_buf_status);
    
    // Check if interrupt is for rx message available
    if (!(int_status & (M2S_INT_RXMSG | M2S_INT_RXLOST)))
        goto no_receive;    

    // Just a printk for RX MSG LOST at this point 
    // TODO: more handling
    if (int_status & M2S_INT_RXLOST)
        printk(KERN_INFO "%s: RX MSG LOST, RX_BUF_STATUS = 0x%08x\n", __func__, rx_buf_status);            
    
    /* ------------------------------------------------------------------------------
     *  Process RX
     * -----------------------------------------------------------------------------*/
    /*
        For polling mode, disable interrupts and NOTIFY NAPI to take control
    */
    if (NAPI_ENABLED)
    {
        // disable rx_msg and rx msg lost interrupts
        val = readl(&M2S_CAN_REG(prv)->int_en);
        writel(val & ~(M2S_INT_RXMSG | M2S_INT_RXLOST), &M2S_CAN_REG(prv)->int_en);
        
        printk(KERN_INFO "%s: Scheduling napi, RX_BUF_STATUS = 0x%08x\n", __func__, rx_buf_status);
        napi_schedule(&prv->napi); 
    }     
    else    
    {    
        /*
            For non-polling mode, check all mailboxes and process available messages
        */     
        mb = 0;
        while (mb < CAN_RX_MAILBOX && rx_buf_status)
        {
            /*
             * if mailbox has pending message 
             */
            if (rx_buf_status & (1 << mb))
            {
                CDBG(KERN_INFO, "%s: RX_BUF_STATUS = 0x%08X mb=%d", __func__, rx_buf_status, mb);
                
                // Check receive message control and command reg
                val = readl(&M2S_CAN_REG(prv)->rx_msg[mb].RXB.L);
                
                // If no message available, move on to next mailbox
                if (!(val & M2S_CAN_RX_MSGAV))
                    continue; 
                
                // process received frame immediately
                m2s_process_rx(ndev, mb);
                
                // update status of receive mailboxes
                rx_buf_status = readl(&M2S_CAN_REG(prv)->rx_buf_status);
            }
            mb++;
        }    
    }
    
    // if no other active interrupts
    if (!(int_status & ~(M2S_INT_RXMSG | M2S_INT_RXLOST)))
        return IRQ_HANDLED;
    
no_receive:

    // Check if interrupt is for tx message transmitted
    if (!(int_status & M2S_INT_TXMSG))
        goto no_xmit;

    /* 
     *  Process TX
     */    
    tx_buf_status = readl(&M2S_CAN_REG(prv)->tx_buf_status);
    for (mb = 0; mb < CAN_TX_MAILBOX; mb++)
    {
        // if mailbox still has pending message 
        if (tx_buf_status & (1 << mb))
            continue;
        
        // if no pending message i.e. mailbox free and mailbox's interrupt enabled
        // then mailbox was recently used for transmission
        txb = readl(&M2S_CAN_REG(prv)->tx_msg[mb].TXB.L);
        if (txb & (M2S_CAN_TXINTEN))
        {
            if (txb & M2S_CAN_TXRQ) // Safety check, txb must not be changed while TxReq is 1
            {
                printk(KERN_INFO "%s: ERROR! Mailbox buffer status is STILL pending TXB[%d] = 0x%08X\n", __func__, mb, txb);
                continue;
            }
            CDBG(KERN_INFO "%s: TXB for mb[%d] =  0x%08x", __func__, mb, txb);
            // clear the mb interrupt enable bit
            writel(M2S_CAN_TX_WPNH | M2S_CAN_TX_WPNL, &M2S_CAN_REG(prv)->tx_msg[mb].TXB.L);
            
            // get the skb from the stack and loop it back locally
            can_get_echo_skb(ndev, mb);
            
            // Update statistics to record packet transmission
            stats->tx_packets++;
            stats->tx_bytes += M2S_CAN_GET_DLC(txb);
            
        }
    }
    // if transmit buffer is cleared and queue buffer was stopped, restart queue
    // Able to accept packets for transmission again:
    val = readl(&M2S_CAN_REG(prv)->tx_buf_status);
    if (val == 0)
        netif_wake_queue(ndev);
    
    // if no other active interrupts
    if (!(int_status & ~(M2S_INT_TXMSG | M2S_INT_RXMSG | M2S_INT_RXLOST)))
        return IRQ_HANDLED;
    
no_xmit:

    /*
     * Handle errors 
     */ 
    if (int_status & M2S_INT_ERRS)
    {
        if (int_status & M2S_INT_BUSOFF)
            printk(KERN_INFO "%s: The CAN controller entered the bus-off error state\n", __func__);
        if (int_status & M2S_INT_CRCERR)
            printk(KERN_INFO "%s: A CAN CRC error was detected\n", __func__);
        if (int_status & M2S_INT_FORMERR)
            printk(KERN_INFO "%s: A CAN format error was detectedt\n", __func__);        
        if (int_status & M2S_INT_ACKERR)
            printk(KERN_INFO "%s: A CAN message acknowledgement error was detected\n", __func__);     
        if (int_status & M2S_INT_STUFFERR)
            printk(KERN_INFO "%s: A CAN bit stuffing error is detected\n", __func__);
        if (int_status & M2S_INT_BITERR)
            printk(KERN_INFO "%s: A CAN bit error is detected\n", __func__);        
        if (int_status & M2S_INT_OVRLOAD)
            printk(KERN_INFO "%s: A CAN overload message is detected\n", __func__);        
        if (int_status & M2S_INT_ARBLOSS)
            printk(KERN_INFO "%s: Message arbitration was lost while sending a message\n", __func__);                
    }
    
    
    if (int_status & M2S_INT_SSTFAIL)
        printk(KERN_INFO "%s: A buffer set for single shot transmission experienced an arbitration loss or a bus error during transmission\n", __func__);
    if (int_status & M2S_INT_STUCK)
        printk(KERN_INFO "%s: Stuck at dominant error, RX input remains stuck at 0 for more than 11 consecutive bit times\n", __func__);
    if (int_status & M2S_INT_RTR)
        printk(KERN_INFO "%s: A RTR auto-reply message was sent\n", __func__);
        

    return IRQ_HANDLED;
}
/******************************************************************************
 * CAN network driver interface
 ******************************************************************************/
static int m2s_set_bittiming(struct net_device *ndev)
{
	struct m2s_can_dev *prv = netdev_priv(ndev);
	struct can_bittiming *bt = &prv->can.bittiming;
	u32 val;
	int rv;

    CDBG(KERN_ALERT "%s: Enter driver %s \n", __func__, DRV_NAME);
    
	/* Enter Initialization mode */
	rv = m2s_chip_init(prv);
	if (rv) 
    {
		dev_err(prv->dev, "enter initialization mode err (%d)\n", rv);
		goto out;
	}

	/* Configure BIT RATE*/
	val = CAN_SET_SJW(bt->sjw) |
	       CAN_SET_TSEG1(bt->prop_seg + bt->phase_seg1 - 1) |
	       CAN_SET_TSEG2(bt->phase_seg2 - 1) |
	       CAN_SET_BITRATE(bt->brp - 1);
    val |= M2S_CAN_CFG_DEFAULT;
	writel(val, &M2S_CAN_REG(prv)->cfg);
    
    CDBG(KERN_ALERT "%s: Exit driver %s \n", __func__, DRV_NAME);
out:
	CDBG("%s: brp=%d,sjw=%d,prop=%d,ps1=%d,ps2=%d,rv=%d\n", __func__,
		bt->brp, bt->sjw, bt->prop_seg, bt->phase_seg1, bt->phase_seg2,
		rv);
    
	return rv;
}


static int m2s_set_mode(struct net_device *dev, enum can_mode mode)
{
	int	rv;

	switch (mode) 
    {
	case CAN_MODE_START:
		rv = m2s_chip_start(dev);
		if (rv)
			break;

		netif_wake_queue(dev);
		rv = 0;
		break;
	default:
		rv = -EOPNOTSUPP;
		break;
	}

	return rv;
}



/******************************************************************************
 * Network device interface
 ******************************************************************************/  
 // Request any system resources needed and tells the interface to come up
static int m2s_open(struct net_device *ndev)
{
	struct m2s_can_dev *prv = netdev_priv(ndev);
	struct device *dev = prv->dev;
	int rv;    
    
    
    //m2s_can_dump(prv); // used for debug only
    CDBG(KERN_ALERT "%s: Enter driver %s \n", __func__, DRV_NAME);
    
 	/*
	 * Check/determine and set bittime
	 */        
    rv = open_candev(ndev); // defined in dev.c
    if (rv) 
    {
        dev_err(dev, "open_candev() err (%d)\n",rv);
        goto out;
    }
    
 	/*
	 * Register interrupt handler
	 */
    if (request_irq(ndev->irq, m2s_irq, IRQF_SHARED, ndev->name, ndev)) 
    {
        dev_err(dev, "request_irq(%d) fail\n", ndev->irq);
        rv = -EAGAIN;
        goto out_candev;
	}
    
 	/*
	 * Start chip and queuing
	 */
	m2s_chip_start(ndev);
    
    // enable NAPI scheduling
	napi_enable(&prv->napi);
    
    // Start interfaces transmit queue 
    // Enables transmission for the device. Usually called when the device is activated 
    // Can be called again later if needed to restart a stopped device
	netif_start_queue(ndev);    
    
    
    rv = 0;
    goto out;
    
out_candev:
    close_candev(ndev);
out:
    return rv;  
};          

static int m2s_close(struct net_device *ndev)
{
    struct m2s_can_dev *prv = netdev_priv(ndev);
    int rv;
    
    // Mark device as being unable to transmit any more packets
    netif_stop_queue(ndev);
    
    //disable NAPI scheduling
    napi_disable(&prv->napi);
    
    rv = m2s_chip_stop(ndev);
    if (rv)
        goto out;
    
    free_irq(ndev->irq, ndev);
    
    close_candev(ndev);
    rv = 0;
    
out:
    return rv;
};
// Based on MSS_CAN_send_message_n
static netdev_tx_t m2s_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct m2s_can_dev *prv = netdev_priv(ndev);
	struct can_frame *cf = (void *)skb->data;
	netdev_tx_t rv;
	u32 val, txb, tx_buf_status;
	int mb;
    u32 dlc;

    // Read TX mailbox status register 
    CDBG(KERN_INFO "%s: TX BUF STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->tx_buf_status));
    CDBG(KERN_INFO "%s: RX BUF STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->rx_buf_status));
    CDBG(KERN_INFO "%s: CAN INT STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->status));
    CDBG(KERN_ALERT "%s M2S_CAN_REG_cmd = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->cmd));
    CDBG(KERN_ALERT "%s ERROR STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->err_status));
    CDBG(KERN_ALERT "%s ERROR CAPTURE = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->ecr));
    
	/*
	 * Get empty TX mailbox
	 */

    tx_buf_status = readl(&M2S_CAN_REG(prv)->tx_buf_status);
	for (mb = 0; mb < CAN_TX_MAILBOX; mb++) 
    {
        if (tx_buf_status >> mb)      // start from next mailbox after lowest priority pending mailbox 
            continue;
        
		val = readl(&M2S_CAN_REG(prv)->tx_msg[mb].TXB.L);
        CDBG(KERN_INFO "%s: tx_msg[%d] = 0x%08x\n", __func__, mb, val);
        
        // break if mailbox available
		if (!(val & M2S_CAN_TXRQ))
			break;
        // tx abort 
        //writel(M2S_CAN_TXABRT, &M2S_CAN_REG(prv)->tx_msg[mb].TXB.L);
	}
    
    if (mb == CAN_TX_MAILBOX - 1) // if used up all mailboxes 
    {
        netif_stop_queue(ndev);
        CDBG(KERN_INFO "%s: QUEUE stopped!",__func__);
    }
    
    /*
     * if no mailboxes available... 
     */
	if (!(mb < CAN_TX_MAILBOX)) 
    {
        printk(KERN_INFO "%s: TX BUF STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->tx_buf_status));
        printk(KERN_INFO "%s: TX ID 0x%X failed to send = 0x%08x 0x%08x\n", __func__, cf->can_id, *(u32 *)(cf->data), *(u32 *)(cf->data+4));
		rv = NETDEV_TX_BUSY;
        
        // All buffers occupied - transmit resources unavailable
        // Stop upper layers from calling the hard_start_xmit routine
        netif_stop_queue(ndev);
        printk("%s: Error, NETDEV_TX_BUSY", __func__);
		goto out;
	}

    // Put the skb on the stack to be looped back locally
    // once the packet has been successfully transmitted 
	can_put_echo_skb(skb, ndev, mb);

	/*
	 * Program mailbox, and trigger transmission
	 */

    // set data length code 
    dlc = cf->can_dlc;
	txb = M2S_CAN_SET_DLC(dlc);

    // write data to mailbox data registers 
    if (dlc > 0)
    {
        writel(*(u32 *)(cf->data), &M2S_CAN_REG(prv)->tx_msg[mb].DATAHIGH);  // write CAN data bytes 3:0 
        if (dlc > 4)
            writel(*(u32 *)(cf->data+4), &M2S_CAN_REG(prv)->tx_msg[mb].DATALOW);     // write CAN data bytes 7:4
    }

    // load identifier 
	if (cf->can_id & CAN_EFF_FLAG) // check for identifier extension bit 
	{
		txb |= M2S_CAN_IDE;
		val = M2S_CAN_SET_IDE(cf->can_id);
	} else
	{
		txb &= ~M2S_CAN_IDE;
		val = M2S_CAN_SET_IDS(cf->can_id);
	}
    
        
    CDBG(KERN_INFO "%s: CAN ID = 0x%08x original can_id=%d\n", __func__, val, cf->can_id);
    writel(val, &M2S_CAN_REG(prv)->tx_msg[mb].ID.L);
        
    
	if (cf->can_id & CAN_RTR_FLAG)
		txb |= M2S_CAN_RX_RTR;
    CDBG("%s: Writing 0x%08x to tx_msg[%d]\n", __func__, txb | M2S_CAN_TXRQ | M2S_CAN_TX_WPNH | M2S_CAN_TX_WPNL, mb);
    
    // Disable write protects on TX control reg, IDE and RTR config, and enable TX interrupts on this mailbox
	writel(txb | M2S_CAN_TXRQ | M2S_CAN_TX_WPNL | M2S_CAN_TX_WPNH | M2S_CAN_TXINTEN, &M2S_CAN_REG(prv)->tx_msg[mb].TXB.L);
    
    // Successful start of transmission to packet scheduler
	rv = NETDEV_TX_OK;

    if (cf->can_id & 0x1) // If last packet of Inovor Bus Protocol Datagram
        CDBG(KERN_INFO "%s: Sent CAN ID %d (%d)", __func__, cf->can_id, cf->can_id >> 1);
    
    CDBG("%s: Read back 0x%08x from tx_msg[%d]\n", __func__, readl(&M2S_CAN_REG(prv)->tx_msg[mb].TXB.L), mb);
    CDBG(KERN_INFO "%s: CAN INT STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->status));
    CDBG(KERN_INFO "%s: RX BUF STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->rx_buf_status));
    CDBG(KERN_ALERT "%s ERROR STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->err_status));
    CDBG(KERN_ALERT "%s ERROR CAPTURE = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->ecr));    
    CDBG(KERN_ALERT "%s DATA HIGH = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->tx_msg[mb].DATAHIGH));    
    CDBG(KERN_ALERT "%s DATA LOW = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->tx_msg[mb].DATALOW));   
    
out:
	CDBG("%s: dlc=%d,mb=%d,rv=%d\n", __func__, cf->can_dlc, mb, rv);

	return rv;
};
/*
    Contingent on capacity to buffer incoming CAN frames until the NAPI handler starts.
    NAPI used to minimise time inside interrupt handler and decrease IRQ load, as multiple
    CAN frames can be processed within a single scheduled NAPI request.

    The budget parameter specifies the REMAINING number of packets the driver is allowed to pass into the network stack 
    on this call before yielding to other system tasks. Drivers should simply respect budget and return the number of packets which were
    actually processed. i.e. NOT SURE IF THIS IS STILL CORRECT: Each driver is responsible for decrementing budget by the total number of packets sent.

    Total number of packets CANNOT EXCEED dev->quota.
    
    Budget is the only indicator of how many packets the poll() function may feed into the kernel 
*/
static int m2s_poll(struct napi_struct *napi, int quota)
{
		struct net_device *ndev = napi->dev;
		struct m2s_can_dev *prv = netdev_priv(ndev);
		unsigned long flags;
		u32 rx_buf_status, val, i;
		int done;
        
        // a driver is allowed to send up to dev->quota packets by the current CPU 
        // before yielding to the network subsystem 
        // rx_work_limit = ndev->quota;
        // printk(KERN_INFO "%s: rx_work_limit = %d\n", __func__, rx_work_limit); 
        
        // Since ti_hecc.c doesn't use ndev->quota, will just observe for now
    
        if (!(netif_running(ndev)))
            return 0;
        
        // Read RX mailbox status register 
        printk(KERN_INFO "%s: RX BUF STATUS = 0x%08x\n", __func__, readl(&M2S_CAN_REG(prv)->rx_buf_status));    
 		i = done = 0;
        
 		// iterate through mailboxes and check if any messages available 
 		rx_buf_status = readl(&M2S_CAN_REG(prv)->rx_buf_status);
        
		while (done < quota && i < CAN_RX_MAILBOX && rx_buf_status) 
        {
            /*
             * if mailbox has pending message 
             */            
            if (rx_buf_status & (1 << i))
            {
                // Check receive message control and command reg
                val = readl(&M2S_CAN_REG(prv)->rx_msg[i].RXB.L);
                
                // If no message available, move on to next mailbox
                if (!(val & M2S_CAN_RX_MSGAV))
                    continue; 
                
                // process received frame 
                done += m2s_process_rx(ndev, i);
                
                // start from first mailbox again in case message has been received since last checked
                i = 0;        
                
                // update status of receive mailboxes
                rx_buf_status = readl(&M2S_CAN_REG(prv)->rx_buf_status);  

                if (done == quota)
                    break;
            }

            i++;
		}
	
		if (done == quota)
			goto more;
	
		/*
		 * If there are no more packets in RX FIFOs, then complete NAPI, and
		 * enable interrupts
		 */
        if (rx_buf_status)
            goto more;
	
        // return to interrupt driven mode, remove device from polling list
		napi_complete(napi);
        printk("%s: napi_complete, done=%d,quot=%d\n", __func__, done, quota);
	
		local_irq_save(flags);
        
		// enable interrupts
        val = readl(&M2S_CAN_REG(prv)->int_en);
        writel(val | M2S_INT_RXMSG, &M2S_CAN_REG(prv)->int_en);
        for (i = 0; i < CAN_RX_MAILBOX; i++) {
            val = readl(&M2S_CAN_REG(prv)->rx_msg[i].RXB.L);
            writel(val | M2S_CAN_RX_ENABLE, &M2S_CAN_REG(prv)->rx_msg[i].RXB.L);
        }

		local_irq_restore(flags);

more:
	printk("%s: done=%d,quot=%d\n", __func__, done, quota);
	return done;			
	
	    
}

static const struct net_device_ops m2s_netdev_ops = {
    .ndo_open       = m2s_open, 
    .ndo_stop       = m2s_close,
    .ndo_start_xmit = m2s_start_xmit,
};



/******************************************************************************
 * Platform driver interface
 * At minimum, the probe() and remove() callbacks must be supplied
 * the other callbacks have to do with power management and should be provided
 * if they are relevant 
 ******************************************************************************/     
static int __init m2s_can_probe(struct platform_device *pdev)
{
    struct net_device *ndev;
    struct m2s_can_dev *prv;
    struct resource *res;
    struct device  *dev;
    void __iomem *addr;
    int rv, irq;
    
    dev = &pdev->dev; 
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
    
	if (!res || irq <= 0) {
		dev_err(dev, "bad reg/irq %p/%d\n", res, irq);
		rv = -ENODEV;
		goto out;
	}    
    printk(KERN_INFO "%s: irq=%08x; res->start=%08x sizeof(regs)=%08x resource_size=%08x\n", __func__, 
        irq, res->start,sizeof(struct m2s_can_cfg_regs), resource_size(res));
    
	if (!request_mem_region(res->start,
				resource_size(res), pdev->name)) {
		dev_err(dev, "request_mem(0x%x,0x%x,%s) fail\n", res->start,
			resource_size(res), pdev->name);
		rv = -EBUSY;
		goto out;
	}    
    
	addr = ioremap_nocache(res->start, resource_size(res));
    // addr = ioremap_nocache(res->start, sizeof(struct m2s_can_cfg_regs));
	if (!addr) {
		dev_err(dev, "ioremap(0x%x,0x%x) fail\n", res->start,
			resource_size(res));
		rv = -ENOMEM;
		goto out_release;
	}    
    
    // allocate and setup space for the CAN network device (defined in dev.c)
	ndev = alloc_candev(sizeof(struct m2s_can_dev), CAN_TX_MAILBOX);
	if (!ndev) {
		dev_err(dev, "alloc_candev(%d,%d) fail\n",
			sizeof(struct m2s_can_dev), CAN_TX_MAILBOX);
		rv = -ENOMEM;
		goto out_unmap;
	}
    
    ndev->netdev_ops = &m2s_netdev_ops;
    ndev->irq = irq;
    ndev->flags |= IFF_ECHO; // echo sent packets
    
    prv = netdev_priv(ndev); // get network device private data
  	prv->can.clock.freq = m2s_clock_get(CLCK_PCLK1);
	prv->can.bittiming_const = &m2s_bittiming_const;
	prv->can.do_set_bittiming = m2s_set_bittiming;
	prv->can.do_set_mode = m2s_set_mode;
	prv->reg = addr;
	prv->ndev = ndev;
	prv->dev = dev;  
    
    /*
        The weight is a limit on the number of packets the driver will pass 
        to the stack in each polling cycle
        recommended to use a weight of 64 (NAPI_POLL_WEIGHT)
    */
    // initalise a napi context
    netif_napi_add(ndev, &prv->napi, m2s_poll, M2S_NAPI_WEIGHT);
    
    dev_set_drvdata(dev, ndev);
    
    SET_NETDEV_DEV(ndev, dev);
    
    // Register the CAN network device
    rv = register_candev(ndev);
    
    if (rv) {
        dev_err(dev, "registering netdev failed (%d)\n", rv);
        goto out_free;
    }
    
    dev_info(dev, "device registered (reg=%p, irq=%d)\n", addr, irq);
    rv = 0;
    goto out;
    
out_free:
	free_netdev(ndev);
out_unmap:
	iounmap(addr);
out_release:
	release_mem_region(res->start, resource_size(res));
out:
	return rv;
}    

static int __devexit  m2s_can_remove(struct platform_device *pdev)
{
    struct net_device *ndev = platform_get_drvdata(pdev);
	struct m2s_can_dev *prv = netdev_priv(ndev);
	struct resource *res; 
    
	unregister_netdev(ndev);
	platform_set_drvdata(pdev, NULL);
	free_netdev(ndev);
	iounmap(prv->reg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));    
    return 0;
}


static struct platform_driver m2s_can_driver = {
    .probe      = m2s_can_probe, 
    .remove     = __devexit_p(m2s_can_remove),
    .driver     = {
        .name   = DRV_NAME,
        .owner  = THIS_MODULE,
    }, // With this setup, any device identifying itself as "m2s_can" will be bound to this driver, no ID table is needed
};
     
/******************************************************************************
 * Kernel module interface
 ******************************************************************************/
 

 static int __init m2s_can_module_init(void)
{
	int ret = 0;

	/*
 	 * Register device
 	 */
    printk(KERN_INFO "%s netdevice driver\n", DRV_NAME);
	ret = platform_driver_register(&m2s_can_driver);
	if (ret < 0) {
		printk(KERN_ALERT "%s: registering device %s with major %d "
				  "failed with %d\n",
		       __func__, DRV_NAME, dev_major, ret);
		goto Done;
	}
	
Done:
	CDBG("name=%s,major=%d\n", DRV_NAME, dev_major);

	return ret;
}
static void __exit m2s_can_module_exit(void)
{
	/*
	 * Unregister device
	 */
	platform_driver_unregister(&m2s_can_driver);

	printk(KERN_INFO "%s: driver removed \n", DRV_NAME);
}

 // Identify module entry and exit points
 module_init(m2s_can_module_init);
 module_exit(m2s_can_module_exit);

MODULE_AUTHOR("Mia Baquiran <mia.baquiran@csiro.au>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRV_NAME " CAN netdevice driver");
