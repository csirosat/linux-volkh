/* linux/drivers/mmc/host/sdhci-iwave.c
 *
 * (C) Copyright 2019 CSIRO
 * Commonwealth Scientific and Industrial Research Organisation
 * Mike Pilawa <Mike.Pilawa@csiro.au>
 *
 * SDHCI driver shim for iWave Systems SD/MMC 2.0 Host Controller IP Core
 *
 * Based on linux/drivers/mmc/host/sdhci-s3c.c which is...
 *
 * Copyright 2008 Openmoko Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdhci-iwave.h>

#include "sdhci.h"

#define DBG(f, x...) \
	pr_debug("[%s()]: " f, __func__,## x)

/**
 * struct sdhci_iwave - iWave SDHCI instance
 * @host: The SDHCI host created
 * @pdev: The platform device we where created from.
 * @ioarea: The resource created when we claimed the IO area.
 * @pdata: The platform data for this controller.
 */
struct sdhci_iwave {
	struct sdhci_host			*host;
	struct platform_device		*pdev;
	struct resource				*ioarea;
	struct sdhci_iwave_platdata	*pdata;
};

static	u16		sdhci_iwave_readw(struct sdhci_host *host, int reg)
{
	int shift = (reg & 0x2) * 8; // same as rshift 1, mask 0x1, mult 16
	int regl  = reg & 0xFFFFFFFC;
	u32	lword = readl(host->ioaddr + regl);
	return ((lword >> shift) & 0xFFFF); // assumes little-endian
}

static	u8		sdhci_iwave_readb(struct sdhci_host *host, int reg)
{	int shift  = (reg & 0x3) * 8;
	int regl  = reg & 0xFFFFFFFC;
	u32	lword = readl(host->ioaddr + regl);
	return ((lword >> shift) & 0xFF); // assumes little-endian
}

///* Only for write debug messages */
//static	void	sdhci_iwave_writel(struct sdhci_host *host, u32 val, int reg)
//{
//	int regl = reg & 0xFFFFFFFC;
//	writel(val, host->ioaddr + regl);
////	DBG("Reg 0x%02x Val 0x%08x\n", reg & 0xFF, val);
//}

static	void	sdhci_iwave_writew(struct sdhci_host *host, u16 val, int reg)
{
	int shift = (reg & 0x2) * 8; // same as rshift 1, mask 0x1, mult 16
	int regl  = reg & 0xFFFFFFFC;
	u32 mask  = 0xFFFF << shift; // assumes little-endian
	u32 word  = val << shift; // assumes little-endian
	u32	lword = readl(host->ioaddr + regl);
	writel((lword & ~mask) | word, host->ioaddr + regl);
//	DBG("Reg 0x%02x Val 0x%04x\n", reg & 0xFF, val);
}

static	void	sdhci_iwave_writeb(struct sdhci_host *host, u8 val, int reg)
{
	int shift = (reg & 0x3) * 8;
	int regl  = reg & 0xFFFFFFFC;
	u32 mask  = 0xFF << shift; // assumes little-endian
	u32 word  = val << shift; // assumes little-endian
	u32	lword = readl(host->ioaddr + regl);
	writel((lword & ~mask) | word, host->ioaddr + regl);
//	DBG("Reg 0x%02x Val 0x%02x\n", reg & 0xFF, val);
}

//static int sdhci_iwave_enable_dma(struct sdhci_host *host)
//{
//	return 0;
//}

static struct sdhci_ops sdhci_iwave_ops = {
	.ops_readl			= NULL,
	.ops_readw			= sdhci_iwave_readw,
	.ops_readb			= sdhci_iwave_readb,
	.ops_writel			= NULL, //sdhci_iwave_writel,
	.ops_writew			= sdhci_iwave_writew,
	.ops_writeb			= sdhci_iwave_writeb,
	.set_clock			= NULL,
	.enable_dma			= NULL, //sdhci_iwave_enable_dma,
	.get_max_clock		= NULL,
	.get_min_clock		= NULL,
	.get_timeout_clock	= NULL,
};

static int __devinit sdhci_iwave_probe(struct platform_device *pdev)
{
	struct sdhci_iwave_platdata *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_iwave *sc;
	struct resource *res;
	int ret, irq;

	if (!pdata) {
		dev_err(dev, "no device data specified\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sdhci_iwave));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	sc = sdhci_priv(host);

	sc->host = host;
	sc->pdev = pdev;
	sc->pdata = pdata;

	platform_set_drvdata(pdev, host);

	sc->ioarea = request_mem_region(res->start, resource_size(res),
					mmc_hostname(host->mmc));
	if (!sc->ioarea) {
		dev_err(dev, "failed to reserve register area\n");
		ret = -ENXIO;
		goto err_req_regs;
	}

	host->ioaddr = ioremap_nocache(res->start, resource_size(res));
	if (!host->ioaddr) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_req_regs;
	}

	host->hw_name = pdata->hw_name;
	host->ops = &sdhci_iwave_ops;
	host->quirks = 0;
	host->irq = irq;

	/* Setup quirks for the controller */

	/* SDMA not supported on this controller */
	host->quirks |= SDHCI_QUIRK_BROKEN_DMA;

	/* ADMA is supported, but use 32-bit minimum quanta for now */
	host->quirks |= SDHCI_QUIRK_32BIT_ADMA_SIZE;

	/* Use sequenced VDD and power enable */
	host->quirks |= SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER;

	/* Transfer Mode and Command registers must be written
	 * atomically due to 32-bit register access requirement */
	host->quirks |= SDHCI_QUIRK_ATOMIC_XFER_MODE_CMD_REGS;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}

	return 0;

 err_add_host:
	release_resource(sc->ioarea);
	kfree(sc->ioarea);

 err_req_regs:
	sdhci_free_host(host);

	return ret;
}

static int __devexit sdhci_iwave_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM

static int sdhci_iwave_suspend(struct platform_device *dev, pm_message_t pm)
{
	struct sdhci_host *host = platform_get_drvdata(dev);

	sdhci_suspend_host(host, pm);
	return 0;
}

static int sdhci_iwave_resume(struct platform_device *dev)
{
	struct sdhci_host *host = platform_get_drvdata(dev);

	sdhci_resume_host(host);
	return 0;
}

#else
#define sdhci_iwave_suspend NULL
#define sdhci_iwave_resume NULL
#endif

static struct platform_driver sdhci_iwave_driver = {
	.probe		= sdhci_iwave_probe,
	.remove		= __devexit_p(sdhci_iwave_remove),
	.suspend	= sdhci_iwave_suspend,
	.resume		= sdhci_iwave_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sdhci-iwave",
	},
};

static int __init sdhci_iwave_init(void)
{
	return platform_driver_register(&sdhci_iwave_driver);
}

static void __exit sdhci_iwave_exit(void)
{
	platform_driver_unregister(&sdhci_iwave_driver);
}

module_init(sdhci_iwave_init);
module_exit(sdhci_iwave_exit);

MODULE_DESCRIPTION("iWave SDHCI IP core driver shim");
MODULE_AUTHOR("Mike Pilawa, <Mike.Pilawa@csiro.au>");
MODULE_LICENSE("GPL v2");
