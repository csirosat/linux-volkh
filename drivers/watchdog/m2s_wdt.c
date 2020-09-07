/*
 * Watchdog driver for Microsemi M2S SoCs.
 *
 * Copyright (C) 2020 CSIRO
 * Commonwealth Scientific and Industrial Research Organisation
 * Mike Pilawa <Mike.Pilawa@csiro.au>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>

#include <mach/m2s.h>
#include <mach/wdt.h>

#define DRV_NAME	"m2s_wdt"

/* Hardware timeout (seconds) */
#define WDT_HW_TIMEOUT		32

/* Kernel ping timer (seconds) */
#define WDT_KERN_TIMEOUT	(WDT_HW_TIMEOUT/2)

/* User-land timeout (seconds) */
#define WDT_USER_TIMEOUT	60
static int timeout = WDT_USER_TIMEOUT;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, " Watchdog user-land timeout in seconds. "
	"(default=" __stringify(WDT_USER_TIMEOUT) ")");

/* Watchdog HW expire mode: 0=reset; 1=interrupt (NMI) */
/* Interrupt mode not currently supported by driver! */
#define WDT_HW_MODE		0
static int hwmode = WDT_HW_MODE;
module_param(hwmode, int, 0);
MODULE_PARM_DESC(hwmode, " Watchdog HW expire mode: 0=reset; 1=interrupt (NMI). "
	"(default=" __stringify(WDT_HW_MODE) ")");

/* Watchdog cannot be stopped once started */
static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, " Watchdog cannot be stopped once started. "
	"(default=" __stringify(WATCHDOG_NOWAYOUT) ")");

/* Driver log-level: Mimics kernel log-levels */
#define	LOG_EMERG	0	/* system is unusable			*/
#define	LOG_ALERT	1	/* action must be taken immediately	*/
#define	LOG_CRIT	2	/* critical conditions			*/
#define	LOG_ERROR	3	/* error conditions			*/
#define	LOG_WARN	4	/* warning conditions			*/
#define	LOG_NOTICE	5	/* normal but significant condition	*/
#define	LOG_INFO	6	/* informational			*/
#define	LOG_DEBUG	7	/* debug-level messages			*/
#define WDT_LOGLEVEL	LOG_INFO
static int loglevel = WDT_LOGLEVEL;
module_param(loglevel, int, 0);
MODULE_PARM_DESC(loglevel, " Driver log-level, mimics kernel log-levels. "
	"(default=" __stringify(WDT_LOGLEVEL) ")");

/*
 * Service to print debug/log messages.
 * Mimics kernel log-levels.
 */
#define d_printk(level, fmt, args...)					\
	if (loglevel >= level) printk("<" __stringify(level) ">"	\
		DRV_NAME ": " fmt, ## args)

/*
 * Private data structure
 */
static struct {
	unsigned long	next_expiry;	/* The next expiry for the user-land timeout */
	unsigned long	open;		/* Watchdog device is open */
	unsigned int	close_okay;	/* Okay to be closed, refresh anyway */
	struct timer_list timer;	/* Kernel timer that pats the HW watchdog */
} m2s_wdt_private;

/* ......................................................................... */

/*
 * Refresh the HW watchdog timer.  (ie. pat the watchdog)
 */
static void m2s_wdt_refresh(void)
{
	unsigned int before, after;

	if (M2S_WDT_REG->wdogstatus & M2S_WDT_REFRESHSTATUS) {
		before = M2S_WDT_REG->wdogvalue;
		M2S_WDT_REG->wdogrefresh = M2S_WDT_REFRESH_VAL;
		after  = M2S_WDT_REG->wdogvalue;
		d_printk(LOG_DEBUG, "HW timer refreshed, before=0x%08x, after=0x%08x\n",
			before, after);
	} else
		d_printk(LOG_NOTICE, "HW timer refresh denied!\n");
}

/*
 * Enable the HW watchdog timer, and do a refresh.
 * If HW timer already enabled returns 1, otherwise returns 0.
 */
static int m2s_wdt_enable(void)
{
	int ret = 0;

	if (M2S_SYSREG->wdog_cr & M2S_WDOG_CR_ENABLE)
		ret = 1;

	if (hwmode)
		M2S_SYSREG->wdog_cr |=  M2S_WDOG_CR_MODE;
	else
		M2S_SYSREG->wdog_cr &= ~M2S_WDOG_CR_MODE;
	M2S_SYSREG->wdog_cr |= M2S_WDOG_CR_ENABLE;

	if (!ret)
		d_printk(LOG_INFO, "HW timer enabled\n");

	/* Make sure the WDT is refreshed as soon as it is enabled */
	m2s_wdt_refresh();

	return ret;
}

/*
 * Disable the HW watchdog timer.
 */
static void m2s_wdt_disable(void)
{
	if (!(M2S_SYSREG->wdog_cr & M2S_WDOG_CR_ENABLE))
		return;

	M2S_SYSREG->wdog_cr &= ~M2S_WDOG_CR_ENABLE;

	d_printk(LOG_INFO, "HW timer disabled\n");
}

/*
 * Kernel timer ping
 */
static void m2s_wdt_ping(unsigned long data)
{
	unsigned long now = jiffies;

	if (time_before(now, m2s_wdt_private.next_expiry) ||
			m2s_wdt_private.close_okay) {
		m2s_wdt_refresh();
		mod_timer(&m2s_wdt_private.timer, now + WDT_KERN_TIMEOUT * HZ);
	} else
		d_printk(LOG_CRIT, "Timed out, system will reset in ~%i seconds!\n",
			WDT_HW_TIMEOUT - WDT_KERN_TIMEOUT);
}

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int m2s_wdt_open(struct inode *inode, struct file *file)
{
	unsigned long now = jiffies;

	if (test_and_set_bit(0, &m2s_wdt_private.open)) {
		d_printk(LOG_ERROR, "Device already open!\n");
		return -EBUSY;
	}

	/* Enable the HW watchdog timer, if not already */
	m2s_wdt_enable();

	m2s_wdt_private.close_okay  = 0;
	m2s_wdt_private.next_expiry = now + timeout * HZ;
	mod_timer(&m2s_wdt_private.timer, now + WDT_KERN_TIMEOUT * HZ);

	d_printk(LOG_INFO, "Device opened\n");

	return nonseekable_open(inode, file);
}

/*
 * Close the watchdog device.
 */
static int m2s_wdt_close(struct inode *inode, struct file *file)
{
	clear_bit(0, &m2s_wdt_private.open);

	/* If we are expecting a close action,
	 * disable the HW timer and kernel timer */
	if (m2s_wdt_private.close_okay) {
		m2s_wdt_disable();
		del_timer(&m2s_wdt_private.timer);
		d_printk(LOG_INFO, "Device closed\n");
	}
	else
		d_printk(LOG_WARN, "Device closed unexpectedly!\n");

	return 0;
}

static const struct watchdog_info m2s_wdt_info = {
	.identity = "M2S Watchdog",
	.options  = WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT,
};

/*
 * Handle commands from user-space.
 */
static long m2s_wdt_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	unsigned long now = jiffies;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &m2s_wdt_info,
				    sizeof(m2s_wdt_info)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;
		d_printk(LOG_DEBUG, "Set-options received, value=0x%x\n",
			(unsigned int)new_value);
		if (new_value & WDIOS_DISABLECARD)
			m2s_wdt_disable();
		if (new_value & WDIOS_ENABLECARD)
			m2s_wdt_enable();
		return 0;

	case WDIOC_KEEPALIVE:
		m2s_wdt_private.next_expiry = now + timeout * HZ;
		d_printk(LOG_DEBUG, "Keep-alive received, now=0x%08x, next_expiry=0x%08x\n",
			(unsigned int)now, (unsigned int)m2s_wdt_private.next_expiry);
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		timeout = new_value;
		m2s_wdt_private.next_expiry = now + timeout * HZ;
		return put_user(timeout, p);

	case WDIOC_GETTIMEOUT:
		return put_user(timeout, p);
	}
	return -ENOTTY;
}

/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t m2s_wdt_write(struct file *file, const char *data, size_t len,
								loff_t *ppos)
{
	unsigned long now = jiffies;
	unsigned int magic_close = 0;
	size_t i;

	if (!len)
		return 0;

	for (i = 0; i < len; i++) {
		char c;
		if (get_user(c, data + i))
			return -EFAULT;

		/* Check for magic character */
		if (!nowayout && c == 'V') {
			magic_close = 42;
			break;
		}
	}

	m2s_wdt_private.close_okay  = magic_close;
	m2s_wdt_private.next_expiry = now + timeout * HZ;

	d_printk(LOG_DEBUG, "Device written, now=0x%08x, next_expiry=0x%08x\n",
		(unsigned int)now, (unsigned int)m2s_wdt_private.next_expiry);

	if (magic_close)
		d_printk(LOG_INFO, "Close expected\n");

	return len;
}

/* ......................................................................... */

static const struct file_operations m2s_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= m2s_wdt_ioctl,
	.open		= m2s_wdt_open,
	.release	= m2s_wdt_close,
	.write		= m2s_wdt_write,
};

static struct miscdevice m2s_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &m2s_wdt_fops,
};

static int __init m2s_wdt_probe(struct platform_device *pdev)
{
	int res;
	unsigned long now = jiffies;

	if (m2s_wdt_miscdev.parent) {
		d_printk(LOG_ERROR, "Device busy!\n");
		return -EBUSY;
	}
	m2s_wdt_miscdev.parent = &pdev->dev;

	res = misc_register(&m2s_wdt_miscdev);
	if (res)
		return res;

	m2s_wdt_private.close_okay  = 42;
	m2s_wdt_private.next_expiry = now + timeout * HZ;
	setup_timer(&m2s_wdt_private.timer, m2s_wdt_ping, 0);
	mod_timer(&m2s_wdt_private.timer, now + WDT_KERN_TIMEOUT * HZ);

	d_printk(LOG_INFO, "Probed (timeout=%d, hwmode=%d, nowayout=%d, "
		"loglevel=%d)\n", timeout, hwmode, nowayout, loglevel);

	/* Enable the HW watchdog timer */
	if (m2s_wdt_enable())
		d_printk(LOG_WARN, "HW timer was already enabled!\n");

	return 0;
}

static int __exit m2s_wdt_remove(struct platform_device *pdev)
{
	int res;

	/* Make sure the HW watchdog timer is disabled */
	m2s_wdt_disable();

	del_timer(&m2s_wdt_private.timer);

	res = misc_deregister(&m2s_wdt_miscdev);
	if (!res)
		m2s_wdt_miscdev.parent = NULL;

	d_printk(LOG_INFO, "Removed\n");

	return res;
}

#ifdef CONFIG_PM

static int m2s_wdt_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int m2s_wdt_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define m2s_wdt_suspend	NULL
#define m2s_wdt_resume	NULL
#endif

static struct platform_driver m2s_wdt_driver = {
	.remove		= __exit_p(m2s_wdt_remove),
	.suspend	= m2s_wdt_suspend,
	.resume		= m2s_wdt_resume,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init m2s_wdt_init(void)
{
	return platform_driver_probe(&m2s_wdt_driver, m2s_wdt_probe);
}

static void __exit m2s_wdt_exit(void)
{
	platform_driver_unregister(&m2s_wdt_driver);
}

module_init(m2s_wdt_init);
module_exit(m2s_wdt_exit);

MODULE_AUTHOR("Mike Pilawa <Mike.Pilawa@csiro.au>");
MODULE_DESCRIPTION("Watchdog driver for Microsemi M2S SoCs");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
