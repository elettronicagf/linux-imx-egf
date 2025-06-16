/*
 * XRM1280 tty serial driver - Copyright (C) 2014 GridPoint
 * Author: Ted Lin <tedlin@maxlinear.com>
 *
 *  Based on SC16IS7xx.c, by Jon Ringle <jringle@gridpoint.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#define DEBUG 
#include "linux/version.h" 
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)	
#include <uapi/linux/sched/types.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
#include <linux/gpio.h>
#else
#include <linux/gpio/driver.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#else
#include <linux/of.h>
#include <linux/of_device.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 16, 0)	
/* This is included under serial_core.h after kernel 3.16. 
 * this define is only for kernel version under 3.16
 */
#define PORT_SC16IS7XX   108
/**
 * enum uart_pm_state - power states for UARTs
 * @UART_PM_STATE_ON: UART is powered, up and operational
 * @UART_PM_STATE_OFF: UART is powered off
 * @UART_PM_STATE_UNDEFINED: sentinel
 */
enum uart_pm_state {
	UART_PM_STATE_ON = 0,
	UART_PM_STATE_OFF = 3, /* number taken from ACPI */
	UART_PM_STATE_UNDEFINED,
};
#endif

#ifdef CONFIG_GPIOLIB
#undef CONFIG_GPIOLIB
#endif

#define XRM1280_NAME			"xrm1280"
#define XRM1280_MAX_DEVS	8

/* XRM1280 register definitions */
#define XRM1280_RHR_REG		(0x00) /* RX FIFO */
#define XRM1280_THR_REG		(0x00) /* TX FIFO */

#define XRM1280_IER_REG		(0x01) /* Interrupt enable */
#define XRM1280_IIR_REG		(0x02) /* Interrupt Identification */
#define XRM1280_FCR_REG		(0x02) /* FIFO control */
#define XRM1280_LCR_REG		(0x03) /* Line Control */
#define XRM1280_MCR_REG		(0x04) /* Modem Control */
#define XRM1280_LSR_REG		(0x05) /* Line Status */
#define XRM1280_MSR_REG		(0x06) /* Modem Status */

/* Access when LCR != 0xBF && EFR[4]=1 */
#define XRM1280_SHR_REG		(0x05) /* Setup/Hysteresis */
#define XRM1280_SFR_REG		(0x06) /* Special Function */

/* Access when LCR != 0xBF, FCTR[6]=0, SFR[0]=0 */
#define XRM1280_SPR_REG		(0x07) /* Scratch Pad */

/* Access when LCR != 0xBF, FCTR[6]=1, SFR[0]=0 */
#define XRM1280_FC_REG		(0x07) /* TX/RX FIFO Counter Read only*/
#define XRM1280_EMSR_REG	(0x07) /* Enhanced Mode Select Register write only*/

/* Access when LCR != 0xBF, FCTR[6]=0, SFR[0]=1 */
#define XRM1280_GPIOLVL_REG		(0x07) /* GPIO Level */

/* Access when LCR = 0xBF, SFR[0]=1 */
#define XRM1280_GPIOINT_REG		(0x04) /* GPIO Interrupt Enable */
#define XRM1280_GPIO3T_REG		(0x05) /* GPIO Three State Control */
#define XRM1280_GPIOINV_REG		(0x06) /* GPIO Polarity Control */
#define XRM1280_GPIOSEL_REG		(0x07) /* GPIO Select */

/* Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF)) */
#define XRM1280_DLL_REG		(0x00) /* Divisor Latch Low */
#define XRM1280_DLH_REG		(0x01) /* Divisor Latch High */

/* Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF)) && (EFR[4]=1) */
#define XRM1280_DLD_REG		(0x02) /* Divisor Fractional */

/* Enhanced Register set: Only if (LCR == 0xBF) */
#define XRM1280_FCTR_REG	(0x01) /* FCTR register */
#define XRM1280_EFR_REG		(0x02) /* Enhanced Features */
#define XRM1280_XON1_REG		(0x04) /* Xon1 word */
#define XRM1280_XON2_REG		(0x05) /* Xon2 word */
#define XRM1280_XOFF1_REG		(0x06) /* Xoff1 word */
#define XRM1280_XOFF2_REG		(0x07) /* Xoff2 word */

/* IER register bits */
#define XRM1280_IER_RDI_BIT		(1 << 0) /* Enable RX data interrupt */
#define XRM1280_IER_THRI_BIT		(1 << 1) /* Enable TX holding register
						  * interrupt */
#define XRM1280_IER_RLSI_BIT		(1 << 2) /* Enable RX line status
						  * interrupt */
#define XRM1280_IER_MSI_BIT		(1 << 3) /* Enable Modem status
						  * interrupt */

/* IER register bits - write only if (EFR[4] == 1) */
#define XRM1280_IER_SLEEP_BIT		(1 << 4) /* Enable Sleep mode */
#define XRM1280_IER_XOFFI_BIT		(1 << 5) /* Enable Xoff interrupt */
#define XRM1280_IER_RTSI_BIT		(1 << 6) /* Enable nRTS interrupt */
#define XRM1280_IER_CTSI_BIT		(1 << 7) /* Enable nCTS interrupt */

/* FCR register bits */
#define XRM1280_FCR_FIFO_BIT		(1 << 0) /* Enable FIFO */
#define XRM1280_FCR_RXRESET_BIT	(1 << 1) /* Reset RX FIFO */
#define XRM1280_FCR_TXRESET_BIT	(1 << 2) /* Reset TX FIFO */
#define XRM1280_FCR_RXLVLL_BIT	(1 << 6) /* RX Trigger level LSB */
#define XRM1280_FCR_RXLVLH_BIT	(1 << 7) /* RX Trigger level MSB */

/* FCR register bits - write only if (EFR[4] == 1) */
#define XRM1280_FCR_TXLVLL_BIT	(1 << 4) /* TX Trigger level LSB */
#define XRM1280_FCR_TXLVLH_BIT	(1 << 5) /* TX Trigger level MSB */

/* IIR register bits */
#define XRM1280_IIR_NO_INT_BIT	(1 << 0) /* No interrupts pending */
#define XRM1280_IIR_ID_MASK		0x3e     /* Mask for the interrupt ID */
#define XRM1280_IIR_THRI_SRC		0x02     /* TX holding register empty */
#define XRM1280_IIR_RDI_SRC		0x04     /* RX data interrupt */
#define XRM1280_IIR_RLSE_SRC		0x06     /* RX line status error */
#define XRM1280_IIR_RTOI_SRC		0x0c     /* RX time-out interrupt */
#define XRM1280_IIR_MSI_SRC		0x00     /* Modem status interrupt
						  * - only on 75x/76x
						  */
#define XRM1280_IIR_INPIN_SRC		0x30     /* Input pin change of state
						  * - only on 75x/76x
						  */
#define XRM1280_IIR_XOFFI_SRC		0x10     /* Received Xoff */
#define XRM1280_IIR_CTSRTS_SRC	0x20     /* nCTS,nRTS change of state
						  * from active (LOW)
						  * to inactive (HIGH)
						  */
/* LCR register bits */
#define XRM1280_LCR_LENGTH0_BIT	(1 << 0) /* Word length bit 0 */
#define XRM1280_LCR_LENGTH1_BIT	(1 << 1) /* Word length bit 1
						  *
						  * Word length bits table:
						  * 00 -> 5 bit words
						  * 01 -> 6 bit words
						  * 10 -> 7 bit words
						  * 11 -> 8 bit words
						  */
#define XRM1280_LCR_STOPLEN_BIT	(1 << 2) /* STOP length bit
						  *
						  * STOP length bit table:
						  * 0 -> 1 stop bit
						  * 1 -> 1-1.5 stop bits if
						  *      word length is 5,
						  *      2 stop bits otherwise
						  */
#define XRM1280_LCR_PARITY_BIT	(1 << 3) /* Parity bit enable */
#define XRM1280_LCR_EVENPARITY_BIT	(1 << 4) /* Even parity bit enable */
#define XRM1280_LCR_FORCEPARITY_BIT	(1 << 5) /* 9-bit multidrop parity */
#define XRM1280_LCR_TXBREAK_BIT	(1 << 6) /* TX break enable */
#define XRM1280_LCR_DLAB_BIT		(1 << 7) /* Divisor Latch enable */
#define XRM1280_LCR_WORD_LEN_5	(0x00)
#define XRM1280_LCR_WORD_LEN_6	(0x01)
#define XRM1280_LCR_WORD_LEN_7	(0x02)
#define XRM1280_LCR_WORD_LEN_8	(0x03)
#define XRM1280_LCR_CONF_MODE_A	XRM1280_LCR_DLAB_BIT /* Special
								* reg set */
#define XRM1280_LCR_CONF_MODE_B	0xBF                   /* Enhanced
								* reg set */

/* MCR register bits */
#define XRM1280_MCR_DTR_BIT		(1 << 0) /* DTR complement
						  * - only on 75x/76x
						  */
#define XRM1280_MCR_RTS_BIT		(1 << 1) /* RTS complement */
#define XRM1280_MCR_TCRTLR_BIT	(1 << 2) /* TCR/TLR register enable */
#define XRM1280_MCR_LOOP_BIT		(1 << 4) /* Enable loopback test mode */
#define XRM1280_MCR_XONANY_BIT	(1 << 5) /* Enable Xon Any
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define XRM1280_MCR_IRDA_BIT		(1 << 6) /* Enable IrDA mode
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define XRM1280_MCR_CLKSEL_BIT	(1 << 7) /* Divide clock by 4
						  * - write enabled
						  * if (EFR[4] == 1)
						  */

/* LSR register bits */
#define XRM1280_LSR_DR_BIT		(1 << 0) /* Receiver data ready */
#define XRM1280_LSR_OE_BIT		(1 << 1) /* Overrun Error */
#define XRM1280_LSR_PE_BIT		(1 << 2) /* Parity Error */
#define XRM1280_LSR_FE_BIT		(1 << 3) /* Frame Error */
#define XRM1280_LSR_BI_BIT		(1 << 4) /* Break Interrupt */
#define XRM1280_LSR_BRK_ERROR_MASK	0x1E     /* BI, FE, PE, OE bits */
#define XRM1280_LSR_THRE_BIT		(1 << 5) /* TX holding register empty */
#define XRM1280_LSR_TEMT_BIT		(1 << 6) /* Transmitter empty */
#define XRM1280_LSR_FIFOE_BIT		(1 << 7) /* Fifo Error */

/* MSR register bits */
#define XRM1280_MSR_DCTS_BIT		(1 << 0) /* Delta CTS Clear To Send */
#define XRM1280_MSR_DDSR_BIT		(1 << 1) /* Delta DSR Data Set Ready
						  * or (IO4)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_DRI_BIT		(1 << 2) /* Delta RI Ring Indicator
						  * or (IO7)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_DCD_BIT		(1 << 3) /* Delta CD Carrier Detect
						  * or (IO6)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_CTS_BIT		(1 << 0) /* CTS */
#define XRM1280_MSR_DSR_BIT		(1 << 1) /* DSR (IO4)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_RI_BIT		(1 << 2) /* RI (IO7)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_CD_BIT		(1 << 3) /* CD (IO6)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_DELTA_MASK	0x0F     /* Any of the delta bits! */

/* DLD register bits */
#define XRM1280_DLD_SAMPLING_8X_BIT		(1 << 4) /* Sampling mode 8x; Ignored when 4x mode is set */
#define XRM1280_DLD_SAMPLING_4X_BIT		(1 << 5) /* Sampling mode 4x */
#define XRM1280_DLD_FRACTION_BITS_MASK		0x0F 	/* Fraction config is in DLD[3:0] */
#define XRM1280_DLD_SAMPLING_MODE_MASK		0x30 	/* Sampling mode setting is in DLD[5:4] */

/* SFR register bits */
#define XRM1280_SFR_ENABLE_GPIO_BIT		(1 << 0) /* Enable GPIO registers */
#define XRM1280_SFR_GPIO_SELECT_BIT		(1 << 1) /* Select GPIO[15:8] or GPIO[7:0] */
#define XRM1280_SFR_ENABLE_GPIO_INT_BIT		(1 << 2) /* Enable GPIO interrupt */
#define XRM1280_SFR_ENABLE_FAST_IR_BIT		(1 << 3) /* Enable IrDA ver1.1 (data rate up to 1.152 Mbps) */
#define XRM1280_SFR_TX_DISABLE_BIT		(1 << 4) /* Disable TX */
#define XRM1280_SFR_RX_DISABLE_BIT		(1 << 5) /* Disable RX */
#define XRM1280_SFR_9BIT_MODE_ENABLE_BIT		(1 << 6) /* Enable 9 bit/multidrop mode */
#define XRM1280_SFR_TX_ADDR_BIT		(1 << 7) /* set 9th bit to 1, LCR[5:3]=111 required */

/* EMSR register bits */
#define XRM1280_EMSR_RS485_INVERT_BIT	(1 << 3) /* RS485 inversion */

/* FCTR register bits */
#define XRM1280_FCTR_SLEEP_PWRDN_MODE_BIT	(1 << 0) /* Switch between sleep (input) & PWRDN# (output) mode */
#define XRM1280_FCTR_IRDA_INVERT_BIT	(1 << 2) /* IrDa RX inversion */
#define XRM1280_FCTR_AUTO_RS485_BIT	(1 << 3) /* Auto RS485 RTS direction */
#define XRM1280_FCTR_ENHANCED_MODE_BIT	(1 << 6) /* Enhanced mode select */

/* EFR register bits */
#define XRM1280_EFR_AUTORTS_BIT	(1 << 6) /* Auto RTS flow ctrl enable */
#define XRM1280_EFR_AUTOCTS_BIT	(1 << 7) /* Auto CTS flow ctrl enable */
#define XRM1280_EFR_XOFF2_DETECT_BIT	(1 << 5) /* Enable Xoff2 detection */
#define XRM1280_EFR_ENABLE_BIT	(1 << 4) /* Enable enhanced functions
						  * and writing to IER[7:4],
						  * FCR[5:4], MCR[7:5], DLD, SHR, SFR
						  */
#define XRM1280_EFR_SWFLOW3_BIT	(1 << 3) /* SWFLOW bit 3 */
#define XRM1280_EFR_SWFLOW2_BIT	(1 << 2) /* SWFLOW bit 2
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no transmitter flow
						  *       control
						  * 01 -> transmitter generates
						  *       XON2 and XOFF2
						  * 10 -> transmitter generates
						  *       XON1 and XOFF1
						  * 11 -> transmitter generates
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */
#define XRM1280_EFR_SWFLOW1_BIT	(1 << 1) /* SWFLOW bit 2 */
#define XRM1280_EFR_SWFLOW0_BIT	(1 << 0) /* SWFLOW bit 3
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no received flow
						  *       control
						  * 01 -> receiver compares
						  *       XON2 and XOFF2
						  * 10 -> receiver compares
						  *       XON1 and XOFF1
						  * 11 -> receiver compares
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */

/* Misc definitions */
#define XRM1280_FIFO_SIZE		(128)
#define XRM1280_REG_SHIFT		2

struct xrm1280_devtype {
	char	name[10];
	int	nr_gpio;
	int	nr_uart;
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)	
#define XRM1280_RECONF_MD			(1 << 0)
#define XRM1280_RECONF_IER		(1 << 1)
#define XRM1280_RECONF_RS485	(1 << 2)

struct xrm1280_one_config {
	unsigned int			flags;
	u8				ier_clear;
};
#endif

struct xrm1280_one {
	struct uart_port		port;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
	struct work_struct		tx_work;
	struct work_struct		md_work;

	struct serial_rs485		rs485;
#else	// Kernel < 4.2
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)		
	u8				line;
#endif	
	struct kthread_work		tx_work;
	struct kthread_work		reg_work;
	struct xrm1280_one_config	config;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)	
	bool				irda_mode;
#endif	
#endif	
};

struct xrm1280_port {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)			
	struct uart_driver		uart;
	struct xrm1280_devtype	*devtype;
#else	
	const struct xrm1280_devtype	*devtype;
#endif
	struct regmap			*regmap;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)			
	struct mutex			mutex;
#endif	
	struct clk			*clk;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip		gpio;
#endif
	unsigned char			buf[XRM1280_FIFO_SIZE];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)	
	struct kthread_worker		kworker;
	struct task_struct		*kworker_task;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)	
	struct kthread_work		irq_work;
#endif		//Kernel < version 5.8	
#endif		//Kernel >= 4.2
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)	
	struct mutex			efr_lock;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 7, 0)	
	struct xrm1280_one		p[];
#else
	struct xrm1280_one		p[0];
#endif
};

#define to_xrm1280_one(p,e)	((container_of((p), struct xrm1280_one, e)))

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)	
static unsigned long xrm1280_lines;

static struct uart_driver xrm1280_uart = {
	.owner		= THIS_MODULE,
	.dev_name	= "ttyXRM",
	.nr		=  XRM1280_MAX_DEVS,
};

#define to_xrm1280_port(p,e)	((container_of((p), struct xrm1280_port, e)))

static int xrm1280_line(struct uart_port *port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	return one->line;
}
#endif

static u8 xrm1280_port_read(struct uart_port *port, u8 reg)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int val = 0;
	int ret;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)	
	regmap_read(s->regmap,
		    (reg << XRM1280_REG_SHIFT) | port->line, &val);
#else
	const u8 line = xrm1280_line(port);

	ret = regmap_read(s->regmap, (reg << XRM1280_REG_SHIFT) | line, &val);
	//printk(KERN_INFO "xrm1280_port_read reg=%02X val=%02X ret=%d",reg,val,ret);
#endif

	return val;
}

static void xrm1280_port_write(struct uart_port *port, u8 reg, u8 val)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	int ret;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)	
	regmap_write(s->regmap,
		     (reg << XRM1280_REG_SHIFT) | port->line, val);
#else
	const u8 line = xrm1280_line(port);

	ret = regmap_write(s->regmap, (reg << XRM1280_REG_SHIFT) | line, val);
	//printk(KERN_INFO "xrm1280_port_write reg=%02X val=%02X ret=%d",reg,val,ret);
#endif				 
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)	
static void xrm1280_fifo_read(struct uart_port *port, unsigned int rxlen)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	const u8 line = xrm1280_line(port);
	u8 addr = (XRM1280_RHR_REG << XRM1280_REG_SHIFT) | line;

	regcache_cache_bypass(s->regmap, true);
	regmap_raw_read(s->regmap, addr, s->buf, rxlen);
	regcache_cache_bypass(s->regmap, false);
}

static void xrm1280_fifo_write(struct uart_port *port, u8 to_send)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	const u8 line = xrm1280_line(port);
	//printk(KERN_INFO "xrm1280_fifo_write to_send=%d",to_send);
	u8 addr = (XRM1280_THR_REG << XRM1280_REG_SHIFT) | line;

	/*
	 * Don't send zero-length data, at least on SPI it confuses the chip
	 * delivering wrong TXLVL data.
	 */
	if (unlikely(!to_send))
		return;

	regcache_cache_bypass(s->regmap, true);
	regmap_raw_write(s->regmap, addr, s->buf, to_send);
	regcache_cache_bypass(s->regmap, false);
}
#endif

static void xrm1280_port_update(struct uart_port *port, u8 reg,
				  u8 mask, u8 val)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)	
	regmap_update_bits(s->regmap,
			   (reg << XRM1280_REG_SHIFT) | port->line,
			   mask, val);
#else			
	const u8 line = xrm1280_line(port);

	regmap_update_bits(s->regmap, (reg << XRM1280_REG_SHIFT) | line,
			   mask, val);
#endif				 
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)	
static int xrm1280_alloc_line(void)
{
	int i;

	BUILD_BUG_ON(XRM1280_MAX_DEVS > BITS_PER_LONG);

	for (i = 0; i < XRM1280_MAX_DEVS; i++)
		if (!test_and_set_bit(i, &xrm1280_lines))
			break;

	return i;
}	
#endif	

static void xrm1280_power(struct uart_port *port, int on)
{
	xrm1280_port_update(port, XRM1280_IER_REG,
			      XRM1280_IER_SLEEP_BIT,
			      on ? 0 : XRM1280_IER_SLEEP_BIT);
}

static const struct xrm1280_devtype xrm1280_devtype = {
	.name		= "XRM1280",
	.nr_gpio	= 0,
	.nr_uart	= 1,
};

static bool xrm1280_regmap_volatile(struct device *dev, unsigned int reg)
{
	switch (reg >> XRM1280_REG_SHIFT) {
	case XRM1280_RHR_REG:
	case XRM1280_IIR_REG:
	case XRM1280_LSR_REG:
	case XRM1280_MSR_REG:
	//case XRM1280_FC_REG:
	//case XRM1280_TXLVL_REG:
	//case XRM1280_RXLVL_REG:
		return true;
	default:
		break;
	}

	return false;
}

static bool xrm1280_regmap_precious(struct device *dev, unsigned int reg)
{
	switch (reg >> XRM1280_REG_SHIFT) {
	case XRM1280_RHR_REG:
		return true;
	default:
		break;
	}

	return false;
}

static int xrm1280_set_baud(struct uart_port *port, int baud)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	u8 lcr;
	u8 prescaler = 0;
	unsigned long clk = port->uartclk;
	unsigned long div_16, div_integer, div_fraction;
	/* 4x or 8x mode can be set in DLD[5:4] if needed for higher baud rate, default is 16x */
	u8 sampling_mode = 0;
	u8 sampling_factor = 16;
	
  //printk("xrm1280_set_baud clk:%d baud=%d\n",clk,baud);
	if ((clk / 16 / baud) > 0xffff) {
		/* If DLL & DLM is not enough, setup prescaler as by 4 */
		prescaler = XRM1280_MCR_CLKSEL_BIT;
		clk /= 4;
	}
	else if ((clk / 16 / baud) < 1 ) {
		/* When divisor < 1, change sampling mode */
		sampling_mode = XRM1280_DLD_SAMPLING_8X_BIT;
		sampling_factor /= 2;
		
		if ((clk / 8 / baud) < 1 ) {
			sampling_mode = XRM1280_DLD_SAMPLING_4X_BIT;
			sampling_factor /= 2;
			
			if ((clk / 4 / baud) < 1 ) {
				/* This baud rate cannot be supported if 4x mode still cannot do */
				printk("xrm1280_set_baud: baud %d is not supported \n", baud);
				return -EINVAL;
			}	
		}
		
		clk *= (16/sampling_factor);
	}		
	
	div_16 = DIV_ROUND_CLOSEST(clk, baud);
	div_fraction = div_16 & 0x0f;	
	div_integer = (div_16 >> 4);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	/* In an amazing feat of design, the Enhanced Features Register shares
	 * the address of the Interrupt Identification Register, and is
	 * switched in by writing a magic value (0xbf) to the Line Control
	 * Register. Any interrupt firing during this time will see the EFR
	 * where it expects the IIR to be, leading to "Unexpected interrupt"
	 * messages.
	 *
	 * Prevent this possibility by claiming a mutex while accessing the
	 * EFR, and claiming the same mutex from within the interrupt handler.
	 * This is similar to disabling the interrupt, but that doesn't work
	 * because the bulk of the interrupt processing is run as a workqueue
	 * job in thread context.
	 */
	mutex_lock(&s->efr_lock);
#endif
	
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Open the LCR divisors for configuration */
	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_B);

	/* Enable enhanced features */
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_write(port, XRM1280_EFR_REG,
			     XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);

	/* Put LCR back to the normal mode */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);	
#endif
	
	xrm1280_port_update(port, XRM1280_MCR_REG,
			      XRM1280_MCR_CLKSEL_BIT,
			      prescaler);

	/* Open the LCR divisors for configuration */
	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_A);

	/* Write the new divisor */
	regcache_cache_bypass(s->regmap, true);

	if(sampling_mode != 0) {
		/* Change in sampling mode */
		xrm1280_port_update(port, XRM1280_DLD_REG, XRM1280_DLD_SAMPLING_MODE_MASK, sampling_mode);	
		printk("xrm1280_set_baud: Sampling mode changed to %dX mode\n",sampling_factor);
	}

	xrm1280_port_write(port, XRM1280_DLH_REG, div_integer / 256);
	xrm1280_port_write(port, XRM1280_DLL_REG, div_integer % 256);
	xrm1280_port_update(port, XRM1280_DLD_REG, XRM1280_DLD_FRACTION_BITS_MASK, div_fraction);	
	printk("xrm1280_set_baud: msb:0x%lx lsb=0x%lx fraction=0x%lx\n",div_integer / 256, div_integer % 256, div_fraction);
	
	regcache_cache_bypass(s->regmap, false);

	/* Put LCR back to the normal mode */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);

	return DIV_ROUND_CLOSEST(clk, div_16);
}

static void xrm1280_handle_rx(struct uart_port *port, unsigned int rxlen,
				unsigned int iir)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int lsr = 0, ch, flag, bytes_read, i;
	bool read_lsr = (iir == XRM1280_IIR_RLSE_SRC) ? true : false;

	if (unlikely(rxlen >= sizeof(s->buf))) {
		dev_warn_ratelimited(port->dev,
				     "ttyXRM%i: Possible RX FIFO overrun: %d\n",
				     port->line, rxlen);
		port->icount.buf_overrun++;
		/* Ensure sanity of RX level */
		rxlen = sizeof(s->buf);
	}

	while (rxlen) {
		/* Only read lsr if there are possible errors in FIFO */
		if (read_lsr) {
			lsr = xrm1280_port_read(port, XRM1280_LSR_REG);
			if (!(lsr & XRM1280_LSR_FIFOE_BIT))
				read_lsr = false; /* No errors left in FIFO */
		} else
			lsr = 0;

		if (read_lsr) {
			s->buf[0] = xrm1280_port_read(port, XRM1280_RHR_REG);
			bytes_read = 1;
		} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
			regcache_cache_bypass(s->regmap, true);
			regmap_raw_read(s->regmap, XRM1280_RHR_REG,
					s->buf, rxlen);
			regcache_cache_bypass(s->regmap, false);
#else
			xrm1280_fifo_read(port, rxlen);
#endif	
			bytes_read = rxlen;
		}

		lsr &= XRM1280_LSR_BRK_ERROR_MASK;

		port->icount.rx++;
		flag = TTY_NORMAL;

		if (unlikely(lsr)) {
			if (lsr & XRM1280_LSR_BI_BIT) {
				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			} else if (lsr & XRM1280_LSR_PE_BIT)
				port->icount.parity++;
			else if (lsr & XRM1280_LSR_FE_BIT)
				port->icount.frame++;
			else if (lsr & XRM1280_LSR_OE_BIT)
				port->icount.overrun++;

			lsr &= port->read_status_mask;
			if (lsr & XRM1280_LSR_BI_BIT)
				flag = TTY_BREAK;
			else if (lsr & XRM1280_LSR_PE_BIT)
				flag = TTY_PARITY;
			else if (lsr & XRM1280_LSR_FE_BIT)
				flag = TTY_FRAME;
			else if (lsr & XRM1280_LSR_OE_BIT)
				flag = TTY_OVERRUN;
		}

		for (i = 0; i < bytes_read; ++i) {
			ch = s->buf[i];
			if (uart_handle_sysrq_char(port, ch))
				continue;

			if (lsr & port->ignore_status_mask)
				continue;
			//printk(KERN_INFO "RX[%d]:0x%02X-0x%02X",i,ch,flag);
			uart_insert_char(port, lsr, XRM1280_LSR_OE_BIT, ch,
					 flag);
		}
		rxlen -= bytes_read;
	}
	
//      tty_flip_buffer_push(port->state->port.tty);
	tty_flip_buffer_push(&port->state->port);
}

static void xrm1280_handle_tx(struct uart_port *port)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int txlen, to_send, i;
  	if (unlikely(port->x_char)) {
		xrm1280_port_write(port, XRM1280_THR_REG, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;
	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	//printk(KERN_INFO "xrm1280 to_send %d\n",to_send);
	if (likely(to_send)) {
		/* Limit to size of TX FIFO */
#if 1
		regcache_cache_bypass(s->regmap, true);
		xrm1280_port_write(port, XRM1280_EMSR_REG,1);// read tx_fifo counter
		txlen = XRM1280_FIFO_SIZE - xrm1280_port_read(port, XRM1280_FC_REG);
		regcache_cache_bypass(s->regmap, false);
		// if(txlen !=XRM1280_FIFO_SIZE)
		// 	printk(KERN_INFO "xrm1280 txlen %d\n",txlen);
#else
		txlen = xrm1280_port_read(port, XRM1280_TXLVL_REG);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)	
		if (txlen > XRM1280_FIFO_SIZE) {
			dev_err_ratelimited(port->dev,
				"chip reports %d free bytes in TX fifo, but it only has %d",
				txlen, XRM1280_FIFO_SIZE);
			txlen = 0;
		}
#endif	
		to_send = (to_send > txlen) ? txlen : to_send;

		/* Add data to send */
		port->icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			s->buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
		regcache_cache_bypass(s->regmap, true);
		regmap_raw_write(s->regmap, XRM1280_THR_REG, s->buf, to_send);
		regcache_cache_bypass(s->regmap, false);
#else
		xrm1280_fifo_write(port, to_send);
#endif	
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 3)
static bool xrm1280_port_irq(struct xrm1280_port *s, int portno)
#else
static void xrm1280_port_irq(struct xrm1280_port *s, int portno)
#endif
{
	struct uart_port *port = &s->p[portno].port;

	do {
		unsigned int iir, rxlen;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
		unsigned int msr;
#endif		

		iir = xrm1280_port_read(port, XRM1280_IIR_REG);
		if (iir & XRM1280_IIR_NO_INT_BIT)
		{	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 3)
			return false;
#else			
			break;
#endif
		}
		
		iir &= XRM1280_IIR_ID_MASK;

		switch (iir) {
		case XRM1280_IIR_RDI_SRC:
		case XRM1280_IIR_RLSE_SRC:
		case XRM1280_IIR_RTOI_SRC:
		case XRM1280_IIR_XOFFI_SRC:
#if 1
			regcache_cache_bypass(s->regmap, true);
			xrm1280_port_write(port, XRM1280_EMSR_REG,0);//set FCTR[7] = 0 for read rx_fifo counter
			rxlen = xrm1280_port_read(port, XRM1280_FC_REG);
			regcache_cache_bypass(s->regmap, false);
#else
			rxlen = xrm1280_port_read(port, XRM1280_RXLVL_REG);
#endif
			if (rxlen)
				xrm1280_handle_rx(port, rxlen, iir);
			break;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
		case XRM1280_IIR_CTSRTS_SRC:
			msr = xrm1280_port_read(port, XRM1280_MSR_REG);
			uart_handle_cts_change(port,
					       !!(msr & XRM1280_MSR_CTS_BIT));
			break;
#endif			
		case XRM1280_IIR_THRI_SRC:
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
			mutex_lock(&s->mutex);
			xrm1280_handle_tx(port);
			mutex_unlock(&s->mutex);
#else
			xrm1280_handle_tx(port);
#endif	
			break;
		default:
			dev_err_ratelimited(port->dev,
					    "ttyXRM%i: Unexpected interrupt: %x",
					    port->line, iir);
			break;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 3)
	} while (0);
	return true;
#else		
	} while (1);
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
static irqreturn_t xrm1280_ist(int irq, void *dev_id)
{
	struct xrm1280_port *s = (struct xrm1280_port *)dev_id;
	int i;

	for (i = 0; i < s->uart.nr; ++i)
		xrm1280_port_irq(s, i);

	return IRQ_HANDLED;
}

static void xrm1280_wq_proc(struct work_struct *ws)
{
	struct xrm1280_one *one = to_xrm1280_one(ws, tx_work);
	struct xrm1280_port *s = dev_get_drvdata(one->port.dev);

	mutex_lock(&s->mutex);
	xrm1280_handle_tx(&one->port);
	mutex_unlock(&s->mutex);
}

static void xrm1280_stop_tx(struct uart_port* port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);
	struct circ_buf *xmit = &one->port.state->xmit;

	/* handle rs485 */
	if (one->rs485.flags & SER_RS485_ENABLED) {
		/* do nothing if current tx not yet completed */
		int lsr = xrm1280_port_read(port, XRM1280_LSR_REG);
		if (!(lsr & XRM1280_LSR_TEMT_BIT))
			return;

		if (uart_circ_empty(xmit) &&
		    (one->rs485.delay_rts_after_send > 0))
			mdelay(one->rs485.delay_rts_after_send);
	}

	xrm1280_port_update(port, XRM1280_IER_REG,
			      XRM1280_IER_THRI_BIT,
			      0);
}

static void xrm1280_stop_rx(struct uart_port* port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	one->port.read_status_mask &= ~XRM1280_LSR_DR_BIT;
	xrm1280_port_update(port, XRM1280_IER_REG,
			      XRM1280_LSR_DR_BIT,
			      0);
}

static void xrm1280_start_tx(struct uart_port *port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	/* handle rs485 */
	if ((one->rs485.flags & SER_RS485_ENABLED) &&
	    (one->rs485.delay_rts_before_send > 0)) {
		mdelay(one->rs485.delay_rts_before_send);
	}

	if (!work_pending(&one->tx_work))
		schedule_work(&one->tx_work);
}

static unsigned int xrm1280_tx_empty(struct uart_port *port)
{
	unsigned int lvl, lsr;

	//lvl = xrm1280_port_read(port, XRM1280_TXLVL_REG);
	
	lsr = xrm1280_port_read(port, XRM1280_LSR_REG);

	return (lsr & XRM1280_LSR_THRE_BIT) ? TIOCSER_TEMT : 0;
}

static void xrm1280_md_proc(struct work_struct *ws)
{
	struct xrm1280_one *one = to_xrm1280_one(ws, md_work);

	xrm1280_port_update(&one->port, XRM1280_MCR_REG,
			      XRM1280_MCR_LOOP_BIT,
			      (one->port.mctrl & TIOCM_LOOP) ?
				      XRM1280_MCR_LOOP_BIT : 0);
}

static void xrm1280_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	schedule_work(&one->md_work);
}

#else	// Above is Kernel < 4.2; Below is when Kernel > 4.2
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)	
static irqreturn_t xrm1280_irq(int irq, void *dev_id)
{
	struct xrm1280_port *s = (struct xrm1280_port *)dev_id;

	mutex_lock(&s->efr_lock);

	while (1) {
		bool keep_polling = false;
		int i;

		for (i = 0; i < s->devtype->nr_uart; ++i)
			keep_polling |= xrm1280_port_irq(s, i);
		if (!keep_polling)
			break;
	}

	mutex_unlock(&s->efr_lock);

	return IRQ_HANDLED;
}
#else		//	Kernel version 5.8
static void xrm1280_ist(struct kthread_work *ws)
{
	struct xrm1280_port *s = to_xrm1280_port(ws, irq_work);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 3)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_lock(&s->efr_lock);
#endif
	while (1) {
		bool keep_polling = false;
		int i;

		for (i = 0; i < s->devtype->nr_uart; ++i)
			keep_polling |= xrm1280_port_irq(s, i);
		if (!keep_polling)
			break;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);
#endif	
#else
	int i;

	for (i = 0; i < s->devtype->nr_uart; ++i)
		xrm1280_port_irq(s, i);
#endif	
}

static irqreturn_t xrm1280_irq(int irq, void *dev_id)
{
	struct xrm1280_port *s = (struct xrm1280_port *)dev_id;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)	
	kthread_queue_work(&s->kworker, &s->irq_work);
#else
	queue_kthread_work(&s->kworker, &s->irq_work);
#endif	

	return IRQ_HANDLED;
}
#endif		//Kernel version 5.8

static void xrm1280_tx_proc(struct kthread_work *ws)
{
	struct uart_port *port = &(to_xrm1280_one(ws, tx_work)->port);

	if ((port->rs485.flags & SER_RS485_ENABLED) &&
	    (port->rs485.delay_rts_before_send > 0))
		msleep(port->rs485.delay_rts_before_send);

	xrm1280_handle_tx(port);
}

static void xrm1280_reconf_rs485(struct uart_port *port)
{
	const u32 fctr_mask = XRM1280_FCTR_AUTO_RS485_BIT | XRM1280_FCTR_ENHANCED_MODE_BIT;
	const u32 emsr_mask = XRM1280_EMSR_RS485_INVERT_BIT;
	u8 lcr = 0;
	u8 fctr = 0;
	u8 emsr = 0;
	struct serial_rs485 *rs485 = &port->rs485;
	unsigned long irqflags;
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	
	spin_lock_irqsave(&port->lock, irqflags);
	if (rs485->flags & SER_RS485_ENABLED) {
		fctr |=	XRM1280_FCTR_AUTO_RS485_BIT;

		if (rs485->flags & SER_RS485_RTS_AFTER_SEND)
			emsr |= XRM1280_EMSR_RS485_INVERT_BIT;
	}
	spin_unlock_irqrestore(&port->lock, irqflags);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_lock(&s->efr_lock);
#endif	

	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Set LCR to mode B (0xBF) to access FCTR */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);

	/* Also enable FCTR[6] for EMSR access */
	fctr |= XRM1280_FCTR_ENHANCED_MODE_BIT;

	regcache_cache_bypass(s->regmap, true);	
	xrm1280_port_update(port, XRM1280_FCTR_REG, fctr_mask, fctr);

	/* Enable EFR[4] for SFR access */
	xrm1280_port_update(port, XRM1280_EFR_REG, XRM1280_EFR_ENABLE_BIT, XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);
	
	/* LCR != 0xBF, FCTR[6] = 1, SFR[0]=0 to access EMSR */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);	
	
	/* make sure lcr is not 0xBF to access EMSR & SFR */
	if(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);
	}	
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);
#endif	
	
	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, 0);	

	regcache_cache_bypass(s->regmap, true);	
	xrm1280_port_update(port, XRM1280_EMSR_REG, emsr_mask, emsr);	
	regcache_cache_bypass(s->regmap, false);
	
	/* Reset SFR after EMSR done */
	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, XRM1280_SFR_ENABLE_GPIO_BIT);	
	
	/* Reset FCTR */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);
	regcache_cache_bypass(s->regmap, true);	
	xrm1280_port_update(port, XRM1280_FCTR_REG, XRM1280_FCTR_ENHANCED_MODE_BIT, 0);
	regcache_cache_bypass(s->regmap, false);	
	
	/* Put LCR back to the normal mode */	
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);	
}

static void xrm1280_reg_proc(struct kthread_work *ws)
{
	struct xrm1280_one *one = to_xrm1280_one(ws, reg_work);
	struct xrm1280_one_config config;
	unsigned long irqflags;

	spin_lock_irqsave(&one->port.lock, irqflags);
	config = one->config;
	memset(&one->config, 0, sizeof(one->config));
	spin_unlock_irqrestore(&one->port.lock, irqflags);

	if (config.flags & XRM1280_RECONF_MD) {
		xrm1280_port_update(&one->port, XRM1280_MCR_REG,
				      XRM1280_MCR_LOOP_BIT,
				      (one->port.mctrl & TIOCM_LOOP) ?
				      XRM1280_MCR_LOOP_BIT : 0);
		xrm1280_port_update(&one->port, XRM1280_MCR_REG,
				      XRM1280_MCR_RTS_BIT,
				      (one->port.mctrl & TIOCM_RTS) ?
				      XRM1280_MCR_RTS_BIT : 0);
		xrm1280_port_update(&one->port, XRM1280_MCR_REG,
				      XRM1280_MCR_DTR_BIT,
				      (one->port.mctrl & TIOCM_DTR) ?
				      XRM1280_MCR_DTR_BIT : 0);
	}
	if (config.flags & XRM1280_RECONF_IER)
		xrm1280_port_update(&one->port, XRM1280_IER_REG,
				      config.ier_clear, 0);

	if (config.flags & XRM1280_RECONF_RS485)
		xrm1280_reconf_rs485(&one->port);
}

static void xrm1280_ier_clear(struct uart_port *port, u8 bit)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	one->config.flags |= XRM1280_RECONF_IER;
	one->config.ier_clear |= bit;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)		
	kthread_queue_work(&s->kworker, &one->reg_work);
#else
	queue_kthread_work(&s->kworker, &one->reg_work);
#endif	
}

static void xrm1280_stop_tx(struct uart_port *port)
{
	xrm1280_ier_clear(port, XRM1280_IER_THRI_BIT);
}

static void xrm1280_stop_rx(struct uart_port *port)
{
	xrm1280_ier_clear(port, XRM1280_IER_RDI_BIT);
}

static void xrm1280_start_tx(struct uart_port *port)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	struct xrm1280_one *one = to_xrm1280_one(port, port);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)		
	kthread_queue_work(&s->kworker, &one->tx_work);
#else
	queue_kthread_work(&s->kworker, &one->tx_work);
#endif		
}

static unsigned int xrm1280_tx_empty(struct uart_port *port)
{
	unsigned int lsr;

	lsr = xrm1280_port_read(port, XRM1280_LSR_REG);

	return (lsr & XRM1280_LSR_TEMT_BIT) ? TIOCSER_TEMT : 0;
}

static void xrm1280_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	one->config.flags |= XRM1280_RECONF_MD;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)		
	kthread_queue_work(&s->kworker, &one->reg_work);
#else
	queue_kthread_work(&s->kworker, &one->reg_work);
#endif			
}
#endif	//Kernel < 4.2

static unsigned int xrm1280_get_mctrl(struct uart_port *port)
{
	/* DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted
	 */
	return TIOCM_DSR | TIOCM_CAR;
}

static void xrm1280_break_ctl(struct uart_port *port, int break_state)
{
	xrm1280_port_update(port, XRM1280_LCR_REG,
			      XRM1280_LCR_TXBREAK_BIT,
			      break_state ? XRM1280_LCR_TXBREAK_BIT : 0);
}

static void xrm1280_set_termios(struct uart_port *port,
				  struct ktermios *termios,
				  const struct ktermios *old)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int lcr, flow = 0;
	int baud;
	/* Mask termios capabilities we don't support */
	termios->c_cflag &= ~CMSPAR;

	/* Word size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = XRM1280_LCR_WORD_LEN_5;
		break;
	case CS6:
		lcr = XRM1280_LCR_WORD_LEN_6;
		break;
	case CS7:
		lcr = XRM1280_LCR_WORD_LEN_7;
		break;
	case CS8:
		lcr = XRM1280_LCR_WORD_LEN_8;
		break;
	default:
		lcr = XRM1280_LCR_WORD_LEN_8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
		break;
	}

	/* Parity */
	if (termios->c_cflag & PARENB) {
		lcr |= XRM1280_LCR_PARITY_BIT;
		if (!(termios->c_cflag & PARODD))
			lcr |= XRM1280_LCR_EVENPARITY_BIT;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= XRM1280_LCR_STOPLEN_BIT; /* 2 stops */

	/* Set read status mask */
	port->read_status_mask = XRM1280_LSR_OE_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= XRM1280_LSR_PE_BIT |
					  XRM1280_LSR_FE_BIT;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= XRM1280_LSR_BI_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= XRM1280_LSR_BI_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= XRM1280_LSR_BRK_ERROR_MASK;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	/* As above, claim the mutex while accessing the EFR. */
	mutex_lock(&s->efr_lock);
#endif	
	
	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_B);

	/* Configure flow control */
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_write(port, XRM1280_XON1_REG, termios->c_cc[VSTART]);
	xrm1280_port_write(port, XRM1280_XOFF1_REG, termios->c_cc[VSTOP]);
	if (termios->c_cflag & CRTSCTS)
		flow |= XRM1280_EFR_AUTOCTS_BIT |
			XRM1280_EFR_AUTORTS_BIT;
	if (termios->c_iflag & IXON)
		flow |= XRM1280_EFR_SWFLOW3_BIT;
	if (termios->c_iflag & IXOFF)
		flow |= XRM1280_EFR_SWFLOW1_BIT;

	xrm1280_port_write(port, XRM1280_EFR_REG, flow);
	regcache_cache_bypass(s->regmap, false);

	/* Update LCR register */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);
#endif	

	/* Get baud rate generator configuration */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 4 / 0xffff,
				  port->uartclk / 16);

	/* Setup baudrate generator */
	baud = xrm1280_set_baud(port, baud);

	if(baud < 0)
	{
		/* baud to set is over range */
		printk(" xrm1280_set_termios: baud not supported, remain unchanged \n");
		return;
	}
	
	/* Update timeout according to new baud rate */
	uart_update_timeout(port, termios->c_cflag, baud);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)	
#if defined(TIOCSRS485) && defined(TIOCGRS485)
static void xrm1280_config_rs485(struct uart_port *port,
				   struct serial_rs485 *rs485)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);
	u8 lcr;

	one->rs485 = *rs485;
	
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Set LCR to mode B (0xBF) to access FCTR */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);

	if (one->rs485.flags & SER_RS485_ENABLED) {
		xrm1280_port_update(port, XRM1280_FCTR_REG, XRM1280_FCTR_AUTO_RS485_BIT, XRM1280_FCTR_AUTO_RS485_BIT);				
	} else {
		xrm1280_port_update(port, XRM1280_FCTR_REG, XRM1280_FCTR_AUTO_RS485_BIT, 0);
	}
	
	/* Put LCR back to the normal mode */	
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);	
}
#endif
#else	// Above is Kernel < 4.1; below is Kernel > 4.1
static int xrm1280_config_rs485(struct uart_port *port,struct ktermios *termios,
				  struct serial_rs485 *rs485)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
	const u32 fctr_mask = XRM1280_FCTR_AUTO_RS485_BIT | XRM1280_FCTR_ENHANCED_MODE_BIT;
	const u32 emsr_mask = XRM1280_EMSR_RS485_INVERT_BIT;
	u8 lcr = 0;
	u8 fctr = 0;
	u8 emsr = 0;
#else		
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	struct xrm1280_one *one = to_xrm1280_one(port, port);
#endif

	if (rs485->flags & SER_RS485_ENABLED) {
		bool rts_during_rx, rts_during_tx;

		rts_during_rx = rs485->flags & SER_RS485_RTS_AFTER_SEND;
		rts_during_tx = rs485->flags & SER_RS485_RTS_ON_SEND;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)		
		fctr |=	XRM1280_FCTR_AUTO_RS485_BIT;
		
		if (!rts_during_rx && rts_during_tx)
			/* default */;
		else if (rts_during_rx && !rts_during_tx)
			emsr |= XRM1280_EMSR_RS485_INVERT_BIT;
		else
#else
		if (rts_during_rx == rts_during_tx)
#endif	
			dev_err(port->dev,
				"unsupported RTS signalling on_send:%d after_send:%d - exactly one of RS485 RTS flags should be set\n",
				rts_during_tx, rts_during_rx);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)						
		/*
		 * RTS signal is handled by HW, it's timing can't be influenced.
		 * However, it's sometimes useful to delay TX even without RTS
		 * control therefore we try to handle .delay_rts_before_send.
		 */
		if (rs485->delay_rts_after_send)
			return -EINVAL;
#endif		
	}

	port->rs485 = *rs485;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)		
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Set LCR to mode B (0xBF) to access FCTR */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);

	/* Also enable FCTR[6] for EMSR access */
  fctr |= XRM1280_FCTR_ENHANCED_MODE_BIT;
	
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_update(port, XRM1280_FCTR_REG, fctr_mask, fctr);

	/* Enable EFR[4] for SFR access SFR[0]=0 */
	xrm1280_port_update(port, XRM1280_EFR_REG, XRM1280_EFR_ENABLE_BIT, XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);
	
	/* LCR != 0xBF, FCTR[6] = 1, SFR[0]=0 to access EMSR */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);	

	/* make sure lcr is not 0xBF to access EMSR */
	if(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);
	}	

	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, 0);	
	
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_update(port, XRM1280_EMSR_REG, emsr_mask, emsr);	
	regcache_cache_bypass(s->regmap, false);
	
	/* Reset SFR after EMSR done */
	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, XRM1280_SFR_ENABLE_GPIO_BIT);	
	
	/* Reset FCTR */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);
	regcache_cache_bypass(s->regmap, true);	
	xrm1280_port_update(port, XRM1280_FCTR_REG, XRM1280_FCTR_ENHANCED_MODE_BIT, 0);
	regcache_cache_bypass(s->regmap, false);
	
	/* Put LCR back to the normal mode */	
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);	
#else	
	one->config.flags |= XRM1280_RECONF_RS485;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)		
	kthread_queue_work(&s->kworker, &one->reg_work);
#else
	queue_kthread_work(&s->kworker, &one->reg_work);
#endif		
#endif

	return 0;
}	
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)	
static int xrm1280_ioctl(struct uart_port *port, unsigned int cmd,
			   unsigned long arg)
{
#if defined(TIOCSRS485) && defined(TIOCGRS485)
		struct serial_rs485 rs485;
	
		switch (cmd) {
		case TIOCSRS485:
			if (copy_from_user(&rs485, (void __user *)arg, sizeof(rs485)))
				return -EFAULT;
	
			xrm1280_config_rs485(port, &rs485);
			return 0;
		case TIOCGRS485:
			if (copy_to_user((void __user *)arg,
					 &(to_xrm1280_one(port, port)->rs485),
					 sizeof(rs485)))
				return -EFAULT;
			return 0;
		default:
			break;
		}
#endif
	   return -ENOIOCTLCMD;


	return -ENOIOCTLCMD;
}
#endif	

static int xrm1280_startup(struct uart_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)	
	struct xrm1280_one *one = to_xrm1280_one(port, port);
	u8 lcr;
#endif	
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int val;
	u8 fifo_count;
	
	xrm1280_power(port, 1);
	
	/* Reset FIFOs*/
	val = XRM1280_FCR_RXRESET_BIT | XRM1280_FCR_TXRESET_BIT;
	xrm1280_port_write(port, XRM1280_FCR_REG, val);
	udelay(5);
	xrm1280_port_write(port, XRM1280_FCR_REG,
			     XRM1280_FCR_FIFO_BIT);

	/* Enable EFR */
	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_B);

	regcache_cache_bypass(s->regmap, true);

	/* Enable write access to enhanced features and internal clock div */
	xrm1280_port_write(port, XRM1280_EFR_REG,
			     XRM1280_EFR_ENABLE_BIT);
			     
	xrm1280_port_write(port, XRM1280_FCTR_REG,0x40);
	regcache_cache_bypass(s->regmap, false);

	/* Now, initialize the UART */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_WORD_LEN_8);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)	
	/* Enable IrDA mode if requested in DT */
	/* This bit must be written with LCR != 0xBF && EFR[4] = 1 */  
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* make sure lcr is not 0xBF to access MCR[6] */
	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_WORD_LEN_8);	// This should be done above already
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	
	
	xrm1280_port_update(port, XRM1280_MCR_REG,
			      XRM1280_MCR_IRDA_BIT,
			      one->irda_mode ?
				XRM1280_MCR_IRDA_BIT : 0);
#endif

	/* Enable the Rx and Tx FIFO */
	xrm1280_port_update(port, XRM1280_SFR_REG,
			      XRM1280_SFR_RX_DISABLE_BIT |
			      XRM1280_SFR_TX_DISABLE_BIT,
			      0);
			      
	regcache_cache_bypass(s->regmap, true);
    xrm1280_port_write(port, XRM1280_EMSR_REG,1);//set FCTR[7] = 0 for read tx_fifo counter
	fifo_count = xrm1280_port_read(port, XRM1280_FC_REG);
	//printk("tx fifo_count =%d\n",fifo_count);
	xrm1280_port_write(port, XRM1280_EMSR_REG,0);//set FCTR[7] = 0 for read rx_fifo counter
	fifo_count = xrm1280_port_read(port, XRM1280_FC_REG);
	//printk("rx fifo_count =%d\n",fifo_count);
	regcache_cache_bypass(s->regmap, false);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
	/* Enable RX, TX, CTS change interrupts */
	val = XRM1280_IER_RDI_BIT | XRM1280_IER_THRI_BIT |
	      XRM1280_IER_CTSI_BIT;
#else
	/* Enable RX, TX interrupts */
	val = XRM1280_IER_RDI_BIT | XRM1280_IER_THRI_BIT;
#endif				
	xrm1280_port_write(port, XRM1280_IER_REG, val);

	return 0;
}

static void xrm1280_shutdown(struct uart_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)	
	struct xrm1280_port *s = dev_get_drvdata(port->dev);	
#endif	
	u8 lcr;
	
	/* Disable all interrupts */
	xrm1280_port_write(port, XRM1280_IER_REG, 0);
	
	/* LCR should not be 0xBF, but check just to make sure SFR can be correctly accessed */
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	// Disable the Divisor bit to avoid LCR==0xBF
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	
	/* Disable TX/RX */	 
	xrm1280_port_update(port, XRM1280_SFR_REG,
					XRM1280_SFR_RX_DISABLE_BIT |
					XRM1280_SFR_TX_DISABLE_BIT,
					XRM1280_SFR_RX_DISABLE_BIT |
					XRM1280_SFR_TX_DISABLE_BIT);				

	xrm1280_power(port, 0);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
  flush_kthread_worker(&s->kworker);
#else	
	kthread_flush_worker(&s->kworker);
#endif
}

static const char *xrm1280_type(struct uart_port *port)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);

	return (port->type == PORT_SC16IS7XX) ? s->devtype->name : NULL;
}

static int xrm1280_request_port(struct uart_port *port)
{
	/* Do nothing */
	return 0;
}

static void xrm1280_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_SC16IS7XX;
}

static int xrm1280_verify_port(struct uart_port *port,
				 struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_SC16IS7XX))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

static void xrm1280_pm(struct uart_port *port, unsigned int state,
			 unsigned int oldstate)
{
	xrm1280_power(port, (state == UART_PM_STATE_ON) ? 1 : 0);
}

static void xrm1280_null_void(struct uart_port *port)
{
	/* Do nothing */
}

static const struct uart_ops xrm1280_ops = {
	.tx_empty	= xrm1280_tx_empty,
	.set_mctrl	= xrm1280_set_mctrl,
	.get_mctrl	= xrm1280_get_mctrl,
	.stop_tx	= xrm1280_stop_tx,
	.start_tx	= xrm1280_start_tx,
	.stop_rx	= xrm1280_stop_rx,
	.break_ctl	= xrm1280_break_ctl,
	.startup	= xrm1280_startup,
	.shutdown	= xrm1280_shutdown,
	.set_termios	= xrm1280_set_termios,
	.type		= xrm1280_type,
	.request_port	= xrm1280_request_port,
	.release_port	= xrm1280_null_void,
	.config_port	= xrm1280_config_port,
	.verify_port	= xrm1280_verify_port,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)	
	.ioctl		= xrm1280_ioctl,
#endif	
	.pm		= xrm1280_pm,
};

#ifdef CONFIG_GPIOLIB
static int xrm1280_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	unsigned int val;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
#else
	struct xrm1280_port *s = gpiochip_get_data(chip);	
#endif	
	struct uart_port *port = &s->p[0].port;
	u8 lcr = 0;
	u8 sfr = XRM1280_SFR_ENABLE_GPIO_BIT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_lock(&s->efr_lock);	
#endif
	/* To access GPIOLVL, LCR != 0xBF, FCTR[6]=0, SFR[0]=1 */
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Setup FCTR[6] first */
	/* Set LCR to mode B (0xBF) to access FCTR; also setup EFR[4] for SFR access */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);	
	
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_update(port, XRM1280_FCTR_REG, XRM1280_FCTR_ENHANCED_MODE_BIT, 0);
	xrm1280_port_update(port, XRM1280_EFR_REG, XRM1280_EFR_ENABLE_BIT, XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);
	
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);
	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	// Disable the Divisor bit to avoid LCR==0xBF
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);	
#endif
	
	if(offset > 7)
	{
		sfr |= XRM1280_SFR_GPIO_SELECT_BIT;
		offset = offset - 8; 	// reconfigure the byte offset value if GPIO[15:8]
	}
	
	/* Set SFR[0] to 1 for GPIO access & GPIO select if GPIO[15:8] is being accessed */
	xrm1280_port_update(port, XRM1280_SFR_REG, (XRM1280_SFR_ENABLE_GPIO_BIT | XRM1280_SFR_GPIO_SELECT_BIT), sfr);
	
	regcache_cache_bypass(s->regmap, true);
	val = xrm1280_port_read(port, XRM1280_GPIOLVL_REG);
	regcache_cache_bypass(s->regmap, false);
	
	/* Reset SFR[0] */
	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, 0);
	
	return !!(val & BIT(offset));
}

static void xrm1280_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
#else
	struct xrm1280_port *s = gpiochip_get_data(chip);	
#endif							
	struct uart_port *port = &s->p[0].port;
	u8 lcr = 0;
	u8 sfr = XRM1280_SFR_ENABLE_GPIO_BIT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_lock(&s->efr_lock);	
#endif	
	/* To access GPIOLVL, LCR != 0xBF, FCTR[6]=0, SFR[0]=1 */
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Setup FCTR[6] first */
	/* Set LCR to mode B (0xBF) to access FCTR; also setup EFR[4] for SFR access */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);	
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_update(port, XRM1280_FCTR_REG, XRM1280_FCTR_ENHANCED_MODE_BIT, 0);
	xrm1280_port_update(port, XRM1280_EFR_REG, XRM1280_EFR_ENABLE_BIT, XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);
	
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);
	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	// Disable the Divisor bit to avoid LCR==0xBF
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);	
#endif
	
	if(offset > 7)
	{
		sfr |= XRM1280_SFR_GPIO_SELECT_BIT;
		offset = offset - 8; 	// reconfigure the byte offset value if GPIO[15:8]
	}
	
	/* Set SFR[0] to 1 for GPIO access & GPIO select if GPIO[15:8] is being accessed */
	xrm1280_port_update(port, XRM1280_SFR_REG, (XRM1280_SFR_ENABLE_GPIO_BIT | XRM1280_SFR_GPIO_SELECT_BIT), sfr);
	
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_update(port, XRM1280_GPIOLVL_REG, BIT(offset),
			      val ? BIT(offset) : 0);
	regcache_cache_bypass(s->regmap, false);					

	/* Reset SFR[0] */
	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, 0);
}

static int xrm1280_gpio_direction_input(struct gpio_chip *chip,
					  unsigned offset)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
#else
	struct xrm1280_port *s = gpiochip_get_data(chip);	
#endif							
	struct uart_port *port = &s->p[0].port;
	u8 lcr = 0;
	u8 sfr = XRM1280_SFR_ENABLE_GPIO_BIT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_lock(&s->efr_lock);	
#endif

	/* To access GPIOSEL, LCR = 0xBF, SFR[0]=1 */
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Set LCR to mode B (0xBF) to setup EFR[4] for SFR access */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);	
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_update(port, XRM1280_EFR_REG, XRM1280_EFR_ENABLE_BIT, XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);
	
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);
	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	// Disable the Divisor bit to avoid LCR==0xBF
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);	
#endif
	
	if(offset > 7)
	{
		sfr |= XRM1280_SFR_GPIO_SELECT_BIT;
		offset = offset - 8; 	// reconfigure the byte offset value if GPIO[15:8]
	}
	
	/* Set SFR[0] to 1 for GPIO access & GPIO select if GPIO[15:8] is being accessed */
	xrm1280_port_update(port, XRM1280_SFR_REG, (XRM1280_SFR_ENABLE_GPIO_BIT | XRM1280_SFR_GPIO_SELECT_BIT), sfr);
	
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	/* Set LCR to mode B (0xBF) for GPIOSEL access */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);	
	
	regcache_cache_bypass(s->regmap, true);
	/* For XRM1280, GPIO as input is Logic 1 as XRM117x is Logic 0 */
	xrm1280_port_update(port, XRM1280_GPIOSEL_REG, BIT(offset), BIT(offset));
	regcache_cache_bypass(s->regmap, false);

	/* Reset LCR */						
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);
	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	// Disable the Divisor bit to avoid LCR==0xBF
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	
	
	/* Reset SFR[0] */
	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, 0);

	return 0;
}

static int xrm1280_gpio_direction_output(struct gpio_chip *chip,
					   unsigned offset, int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)	
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
#else
	struct xrm1280_port *s = gpiochip_get_data(chip);	
#endif								
	struct uart_port *port = &s->p[0].port;
	u8 lcr = 0;
	u8 sfr = XRM1280_SFR_ENABLE_GPIO_BIT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_lock(&s->efr_lock);	
#endif
	
	/* To access GPIOLVL, LCR != 0xBF, SFR[0]=1, FCTR[6]=0 */
	/* To access GPIOSEL, LCR = 0xBF, SFR[0]=1 */
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Set LCR to mode B (0xBF) to setup EFR[4] for SFR access */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);	
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_update(port, XRM1280_FCTR_REG, XRM1280_FCTR_ENHANCED_MODE_BIT, 0);
	xrm1280_port_update(port, XRM1280_EFR_REG, XRM1280_EFR_ENABLE_BIT, XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);
	
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);
	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	// Disable the Divisor bit to avoid LCR==0xBF
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)
	mutex_unlock(&s->efr_lock);	
#endif
	
	if(offset > 7)
	{
		sfr |= XRM1280_SFR_GPIO_SELECT_BIT;
		offset = offset - 8; 	// reconfigure the byte offset value if GPIO[15:8]
	}
	
	/* Set SFR[0] to 1 for GPIO access & GPIO select if GPIO[15:8] is being accessed */
	xrm1280_port_update(port, XRM1280_SFR_REG, (XRM1280_SFR_ENABLE_GPIO_BIT | XRM1280_SFR_GPIO_SELECT_BIT), sfr);
	
	regcache_cache_bypass(s->regmap, true);
	/* Set corresponding GPIOLVL */
	xrm1280_port_update(port, XRM1280_GPIOLVL_REG, BIT(offset),
			      val ? BIT(offset) : 0);
	regcache_cache_bypass(s->regmap, false);					
						
	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	/* Set LCR to mode B (0xBF) for GPIOSEL access */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_CONF_MODE_B);	
	
	regcache_cache_bypass(s->regmap, true);
	/* For XRM1280, GPIO as output is Logic 0 opposing to XRM117x's Logic 1 */
	xrm1280_port_update(port, XRM1280_GPIOSEL_REG, BIT(offset), 0);
	regcache_cache_bypass(s->regmap, false);

	/* Reset LCR */						
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);
	while(lcr == XRM1280_LCR_CONF_MODE_B)
	{
		xrm1280_port_update(port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	// Disable the Divisor bit to avoid LCR==0xBF
		lcr = xrm1280_port_read(port, XRM1280_LCR_REG);
	}	
	
	/* Reset SFR[0] */
	xrm1280_port_update(port, XRM1280_SFR_REG, XRM1280_SFR_ENABLE_GPIO_BIT, 0);

	return 0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)	
static int xrm1280_probe(struct device *dev,
			   struct xrm1280_devtype *devtype,
			   struct regmap *regmap, int irq, unsigned long flags)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)	
static int xrm1280_probe(struct device *dev,
			   const struct xrm1280_devtype *devtype,
			   struct regmap *regmap, int irq)
#else
static int xrm1280_probe(struct device *dev,
			   const struct xrm1280_devtype *devtype,
			   struct regmap *regmap, int irq, unsigned long flags)	
#endif	
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)	
	struct sched_param sched_param = { .sched_priority = MAX_RT_PRIO / 2 };
#endif	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	unsigned long freq = 0, *pfreq = dev_get_platdata(dev);
	u32 uartclk = 0;
#else
	unsigned long freq, *pfreq = dev_get_platdata(dev);
#endif
	int i, ret;
	struct xrm1280_port *s;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)		
	unsigned int val;
#endif
	u8 lcr;
	
  if (IS_ERR(regmap))
		return PTR_ERR(regmap);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)		
	/*
	 * This device does not have an identification register that would
	 * tell us if we are really connected to the correct device.
	 * The best we can do is to check if communication is at all possible.
	 */
	ret = regmap_read(regmap,
			  XRM1280_LSR_REG << XRM1280_REG_SHIFT, &val);
	if (ret < 0)
		return ret;
#endif
	
	/* Alloc port structure */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0)
	s = devm_kzalloc(dev, struct_size(s, p, devtype->nr_uart), GFP_KERNEL);
#else	
	s = devm_kzalloc(dev, sizeof(*s) +
			 sizeof(struct xrm1280_one) * devtype->nr_uart,
			 GFP_KERNEL);
#endif			 

	if (!s) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	/* Always ask for fixed clock rate from a property. */
	device_property_read_u32(dev, "clock-frequency", &uartclk);
#endif
	
 	s->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->clk)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
		if (uartclk)
			freq = uartclk;
#endif		
		if (pfreq)
			freq = *pfreq;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
		if (freq)
			dev_dbg(dev, "Clock frequency: %luHz\n", freq);
#endif		
		else
			return PTR_ERR(s->clk);
	} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0)
		ret = clk_prepare_enable(s->clk);
		if (ret)
			return ret;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)	
		clk_prepare_enable(s->clk);
#endif	//KERNEL > 4.18 or 4.1~4.18
		freq = clk_get_rate(s->clk);
	}
  
	freq = 18432000; //freq = 14745600;	/* Fix XTAL frequency setting if needed, typically using 24 MHz 
	s->regmap = regmap;
	s->devtype = devtype;
	dev_set_drvdata(dev, s);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 82)	
	mutex_init(&s->efr_lock);
#endif	
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)	
	/* Register UART driver */
	s->uart.owner		= THIS_MODULE;
	s->uart.dev_name	= "ttyXRM";
	s->uart.nr		= devtype->nr_uart;
	ret = uart_register_driver(&s->uart);
	if (ret) {
		dev_err(dev, "Registering UART driver failed\n");
		goto out_clk;
	}
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	init_kthread_worker(&s->kworker);
	init_kthread_work(&s->irq_work, xrm1280_ist);
#else		
	kthread_init_worker(&s->kworker);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)		
	kthread_init_work(&s->irq_work, xrm1280_ist);
#endif	//Kernel version 5.8
#endif	//Kernel < 4.9
	s->kworker_task = kthread_run(kthread_worker_fn, &s->kworker,
				      "xrm1280");
	if (IS_ERR(s->kworker_task)) {
		ret = PTR_ERR(s->kworker_task);
		goto out_clk;
	}
	sched_setscheduler(s->kworker_task, SCHED_FIFO, &sched_param);
#endif	// Kernel < 4.3
	
#ifdef CONFIG_GPIOLIB
	if (devtype->nr_gpio) {
		/* Setup GPIO cotroller */
		s->gpio.owner		 = THIS_MODULE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)	
		s->gpio.dev		 = dev;
#else
		s->gpio.parent		 = dev;
#endif	
		s->gpio.label		 = dev_name(dev);
		s->gpio.direction_input	 = xrm1280_gpio_direction_input;
		s->gpio.get		 = xrm1280_gpio_get;
		s->gpio.direction_output = xrm1280_gpio_direction_output;
		s->gpio.set		 = xrm1280_gpio_set;
		s->gpio.base		 = -1;
		s->gpio.ngpio		 = devtype->nr_gpio;
		s->gpio.can_sleep	 = 1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)		
		ret = gpiochip_add(&s->gpio);
#else
		ret = gpiochip_add_data(&s->gpio, s);
#endif	
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)			
		if (ret)
			goto out_uart;
#else
		if (ret)
			goto out_thread;
#endif	
	}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
	mutex_init(&s->mutex);
#endif	

	for (i = 0; i < devtype->nr_uart; ++i) {
		/* Initialize port data */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)			
		s->p[i].port.line	= i;
#else
		s->p[i].line		= i;
		s->p[i].port.line	= xrm1280_alloc_line();
		if (s->p[i].port.line >= XRM1280_MAX_DEVS) {
			ret = -ENOMEM;
			goto out_ports;
		}
#endif	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)	
		s->p[i].port.rs485_config = xrm1280_config_rs485;
#endif		
		s->p[i].port.dev	= dev;
		s->p[i].port.irq	= irq;
		s->p[i].port.type	= PORT_SC16IS7XX;
		s->p[i].port.fifosize	= XRM1280_FIFO_SIZE;
		s->p[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		s->p[i].port.iotype	= UPIO_PORT;
		s->p[i].port.uartclk	= freq;
		s->p[i].port.ops	= &xrm1280_ops;
		/* LCR should not be 0xBF, but check just to make sure SFR can be correctly accessed */
		lcr = xrm1280_port_read(&s->p[i].port, XRM1280_LCR_REG);

		while((lcr == XRM1280_LCR_CONF_MODE_B) || (lcr == XRM1280_LCR_CONF_MODE_A))
		{
			/* Avoid LCR=0xBF or LCR[7]=1. This is to make sure IER & SFR is correctly accessed */
			xrm1280_port_update(&s->p[i].port, XRM1280_LCR_REG, XRM1280_LCR_DLAB_BIT, 0);	
			lcr = xrm1280_port_read(&s->p[i].port, XRM1280_LCR_REG);
		}	
		/* Disable all interrupts */
		xrm1280_port_write(&s->p[i].port, XRM1280_IER_REG, 0);
		/* Disable TX/RX */
		xrm1280_port_write(&s->p[i].port, XRM1280_SFR_REG,
							XRM1280_SFR_RX_DISABLE_BIT |
							XRM1280_SFR_TX_DISABLE_BIT);	
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)						 
		/* Initialize queue for start TX */
		INIT_WORK(&s->p[i].tx_work, xrm1280_wq_proc);
		/* Initialize queue for changing mode */
		INIT_WORK(&s->p[i].md_work, xrm1280_md_proc);
		/* Register port */
		uart_add_one_port(&s->uart, &s->p[i].port);
#else
		/* Initialize kthread work structs */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)	
		init_kthread_work(&s->p[i].tx_work, xrm1280_tx_proc);
		init_kthread_work(&s->p[i].reg_work, xrm1280_reg_proc);
#else
		kthread_init_work(&s->p[i].tx_work, xrm1280_tx_proc);
		kthread_init_work(&s->p[i].reg_work, xrm1280_reg_proc);
#endif
		/* Register port */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)			
		uart_add_one_port(&xrm1280_uart, &s->p[i].port);
#else		
		uart_add_one_port(&s->uart, &s->p[i].port);
#endif

		/* Enable EFR */
		xrm1280_port_write(&s->p[i].port, XRM1280_LCR_REG,
				     XRM1280_LCR_CONF_MODE_B);

		regcache_cache_bypass(s->regmap, true);

		/* Enable write access to enhanced features */
		xrm1280_port_write(&s->p[i].port, XRM1280_EFR_REG,
				     XRM1280_EFR_ENABLE_BIT);

		regcache_cache_bypass(s->regmap, false);

		/* Restore access to general registers */
		xrm1280_port_write(&s->p[i].port, XRM1280_LCR_REG, 0x00);	
#endif	
		/* Go to suspend mode */
		xrm1280_power(&s->p[i].port, 0);
	}
 
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
	/* Setup interrupt */
	ret = devm_request_threaded_irq(dev, irq, NULL, xrm1280_ist,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING | flags, dev_name(dev), s);
	//printk("xrm1280_probe devm_request_threaded_irq =%d\n",ret);
	
	if (!ret)
		return 0;

	mutex_destroy(&s->mutex);

#ifdef CONFIG_GPIOLIB
	if (devtype->nr_gpio)
		gpiochip_remove(&s->gpio);

out_uart:
#endif
	uart_unregister_driver(&s->uart);
#else		//KERNEL < 4.2
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)	
	if (dev->of_node) {
		struct property *prop;
		const __be32 *p;
		u32 u;

		of_property_for_each_u32(dev->of_node, "irda-mode-ports",
					 prop, p, u)
			if (u < devtype->nr_uart)
				s->p[u].irda_mode = true;
	}

	/*
	 * Setup interrupt. We first try to acquire the IRQ line as level IRQ.
	 * If that succeeds, we can allow sharing the interrupt as well.
	 * In case the interrupt controller doesn't support that, we fall
	 * back to a non-shared falling-edge trigger.
	 */
	ret = devm_request_threaded_irq(dev, irq, NULL, xrm1280_irq,
					IRQF_TRIGGER_LOW | IRQF_SHARED |
					IRQF_ONESHOT,
					dev_name(dev), s);
	if (!ret)
		return 0;

	ret = devm_request_threaded_irq(dev, irq, NULL, xrm1280_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					dev_name(dev), s);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 2)	
	/* Setup interrupt */	
	ret = devm_request_irq(dev, irq, xrm1280_irq,
			       IRQF_ONESHOT | flags, dev_name(dev), s);
#else
	/* Setup interrupt */	
	ret = devm_request_irq(dev, irq, xrm1280_irq,
			       flags, dev_name(dev), s);
#endif	//Kernel version 5.8				 
	if (!ret)
		return 0;

out_ports:
	for (i--; i >= 0; i--) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)			
		uart_remove_one_port(&xrm1280_uart, &s->p[i].port);
		clear_bit(s->p[i].port.line, &xrm1280_lines);
#else
		uart_remove_one_port(&s->uart, &s->p[i].port);	
#endif		
	}

#ifdef CONFIG_GPIOLIB
	if (devtype->nr_gpio)
		gpiochip_remove(&s->gpio);

out_thread:
#endif
	kthread_stop(s->kworker_task);
#endif	//KERNEL < 4.2

out_clk:
	if (!IS_ERR(s->clk))
		clk_disable_unprepare(s->clk);

	return ret;
}

static void xrm1280_remove(struct device *dev)
{
	struct xrm1280_port *s = dev_get_drvdata(dev);
	int i;

#ifdef CONFIG_GPIOLIB
	if (s->devtype->nr_gpio)
		gpiochip_remove(&s->gpio);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)	
	for (i = 0; i < s->uart.nr; i++) {
		cancel_work_sync(&s->p[i].tx_work);
		cancel_work_sync(&s->p[i].md_work);
		uart_remove_one_port(&s->uart, &s->p[i].port);
		xrm1280_power(&s->p[i].port, 0);
	}

	mutex_destroy(&s->mutex);
#else
	for (i = 0; i < s->devtype->nr_uart; i++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)			
		uart_remove_one_port(&s->uart, &s->p[i].port);
#else
		uart_remove_one_port(&xrm1280_uart, &s->p[i].port);
#endif		
		clear_bit(s->p[i].port.line, &xrm1280_lines);
		xrm1280_power(&s->p[i].port, 0);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	flush_kthread_worker(&s->kworker);		
#else
	kthread_flush_worker(&s->kworker);
#endif	
	kthread_stop(s->kworker_task);
#endif	

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)	
	uart_unregister_driver(&s->uart);
#endif
	
	if (!IS_ERR(s->clk))
		clk_disable_unprepare(s->clk);

	return;
}

static const struct of_device_id __maybe_unused xrm1280_dt_ids[] = {
	{ .compatible = "exar,xrm1280",	.data = &xrm1280_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(of, xrm1280_dt_ids);

static struct regmap_config regcfg = {
	.reg_bits = 7,
	.pad_bits = 1,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = xrm1280_regmap_volatile,
	.precious_reg = xrm1280_regmap_precious,
};

#ifdef CONFIG_SERIAL_XR20M1280_SPI
static int xrm1280_spi_probe(struct spi_device *spi)
{
	const struct xrm1280_devtype *devtype;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)			
	unsigned long flags = 0;
#endif
	struct regmap *regmap;
	int ret;
	
	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	/* only supports mode 0 on SC16IS762 */
	spi->mode		= spi->mode ? : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? : 15000000;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	if (spi->dev.of_node) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)	
		devtype = device_get_match_data(&spi->dev);
		if (!devtype)
			return -ENODEV;
#else		
		const struct of_device_id *of_id =
			of_match_device(xrm1280_dt_ids, &spi->dev);

		if (!of_id)
			return -ENODEV;

		devtype = (struct xrm1280_devtype *)of_id->data;
#endif			
	} else {
		const struct spi_device_id *id_entry = spi_get_device_id(spi);

		devtype = (struct xrm1280_devtype *)id_entry->driver_data;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)			
		flags = IRQF_TRIGGER_FALLING;
#endif	
	}

	regcfg.max_register = (0xf << XRM1280_REG_SHIFT) |
			      (devtype->nr_uart - 1);
	regmap = devm_regmap_init_spi(spi, &regcfg);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)		
	return xrm1280_probe(&spi->dev, devtype, regmap, spi->irq);
#else
	return xrm1280_probe(&spi->dev, devtype, regmap, spi->irq, flags);
#endif
}

static void xrm1280_spi_remove(struct spi_device *spi)
{
	xrm1280_remove(&spi->dev);
}

static const struct spi_device_id xrm1280_spi_id_table[] = {
	{ "xrm1280",	(kernel_ulong_t)&xrm1280_devtype, },
	{ }
};

MODULE_DEVICE_TABLE(spi, xrm1280_spi_id_table);

static struct spi_driver xrm1280_spi_uart_driver = {
	.driver = {
		.name		= XRM1280_NAME,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)	
		.of_match_table	= xrm1280_dt_ids,
#else		
		.of_match_table	= of_match_ptr(xrm1280_dt_ids),
#endif	
	},
	.probe		= xrm1280_spi_probe,
	.remove		= xrm1280_spi_remove,
	.id_table	= xrm1280_spi_id_table,
};

MODULE_ALIAS("spi:xrm1280");
#endif	//CONFIG_SERIAL_XR20M1280_SPI

#ifdef CONFIG_SERIAL_XR20M1280_I2C
static int xrm1280_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)
	struct xrm1280_devtype *devtype;
#else
	const struct xrm1280_devtype *devtype;
#endif	
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)			
	unsigned long flags = 0;
#endif
	struct regmap *regmap;
	int ret;
   	if (i2c->dev.of_node) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)	
		devtype = device_get_match_data(&i2c->dev);
		if (!devtype)
			return -ENODEV;
#else		//KERNEL >= 5.2
		const struct of_device_id *of_id =
				of_match_device(xrm1280_dt_ids, &i2c->dev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
		if (!of_id)
			return -ENODEV;
#endif	
				
		devtype = (struct xrm1280_devtype *)of_id->data;
#endif		//KERNEL < 5.2
	} else {
		devtype = (struct xrm1280_devtype *)id->driver_data;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)			
		flags = IRQF_TRIGGER_FALLING;
#endif	
	}

	regcfg.max_register = (0xf << XRM1280_REG_SHIFT) |
			      (devtype->nr_uart - 1);
	regmap = devm_regmap_init_i2c(i2c, &regcfg);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)		
	ret = xrm1280_probe(&i2c->dev, devtype, regmap, i2c->irq);
#else	
	ret = xrm1280_probe(&i2c->dev, devtype, regmap, i2c->irq, flags);
#endif
	return ret;
}

static void xrm1280_i2c_remove(struct i2c_client *client)
{
	xrm1280_remove(&client->dev);
}

static const struct i2c_device_id xrm1280_i2c_id_table[] = {
	{ "xrm1280",	(kernel_ulong_t)&xrm1280_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, xrm1280_i2c_id_table);

static struct i2c_driver xrm1280_i2c_uart_driver = {
	.driver = {
		.name		= XRM1280_NAME,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)
		.owner		= THIS_MODULE,
#endif		
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)	
		.of_match_table	= xrm1280_dt_ids,
#else
		.of_match_table	= of_match_ptr(xrm1280_dt_ids),
#endif	
	},
	.probe		= xrm1280_i2c_probe,
	.remove		= xrm1280_i2c_remove,
	.id_table	= xrm1280_i2c_id_table,
};
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
module_i2c_driver(xrm1280_i2c_uart_driver);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
MODULE_ALIAS("i2c:xrm1280");
#endif
#endif	//CONFIG_SERIAL_XR20M1280_I2C

static int __init xrm1280_init(void)
{
	int ret;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)	
	ret = uart_register_driver(&xrm1280_uart);
	if (ret) {
		pr_err("Registering UART driver failed\n");
		return ret;
	}
#endif	

#ifdef CONFIG_SERIAL_XR20M1280_I2C
	ret = i2c_add_driver(&xrm1280_i2c_uart_driver);
	if (ret < 0) {
		pr_err("failed to init xrm1280 i2c --> %d\n", ret);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 12)	
		goto err_i2c;		
#else		
		return ret;
#endif	
	}
#endif

#ifdef CONFIG_SERIAL_XR20M1280_SPI
	ret = spi_register_driver(&xrm1280_spi_uart_driver);
	if (ret < 0) {
		pr_err("failed to init xrm1280 spi --> %d\n", ret);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 12)	
		goto err_spi;
#else			
		return ret;
#endif	
	}
#endif

	return ret;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 12)	
#ifdef CONFIG_SERIAL_XR20M1280_SPI
err_spi:
#endif
#ifdef CONFIG_SERIAL_XR20M1280_I2C
	i2c_del_driver(&xrm1280_i2c_uart_driver);
err_i2c:	
#endif
	uart_unregister_driver(&xrm1280_uart);
	return ret;
#endif 	//KERNEL >= 5.0.12	
}
module_init(xrm1280_init);

static void __exit xrm1280_exit(void)
{
#ifdef CONFIG_SERIAL_XR20M1280_I2C
	i2c_del_driver(&xrm1280_i2c_uart_driver);
#endif

#ifdef CONFIG_SERIAL_XR20M1280_SPI
	spi_unregister_driver(&xrm1280_spi_uart_driver);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)	
	uart_unregister_driver(&xrm1280_uart);
#endif	
}
module_exit(xrm1280_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ted Lin <tedlin@maxlinear.com>");
MODULE_DESCRIPTION("XR20M1280 SPI/I2C serial driver");
