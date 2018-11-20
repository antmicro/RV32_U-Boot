/*
 * LiteUART driver.
 *
 * stub based on: hifive
 *
 * Copyright (C) 2018 Microsemi Corporation
 *
 * Modified to support C structur SoC access by
 * Padmarao Begari <padmarao.begari@microsemi.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */
#include <common.h>
#include <dm.h>
#include <errno.h>
#include <watchdog.h>
#include <serial.h>
#include <linux/compiler.h>

#include <asm/io.h>
#ifdef CONFIG_DM_SERIAL
#include <asm/arch/msc_serial.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

/* HiFive unleashed UART register footprint */
typedef struct lite_uart {
    u32 rxtxdata;
    u32 txfull;
    u32 rxempty;
    u32 eventstatus;
    u32 eventpending;
    u32 eventenable;
} lite_uart_t;

/* Information about a serial port */
struct lite_serial_platdata {
	uint32_t base_addr;
};

/* If Driver Model for Serial is not defined */
#ifndef CONFIG_DM_SERIAL
/* Set HiFive UART baud rate */
static void lite_uart_setbrg(void)
{
#if 0
	lite_uart_t *usart = (lite_uart_t *)LITE_UART_BASE_ADDR;
    u32 baud_value;
   /*
    * BAUD_VALUE = (CLOCK / BAUD_RATE) - 1
    */
    baud_value = (LITE_PERIPH_CLK_FREQ / CONFIG_BAUDRATE) -1;

    writel(baud_value, &usart->div);
#endif
}

static int lite_uart_init(void)
{
	lite_uart_t *usart = (lite_uart_t *)LITE_UART_BASE_ADDR;
	//lite_uart_setbrg();
	writel(0, &usart->eventstatus);
	writel(0, &usart->eventpending);
	writel(0, &usart->eventenable);
    return 0;
}

static void lite_uart_putc(char ch)
{
	lite_uart_t *usart = (lite_uart_t *)LITE_UART_BASE_ADDR;

    if (ch == '\n')
       serial_putc('\r');

	while (readl(&usart->txfull));
	writel(ch, &usart->rxtxdata);
}

#define UART_EV_TX          (1 << 0)
#define UART_EV_RX          (1 << 1)

static int lite_uart_getc(void)
{

	lite_uart_t *usart = (lite_uart_t *)LITE_UART_BASE_ADDR;
	if (usart == NULL) return 0;
	while (!(readl(&usart->eventpending) & UART_EV_RX));
	while (readl(&usart->rxempty));
	int ch = readl(&usart->rxtxdata) & 0xFF;
	writel(UART_EV_RX, &usart->eventpending);
	writel(0, &usart->rxempty);
        return ch;
}

static int lite_uart_tstc(void)
{
        lite_uart_t *usart = (lite_uart_t *)LITE_UART_BASE_ADDR;
	if (usart == NULL) return 0;
        return !!(readl(&usart->eventpending) & UART_EV_RX);
}

static struct serial_device lite_uart_drv = {
    .name = "lite_uart",
    .start = lite_uart_init,
    .stop = NULL,
    .setbrg = lite_uart_setbrg,
    .putc = lite_uart_putc,
    .puts = default_serial_puts,
    .getc = lite_uart_getc,
    .tstc = lite_uart_tstc,
};

void lite_uart_initialize(void)
{
    serial_register(&lite_uart_drv);
}

struct serial_device *default_serial_console(void)
{
    return &lite_uart_drv;
}
#endif

#ifdef CONFIG_DM_SERIAL
#error dupa
enum serial_clk_type {
	CLK_TYPE_NORMAL = 0,
	CLK_TYPE_DBGU,
};

struct lite_serial_priv {
	lite_uart_t *usart;
	ulong usart_clk_rate;
};

static void _lite_serial_set_brg(lite_uart_t *usart,
				  ulong usart_clk_rate, int baudrate)
{
    u32 divisor;
   /*
    * BAUD_VALUE = (CLOCK / BAUD_RATE) - 1
    */
    divisor = (usart_clk_rate / baudrate) -1;

///    writel(divisor, &usart->div);

}

void _lite_serial_init(lite_uart_t *usart,
			ulong usart_clk_rate, int baudrate)
{

	lite_uart_setbrg(usart, usart_clk_rate, baudrate);
	writel(UART_TXEN, &usart->txctrl);
	writel(UART_RXEN, &usart->rxctrl);

}

int lite_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct lite_serial_priv *priv = dev_get_priv(dev);

	_lite_serial_set_brg(priv->usart, priv->usart_clk_rate, baudrate);

	return 0;
}

static int lite_serial_getc(struct udevice *dev)
{
	struct lite_serial_priv *priv = dev_get_priv(dev);

	int ch;
	ch = readl(&priv->usart->rxdata);

	while((ch & UART_RXFIFO_EMPTY) != 0)
	{
		ch = readl(&priv->usart->rxdata);
	}
	return ch;
}

static int lite_serial_putc(struct udevice *dev, const char ch)
{
	struct lite_serial_priv *priv = dev_get_priv(dev);

	while (readl(&priv->usart->txdata) & UART_TXFIFO_FULL);
	writel(ch, &priv->usart->txdata);

	return 0;
}

static int lite_serial_pending(struct udevice *dev, bool input)
{
	struct lite_serial_priv *priv = dev_get_priv(dev);
	uint32_t csr = readl(&priv->usart->csr);

	if (input)
		return readl(&priv->usart->rxdata) & UART_RXFIFO_EMPTY ? 0 : 1;
	else
		return readl(&priv->usart->txdata) & UART_TXFIFO_FULL ? 0 : 1;
}

static const struct dm_serial_ops atmel_serial_ops = {
	.putc = lite_serial_putc,
/*	.pending = lite_serial_pending,*/
	.getc = lite_serial_getc,
	.setbrg = lite_serial_setbrg
};
#if 0 //padma
static int atmel_serial_enable_clk(struct udevice *dev)
{
	struct atmel_serial_priv *priv = dev_get_priv(dev);
	struct clk clk;
	ulong clk_rate;
	int ret;

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret)
		return -EINVAL;

	if (dev_get_driver_data(dev) == CLK_TYPE_NORMAL) {
		ret = clk_enable(&clk);
		if (ret)
			return ret;
	}

	clk_rate = clk_get_rate(&clk);
	if (!clk_rate)
		return -EINVAL;

	priv->usart_clk_rate = clk_rate;

	clk_free(&clk);

	return 0;
}
#endif
static int lite_serial_probe(struct udevice *dev)
{
	struct lite_serial_platdata *plat = dev->platdata;
	struct lite_serial_priv *priv = dev_get_priv(dev);
	int ret;
#if CONFIG_IS_ENABLED(OF_CONTROL)
	fdt_addr_t addr_base;

	addr_base = devfdt_get_addr(dev);
	if (addr_base == FDT_ADDR_T_NONE)
		return -ENODEV;

	plat->base_addr = (uint32_t)addr_base;
#endif
	priv->usart = (lite_uart_t *)plat->base_addr;

/*	ret = atmel_serial_enable_clk(dev);
	if (ret)
		return ret;
*/
	_lite_serial_init(priv->usart, LITE_PERIPH_CLK_FREQ, gd->baudrate);

	return 0;
}

#if CONFIG_IS_ENABLED(OF_CONTROL)
static const struct udevice_id lite_serial_ids[] = {
	{
		.compatible = "liteuart"
	},
	{ }
};
#endif

U_BOOT_DRIVER(serial_lite) = {
	.name	= "serial_lite",
	.id	= UCLASS_SERIAL,
#if CONFIG_IS_ENABLED(OF_CONTROL)
	.of_match = lite_serial_ids,
	.platdata_auto_alloc_size = sizeof(struct lite_serial_platdata),
#endif
	.probe = lite_serial_probe,
	.ops	= &lite_serial_ops,
	.flags = DM_FLAG_PRE_RELOC,
	.priv_auto_alloc_size	= sizeof(struct lite_serial_priv),
};
#endif

