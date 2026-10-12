// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx USART driver
 *
 * One-port polling/IRQ driven serial driver for the N32H7xx USART block.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>


#define N32H7XX_USART_CTRL1	0x00
#define N32H7XX_USART_CTRL2	0x04
#define N32H7XX_USART_CTRL3	0x08
#define N32H7XX_USART_STS	0x0c
#define N32H7XX_USART_DAT	0x10
#define N32H7XX_USART_BRCF	0x14
#define N32H7XX_USART_FIFO	0x1c

#define N32H7XX_USART_CTRL1_UEN		BIT(0)
#define N32H7XX_USART_CTRL1_RXEN	BIT(1)
#define N32H7XX_USART_CTRL1_TXEN	BIT(2)
#define N32H7XX_USART_CTRL1_PSEL	BIT(3)
#define N32H7XX_USART_CTRL1_PCEN	BIT(4)
#define N32H7XX_USART_CTRL1_WL		BIT(5)
#define N32H7XX_USART_CTRL1_RXDNEIEN	BIT(9)
#define N32H7XX_USART_CTRL1_TXDEIEN	BIT(10)
#define N32H7XX_USART_CTRL1_OSPM		BIT(26)

#define N32H7XX_USART_FIFO_EN		BIT(0)
#define N32H7XX_USART_FIFO_CLR		BIT(1)
#define N32H7XX_USART_FIFO_RXFTIEN	BIT(12)
#define N32H7XX_USART_FIFO_RXCNT	GENMASK(17, 14)

#define N32H7XX_USART_CTRL2_STPB	GENMASK(6, 5)

#define N32H7XX_USART_CTRL3_CTSEN	BIT(0)
#define N32H7XX_USART_CTRL3_RTSEN	BIT(2)
#define N32H7XX_USART_CTRL3_ERRIEN	BIT(6)

#define N32H7XX_USART_STS_TXDE		BIT(7)
#define N32H7XX_USART_STS_TXC		BIT(8)
#define N32H7XX_USART_STS_RXDNE		BIT(9)
#define N32H7XX_USART_STS_RXFT		BIT(4)
#define N32H7XX_USART_STS_PEF		BIT(12)
#define N32H7XX_USART_STS_OREF		BIT(13)
#define N32H7XX_USART_STS_NEF		BIT(14)
#define N32H7XX_USART_STS_FEF		BIT(15)

/* Latched receive errors.  They are cleared by reading STS, then DAT. */
#define N32H7XX_USART_STS_ERR		(N32H7XX_USART_STS_OREF | \
					 N32H7XX_USART_STS_PEF | \
					 N32H7XX_USART_STS_NEF | \
					 N32H7XX_USART_STS_FEF)

/*
 * Hardware FIFO depth.  The
 * receive threshold is programmed to one byte in startup() so a single
 * keystroke raises RXFT immediately.
 */
#define N32H7XX_USART_RX_FIFO_SIZE	8

#define N32H7XX_UART_NAME	"ttyS"

struct n32h7xx_uart_port {
	struct uart_port port;
	struct clk *clk;
};

static struct uart_driver n32h7xx_uart_driver;

static void n32h7xx_usart_putc(struct uart_port *port, unsigned char ch)
{
	while (!(readl_relaxed(port->membase + N32H7XX_USART_STS) &
		 N32H7XX_USART_STS_TXDE))
		cpu_relax();

	writel_relaxed(ch, port->membase + N32H7XX_USART_DAT);
}

static unsigned int n32h7xx_usart_tx_empty(struct uart_port *port)
{
	return (readl_relaxed(port->membase + N32H7XX_USART_STS) &
		N32H7XX_USART_STS_TXC) ? TIOCSER_TEMT : 0;
}

static void n32h7xx_usart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int n32h7xx_usart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

static void n32h7xx_usart_stop_tx(struct uart_port *port)
{
	u32 ctrl1 = readl_relaxed(port->membase + N32H7XX_USART_CTRL1);

	ctrl1 &= ~N32H7XX_USART_CTRL1_TXDEIEN;
	writel_relaxed(ctrl1, port->membase + N32H7XX_USART_CTRL1);
}

static void n32h7xx_usart_tx_chars(struct uart_port *port)
{
	unsigned char ch;

	uart_port_tx(port, ch,
		     (readl_relaxed(port->membase + N32H7XX_USART_STS) &
		      N32H7XX_USART_STS_TXDE),
		     n32h7xx_usart_putc(port, ch));
}

static void n32h7xx_usart_start_tx(struct uart_port *port)
{
	u32 ctrl1 = readl_relaxed(port->membase + N32H7XX_USART_CTRL1);

	ctrl1 |= N32H7XX_USART_CTRL1_TXDEIEN;
	writel_relaxed(ctrl1, port->membase + N32H7XX_USART_CTRL1);
	n32h7xx_usart_tx_chars(port);
}

static void n32h7xx_usart_stop_rx(struct uart_port *port)
{
	u32 ctrl1 = readl_relaxed(port->membase + N32H7XX_USART_CTRL1);
	u32 ctrl3 = readl_relaxed(port->membase + N32H7XX_USART_CTRL3);
	u32 fifo = readl_relaxed(port->membase + N32H7XX_USART_FIFO);

	ctrl1 &= ~(N32H7XX_USART_CTRL1_RXDNEIEN | N32H7XX_USART_CTRL1_RXEN);
	ctrl3 &= ~N32H7XX_USART_CTRL3_ERRIEN;
	fifo &= ~N32H7XX_USART_FIFO_RXFTIEN;
	writel_relaxed(ctrl1, port->membase + N32H7XX_USART_CTRL1);
	writel_relaxed(ctrl3, port->membase + N32H7XX_USART_CTRL3);
	writel_relaxed(fifo, port->membase + N32H7XX_USART_FIFO);
}

static void n32h7xx_usart_break_ctl(struct uart_port *port, int ctl)
{
}

static void n32h7xx_usart_rx_chars(struct uart_port *port)
{
	unsigned int budget = 256;

	/* RXDNE describes DAT; RXCNT also covers bytes queued in the FIFO.
	 * Re-read both after every byte, including bytes arriving during drain.
	 */
	while (budget--) {
		u32 status = readl_relaxed(port->membase + N32H7XX_USART_STS);
		u32 fifo = readl_relaxed(port->membase + N32H7XX_USART_FIFO);
		bool available = (status & N32H7XX_USART_STS_RXDNE) ||
			((fifo & N32H7XX_USART_FIFO_EN) &&
			 (fifo & N32H7XX_USART_FIFO_RXCNT));
		unsigned int ch;
		char flag = TTY_NORMAL;

		if (!available && !(status & N32H7XX_USART_STS_ERR))
			break;

		/* STS followed by DAT clears receive errors, including an overrun
		 * without RXDNE. Do not insert a stale DAT value in that case.
		 */
		ch = readl_relaxed(port->membase + N32H7XX_USART_DAT) & 0xff;
		if (status & N32H7XX_USART_STS_OREF)
			port->icount.overrun++;
		if (status & N32H7XX_USART_STS_FEF)
			port->icount.frame++;
		if (status & N32H7XX_USART_STS_PEF)
			port->icount.parity++;

		if (!available)
			continue;

		port->icount.rx++;
		status &= port->read_status_mask;
		if (status & N32H7XX_USART_STS_PEF)
			flag = TTY_PARITY;
		else if (status & (N32H7XX_USART_STS_FEF | N32H7XX_USART_STS_NEF))
			flag = TTY_FRAME;

		if (uart_handle_sysrq_char(port, ch))
			continue;
		uart_insert_char(port, status, N32H7XX_USART_STS_OREF, ch, flag);
	}

	tty_flip_buffer_push(&port->state->port);
}

/* Console output holds the port lock with local IRQs disabled.  At
 * 115200 baud even a short printk exceeds the eight-byte RX FIFO capacity.
 * Service reception while polling TX, using the same lock as the IRQ path.
 * Earlycon and an unlocked oops path keep the write-only callback.
 */
static void n32h7xx_usart_console_rx(struct uart_port *port)
{
	u32 status = readl_relaxed(port->membase + N32H7XX_USART_STS);

	if (status & (N32H7XX_USART_STS_RXDNE | N32H7XX_USART_STS_RXFT |
		      N32H7XX_USART_STS_ERR))
		n32h7xx_usart_rx_chars(port);
}

static void n32h7xx_usart_console_putc(struct uart_port *port, unsigned char ch)
{
	do {
		n32h7xx_usart_console_rx(port);
	} while (!(readl_relaxed(port->membase + N32H7XX_USART_STS) &
		   N32H7XX_USART_STS_TXDE));

	writel_relaxed(ch, port->membase + N32H7XX_USART_DAT);
}

static irqreturn_t n32h7xx_usart_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	u32 status, ctrl1, ctrl3, fifo;
	bool handled = false;

	uart_port_lock(port);
	status = readl_relaxed(port->membase + N32H7XX_USART_STS);
	ctrl1 = readl_relaxed(port->membase + N32H7XX_USART_CTRL1);
	ctrl3 = readl_relaxed(port->membase + N32H7XX_USART_CTRL3);
	fifo = readl_relaxed(port->membase + N32H7XX_USART_FIFO);

	if (((ctrl1 & N32H7XX_USART_CTRL1_RXDNEIEN) &&
	     (status & N32H7XX_USART_STS_RXDNE)) ||
	    ((fifo & N32H7XX_USART_FIFO_RXFTIEN) &&
	     (status & N32H7XX_USART_STS_RXFT)) ||
	    ((ctrl3 & N32H7XX_USART_CTRL3_ERRIEN) &&
	     (status & N32H7XX_USART_STS_ERR))) {
		n32h7xx_usart_rx_chars(port);
		handled = true;
	}

	if ((ctrl1 & N32H7XX_USART_CTRL1_TXDEIEN) &&
	    (status & N32H7XX_USART_STS_TXDE)) {
		n32h7xx_usart_tx_chars(port);
		handled = true;
	}

	uart_port_unlock(port);
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int n32h7xx_usart_startup(struct uart_port *port)
{
	u32 ctrl1;
	int ret;

	ret = request_irq(port->irq, n32h7xx_usart_irq, 0, port->name, port);
	if (ret)
		return ret;

	/*
	 * Quiesce reception and clear stale boot-time data and errors before
	 * arming the receive interrupts.
	 */
	ctrl1 = readl_relaxed(port->membase + N32H7XX_USART_CTRL1);
	ctrl1 &= ~N32H7XX_USART_CTRL1_RXEN;
	writel_relaxed(ctrl1, port->membase + N32H7XX_USART_CTRL1);

	/*
	 * Enable the eight-byte FIFO with a one-byte receive threshold so a
	 * single keystroke is delivered immediately.
	 */
	writel_relaxed(N32H7XX_USART_FIFO_CLR,
		       port->membase + N32H7XX_USART_FIFO);
	writel_relaxed(N32H7XX_USART_FIFO_EN | N32H7XX_USART_FIFO_RXFTIEN,
		       port->membase + N32H7XX_USART_FIFO);
	(void)readl_relaxed(port->membase + N32H7XX_USART_STS);
	(void)readl_relaxed(port->membase + N32H7XX_USART_DAT);

	ctrl1 |= N32H7XX_USART_CTRL1_UEN |
		 N32H7XX_USART_CTRL1_RXEN |
		 N32H7XX_USART_CTRL1_TXEN |
		 N32H7XX_USART_CTRL1_RXDNEIEN;
	ctrl1 &= ~N32H7XX_USART_CTRL1_TXDEIEN;
	writel_relaxed(ctrl1, port->membase + N32H7XX_USART_CTRL1);
	writel_relaxed(readl_relaxed(port->membase + N32H7XX_USART_CTRL3) |
		       N32H7XX_USART_CTRL3_ERRIEN,
		       port->membase + N32H7XX_USART_CTRL3);

	return 0;
}

static void n32h7xx_usart_shutdown(struct uart_port *port)
{
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	n32h7xx_usart_stop_rx(port);
	n32h7xx_usart_stop_tx(port);
	uart_port_unlock_irqrestore(port, flags);
	free_irq(port->irq, port);
}

static void n32h7xx_usart_set_termios(struct uart_port *port,
				      struct ktermios *termios,
				      const struct ktermios *old)
{
	unsigned int baud;
	u32 divider;
	u32 ctrl1;
	u32 ctrl2;
	u32 ctrl3;
	unsigned long flags;

	termios->c_cflag &= ~(CRTSCTS | CMSPAR);
	termios->c_cflag &= ~CSIZE;
	termios->c_cflag |= CS8;
	termios->c_cflag &= ~PARENB;
	termios->c_cflag &= ~CSTOPB;

	baud = uart_get_baud_rate(port, termios, old, 300, port->uartclk / 16);
	divider = DIV_ROUND_CLOSEST(port->uartclk, baud);

	port->read_status_mask = N32H7XX_USART_STS_RXDNE | N32H7XX_USART_STS_OREF;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= N32H7XX_USART_STS_PEF |
					  N32H7XX_USART_STS_FEF | N32H7XX_USART_STS_NEF;
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= N32H7XX_USART_STS_PEF |
					    N32H7XX_USART_STS_FEF | N32H7XX_USART_STS_NEF;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= N32H7XX_USART_STS_RXDNE;

	uart_port_lock_irqsave(port, &flags);

	ctrl1 = readl_relaxed(port->membase + N32H7XX_USART_CTRL1);
	ctrl1 &= ~(N32H7XX_USART_CTRL1_WL |
		   N32H7XX_USART_CTRL1_PCEN |
		   N32H7XX_USART_CTRL1_PSEL |
		   N32H7XX_USART_CTRL1_OSPM);
	ctrl1 |= N32H7XX_USART_CTRL1_UEN |
		 N32H7XX_USART_CTRL1_RXEN |
		 N32H7XX_USART_CTRL1_TXEN;
	writel_relaxed(ctrl1, port->membase + N32H7XX_USART_CTRL1);

	ctrl2 = readl_relaxed(port->membase + N32H7XX_USART_CTRL2);
	ctrl2 &= ~N32H7XX_USART_CTRL2_STPB;
	writel_relaxed(ctrl2, port->membase + N32H7XX_USART_CTRL2);

	ctrl3 = readl_relaxed(port->membase + N32H7XX_USART_CTRL3);
	ctrl3 &= ~(N32H7XX_USART_CTRL3_CTSEN |
		   N32H7XX_USART_CTRL3_RTSEN);
	writel_relaxed(ctrl3, port->membase + N32H7XX_USART_CTRL3);

	/* BRCF uses one integer bitfield and four fractional bits. */
	writel_relaxed(((divider / 16) << 4) | (divider % 16),
		       port->membase + N32H7XX_USART_BRCF);

	uart_update_timeout(port, termios->c_cflag, baud);
	uart_port_unlock_irqrestore(port, flags);

	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}

static const char *n32h7xx_usart_type(struct uart_port *port)
{
	return port->type == PORT_16550A ? "n32h7xx-usart" : NULL;
}

static void n32h7xx_usart_release_port(struct uart_port *port)
{
}

static int n32h7xx_usart_request_port(struct uart_port *port)
{
	return 0;
}

static void n32h7xx_usart_config_port(struct uart_port *port, int type)
{
	if (type & UART_CONFIG_TYPE && !n32h7xx_usart_request_port(port))
		port->type = PORT_16550A;
}

static int n32h7xx_usart_verify_port(struct uart_port *port,
				     struct serial_struct *serinfo)
{
	return -EINVAL;
}

static const struct uart_ops n32h7xx_usart_ops = {
	.tx_empty	= n32h7xx_usart_tx_empty,
	.set_mctrl	= n32h7xx_usart_set_mctrl,
	.get_mctrl	= n32h7xx_usart_get_mctrl,
	.stop_tx	= n32h7xx_usart_stop_tx,
	.start_tx	= n32h7xx_usart_start_tx,
	.stop_rx	= n32h7xx_usart_stop_rx,
	.break_ctl	= n32h7xx_usart_break_ctl,
	.startup	= n32h7xx_usart_startup,
	.shutdown	= n32h7xx_usart_shutdown,
	.set_termios	= n32h7xx_usart_set_termios,
	.type		= n32h7xx_usart_type,
	.release_port	= n32h7xx_usart_release_port,
	.request_port	= n32h7xx_usart_request_port,
	.config_port	= n32h7xx_usart_config_port,
	.verify_port	= n32h7xx_usart_verify_port,
};

static void n32h7xx_usart_early_write(struct console *con, const char *s,
				      unsigned int count)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, count, n32h7xx_usart_putc);
}

static int __init n32h7xx_usart_early_setup(struct earlycon_device *device,
					    const char *options)
{
	u32 baud = device->baud ? device->baud : 115200;
	u32 divider;

	if (!device->port.membase || !device->port.uartclk)
		return -ENODEV;

	divider = DIV_ROUND_CLOSEST(device->port.uartclk, baud);

	writel(0, device->port.membase + N32H7XX_USART_CTRL1);
	writel(0, device->port.membase + N32H7XX_USART_CTRL2);
	writel(0, device->port.membase + N32H7XX_USART_CTRL3);
	writel(((divider / 16) << 4) | (divider % 16),
	       device->port.membase + N32H7XX_USART_BRCF);
	/* Write-only console: leave RX disabled so a byte arriving before the
	 * tty driver opens cannot latch an unserviced overrun.
	 */
	writel(N32H7XX_USART_CTRL1_UEN | N32H7XX_USART_CTRL1_TXEN,
	       device->port.membase + N32H7XX_USART_CTRL1);

	device->con->write = n32h7xx_usart_early_write;
	return 0;
}

OF_EARLYCON_DECLARE(n32h7xx_usart, "nsing,n32h7xx-usart",
		    n32h7xx_usart_early_setup);
EARLYCON_DECLARE(n32h7xx_usart, n32h7xx_usart_early_setup);

static void n32h7xx_usart_console_write(struct console *co, const char *s,
					unsigned int count)
{
	struct uart_port *port = n32h7xx_uart_driver.state[co->index].uart_port;
	unsigned long flags;
	bool locked = true;
	u32 ctrl1;

	if (oops_in_progress)
		locked = uart_port_trylock_irqsave(port, &flags);
	else
		uart_port_lock_irqsave(port, &flags);

	ctrl1 = readl_relaxed(port->membase + N32H7XX_USART_CTRL1);
	writel_relaxed(ctrl1 & ~N32H7XX_USART_CTRL1_TXDEIEN,
		       port->membase + N32H7XX_USART_CTRL1);
	uart_console_write(port, s, count,
			   locked && port->state ? n32h7xx_usart_console_putc :
						   n32h7xx_usart_putc);
	while (!(readl_relaxed(port->membase + N32H7XX_USART_STS) &
		 N32H7XX_USART_STS_TXC)) {
		if (locked && port->state)
			n32h7xx_usart_console_rx(port);
		cpu_relax();
	}
	writel_relaxed(ctrl1, port->membase + N32H7XX_USART_CTRL1);

	if (locked)
		uart_port_unlock_irqrestore(port, flags);
}

static int __init n32h7xx_usart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= n32h7xx_uart_driver.nr)
		return -ENODEV;

	port = n32h7xx_uart_driver.state[co->index].uart_port;
	if (!port)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console n32h7xx_usart_console = {
	.name		= N32H7XX_UART_NAME,
	.device		= uart_console_device,
	.write		= n32h7xx_usart_console_write,
	.setup		= n32h7xx_usart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &n32h7xx_uart_driver,
};

static struct uart_driver n32h7xx_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "n32h7xx-usart",
	.dev_name	= N32H7XX_UART_NAME,
	/*
	 * Claim the conventional ttyS0 device number.  With major/minor left
	 * at zero the TTY layer hands out a dynamically allocated major, so
	 * the static /dev/ttyS0 node in the initramfs (character 4:64) points
	 * at nothing and open() fails with ENXIO until devtmpfs is mounted.
	 * Being the only ttyS provider here, taking the standard range makes
	 * the node agree with devtmpfs, the initramfs and every userspace
	 * assumption about ttyS0.
	 */
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= 1,
	.cons		= &n32h7xx_usart_console,
};

static int n32h7xx_usart_probe(struct platform_device *pdev)
{
	struct n32h7xx_uart_port *n32_port;
	struct resource *res;
	int ret;

	n32_port = devm_kzalloc(&pdev->dev, sizeof(*n32_port), GFP_KERNEL);
	if (!n32_port)
		return -ENOMEM;

	n32_port->port.membase = devm_platform_get_and_ioremap_resource(pdev, 0,
									&res);
	if (IS_ERR(n32_port->port.membase))
		return PTR_ERR(n32_port->port.membase);

	n32_port->port.mapbase = res->start;
	n32_port->port.mapsize = resource_size(res);
	n32_port->port.iotype = UPIO_MEM32;
	n32_port->port.irq = platform_get_irq(pdev, 0);
	if (n32_port->port.irq < 0)
		return n32_port->port.irq;

	n32_port->clk = devm_clk_get(&pdev->dev, "usart");
	if (IS_ERR(n32_port->clk))
		return PTR_ERR(n32_port->clk);

	ret = clk_prepare_enable(n32_port->clk);
	if (ret)
		return ret;

	n32_port->port.uartclk = clk_get_rate(n32_port->clk);
	n32_port->port.fifosize = N32H7XX_USART_RX_FIFO_SIZE;

	/*
	 * The pins are a board property, so they come from the device tree
	 * pin group named by "pinctrl-0"; the pinctrl core applies it before
	 * this probe runs.
	 */
	n32_port->port.flags = UPF_BOOT_AUTOCONF;
	n32_port->port.type = PORT_16550A;
	n32_port->port.ops = &n32h7xx_usart_ops;
	n32_port->port.dev = &pdev->dev;
	n32_port->port.name = dev_name(&pdev->dev);
	n32_port->port.line = 0;

	ret = uart_add_one_port(&n32h7xx_uart_driver, &n32_port->port);
	if (ret) {
		clk_disable_unprepare(n32_port->clk);
		return ret;
	}

	platform_set_drvdata(pdev, n32_port);
	return 0;
}

static void n32h7xx_usart_remove(struct platform_device *pdev)
{
	struct n32h7xx_uart_port *n32_port = platform_get_drvdata(pdev);

	uart_remove_one_port(&n32h7xx_uart_driver, &n32_port->port);
	clk_disable_unprepare(n32_port->clk);
}

static const struct of_device_id n32h7xx_usart_of_match[] = {
	{ .compatible = "nsing,n32h7xx-usart" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_usart_of_match);

static struct platform_driver n32h7xx_usart_driver = {
	.probe = n32h7xx_usart_probe,
	.remove = n32h7xx_usart_remove,
	.driver = {
		.name = "n32h7xx-usart",
		.of_match_table = n32h7xx_usart_of_match,
	},
};

static int __init n32h7xx_usart_init(void)
{
	int ret;

	ret = uart_register_driver(&n32h7xx_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&n32h7xx_usart_driver);
	if (ret)
		uart_unregister_driver(&n32h7xx_uart_driver);

	return ret;
}
device_initcall(n32h7xx_usart_init);

static void __exit n32h7xx_usart_exit(void)
{
	platform_driver_unregister(&n32h7xx_usart_driver);
	uart_unregister_driver(&n32h7xx_uart_driver);
}
module_exit(n32h7xx_usart_exit);

MODULE_AUTHOR("NSING");
MODULE_DESCRIPTION("NSING N32H7xx USART driver");
MODULE_LICENSE("GPL");
