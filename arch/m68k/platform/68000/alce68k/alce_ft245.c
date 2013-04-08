/* alce_ft245.c: Serial port driver for the
 * FTDI245 module in Alcetronics M68K board.
 *
 * 2012.04.17, ljalvs@gmail.com, Initial version for FTDI245 module
 *                               at Alcetronics M68K board.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/io.h>

#include <asm/alce68k.h>



#define DRV_NAME "ft245uart"
#define FT245_SERIAL_DEV_NAME "ttyS"
/*#define FT245_SERIAL_MAJOR 4*/
#define FT245_SERIAL_MINOR 64

#define PORT_FT245     2000

struct ft245_uart_port {
	struct uart_port	port;
};

static struct ft245_uart_port ft245_uart_ports[1];
#define	FT245_MAXPORTS	ARRAY_SIZE(ft245_uart_ports)


#if defined(CONFIG_SERIAL_ALCE_CONSOLE)
static struct console ft245_console;
#define	FT245_CONSOLE	(&ft245_console)
#else
#define	FT245_CONSOLE	NULL
#endif


/* UART driver structure. */
static struct uart_driver ft245_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRV_NAME,
	.dev_name	= FT245_SERIAL_DEV_NAME,
	.major		= TTY_MAJOR,
	.minor		= FT245_SERIAL_MINOR,
	.nr		= FT245_MAXPORTS,
	.cons		= FT245_CONSOLE,
};



static void ft245_stop_rx(struct uart_port *port)
{
	INT_CTRL &= ~FTDI_RXIE;
}


static void ft245_stop_tx(struct uart_port *port)
{
	INT_CTRL &= ~FTDI_TXIE;
}

static unsigned int ft245_tx_empty(struct uart_port *port)
{
	return (FTDI_STAT & FTDI_TXE) ? 0 : TIOCSER_TEMT;
}


static void ft245_tx_chars(struct ft245_uart_port *pp)
{
	struct uart_port *port = &pp->port;
	struct circ_buf *xmit = &port->state->xmit;


	if (port->x_char) {
		/* Send special char - probably flow control */
		FTDI_DATA = port->x_char;
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		INT_CTRL &= ~FTDI_TXIE;
		return;
	}

	while (!(FTDI_STAT & FTDI_TXE)) {
		FTDI_DATA = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE -1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		INT_CTRL &= ~FTDI_TXIE;

}


static void ft245_start_tx(struct uart_port *port)
{
	INT_CTRL |= FTDI_TXIE;
}


static void ft245_rx_chars(struct ft245_uart_port *pp)
{
	struct uart_port *port = &pp->port;
	unsigned char ch, flag;
	
	while (!(FTDI_STAT & FTDI_RXF)) {
		ch = FTDI_DATA;
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (uart_handle_sysrq_char(port, ch))
			continue;
		//uart_insert_char(&port, status, MCFUART_USR_RXOVERRUN, ch, flag);
		tty_insert_flip_char(&port->state->port, ch, flag);
		//uart_insert_char(&port, status, 0, ch, flag);
	}

	tty_flip_buffer_push(&port->state->port);
}



static irqreturn_t ft245_interrupt(int irq, void *data)
{
	struct uart_port *port = data;
	struct ft245_uart_port *pp = container_of(port, struct ft245_uart_port, port);
	irqreturn_t ret = IRQ_NONE;
	
	unsigned long flags;


//	spin_lock_irqsave(&port->lock, flags);
	
	if (!(FTDI_STAT & FTDI_RXF)) {
		ft245_rx_chars(pp);
		ret = IRQ_HANDLED;
	} else 

	if (!(FTDI_STAT & FTDI_TXE)) {
		ft245_tx_chars(pp);
		ret = IRQ_HANDLED;
	}
//	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}


static unsigned int ft245_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR; 
}

static void ft245_set_mctrl(struct uart_port *port, unsigned int sigs)
{
}

static void ft245_enable_ms(struct uart_port *port)
{
}

static void ft245_break_ctl(struct uart_port *port, int break_state)
{
}

static int ft245_startup(struct uart_port *port)
{
	//struct ft245_uart_port *pp = container_of(port, struct ft245_uart_port, port);
	unsigned long flags;

	
	//local_irq_save(flags);
	spin_lock_irqsave(&port->lock, flags);
	//spin_lock(&port->lock);

	INT_CTRL &= ~(FTDI_IEN | FTDI_RXIE | FTDI_TXIE);

	if (request_irq(port->irq, ft245_interrupt, 0, "FT245UART", port))
		printk(KERN_ERR "FT245UART: unable to attach FT245UART %d "
			"interrupt vector=%d\n", port->line, port->irq);

	/* Enable RX interrupts now */
	INT_CTRL |= FTDI_IEN | FTDI_RXIE;

	spin_unlock_irqrestore(&port->lock, flags);
	//spin_unlock(&port->lock);
	//local_irq_restore(flags);

	return 0;
}

static void ft245_shutdown(struct uart_port *port)
{
	//struct ft245_uart_port *uart = (struct arc_uart_port *)port;
	unsigned long flags;

 	spin_lock_irqsave(&port->lock, flags);

	/* Disable all interrupts now */
	INT_CTRL &= ~(FTDI_IEN | FTDI_RXIE | FTDI_TXIE);
	free_irq(port->irq, port);
	
 	spin_unlock_irqrestore(&port->lock, flags);
}


static void ft245_set_termios(struct uart_port *port, struct ktermios *termios,
	struct ktermios *old)
{
}


static const char *ft245_type(struct uart_port *port)
{
	return (port->type == PORT_FT245) ? "FTDI 245 serial usb module" : NULL;
}

static void ft245_release_port(struct uart_port *port)
{
	/* Nothing to release... */
}

static int ft245_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_FT245))
		return -EINVAL;
	return 0;
}

static int ft245_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

static void ft245_config_port(struct uart_port *port, int flags)
{
	port->type = PORT_FT245;
	port->fifosize = FTDI_TXFIFOSIZE;

	/* Clear mask, so no surprise interrupts. */
	INT_CTRL &= ~(FTDI_IEN | FTDI_RXIE | FTDI_TXIE);
}



#ifdef CONFIG_CONSOLE_POLL

static void ft245_poll_putchar(struct uart_port *port, unsigned char chr)
{
	//struct arc_uart_port *uart = (struct arc_uart_port *)port;

	/*while (!(UART_GET_STATUS(uart) & TXEMPTY))
		cpu_relax();*/

	FTDI_DATA = chr;
}

static int ft245_poll_getchar(struct uart_port *port)
{
	//struct arc_uart_port *uart = (struct arc_uart_port *)port;
	unsigned char chr;

	while ((FTDI_STAT & FTDI_RXF))
		cpu_relax();

	chr = FTDI_DATA;
	return chr;
}
#endif


/*
 *	Define the basic serial functions we support.
 */
static const struct uart_ops ft245_uart_ops = {
	.tx_empty	= ft245_tx_empty,
	.get_mctrl	= ft245_get_mctrl,
	.set_mctrl	= ft245_set_mctrl,
	.start_tx	= ft245_start_tx,
	.stop_tx		= ft245_stop_tx,
	.stop_rx		= ft245_stop_rx,
	.enable_ms	= ft245_enable_ms,
	.break_ctl	= ft245_break_ctl,
	.startup		= ft245_startup,
	.shutdown	= ft245_shutdown,
	.set_termios	= ft245_set_termios,
	.type		= ft245_type,
	.release_port	= ft245_release_port,
	.request_port	= ft245_request_port,
	.config_port	= ft245_config_port,
	.verify_port	= ft245_verify_port,
	
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char 	= ft245_poll_putchar,
	.poll_get_char 	= ft245_poll_getchar,
#endif
	
};



#if defined(CONFIG_SERIAL_ALCE_CONSOLE)

static int ft245_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';


	if ((co->index < 0) || (co->index >= FT245_MAXPORTS))
		co->index = 0;
	port = &ft245_uart_ports[co->index].port;
	if (port->membase == 0)
		return -ENODEV;
	//co->index = 0;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}
/*
static void ft245_console_putc(struct console *co, const char c)
{
	
	FTDI_DATA = c;
	
}
*/

static void ft245_console_write(struct console *co, const char *s, unsigned int count)
{
	for (; (count); count--, s++) {
		FTDI_DATA = *s; //ft245_console_putc(co, *s);
		if (*s == '\n')
			FTDI_DATA = '\r'; //ft245_console_putc(co, '\r');
	}
}

static struct console ft245_console = {
	.name		= FT245_SERIAL_DEV_NAME,
	.write		= ft245_console_write,
	.device		= uart_console_device,
	.setup		= ft245_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &ft245_driver,
};


static __init void early_serial_write(struct console *con, const char *s, unsigned int n)
{
	//struct uart_port *port = &arc_uart_ports[con->index].port;
	unsigned int i;

	for (i = 0; i < n; i++, s++) {
		FTDI_DATA = *s;
		if (*s == '\n')
			FTDI_DATA = '\r';
	}
}

static struct __initdata console ft245_early_serial_console = {
	.name = "early_FT245uart",
	.write = early_serial_write,
	.flags = CON_PRINTBUFFER | CON_BOOT,
	.index = -1
};

static int ft245_probe_earlyprintk(struct platform_device *pdev)
{
	ft245_early_serial_console.index = pdev->id;
	register_console(&ft245_early_serial_console);
	return 0;
}


#endif




static int ft245_probe(struct platform_device *pdev)
{
	struct alce_platform_uart *platp = pdev->dev.platform_data;
	struct uart_port *port;
	int i;

	
	if (is_early_platform_device(pdev))
		return ft245_probe_earlyprintk(pdev);	
	
	
	for (i = 0; ((i < FT245_MAXPORTS) && (platp[i].mapbase)); i++) {
		port = &ft245_uart_ports[i].port;

		port->line = i;
		port->type = PORT_FT245;
		port->mapbase = platp[i].mapbase;
		port->membase = (platp[i].membase) ? platp[i].membase :
			(unsigned char __iomem *) platp[i].mapbase;
		port->iotype = SERIAL_IO_MEM;
		port->irq = platp[i].irq;
		port->uartclk = 12000000;
		port->ops = &ft245_uart_ops;
		port->flags = ASYNC_BOOT_AUTOCONF;

		uart_add_one_port(&ft245_driver, port);
	}

	return 0;
}

static int ft245_remove(struct platform_device *pdev)
{
	/*struct uart_port *port;
	int i;

	for (i = 0; (i < FT245_MAXPORTS); i++) {
		port = &ft245_uart_ports[i].port;
		if (port)
			uart_remove_one_port(&ft245_driver, port);
	}
	*/
	return 0;
}


static struct platform_driver ft245_platform_driver = {
	.probe		= ft245_probe,
	.remove		= ft245_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};


#if defined(CONFIG_SERIAL_ALCE_CONSOLE)
early_platform_init("earlyprintk", &ft245_platform_driver);
#endif


static int __init ft245_init(void)
{
	int rc;

	printk("FTDI 245 USB module serial driver\n");

	rc = uart_register_driver(&ft245_driver);
	if (rc)
		return rc;
	rc = platform_driver_register(&ft245_platform_driver);
	if (rc)
		return rc;
	return 0;
}

static void __exit ft245_exit(void)
{
	platform_driver_unregister(&ft245_platform_driver);
	uart_unregister_driver(&ft245_driver);
}


module_init(ft245_init);
module_exit(ft245_exit);

MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_DESCRIPTION("FTDI 245 USB module serial driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ft245uart");

/****************************************************************************/

