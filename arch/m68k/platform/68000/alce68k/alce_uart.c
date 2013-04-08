/* alce_uart.c: Serial port driver for the
 * FPGA uart module in Alcetronics M68K board.
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



#define DRV_NAME "fpgauart"
#define FPGA_SERIAL_DEV_NAME "ttyU"
#define FPGA_SERIAL_MAJOR 200
#define FPGA_SERIAL_MINOR 0

#define PORT_M68K_FPGAUART     2001

struct fpga_uart_port {
	struct uart_port	port;
};

static struct fpga_uart_port fpga_uart_ports[1];
#define	FPGAUART_MAXPORTS	ARRAY_SIZE(fpga_uart_ports)


#if defined(CONFIG_ALCE_FPGAUART_CONSOLE)
static struct console fpga_uart_console;
#define	FPGAUART_CONSOLE	(&fpga_uart_console)
#else
#define	FPGAUART_CONSOLE	NULL
#endif


/* UART driver structure. */
static struct uart_driver fpgauart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRV_NAME,
	.dev_name	= FPGA_SERIAL_DEV_NAME,
	.major		= FPGA_SERIAL_MAJOR,
	.minor		= FPGA_SERIAL_MINOR,
	.nr		= FPGAUART_MAXPORTS,
	.cons		= FPGAUART_CONSOLE,
};



static void fpgauart_stop_rx(struct uart_port *port)
{
  INT_CTRL &= ~UART_IEN;
}


static void fpgauart_stop_tx(struct uart_port *port)
{
  
}

static unsigned int fpgauart_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}


static void fpgauart_tx_chars(struct fpga_uart_port *pp)
{
	struct uart_port *port = &pp->port;
	struct circ_buf *xmit = &port->state->xmit;


	if (port->x_char) {
		/* Send special char - probably flow control */
		UART_DATA = port->x_char;
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		//INT_CTRL &= ~FTDI_TXIE;
		return;
	}

	while (1) {
	  while (UART_STAT & UART_TXB);
	  UART_DATA = xmit->buf[xmit->tail];
	  xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE -1);
	  port->icount.tx++;
	  if (uart_circ_empty(xmit))
			break;
	}
	
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	//if (uart_circ_empty(xmit))
		//INT_CTRL &= ~FTDI_TXIE;

}

static void fpgauart_start_tx(struct uart_port *port)
{
	struct fpga_uart_port *pp = container_of(port, struct fpga_uart_port, port);
	fpgauart_tx_chars(pp);
	//INT_CTRL |= FTDI_TXIE;
}

static void fpgauart_rx_chars(struct fpga_uart_port *pp)
{
	struct uart_port *port = &pp->port;
	unsigned char ch, flag;
	
	while (UART_STAT & UART_RXF) {
		ch = UART_DATA;
		
		flag = TTY_NORMAL;
		port->icount.rx++;

		tty_insert_flip_char(&port->state->port, ch, flag);
		
	}

	tty_flip_buffer_push(&port->state->port);
}


static irqreturn_t fpgauart_interrupt(int irq, void *data)
{
	struct uart_port *port = data;
	struct fpga_uart_port *pp = container_of(port, struct fpga_uart_port, port);
	irqreturn_t ret = IRQ_NONE;

	spin_lock(&port->lock);
	
	if (UART_STAT & UART_RXF) {
		fpgauart_rx_chars(pp);
		ret = IRQ_HANDLED;
	}/* else 
	if (!(FTDI_STAT & FTDI_TXE)) {
		fpgauartX_tx_chars(pp);
		ret = IRQ_HANDLED;
	}*/
	spin_unlock(&port->lock);

	return ret;
}

static unsigned int fpgauart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR; 
}

static void fpgauart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
}

static void fpgauart_enable_ms(struct uart_port *port)
{
}

static void fpgauart_break_ctl(struct uart_port *port, int break_state)
{
}

static int fpgauart_startup(struct uart_port *port)
{
	//struct fpgauartX_uart *pp = container_of(port, struct fpgauartX_uart, port);
	//unsigned long flags;
	//printk("start init\n");

	//spin_lock_irqsave(&port->lock, flags);
	
	INT_CTRL &= ~UART_IEN;
	//spin_lock_irqsave(&port->lock, flags);

	if (request_irq(port->irq, fpgauart_interrupt, 0, "FPGAUART", port))
		printk(KERN_ERR "MCF: unable to attach FPGAUART %d "
			"interrupt vector=%d\n", port->line, port->irq);


	INT_CTRL |= UART_IEN;

	//spin_unlock_irqrestore(&port->lock, flags);

	//printk("start done\n");
	return 0;
}

static void fpgauart_shutdown(struct uart_port *port)
{
	//struct fpgauartX_uart *pp = container_of(port, struct fpgauartX_uart, port);
	//unsigned long flags;

	//spin_lock_irqsave(&port->lock, flags);

	/* Disable all interrupts now */
	INT_CTRL &= ~UART_IEN;

	//spin_unlock_irqrestore(&port->lock, flags);
	free_irq(port->irq, port);
}

/****************************************************************************/

static void fpgauart_set_termios(struct uart_port *port, struct ktermios *termios,
	struct ktermios *old)
{
}


static const char *fpgauart_type(struct uart_port *port)
{
	return (port->type == PORT_M68K_FPGAUART) ? "Simple FPGA UART" : NULL;
}

static void fpgauart_release_port(struct uart_port *port)
{
	/* Nothing to release... */
}

static int fpgauart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_M68K_FPGAUART))
		return -EINVAL;
	return 0;
}

static int fpgauart_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

static void fpgauart_config_port(struct uart_port *port, int flags)
{
	port->type = PORT_M68K_FPGAUART;
	port->fifosize = 16;
	
	/* Clear mask, so no surprise interrupts. */
	INT_CTRL &= ~UART_IEN;
}


/*
 *	Define the basic serial functions we support.
 */
static const struct uart_ops fpgauart_uart_ops = {
	.tx_empty	= fpgauart_tx_empty,
	.get_mctrl	= fpgauart_get_mctrl,
	.set_mctrl	= fpgauart_set_mctrl,
	.start_tx	= fpgauart_start_tx,
	.stop_tx		= fpgauart_stop_tx,
	.stop_rx		= fpgauart_stop_rx,
	.enable_ms	= fpgauart_enable_ms,
	.break_ctl	= fpgauart_break_ctl,
	.startup		= fpgauart_startup,
	.shutdown	= fpgauart_shutdown,
	.set_termios	= fpgauart_set_termios,
	.type		= fpgauart_type,
	.request_port	= fpgauart_request_port,
	.release_port	= fpgauart_release_port,
	.config_port	= fpgauart_config_port,
	.verify_port	= fpgauart_verify_port,
};


#if defined(CONFIG_ALCE_FPGAUART_CONSOLE)

#endif




static int fpgauart_probe(struct platform_device *pdev)
{
	struct alce_platform_uart *platp = pdev->dev.platform_data;
	struct uart_port *port;
	int i;

	for (i = 0; ((i < FPGAUART_MAXPORTS) && (platp[i].mapbase)); i++) {
		port = &fpga_uart_ports[i].port;

		port->line = i;
		port->type = PORT_M68K_FPGAUART;
		port->mapbase = platp[i].mapbase;
		port->membase = (platp[i].membase) ? platp[i].membase :
			(unsigned char __iomem *) platp[i].mapbase;
		port->iotype = SERIAL_IO_MEM;
		port->irq = platp[i].irq;
		port->uartclk = 12000000; //MCF_BUSCLK;
		port->ops = &fpgauart_uart_ops;
		port->flags = ASYNC_BOOT_AUTOCONF;

		uart_add_one_port(&fpgauart_driver, port);
	}

	return 0;
}

static int fpgauart_remove(struct platform_device *pdev)
{
	/*struct uart_port *port;
	int i;

	for (i = 0; (i < FPGAUART_MAXPORTS); i++) {
		port = &fpgauartX_ports[i].port;
		if (port)
			uart_remove_one_port(&fpgauart_driver, port);
	}*/

	return 0;
}


static struct platform_driver fpgauart_platform_driver = {
	.probe		= fpgauart_probe,
	.remove		= fpgauart_remove,
	.driver		= {
		.name	= "fpgauart",
		.owner	= THIS_MODULE,
	},
};

/****************************************************************************/

static int __init fpgauart_init(void)
{
	int rc;

	printk("FPGAUART serial driver\n");

	rc = uart_register_driver(&fpgauart_driver);
	if (rc)
		return rc;
	rc = platform_driver_register(&fpgauart_platform_driver);
	if (rc)
		return rc;
	return 0;
}

/****************************************************************************/

static void __exit fpgauart_exit(void)
{
	platform_driver_unregister(&fpgauart_platform_driver);
	uart_unregister_driver(&fpgauart_driver);
}

/****************************************************************************/

module_init(fpgauart_init);
module_exit(fpgauart_exit);

MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_DESCRIPTION("FPGA UART serial driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:fpgauart");

/****************************************************************************/

