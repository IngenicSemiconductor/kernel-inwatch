/*
 *  Based on drivers/serial/8250.c by Russell King.
 *
 *  Author:	Nicolas Pitre
 *  Created:	Feb 20, 2003
 *  Copyright:	(C) 2003 Monta Vista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Note 1: This driver is made separate from the already too overloaded
 * 8250.c because it needs some kirks of its own and that'll make it
 * easier to add DMA support.
 *
 * Note 2: I'm too sick of device allocation policies for serial ports.
 * If someone else wants to request an "official" allocation of major/minor
 * for this driver please be my guest.  And don't forget that new hardware
 * to come from Intel might have more than 3 or 4 of those UARTs.  Let's
 * hope for a better port registration and dynamic device allocation scheme
 * with the serial core maintainer satisfaction to appear soon.
 */


#if defined(CONFIG_SERIAL_JZ47XX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/serial_core.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <mach/jzdma.h>
#include "jz_uart.h"

#define PORT_NR 5
#define DMA_BUFFER 1024
#define COUNT_DMA_BUFFER 2048
static unsigned short quot1[3] = {0}; /* quot[0]:baud_div, quot[1]:umr, quot[2]:uacr */
struct uart_jz47xx_port {
	struct uart_port        port;
	unsigned char           ier;
	unsigned char           lcr;
	unsigned char           mcr;
	unsigned int            lsr_break_flag;
	struct clk		*clk;
	char			name[16];

	int buf_id;
#ifdef CONFIG_SERIAL_JZ47XX_PDMA_UART
	unsigned int use_pdma;
#endif
	struct  tasklet_struct   tasklet_pio_rx;
};

static struct uart_jz47xx_port *serial_jz47xx_ports[PORT_NR];
static struct uart_driver serial_jz47xx_reg;

static inline void check_modem_status(struct uart_jz47xx_port *up);
static unsigned short *serial47xx_get_divisor(struct uart_port *port, unsigned int baud);
static inline void serial_dl_write(struct uart_port *up, int value);
/*
*Function:read register
*Parameter:struct uart_jz47xx_port *up, int offset
*Return:unsigned int:Register address
*/
static inline unsigned int serial_in(struct uart_jz47xx_port *up, int offset)
{
	offset <<= 2;
	return readl(up->port.membase + offset);
}

/*
*Function:write register
*Parameter:struct uart_jz47xx_port *up, int offset,int value:write value
*Return:void
*/

static inline void serial_out(struct uart_jz47xx_port *up, int offset, int value)
{
	offset <<= 2;
	writel(value, up->port.membase + offset);
}

/*
*Function: Enable Modem status interrupt
*Parameter:struct uart_jz47xx_port
*Return:void
*/
static void serial_jz47xx_enable_ms(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;

	up->ier |= UART_IER_MSI;// Enable Modem status interrupt
	serial_out(up, UART_IER, up->ier);
}

/*
*Function:stop transmitting
*Parameter:struct uart_jz47xx_port
*Return:void
*/
static void serial_jz47xx_stop_tx(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;

	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;// Disable the transmit data request interrupt
		serial_out(up, UART_IER, up->ier);
	}
}

/*
*Function:stop receiving
*Parameter:struct uart_jz47xx_port *up
*Return:void
*/
static void serial_jz47xx_stop_rx(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

/*
*Function:receive char
*Parameter:unsigned long data,unsigned int status
*Return:void
*/
static inline void receive_chars(unsigned long data)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)data;
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned int ch, flag;
	int max_count = 256;
	unsigned int status= serial_in(up, UART_LSR);
	while ((status & UART_LSR_DR) && (max_count-- > 0))
		//ready to receive data and max_count>0
	{
		ch = serial_in(up, UART_RX);//read RX_Register
		flag = TTY_NORMAL;// TTY_NORMAL=0
		up->port.icount.rx++;

		/*  Break interrupt error | prrity error | Frame error | overun error */
		if (unlikely(status & (UART_LSR_BI | UART_LSR_PE |
						UART_LSR_FE | UART_LSR_OE))) {
			printk("jz_uart: port %d error status(UART_LSR) 0x%x\n",
					up->port.line, status);

			if (status & UART_LSR_BI) {
				status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (status & UART_LSR_FE)
				up->port.icount.frame++;
			if (status & UART_LSR_OE)
				up->port.icount.overrun++;
			/*
			 * Mask off conditions which should be ignored.
			 */
			status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_JZ47XX_CONSOLE
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (status & UART_LSR_BI) {
				flag = TTY_BREAK;
			} else if (status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (status & UART_LSR_FE)
				flag = TTY_FRAME;
		}
/*
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;
*/
		uart_insert_char(&up->port, status, UART_LSR_OE, ch, flag);

ignore_char:
		status = serial_in(up, UART_LSR);
	}
	tty_flip_buffer_push(tty->port);
/*end to cancle*/
}
/* transmit one char*/
static void transmit_chars(struct uart_jz47xx_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;
	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);//transmit char=port.x_char
		up->port.icount.tx++;
		up->port.x_char = 0;//transmit finish x_char=0
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {//xmit is empty or stop tx
		serial_jz47xx_stop_tx(&up->port);
		return;
	}

	/* try to tx char until xmit is empty or count=0*/
	count = up->port.fifosize / 2;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	/*if circ_chars is less than WARKUP_CHARS,then warkup 	*/
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)//get the renainder of circ_chars
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		serial_jz47xx_stop_tx(&up->port);
}
DEFINE_SPINLOCK(t_lock);
static void serial_jz47xx_start_tx(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned long flags;
	spin_lock_irqsave(&t_lock,flags);
	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
	/* if(!strcmp(up->name,"uart1")) */
	/* { */
	/* 	udelay(100); */
	/* 	printk("uart1 send data!!!\n"); */
	/* } */
	spin_unlock_irqrestore(&t_lock,flags);
}

static inline void check_modem_status(struct uart_jz47xx_port *up)
{
	int status;
	status = serial_in(up, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

	wake_up_interruptible(&up->port.state->port.delta_msr_wait);
}
/*
 * This handles the interrupt from one port.
 */
static inline irqreturn_t serial_jz47xx_irq(int irq, void *dev_id)
{
	struct uart_jz47xx_port *up = dev_id;
	unsigned int iir, lsr;
	iir = serial_in(up, UART_IIR);
	lsr = serial_in(up, UART_LSR);
	if (iir & UART_IIR_NO_INT)
		return IRQ_NONE;

	if (lsr & UART_LSR_DR)
		receive_chars((unsigned long)up);
	check_modem_status(up);
	if (lsr & UART_LSR_THRE)
		transmit_chars(up);
	return IRQ_HANDLED;
}

#ifdef CONFIG_SERIAL_JZ47XX_PDMA_UART
#include "../../dma/jzdma/common_firmware/include/tcsm.h"
static void transmit_chars_to_pdma(struct uart_jz47xx_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;

	volatile unsigned char *buf = (volatile unsigned char *)PDMA_TO_CPU(TCSM_UART_TBUF_ADDR);
	unsigned int waddr = *(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_TBUF_RP);
	unsigned int count = *(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_TBUF_COUNT);
	if (up->port.x_char) {
		buf[count] = up->port.x_char;
		count = (count + 1) & (TCSM_UART_BUF_LEN - 1);
		up->port.icount.tx++;
		up->port.x_char = 0;//transmit finish x_char=0
		*(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_TBUF_COUNT) = count;
		return;
	}

	if (((count == waddr) && uart_circ_empty(xmit)) || uart_tx_stopped(&up->port)) {//xmit is empty or stop tx
		serial_jz47xx_stop_tx(&up->port);
		return;
	}
	do {
		if (uart_circ_empty(xmit)) // FIFO FULL
			break;
		buf[count] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		count = (count + 1) & (TCSM_UART_BUF_LEN - 1);
	}while (count != waddr);

	*(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_TBUF_COUNT) = count;
	if(count >= waddr)
		count = count - waddr;
	else
		count = TCSM_UART_BUF_LEN - waddr + count;

	/* if circ_chars is less than WARKUP_CHARS,then warkup */
	if ((count + uart_circ_chars_pending(xmit)) < WAKEUP_CHARS)//get the renainder of circ_chars
		uart_write_wakeup(&up->port);
}
static void receive_chars_from_pdma(struct uart_jz47xx_port *up)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned int ch, flag, status;
	int max_count = 256;

	unsigned int raddr = *(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_RBUF_WP);
	unsigned int count = *(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_RBUF_COUNT);
	volatile unsigned char *buf = (volatile unsigned char *)PDMA_TO_CPU(TCSM_UART_RBUF_ADDR);
	/* printk("raddr = %d count = %d\n",raddr,count); */
	/* printk("waddr = %x 0x14 = %X\n",raddr,*(volatile unsigned int *)0xb3422014); */
	while ( (raddr != count) && (max_count-- > 0)) {
		ch = buf[count]; // read RX_Register
		status = buf[count + 1];
		flag = TTY_NORMAL; // TTY_NORMAL=0
		up->port.icount.rx++;

		/*  Break interrupt error | prrity error | Frame error | overun error */
		if (unlikely(status & (UART_LSR_BI | UART_LSR_PE | UART_LSR_FE | UART_LSR_OE))) {
			if (status & UART_LSR_BI) {
				status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (status & UART_LSR_FE)
				up->port.icount.frame++;
			if (status & UART_LSR_OE)
				up->port.icount.overrun++;
			/*
			 * Mask off conditions which should be ignored.
			 */
			status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_JZ47XX_CONSOLE
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (status & UART_LSR_BI) {
				flag = TTY_BREAK;
			} else if (status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (status & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		/* printk("ch = %x status = %x\n",ch,status); */
		uart_insert_char(&up->port, status, UART_LSR_OE, ch, flag);
ignore_char:
		//raddr = (raddr + 2) & (TCSM_UART_BUF_LEN - 1);
		count = (count + 2) & (TCSM_UART_BUF_LEN - 1);

	}
	if(max_count != 256) {
		tty_flip_buffer_push(tty->port);
		*(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_RBUF_COUNT) = count;
	}
	/* printk("uier = %X\n",*(volatile unsigned int *)0xb0031004); */
	//*(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_RBUF_RP) = raddr;
	/* printk("d raddr = %d count = %d\n",raddr,count); */
}

void pdma_handle_irq(int id, unsigned int status)
{
	struct uart_jz47xx_port *up = serial_jz47xx_ports[id];
	if (status & TCSM_UART_NEED_READ)
		receive_chars_from_pdma(up);
	check_modem_status(up);
	if (status & TCSM_UART_NEED_WRITE) {
		transmit_chars_to_pdma(up);
		serial_out(up, UART_IER, up->ier);
	}
		//transmit_chars_to_pdma(up);
}
EXPORT_SYMBOL(pdma_handle_irq);
#endif

static unsigned int serial_jz47xx_tx_empty(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ret;
}

/*get modem control*/
static unsigned int serial_jz47xx_get_mctrl(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned char status;
	unsigned int ret;

	status = serial_in(up, UART_MSR);

	ret = 0;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_jz47xx_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr |= up->mcr;

	if (mctrl & UART_MCR_MDCE)
		mcr = mctrl;

	serial_out(up, UART_MCR, mcr);
}

static void serial_jz47xx_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void init_hw_stopped_status(struct uart_port *uport)
{
	struct tty_port tport = uport->state->port;
	struct tty_struct *tty = tport.tty;

	if (tport.flags & ASYNC_CTS_FLOW) {
		unsigned int mctrl;
		spin_lock_irq(&uport->lock);
		if (!((mctrl = uport->ops->get_mctrl(uport)) & TIOCM_CTS))
			tty->hw_stopped = 1;
		else
			tty->hw_stopped = 0;
		spin_unlock_irq(&uport->lock);
	}
}

static int serial_jz47xx_startup(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned long flags;
	int retval;
	up->port.uartclk = clk_get_rate(up->clk);//get clk
	/*
	 * Allocate the IRQ
	 */
#ifdef CONFIG_SERIAL_JZ47XX_PDMA_UART
	if(up->use_pdma) {
		retval = 0;
		//printk("Alloc pdma irq now!\n");
	} else
#endif
	retval = request_irq(up->port.irq, serial_jz47xx_irq, 0, up->name, up);//request irq
	if (retval)
		return retval;

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO
		   | UART_FCR_CLEAR_RCVR
		   | UART_FCR_CLEAR_XMIT
		   | UART_FCR_UME);
	serial_out(up, UART_FCR, 0);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);
	serial_out(up, UART_ISR, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl |= TIOCM_OUT2;
	serial_jz47xx_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termos(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI | UART_IER_RTOIE;
	serial_out(up, UART_IER, up->ier);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	init_hw_stopped_status(port);
	printk("startup uart %s\n",up->name);
	return 0;
}

static void serial_jz47xx_shutdown(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned long flags;
#ifdef CONFIG_SERIAL_JZ47XX_PDMA_UART
	if(up->use_pdma) {
		// printk("No need to free irq!\n");
	} else
#endif
	free_irq(up->port.irq, up);

	/*
	 * Disable interrupts from this port
	 */

	spin_lock_irqsave(&up->port.lock, flags);
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	up->port.mctrl &= ~TIOCM_OUT2;
	serial_jz47xx_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO
		   | UART_FCR_CLEAR_RCVR
		   | UART_FCR_CLEAR_XMIT
		   | UART_FCR_UME);
}

static void serial_jz47xx_set_termios(struct uart_port *port, struct ktermios *termios,struct ktermios *old)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	unsigned char cval=0;
	unsigned long flags;
	unsigned int baud;
	unsigned short *quot1;

	switch (termios->c_cflag & CSIZE) {
		case CS5:
			cval = UART_LCR_WLEN5;
			break;
		case CS6:
			cval = UART_LCR_WLEN6;
			break;
		case CS7:
			cval = UART_LCR_WLEN7;
			break;
		default:
		case CS8:
			cval = UART_LCR_WLEN8;
			break;
	}

	if (termios->c_cflag & CSTOPB){
		cval |= UART_LCR_STOP;
	  }
	if (termios->c_cflag & PARENB){
		cval |= UART_LCR_PARITY;
	  }
	if (!(termios->c_cflag & PARODD)){
		cval |= UART_LCR_EPAR;
	  }
	serial_out(up, UART_LCR, cval);//write cval to UART_LCR
	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk);//get BaudRate
	quot1 = serial47xx_get_divisor(port, baud);
	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);
	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);
	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	/*
	 *enable modem status interrupts and enable modem function and control by hardware
	 */
	if (UART_ENABLE_MS(&up->port, termios->c_cflag)) {
		up->ier |= UART_IER_MSI;
		serial_out(up, UART_IER, up->ier);
		up->port.mctrl = UART_MCR_MDCE | UART_MCR_FCM;
		serial_jz47xx_set_mctrl(&up->port, up->port.mctrl);
	}
	/*
	 *disable modem status interrupts and clear MCR bits of modem function and control by hardware
	 */
	else {
		up->ier &= ~UART_IER_MSI;
		serial_out(up, UART_IER, up->ier);
		up->port.mctrl &= ~(UART_MCR_MDCE | UART_MCR_FCM);
		serial_jz47xx_set_mctrl(&up->port, up->port.mctrl);
	     }
#if 0
	serial_out(up, UART_LCR, cval | UART_LCR_DLAB);	/* set DLAB */
	serial_out(up, UART_DLL, quot & 0xff);		/* LS of divisor */

	/*
	 * work around Errata #75 according to Intel(R) PXA27x Processor Family
	 * Specification Update (Nov 2005)
	 */
	dll = serial_in(up, UART_DLL);
	WARN_ON(dll != (quot & 0xff));

	serial_out(up, UART_DLM, quot >> 8);		/* MS of divisor */
	serial_out(up, UART_LCR, cval);			/* reset DLAB */
#endif
	serial_dl_write(port, quot1[0]);
	serial_out(up,UART_UMR, quot1[1]);//UART send or receive one bit takes quot1[1] cycles
	serial_out(up,UART_UACR, quot1[2]);

	up->lcr = cval;					/* Save LCR */
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_11 | UART_FCR_UME);
	spin_unlock_irqrestore(&up->port.lock, flags);

}

static inline void serial_dl_write(struct uart_port *up, int value)
{
	struct uart_jz47xx_port *port = (struct uart_jz47xx_port*)up;
	int lcr = serial_in(port,UART_LCR);
	serial_out(port,UART_LCR, UART_LCR_DLAB);
	serial_out(port,UART_DLL, value & 0xff);
	serial_out(port,UART_DLM, (value >> 8 )& 0xff);
	serial_out(port,UART_LCR, lcr);
}

static unsigned short *serial47xx_get_divisor(struct uart_port *port, unsigned int baud)
{
	int err, sum, i, j;
	int a[12], b[12];
	unsigned short div, umr, uacr;
	unsigned short umr_best, div_best, uacr_best;
	unsigned long long t0, t1, t2, t3;

	sum = 0;
	umr_best = div_best = uacr_best = 0;
	div = 1;

	if ((port->uartclk % (16 * baud)) == 0) {
		quot1[0] = port->uartclk / (16 * baud);
		quot1[1] = 16;
		quot1[2] = 0;
		return quot1;
	}

	while (1) {
		umr = port->uartclk / (baud * div);
		if (umr > 32) {
			div++;
			continue;
		}
		if (umr < 4) {
			break;
		}
		for (i = 0; i < 12; i++) {
			a[i] = umr;
			b[i] = 0;
			sum = 0;
			for (j = 0; j <= i; j++) {
				sum += a[j];
			}

			/* the precision could be 1/2^(36) due to the value of t0 */
			t0 = 0x1000000000LL;
			t1 = (i + 1) * t0;
			t2 = (sum * div) * t0;
			t3 = div * t0;
			do_div(t1, baud);
			do_div(t2, port->uartclk);
			do_div(t3, (2 * port->uartclk));
			err = t1 - t2 - t3;

			if (err > 0) {
				a[i] += 1;
				b[i] = 1;
			}
		}

		uacr = 0;
		for (i = 0; i < 12; i++) {
			if (b[i] == 1) {
				uacr |= 1 << i;
			}
		}
		if (div_best ==0){
			div_best = div;
			umr_best = umr;
			uacr_best = uacr;
		}

		/* the best value of umr should be near 16, and the value of uacr should better be smaller */
		if (abs(umr - 16) < abs(umr_best - 16) || (abs(umr - 16) == abs(umr_best - 16) && uacr_best > uacr))
		{
			div_best = div;
			umr_best = umr;
			uacr_best = uacr;
		}
		div++;
	}

	quot1[0] = div_best;
	quot1[1] = umr_best;
	quot1[2] = uacr_best;

	return quot1;
}

static void serial_jz47xx_release_port(struct uart_port *port)
{
}

static int serial_jz47xx_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_jz47xx_config_port(struct uart_port *port, int flags)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	up->port.type = PORT_JZ47XX;
}

static int serial_jz47xx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

static const char *serial_jz47xx_type(struct uart_port *port)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;
	return up->name;
}
static struct uart_jz47xx_port *serial_jz47xx_ports[PORT_NR];
static struct uart_driver serial_jz47xx_reg;

#ifdef CONFIG_SERIAL_JZ47XX_CONSOLE

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/*
 *	Wait for transmitter & holding register to empty
 */
static inline void wait_for_xmitr(struct uart_jz47xx_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & BOTH_EMPTY) != BOTH_EMPTY);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		while (--tmout &&
				((serial_in(up, UART_MSR) & UART_MSR_CTS) == 0))//no CTS
			udelay(1);
	}
}

static void serial_jz47xx_console_putchar(struct uart_port *port, int ch)
{
	struct uart_jz47xx_port *up = (struct uart_jz47xx_port *)port;

	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void serial_jz47xx_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_jz47xx_port *up = serial_jz47xx_ports[co->index];
	unsigned int ier;

	/*
	 *	First save the IER then disable the interrupts
	 */
	spin_lock(&up->port.lock);
	ier = up->ier;
	up->ier &= ~UART_IER_THRI;
	serial_out(up, UART_IER, up->ier);

	uart_console_write(&up->port, s, count, serial_jz47xx_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up);
	up->ier = ier;
	serial_out(up, UART_IER, ier);
	spin_unlock(&up->port.lock);
}

struct uart_port *serial_get_uart_port(int port_index)
{
    struct uart_state *state = serial_jz47xx_reg.state + port_index;

    /* The uart_driver structure stores the states in an array.
     * Thus the corresponding offset from the drv->state returns
     * the state for the uart_port that is requested
     */
    if (port_index == state->uart_port->line)
        return state->uart_port;

    return NULL;
}
EXPORT_SYMBOL(serial_get_uart_port);

static int __init serial_jz47xx_console_setup(struct console *co, char *options)
{
	struct uart_jz47xx_port *up;
	int baud = 57600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index == -1 || co->index >= serial_jz47xx_reg.nr)
		co->index = 0;
	up = serial_jz47xx_ports[co->index];
	if (!up)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console serial_jz47xx_console = {
	.name		= "ttyS",
	.write		= serial_jz47xx_console_write,
	.device		= uart_console_device,
	.setup		= serial_jz47xx_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_jz47xx_reg,
};

#define JZ47XX_CONSOLE	&serial_jz47xx_console
#else
#define JZ47XX_CONSOLE	NULL
#endif

struct uart_ops serial_jz47xx_pops = {
	.tx_empty	= serial_jz47xx_tx_empty,//TX Buffer empty
	.set_mctrl	= serial_jz47xx_set_mctrl,//set Modem control
	.get_mctrl	= serial_jz47xx_get_mctrl,//get Modem control
	.stop_tx	= serial_jz47xx_stop_tx,//stop TX
	.start_tx	= serial_jz47xx_start_tx,//start tx
	.stop_rx	= serial_jz47xx_stop_rx,//stop rx
	.enable_ms	= serial_jz47xx_enable_ms,//modem status enable
	.break_ctl	= serial_jz47xx_break_ctl,//
	.startup	= serial_jz47xx_startup,//start endport
	.shutdown	= serial_jz47xx_shutdown,//shutdown endport
	.set_termios	= serial_jz47xx_set_termios,//change para of endport
	.type		= serial_jz47xx_type,//
	.release_port	= serial_jz47xx_release_port,//release port I/O
	.request_port	= serial_jz47xx_request_port,//
	.config_port	= serial_jz47xx_config_port,//
	.verify_port	= serial_jz47xx_verify_port,//
};

static struct uart_driver serial_jz47xx_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "JZ47XX serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= PORT_NR,
	.cons		= JZ47XX_CONSOLE,
};

#ifdef CONFIG_PM
static int serial_jz47xx_suspend(struct device *dev)
{
	struct uart_jz47xx_port *up = dev_get_drvdata(dev);

	if (up)
		uart_suspend_port(&serial_jz47xx_reg, &up->port);

	return 0;
}

static int serial_jz47xx_resume(struct device *dev)
{
	struct uart_jz47xx_port *up = dev_get_drvdata(dev);

	if (up)
		uart_resume_port(&serial_jz47xx_reg, &up->port);

	return 0;
}

static const struct dev_pm_ops serial_jz47xx_pm_ops = {
	.suspend	= serial_jz47xx_suspend,
	.resume		= serial_jz47xx_resume,
};
#endif

static int serial_jz47xx_probe(struct platform_device *dev)
{
	struct uart_jz47xx_port *up;
	struct resource *mmres, *irqres;
	int ret;

	mmres = platform_get_resource(dev, IORESOURCE_MEM, 0);
	irqres = platform_get_resource(dev, IORESOURCE_IRQ, 0);
	if (!mmres || !irqres)
		return -ENODEV;
	up = kzalloc(sizeof(struct uart_jz47xx_port), GFP_KERNEL);
	if (!up)
		return -ENOMEM;

	sprintf(up->name,"uart%d",dev->id);

	up->clk = clk_get(&dev->dev, up->name);
	if (IS_ERR(up->clk)) {
		ret = PTR_ERR(up->clk);
		goto err_free;
	}

	clk_enable(up->clk);

	up->port.type = PORT_JZ47XX;
	up->port.iotype = UPIO_MEM;
	up->port.mapbase = mmres->start;
	up->port.irq = irqres->start;
	up->port.fifosize = 64;
	up->port.ops = &serial_jz47xx_pops;
	up->port.line = dev->id;
	up->port.dev = &dev->dev;
	up->port.flags = UPF_IOREMAP | UPF_BOOT_AUTOCONF;
	up->port.uartclk = clk_get_rate(up->clk);
	up->port.membase = ioremap(mmres->start, mmres->end - mmres->start + 1);
	if (!up->port.membase) {
		ret = -ENOMEM;
		goto err_clk;
	}
#ifdef CONFIG_SERIAL_JZ47XX_PDMA_UART
	if (irqres->name && !strcmp(irqres->name, "pdma")) {
		printk("%s used pdma\n", up->name);
		*(volatile unsigned int *)PDMA_TO_CPU(TCSM_UART_DEVICE_NUM) = dev->id;
		up->use_pdma = 1;
	} else
		up->use_pdma = 0;
#endif

	serial_jz47xx_ports[dev->id] = up;
	uart_add_one_port(&serial_jz47xx_reg, &up->port);
	platform_set_drvdata(dev, up);

	return 0;

err_clk:
	clk_put(up->clk);
err_free:
	kfree(up);
	return ret;
}

static int serial_jz47xx_remove(struct platform_device *dev)
{
	struct uart_jz47xx_port *up = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	uart_remove_one_port(&serial_jz47xx_reg, &up->port);
	clk_put(up->clk);
	kfree(up);

	return 0;
}

static struct platform_driver serial_jz47xx_driver = {
	.probe          = serial_jz47xx_probe,
	.remove         = serial_jz47xx_remove,

	.driver		= {
		.name	= "jz-uart",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &serial_jz47xx_pm_ops,
#endif
	},
};

int __init serial_jz47xx_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_jz47xx_reg);
	if (ret != 0)
		return ret;

	ret = platform_driver_register(&serial_jz47xx_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_jz47xx_reg);

	return ret;
}

void __exit serial_jz47xx_exit(void)
{
	platform_driver_unregister(&serial_jz47xx_driver);
	uart_unregister_driver(&serial_jz47xx_reg);
}


#ifdef CONFIG_EARLY_INIT_RUN
rootfs_initcall(serial_jz47xx_init);

#else
module_init(serial_jz47xx_init);

#endif

module_exit(serial_jz47xx_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz47xx-uart");
