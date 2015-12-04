#include <common.h>
# include <asm/jzsoc.h>
#include <tcsm.h>
#include <uart.h>
#include <intc.h>

// volatile unsigned int *debug = (volatile unsigned int *)TCSM_UART_DEBUG;
void uart_init(void)
{
	REG32(TCSM_UART_TBUF_COUNT) = 0;
	REG32(TCSM_UART_TBUF_WP) = 0;
	REG32(TCSM_UART_TBUF_RP) = 0;

	REG32(TCSM_UART_RBUF_COUNT) = 0;
	REG32(TCSM_UART_RBUF_WP) = 0;
	REG32(TCSM_UART_RBUF_RP) = 0;
	REG32(TCSM_UART_DEVICE_NUM) = 1;
}

#define UART_REG32(addr)  REG32(0x10030000 + REG32(TCSM_UART_DEVICE_NUM) * 0x1000 + (addr))
#define UART_REG8(addr)   REG8(0x10030000 + REG32(TCSM_UART_DEVICE_NUM) * 0x1000 + (addr))
#define INTC_UART0_BIT 19
#define INTC_UART(n) (1 << (INTC_UART0_BIT - n))

int handle_uart_irq(void)
{
	struct mailbox_pend_addr_s *mailbox_pend_addr = (struct mailbox_pend_addr_s *)MAILBOX_PEND_ADDR;
	unsigned int dpr1 = INTC_REG32(INTC_DPR1);
	if (dpr1 & INTC_UART(REG32(TCSM_UART_DEVICE_NUM))) { // UART0 interrupt
		unsigned int iir = UART_REG32(UART_IIR);
		unsigned int lsr = UART_REG32(UART_LSR);
		int waddr = REG32(TCSM_UART_RBUF_WP);
		if (lsr & LSR_DRY) { // read
			volatile unsigned char *buf = (volatile unsigned char *)TCSM_UART_RBUF_ADDR;
			unsigned char status = UART_REG8(UART_LSR);
			int count = REG32(TCSM_UART_RBUF_COUNT);
			while (status & LSR_DRY) {
				buf[waddr] = UART_REG8(UART_RBR); // data
				buf[waddr + 1] = status; // flag
				waddr = (waddr + 2) & (TCSM_UART_BUF_LEN - 1);
				status = UART_REG8(UART_LSR);
			}

			*(volatile unsigned int *)TCSM_UART_RBUF_WP = waddr;
			if(waddr >= count)
				count = waddr - count;
			else
				count = TCSM_UART_BUF_LEN - count + waddr;

			if (count > (TCSM_UART_BUF_LEN - 64)) // full data int to CPU
				mailbox_pend_addr->mcu_state |= TCSM_UART_NEED_READ;

		}

		if (lsr & LSR_TDRQ) { // trans
			volatile unsigned char *buf = (volatile unsigned char *)TCSM_UART_TBUF_ADDR;
			unsigned int raddr = REG32(TCSM_UART_TBUF_RP);
			unsigned int count = REG32(TCSM_UART_TBUF_COUNT);
			int i = 0;
			while (raddr != count && i < 32 ) {
				UART_REG8(UART_THR) = buf[raddr];
				i++;
				raddr = (raddr + 1) & (TCSM_UART_BUF_LEN - 1);
			}

			*(volatile unsigned int *)TCSM_UART_TBUF_RP = raddr;
			if(raddr == count) {
				mailbox_pend_addr->mcu_state |= TCSM_UART_NEED_WRITE; // data request int to cpu
				UART_REG8(UART_IER) &= ~IER_TDRIE;
			}
			if(count >= raddr)
				count = count - raddr;
			else
				count = TCSM_UART_BUF_LEN - raddr + count;
			if (count == 64) {
				mailbox_pend_addr->mcu_state |= TCSM_UART_NEED_WRITE; // data request int to cpu
			}
		}

		if ((iir & IIR_INID) == IIR_RECEIVE_TIMEOUT)  // read over, send int to CPU
		{
			mailbox_pend_addr->mcu_state |= TCSM_UART_NEED_READ;
		}else {
			int count = REG32(TCSM_UART_RBUF_COUNT);
			if(count != waddr && !(UART_REG8(UART_LSR) & LSR_DRY)) {
				mailbox_pend_addr->mcu_state |= TCSM_UART_NEED_READ;
			}
		}
		if (mailbox_pend_addr->mcu_state)
			return 1;
	}

	return 0;
}
