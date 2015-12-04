#ifndef __TCSM_H__
#define __TCSM_H__

#define PDMA_TO_CPU(addr)       (0xB3422000 + (addr & 0xFFFF))
#define CPU_TO_PDMA(addr)       (0xF4000000 + (addr & 0xFFFF) - 0x2000)

struct mailbox_pend_addr_s {
	unsigned int cpu_state;
	unsigned int mcu_state;
};

// IRQ Pending Flag Space : up to 64 irq
#define MAILBOX_PEND_ADDR       0xF4007000

// UART Allocate Space
#define TCSM_UART_BASE_ADDR     0xF4007100
#define TCSM_UART_TBUF_COUNT    (TCSM_UART_BASE_ADDR)
#define TCSM_UART_TBUF_WP       (TCSM_UART_TBUF_COUNT + 0x4)
#define TCSM_UART_TBUF_RP       (TCSM_UART_TBUF_WP + 0x4)
#define TCSM_UART_RBUF_COUNT    (TCSM_UART_TBUF_RP + 0x4)
#define TCSM_UART_RBUF_WP       (TCSM_UART_RBUF_COUNT + 0x4)
#define TCSM_UART_RBUF_RP       (TCSM_UART_RBUF_WP + 0x4)
#define TCSM_UART_DEVICE_NUM    (TCSM_UART_RBUF_RP + 0x4)
#define TCSM_UART_DEBUG         (TCSM_UART_DEVICE_NUM + 0x4)

#define TCSM_UART_NEED_READ     (0x1 << 0)
#define TCSM_UART_NEED_WRITE    (0x1 << 1)

#define TCSM_UART_BUF_LEN       0x800
#define TCSM_UART_TBUF_ADDR     0xF4006000
#define TCSM_UART_RBUF_ADDR     (TCSM_UART_TBUF_ADDR + TCSM_UART_BUF_LEN)
#define TCSM_UART_BUF_END       (TCSM_UART_RBUF_ADDR + TCSM_UART_BUF_LEN)

#endif
