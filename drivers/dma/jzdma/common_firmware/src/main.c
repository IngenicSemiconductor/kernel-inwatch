/*
 * main.c
 */
#include <common.h>
#include <asm/jzsoc.h>
#include <pdma.h>
#include <delay.h>
#include <tcsm.h>

#if defined(HANDLE_UART)
#include <uart.h>
#endif

static void handle_init(void)
{
#if defined(HANDLE_UART)
	uart_init();
#endif
}

static volatile unsigned int mailbox_request;
static noinline void mcu_init(void)
{
	/* clear mailbox irq pending */
	REG32(PDMAC_DMNMB) = 0;
	REG32(PDMAC_DMSMB) = 0;
	REG32(PDMAC_DMINT) = PDMAC_DMINT_S_IMSK;

	/* clear irq mask for channel irq */
	REG32(PDMAC_DCIRQM) = 0;
	mailbox_request = 0;
}
extern volatile unsigned int need_send_mailbox;
static void mcu_sleep()
{
	struct mailbox_pend_addr_s *mailbox_pend_addr = (struct mailbox_pend_addr_s *)MAILBOX_PEND_ADDR;
	__pdma_irq_disable();
	if(!(REG32(PDMAC_DMINT) & PDMAC_DMINT_N_IP))
	{
		if(need_send_mailbox)
		{
			need_send_mailbox = 0;
			mailbox_pend_addr->cpu_state = mailbox_pend_addr->mcu_state;
			mailbox_pend_addr->mcu_state = 0;
			REG32(PDMAC_DMNMB) = 0xFFFFFFFF; // Create an normal mailbox irq to CPU
		}else{
			__pdma_mwait();

		}
	}
	__pdma_irq_enable();
}
int main(void)
{
	mcu_init();
	handle_init();
	while(1) {
		mcu_sleep();
	}
	return 0;
}
