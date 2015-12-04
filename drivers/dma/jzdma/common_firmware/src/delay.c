/* 
 * MCU delay function
 */

void mcu_delay(unsigned int cycle)
{
	__asm__ __volatile__ (
			"       .set    noreorder           \n"
			"       .align  3                   \n"
			"1:     bnez    %0, 1b              \n"
			"       subu    %0, 1               \n"
			"       .set    reorder             \n"
			: "=r" (cycle)
			: "0" (cycle));
}
