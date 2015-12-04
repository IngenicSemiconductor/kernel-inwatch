OUTPUT_ARCH(mips)
ENTRY(_reset)
MEMORY{
	tcsm_bank0(rw)  : ORIGIN = 0xF4000000 , LENGTH = 0x1F3C
	tcsm_bank3_1(rw)  : ORIGIN = 0xF4001F3C , LENGTH = 0xC4
	tcsm_bank4_0(rw)  : ORIGIN = 0xF4002000 , LENGTH = 0xC4
	tcsm_bank4_1(rw)  : ORIGIN = 0xF40020C4 , LENGTH = 0x730
/*
	tcsm_bank5_0(rw)  : ORIGIN = 0xF4002800 , LENGTH = 0x500
	tcsm_bank5_1(rw)  : ORIGIN = 0xF4002d00 , LENGTH = 0x300
	tcsm_bank6_0(rw)  : ORIGIN = 0xF4003000 , LENGTH = 0x500
	tcsm_bank6_1(rw)  : ORIGIN = 0xF4003500 , LENGTH = 0x300
	tcsm_bank7_1(rw)  : ORIGIN = 0xF4003800 , LENGTH = 0x800
*/
}

SECTIONS
{
	.tcsm_bank0 : {
		. = ALIGN(4);
                *(.text)
                *(.data)
                CONSTRUCTORS
                _gp = ALIGN(4);
                *(.sdata)
                *(.sdata.*)
                *(.sbss)
                *(.bss)
                *(.scommon)
                *(.rodata)
	} > tcsm_bank0

	.tcsm_bank3_1 : {
		*(.tcsm_data3)
	} > tcsm_bank3_1

	.tcsm_bank4_0 : {
		*(.tcsm_data4)
	} > tcsm_bank4_0

	.tcsm_bank4_1 : {
		*(.tcsm_bank4_1)
	} > tcsm_bank4_1
/*
	.tcsm_bank5_0 : {
		*(.tcsm_data5)
	} > tcsm_bank5_0

	.tcsm_bank5_1 : {
		*(.tcsm_bank5_1)
	} > tcsm_bank5_1

	.tcsm_bank6_0 : {
		*(.tcsm_data6)
	} > tcsm_bank6_0

	.tcsm_bank6_1 : {
		*(.tcsm_bank6_1)
	} > tcsm_bank6_1

	.tcsm_bank7_1 : {
		*(.tcsm_bank7_1)
	} > tcsm_bank7_1
*/
	/DISCARD/ : {
		*(COMMON)
		*(.pdri)
		*(.comment)
		*(.gnu.attributes)
		*(.reginfo)
	}
}
