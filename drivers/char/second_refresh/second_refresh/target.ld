OUTPUT_ARCH(mips)
ENTRY(start)

MEMORY{
	/* TOTAL MEM :6 * 1024 * 1024 Bytes */
	/* code. data. ... */
	second_refresh(rw): ORIGIN = 0x20000000, LENGTH = 0x600000
}

SECTIONS
{

	.second_refresh : {
		. = ALIGN(4);
			*(.text)
			/**(.data)*/

			*(.bss)
			*(.rodata.*)
	} > second_refresh

	/DISCARD/ : {
		*(COMMON)
		*(.pdri)
		*(.comment)
		*(.gnu.attributes)
		*(.reginfo)
	}
}
