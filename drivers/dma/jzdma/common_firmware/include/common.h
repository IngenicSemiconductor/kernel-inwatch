/*
 * common.h
 */
#ifndef __COMMON_H__
#define __COMMON_H__

#ifndef noinline
#define noinline __attribute__((noinline))
#endif

#ifndef __section
# define __section(S) __attribute__ ((__section__(#S)))
#endif

// #define __bank4		__section(.tcsm_bank4_1)
// #define __bank5		__section(.tcsm_bank5_1)
// #define __bank6		__section(.tcsm_bank6_1)
// #define __bank7		__section(.tcsm_bank7_1)

#define NULL		0
#define ALIGN_ADDR_WORD(addr)	(void *)((((unsigned int)(addr) >> 2) + 1) << 2)

#define MCU_SOFT_IRQ		(1 << 2)
#define MCU_CHANNEL_IRQ		(1 << 1)
#define MCU_INTC_IRQ		(1 << 0)

typedef		char s8;
typedef	unsigned char	u8;
typedef		short s16;
typedef unsigned short	u16;
typedef		int s32;
typedef unsigned int	u32;

#endif /* __COMMON_H__ */
