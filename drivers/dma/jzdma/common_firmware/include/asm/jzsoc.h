/*
 *  include/asm/jzsoc.h
 *
 *  Ingenic's JZXXXX SoC common include.
 *
 *  Copyright (C) 2006 - 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZSOC_H__
#define __ASM_JZSOC_H__

/*
 * SoC include
 */
#ifdef INTERFACE_NEMC
#include <asm/mach/jz4780-base.h>
#endif
#ifdef INTERFACE_NFI
#include <asm/mach/jz4785-base.h>
#endif
#include <asm/mach/gpio.h>
#include <asm/mach/bch.h>
#include <asm/mach/misc.h>
#ifdef INTERFACE_NEMC
#include <asm/mach/nemc.h>
#endif
#ifdef INTERFACE_NFI
#include <asm/mach/nfi.h>
#endif
#include <asm/mach/pdma.h>

#endif /* __ASM_JZSOC_H__ */
