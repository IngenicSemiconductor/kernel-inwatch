
/*
 * I2C adapter for the INGENIC I2C bus access.
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <asm/current.h>
#include <mach/platform.h>
#include <soc/gpio.h>

#define CONFIG_I2C_JZ4775_WAIT_MS 500

#define	I2C_CTRL		(0x00)
#define	I2C_TAR			(0x04)
#define	I2C_SAR			(0x08)
#define	I2C_DC			(0x10)
#define	I2C_SHCNT		(0x14)
#define	I2C_SLCNT		(0x18)
#define	I2C_FHCNT		(0x1C)
#define	I2C_FLCNT		(0x20)
#define	I2C_INTST		(0x2C)
#define	I2C_INTM		(0x30)
#define I2C_RAW_INTST		(0x34)
#define I2C_RXTL		(0x38)
#define I2C_TXTL		(0x3c)
#define	I2C_CINTR		(0x40)
#define	I2C_CRXUF		(0x44)
#define	I2C_CRXOF		(0x48)
#define	I2C_CTXOF		(0x4C)
#define	I2C_CRXREQ		(0x50)
#define	I2C_CTXABRT		(0x54)
#define	I2C_CRXDONE		(0x58)
#define	I2C_CACT		(0x5C)
#define	I2C_CSTP		(0x60)
#define	I2C_CSTT		(0x64)
#define	I2C_CGC			(0x68)
#define	I2C_ENB			(0x6C)
#define	I2C_STA			(0x70)
#define I2C_SDAHD		(0x7C)
#define I2C_TXFLR		(0x74)
#define I2C_RXFLR		(0x78)
#define	I2C_TXABRT		(0x80)
#define I2C_DMACR		(0x88)
#define I2C_DMATDLR		(0x8c)
#define I2C_DMARDLR		(0x90)
#define	I2C_SDASU		(0x94)
#define	I2C_ACKGC		(0x98)
#define	I2C_ENSTA		(0x9C)
#define I2C_FLT			(0xA0)

/* I2C Control Register (I2C_CTRL) */
#define I2C_CTRL_SLVDIS		(1 << 6) /* after reset slave is disabled*/
#define I2C_CTRL_RESTART		(1 << 5)
#define I2C_CTRL_MATP		(1 << 4) /* 1: 10bit address 0: 7bit addressing*/
#define I2C_CTRL_SATP		(1 << 3) /* 1: 10bit address 0: 7bit address*/
#define I2C_CTRL_SPDF		(2 << 1) /* fast mode 400kbps */
#define I2C_CTRL_SPDS		(1 << 1) /* standard mode 100kbps */
#define I2C_CTRL_MD		(1 << 0) /* master enabled*/

/* I2C Target Address Register (I2C_TAR) */
#define I2C_TAR_MATP		(1 << 12) /* master mode, 0: 7-bit address,
						1: 10-bit address */

/* I2C Status Register (I2C_STA) */
#define I2C_STA_SLVACT		(1 << 6) /* Slave FSM is not in IDLE state */
#define I2C_STA_MSTACT		(1 << 5) /* Master FSM is not in IDLE state */
#define I2C_STA_RFF		(1 << 4) /* RFIFO if full */
#define I2C_STA_RFNE		(1 << 3) /* RFIFO is not empty */
#define I2C_STA_TFE		(1 << 2) /* TFIFO is empty */
#define I2C_STA_TFNF		(1 << 1) /* TFIFO is not full  */
#define I2C_STA_ACT		(1 << 0) /* I2C Activity Status */

enum i2c_device_type {
	MASTER,
	SLAVE,
};

enum i2c_address_type {
	ADDR_7BIT,
	ADDR_10BIT,
};

/* four operate mode, master read and write, slave read and write */
enum i2c_operate_mode {
	M_READ,
	M_WRITE,
	S_READ,
	S_WRITE,
};

struct jz_i2c_gpios {
    char name[20];
    int port;
    int func;
    unsigned long pins;
};

/* I2C Transmit Abort Status Register (I2C_TXABRT) */
static char *abrt_src[] = {
	"I2C_TXABRT_ABRT_7B_ADDR_NOACK",
	"I2C_TXABRT_ABRT_10ADDR1_NOACK",
	"I2C_TXABRT_ABRT_10ADDR2_NOACK",
	"I2C_TXABRT_ABRT_XDATA_NOACK",
	"I2C_TXABRT_ABRT_GCALL_NOACK",
	"I2C_TXABRT_ABRT_GCALL_READ",
	"I2C_TXABRT_ABRT_HS_ACKD",
	"I2C_TXABRT_SBYTE_ACKDET",
	"I2C_TXABRT_ABRT_HS_NORSTRT",
	"I2C_TXABRT_SBYTE_NORSTRT",
	"I2C_TXABRT_ABRT_10B_RD_NORSTRT",
	"I2C_TXABRT_ABRT_MASTER_DIS",
	"I2C_TXABRT_ARB_LOST",
	"I2C_TXABRT_SLVFLUSH_TXFIFO",
	"I2C_TXABRT_SLV_ARBLOST",
	"I2C_TXABRT_SLVRD_INTX",
};

/* i2c interrupt status (I2C_INTST) */
#define I2C_INTST_IGC					(1 << 11)
#define I2C_INTST_ISTT					(1 << 10)
#define I2C_INTST_ISTP					(1 << 9)
#define I2C_INTST_IACT					(1 << 8)
#define I2C_INTST_RXDN					(1 << 7)
#define I2C_INTST_TXABT					(1 << 6)
#define I2C_INTST_RDREQ					(1 << 5)
#define I2C_INTST_TXEMP					(1 << 4)
#define I2C_INTST_TXOF					(1 << 3)
#define I2C_INTST_RXFL					(1 << 2)
#define I2C_INTST_RXOF					(1 << 1)
#define I2C_INTST_RXUF					(1 << 0)

/* i2c interrupt mask status (I2C_INTM) */
#define I2C_INTM_MIGC			(1 << 11)
#define I2C_INTM_MISTT			(1 << 10)
#define I2C_INTM_MISTP			(1 << 9)
#define I2C_INTM_MIACT			(1 << 8)
#define I2C_INTM_MRXDN			(1 << 7)
#define I2C_INTM_MTXABT			(1 << 6)
#define I2C_INTM_MRDREQ			(1 << 5)
#define I2C_INTM_MTXEMP			(1 << 4)
#define I2C_INTM_MTXOF			(1 << 3)
#define I2C_INTM_MRXFL			(1 << 2)
#define I2C_INTM_MRXOF			(1 << 1)
#define I2C_INTM_MRXUF			(1 << 0)

#define I2C_DC_RESTART			(1 << 10)
#define I2C_DC_STP			(1 << 9)
#define I2C_DC_READ				(1 << 8)

#define I2C_ENB_I2CENB			(1 << 0) /* Enable the i2c */

/* I2C standard mode high count register(I2CSHCNT) */
#define I2CSHCNT_ADJUST(n)		(((n) - 8) < 6 ? 6 : ((n) - 8))
/* I2C standard mode low count register(I2CSLCNT) */
#define I2CSLCNT_ADJUST(n)		(((n) - 1) < 8 ? 8 : ((n) - 1))
/* I2C fast mode high count register(I2CFHCNT) */
#define I2CFHCNT_ADJUST(n)		(((n) - 8) < 6 ? 6 : ((n) - 8))
/* I2C fast mode low count register(I2CFLCNT) */
#define I2CFLCNT_ADJUST(n)		(((n) - 1) < 8 ? 8 : ((n) - 1))

#define I2C_FIFO_LEN 64
#define TRANS_UNIT (I2C_FIFO_LEN/2)

#define TIMEOUT 0xff

#define BUFSIZE 200

//#define CONFIG_I2C_DEBUG

struct i2c_sg_data {
	struct scatterlist *sg;
	int sg_len;
	int flag;
};

struct jz_i2c {
	void __iomem *iomem;
	int irq;
	struct clk *clk;
	struct i2c_adapter adap;

	unsigned char *rbuf;
	unsigned char *wbuf;
	int rd_len;
	int wt_len;
	int rdcmd_len;

	int rx_level;

	int abort_src;

	int read_finsh;
	int write_finsh;
	int need_restart;
	int need_stop;
	enum i2c_operate_mode operate;

#ifdef CONFIG_I2C_DEBUG
	int data_buf[BUFSIZE];
	int cmd_buf[BUFSIZE];
	int cmd;
#endif
	struct i2c_sg_data *data;
	struct completion r_complete;
	struct completion w_complete;
	unsigned int bus_speed;
	struct mutex lock;

	pid_t suspend_thread_pid;
	unsigned int block_other_thread:1;
	unsigned int is_suspend:1;
};

#ifdef CONFIG_I2C_DEBUG

#define PRINT_REG_WITH_ID(reg_name, id)					\
	dev_info(&(i2c->adap.dev), "--"#reg_name "0x%08x\n", i2c_readl(id, reg_name))

static void jz_dump_i2c_regs(struct jz_i2c *i2c) {
	struct jz_i2c *i2c_id = i2c;

	PRINT_REG_WITH_ID(I2C_CTRL, i2c_id);
	PRINT_REG_WITH_ID(I2C_INTM, i2c_id);
	PRINT_REG_WITH_ID(I2C_RXTL, i2c_id);
	PRINT_REG_WITH_ID(I2C_TXTL, i2c_id);
	PRINT_REG_WITH_ID(I2C_RXFLR, i2c_id);

	return;

	PRINT_REG_WITH_ID(I2C_CTRL, i2c_id);
	PRINT_REG_WITH_ID(I2C_TAR, i2c_id);
	PRINT_REG_WITH_ID(I2C_SAR, i2c_id);
	//	PRINT_REG_WITH_ID(I2C_DC, i2c_id);
	PRINT_REG_WITH_ID(I2C_SHCNT, i2c_id);
	PRINT_REG_WITH_ID(I2C_SLCNT, i2c_id);
	PRINT_REG_WITH_ID(I2C_FHCNT, i2c_id);
	PRINT_REG_WITH_ID(I2C_FLCNT, i2c_id);
	PRINT_REG_WITH_ID(I2C_INTST, i2c_id);
	PRINT_REG_WITH_ID(I2C_INTM, i2c_id);
	PRINT_REG_WITH_ID(I2C_RXTL, i2c_id);
	PRINT_REG_WITH_ID(I2C_TXTL, i2c_id);
	PRINT_REG_WITH_ID(I2C_CINTR, i2c_id);
	PRINT_REG_WITH_ID(I2C_CRXUF, i2c_id);
	PRINT_REG_WITH_ID(I2C_CRXOF, i2c_id);
	PRINT_REG_WITH_ID(I2C_CTXOF, i2c_id);
	PRINT_REG_WITH_ID(I2C_CRXREQ, i2c_id);
	PRINT_REG_WITH_ID(I2C_CTXABRT, i2c_id);
	PRINT_REG_WITH_ID(I2C_CRXDONE, i2c_id);
	PRINT_REG_WITH_ID(I2C_CACT, i2c_id);
	PRINT_REG_WITH_ID(I2C_CSTP, i2c_id);
	PRINT_REG_WITH_ID(I2C_CSTT, i2c_id);
	PRINT_REG_WITH_ID(I2C_CGC, i2c_id);
	PRINT_REG_WITH_ID(I2C_ENB, i2c_id);
	PRINT_REG_WITH_ID(I2C_STA, i2c_id);
	/*debug trans & recive fifo count */
	PRINT_REG_WITH_ID(I2C_TXFLR, i2c_id);
	PRINT_REG_WITH_ID(I2C_RXFLR, i2c_id);

	PRINT_REG_WITH_ID(I2C_TXABRT, i2c_id);
	PRINT_REG_WITH_ID(I2C_DMACR, i2c_id);
	PRINT_REG_WITH_ID(I2C_DMATDLR, i2c_id);
	PRINT_REG_WITH_ID(I2C_DMARDLR, i2c_id);
	PRINT_REG_WITH_ID(I2C_SDASU, i2c_id);
	PRINT_REG_WITH_ID(I2C_ACKGC, i2c_id);
	PRINT_REG_WITH_ID(I2C_ENSTA, i2c_id);
	PRINT_REG_WITH_ID(I2C_SDAHD, i2c_id);
}
#endif

//static inline unsigned short i2c_readl(struct jz_i2c *i2c,
unsigned short i2c_readl(struct jz_i2c *i2c,
					unsigned short offset)
{
#ifdef CONFIG_I2C_DEBUG
	if (offset == I2C_DC) {
		i2c->data_buf[i2c->cmd % BUFSIZE] += 1;
	}
#endif
	//return (*(volatile unsigned short *)(i2c->iomem + offset));
	return readl(i2c->iomem + offset);
}

static inline void i2c_writel(struct jz_i2c *i2c, unsigned short offset,
				unsigned short value)
{
	//(*(volatile unsigned short *) (i2c->iomem + offset)) = (value);
	writel(value, i2c->iomem + offset);
}

static inline unsigned short i2c_readw(struct jz_i2c *i2c,
					unsigned short offset)
{
	return readw(i2c->iomem + offset);
}

static inline void i2c_writew(struct jz_i2c *i2c, unsigned short offset,
				unsigned short value)
{
	writew(value, i2c->iomem + offset);
}

static inline unsigned short i2c_readb(struct jz_i2c *i2c,
					unsigned short offset)
{
	return readb(i2c->iomem + offset);
}

static inline void i2c_writeb(struct jz_i2c *i2c, unsigned short offset,
				unsigned short value)
{
	writeb(value, i2c->iomem + offset);
}

static inline void set_i2c0_internal_pull(void)
{
#if (defined(CONFIG_I2C0_V12_JZ))
    struct jz_i2c_gpios gpios = I2C0_PORTD;
    jzgpio_ctrl_pull(gpios.port, 1, gpios.pins);
#endif
}

static inline void set_i2c1_internal_pull(void)
{
#if (defined(CONFIG_I2C1_V12_JZ))
    struct jz_i2c_gpios gpios = I2C1_PORTE;
    jzgpio_ctrl_pull(gpios.port, 1, gpios.pins);
#endif
}

static inline void set_i2c2_internal_pull(void)
{
#if (defined(CONFIG_I2C2_V12_JZ))
    struct jz_i2c_gpios gpios =
#ifdef CONFIG_I2C2_PA
    I2C2_PORTA;
#else
    I2C2_PORTE;
#endif
    jzgpio_ctrl_pull(gpios.port, 1, gpios.pins);
#endif
}

static inline void set_i2c3_internal_pull(void)
{
#if (defined(CONFIG_I2C3_V12_JZ))
    struct jz_i2c_gpios gpios =
#ifdef CONFIG_I2C3_PB
    I2C3_PORTB;
#else
    I2C3_PORTC;
#endif
    jzgpio_ctrl_pull(gpios.port, 1, gpios.pins);
#endif
}

static int jz_i2c_disable(struct jz_i2c *i2c)
{
	int ret = 0;
	int timeout = TIMEOUT;

	i2c_writeb(i2c, I2C_ENB, 0);
	while ((i2c_readb(i2c, I2C_ENSTA) & I2C_ENB_I2CENB) && (--timeout > 0))
		msleep(1);

	if (!timeout) {
		dev_err(&(i2c->adap.dev),"disable i2c%d failed\n", i2c->adap.nr);
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int jz_i2c_enable(struct jz_i2c *i2c)
{
	int ret = 0;
	int timeout = TIMEOUT;

	i2c_writeb(i2c, I2C_ENB, 1);
	while (!(i2c_readb(i2c, I2C_ENSTA) & I2C_ENB_I2CENB) && (--timeout > 0))
		msleep(1);

	if (!timeout) {
		dev_err(&(i2c->adap.dev),"enable i2c%d failed\n", i2c->adap.nr);
		ret = -ETIMEDOUT;
	}

	return ret;
}

static void set_txemp_irq(struct jz_i2c *i2c)
{
	unsigned short	tmp;
	tmp = i2c_readw(i2c, I2C_INTM);
	tmp |= I2C_INTM_MTXEMP;
	i2c_writew(i2c, I2C_INTM, tmp);
}

static void clear_txemp_irq(struct jz_i2c *i2c)
{
	unsigned short	tmp;
	tmp = i2c_readw(i2c, I2C_INTM);
	tmp &= ~I2C_INTM_MTXEMP;
	i2c_writew(i2c, I2C_INTM, tmp);
}

static void set_rxfl_irq(struct jz_i2c *i2c)
{
	unsigned short	tmp;
	tmp = i2c_readw(i2c, I2C_INTM);
	tmp |= I2C_INTM_MRXFL;
	i2c_writew(i2c, I2C_INTM, tmp);
}

static void clear_rxfl_irq(struct jz_i2c *i2c)
{
	unsigned short	tmp;
	tmp = i2c_readw(i2c, I2C_INTM);
	tmp &= ~I2C_INTM_MRXFL;
	i2c_writew(i2c, I2C_INTM, tmp);
}

static void i2c_set_rxtl(struct jz_i2c *i2c, unsigned int trigger_level)
{
	i2c_writeb(i2c, I2C_RXTL, trigger_level - 1);
}

static void i2c_set_txtl(struct jz_i2c *i2c, unsigned int trigger_level)
{
	i2c_writeb(i2c, I2C_TXTL, trigger_level);
}

static void clear_stop_intr_flag(struct jz_i2c *i2c)
{
	i2c_readb(i2c, I2C_CSTP);
}

static void clear_abort_intr_flag(struct jz_i2c *i2c)
{
	i2c_readb(i2c, I2C_CTXABRT);
}

static void clear_soft_intr_flag(struct jz_i2c *i2c)
{
	i2c_readb(i2c, I2C_CINTR);
}

static void jz_i2c_reset(struct jz_i2c *i2c)
{
	clear_soft_intr_flag(i2c);

	jz_i2c_disable(i2c);
	udelay(10);
	jz_i2c_enable(i2c);

	i2c_writew(i2c, I2C_INTM, 0);
}

static void i2c_send_rcmd(struct jz_i2c *i2c, int cmd_count)
{
	int i;

	for (i = 0; i < cmd_count; i++) {
		i2c_writew(i2c, I2C_DC, I2C_DC_READ);
	}
	i2c->rdcmd_len -= cmd_count;
}

static void i2c_send_rcmd_with_stop(struct jz_i2c *i2c)
{
	i2c_writew(i2c, I2C_DC, I2C_DC_STP | I2C_DC_READ);
	i2c->rdcmd_len = 0;
}

static void i2c_send_rcmd_with_restart(struct jz_i2c *i2c)
{
	i2c_writew(i2c, I2C_DC, I2C_DC_RESTART | I2C_DC_READ);
	i2c->rdcmd_len -=1;
}

static void i2c_send_rcmd_with_stop_restart(struct jz_i2c *i2c)
{
	i2c_writew(i2c, I2C_DC, I2C_DC_RESTART | I2C_DC_STP | I2C_DC_READ);
	i2c->rdcmd_len = 0;
}

static void i2c_write_data(struct jz_i2c *i2c, int len)
{
	int i;
	unsigned short tmp;

	for (i = 0; i < len; i++) {
		tmp = *i2c->wbuf++ & 0xff;
		i2c_writew(i2c, I2C_DC, tmp);
	}
	i2c->wt_len -= len;
}

static void i2c_write_data_with_stop(struct jz_i2c *i2c)
{
	unsigned short data;

	data = *i2c->wbuf & 0xff;
	i2c_writew(i2c, I2C_DC, data | I2C_DC_STP);
	i2c->wt_len = 0;
}

static void i2c_write_data_with_restart(struct jz_i2c *i2c)
{
	unsigned short data;

	data = *i2c->wbuf++ & 0xff;
	i2c_writew(i2c, I2C_DC, data | I2C_DC_RESTART);
	i2c->wt_len -= 1 ;
}

static void i2c_write_data_with_stop_restart(struct jz_i2c *i2c)
{
	unsigned short data;

	data = *i2c->wbuf & 0xff;
	i2c_writew(i2c, I2C_DC, data | I2C_DC_STP | I2C_DC_RESTART);
	i2c->wt_len = 0;
}

static void i2c_read_more_buflen(struct jz_i2c *i2c) 
{	
	i2c->rx_level = TRANS_UNIT;
	i2c_set_rxtl(i2c, TRANS_UNIT);
	set_rxfl_irq(i2c);

	if (i2c->need_restart) {
		i2c_send_rcmd_with_restart(i2c);
		i2c_send_rcmd(i2c, I2C_FIFO_LEN - 1);
	} else {
		i2c_send_rcmd(i2c, I2C_FIFO_LEN);
	}
}

static void i2c_read_less_buflen(struct jz_i2c *i2c)
{
	if (i2c->need_stop) {
		if (i2c->need_restart) { 
			if (i2c->rdcmd_len == 1) {
				i2c_send_rcmd_with_stop_restart(i2c);
			}else {
				i2c_send_rcmd_with_restart(i2c);
				i2c_send_rcmd(i2c, i2c->rdcmd_len - 1);
				i2c_send_rcmd_with_stop(i2c);
			}
		} else {
			i2c_send_rcmd(i2c, i2c->rdcmd_len - 1);
			i2c_send_rcmd_with_stop(i2c);
		}
	} else {
			i2c->rx_level = i2c->rd_len;
			i2c_set_rxtl(i2c, i2c->rx_level);
			i2c->read_finsh = 1;
			set_rxfl_irq(i2c);
			
			if (i2c->need_restart) {
				i2c_send_rcmd_with_restart(i2c);
			}
			i2c_send_rcmd(i2c, i2c->rdcmd_len);
		}
}

static void i2c_write_more_buflen(struct jz_i2c *i2c) 
{	
	i2c_set_txtl(i2c, TRANS_UNIT);
	if (i2c->need_restart) {
		i2c_write_data_with_restart(i2c);
		i2c_write_data(i2c, I2C_FIFO_LEN - 1);
	} else {
		i2c_write_data(i2c, I2C_FIFO_LEN);
	}
	set_txemp_irq(i2c);
}

static void i2c_write_less_buflen(struct jz_i2c *i2c)
{
	if (i2c->need_stop) {
		if (i2c->need_restart) { 
			if (i2c->wt_len == 1) {
				i2c_write_data_with_stop_restart(i2c);
			} else {
				i2c_write_data_with_restart(i2c);	
				i2c_write_data(i2c, i2c->wt_len - 1);
				i2c_write_data_with_stop(i2c);
			}
		} else {
			i2c_write_data(i2c, i2c->wt_len - 1);
			i2c_write_data_with_stop(i2c);
		}
	} else {
			i2c_set_txtl(i2c, 0);
			i2c->write_finsh = 1;
			if (i2c->need_restart) {
				i2c_write_data_with_restart(i2c);	
			}
			i2c_write_data(i2c, i2c->wt_len);
			set_txemp_irq(i2c);
	}
}

static void handle_txabt_irq(struct jz_i2c *i2c)
{
	i2c_writew(i2c, I2C_INTM, 0);

	i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);

	clear_stop_intr_flag(i2c);
	clear_abort_intr_flag(i2c);

	if (i2c->operate == M_READ ) {
		complete(&i2c->r_complete);
	} else {
		complete(&i2c->w_complete);
	}
}

static void handle_stop_irq(struct jz_i2c *i2c)
{
	unsigned short tmp;

	i2c_writew(i2c, I2C_INTM, 0);

	i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);

	clear_stop_intr_flag(i2c);
	clear_abort_intr_flag(i2c);

	switch (i2c->operate) {
		case M_READ:
			while (i2c->rd_len > 0) {
				tmp = i2c_readw(i2c, I2C_DC) & 0xff;
				*i2c->rbuf++ = tmp;
				i2c->rd_len--;
			}
			complete(&i2c->r_complete);
			break;

		case M_WRITE:
			complete(&i2c->w_complete);
			break;

		default:
			dev_info(&(i2c->adap.dev),"the operation is %d",
					i2c->operate);
			break;
	}
}

static void handle_rxfl_irq(struct jz_i2c *i2c)
{
	unsigned short tmp;
	int level_count = i2c->rx_level;	

	while (level_count > 0) {
		tmp = i2c_readw(i2c, I2C_DC) & 0xff;
		*i2c->rbuf++ = tmp;
		i2c->rd_len--;
		level_count--;
	}
	if (i2c->read_finsh) {
		i2c_writew(i2c, I2C_INTM, 0);
		i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
		complete(&i2c->r_complete);
	} else if (i2c->rdcmd_len > TRANS_UNIT) {
		i2c_send_rcmd(i2c, TRANS_UNIT);
	} else {
		if (i2c->need_stop) {
			clear_rxfl_irq(i2c);
			i2c_send_rcmd(i2c, i2c->rdcmd_len - 1);
			i2c_send_rcmd_with_stop(i2c);
		} else {
			i2c->rx_level = i2c->rd_len;
			i2c_set_rxtl(i2c, i2c->rx_level);
			i2c->read_finsh = 1;
			i2c_send_rcmd(i2c, i2c->rd_len);
		}
	}
}

static void handle_txemp_irq(struct jz_i2c *i2c)
{
	if (i2c->write_finsh) {
		i2c_writew(i2c, I2C_INTM, 0);
		i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
		complete(&i2c->w_complete);
	} else if (i2c->wt_len > TRANS_UNIT) {
		i2c_write_data(i2c, TRANS_UNIT);
	} else {
		if (i2c->need_stop) {
			clear_txemp_irq(i2c);
			i2c_write_data(i2c, i2c->wt_len - 1);
			i2c_write_data_with_stop(i2c);
		} else {
			i2c_set_txtl(i2c, 0);
			i2c->write_finsh = 1;
			i2c_write_data(i2c, i2c->wt_len);
		}	
	}
}

static irqreturn_t jz_i2c_irq(int irqno, void *dev_id)
{
	unsigned short pending;
	struct jz_i2c *i2c = dev_id;

	pending = i2c_readw(i2c, I2C_INTST);

	if (likely(pending & I2C_INTST_ISTP)) {
		handle_stop_irq(i2c);
	} else if (pending & I2C_INTST_RXFL) {
		handle_rxfl_irq(i2c);
	} else if (pending & I2C_INTST_TXEMP) {
		handle_txemp_irq(i2c);
	} else if (pending & I2C_INTST_TXABT) {
		handle_txabt_irq(i2c);
	}

	return IRQ_HANDLED;
}

static void txabrt(struct jz_i2c *i2c, int src)
{
#ifdef CONFIG_I2C_DEBUG
	int i;

	dev_err(&(i2c->adap.dev), "--I2C txabrt:\n");

	dev_err(&(i2c->adap.dev), "--I2C device addr=%x\n",
		i2c_readw(i2c, I2C_TAR));
	dev_err(&(i2c->adap.dev), "--I2C send cmd count:%d	%d\n",
		i2c->cmd, i2c->cmd_buf[i2c->cmd]);
	dev_err(&(i2c->adap.dev), "--I2C receive data count:%d	%d\n",
		i2c->cmd, i2c->data_buf[i2c->cmd]);
	jz_dump_i2c_regs(i2c);

	for (i = 0; i < 16; i++) {
		if (src & (0x1 << i))
			dev_info(&(i2c->adap.dev), "--I2C TXABRT[%d]=%s\n",
					i, abrt_src[i]);
	}
#endif

	return;
}

static int i2c_check_error(struct jz_i2c *i2c)
{
	int ret = 0;

	if (i2c->abort_src) {
		txabrt(i2c, i2c->abort_src);
		if (i2c->abort_src > 0x1 && i2c->abort_src < 0x10)
			ret = -ENXIO;
		else
			ret = -EIO;
		if (i2c->abort_src & 8) {
			ret = -EAGAIN;
		}
	}

	return ret;
}

static int handle_txabt_noirq(struct jz_i2c *i2c)
{
	i2c_writew(i2c, I2C_INTM, 0);

	i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);

	clear_stop_intr_flag(i2c);
	clear_abort_intr_flag(i2c);

	return 1;
}

static int handle_stop_noirq(struct jz_i2c *i2c)
{
	unsigned short tmp;

	i2c_writew(i2c, I2C_INTM, 0);

	i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);

	clear_stop_intr_flag(i2c);
	clear_abort_intr_flag(i2c);

	switch (i2c->operate) {
		case M_READ:
			while (i2c->rd_len > 0) {
				tmp = i2c_readw(i2c, I2C_DC) & 0xff;
				*i2c->rbuf++ = tmp;
				i2c->rd_len--;
			}
			break;

		case M_WRITE:
			break;

		default:
			dev_info(&(i2c->adap.dev),"the operation is %d",
					i2c->operate);
			break;
	}
	return 1;
}

static int handle_rxfl_noirq(struct jz_i2c *i2c)
{
	unsigned short tmp;
	int level_count = i2c->rx_level;

	while (level_count > 0) {
		tmp = i2c_readw(i2c, I2C_DC) & 0xff;
		*i2c->rbuf++ = tmp;
		i2c->rd_len--;
		level_count--;
	}
	if (i2c->read_finsh) {
		i2c_writew(i2c, I2C_INTM, 0);
		i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
		return 1;
	} else if (i2c->rdcmd_len > TRANS_UNIT) {
		i2c_send_rcmd(i2c, TRANS_UNIT);
	} else {
		if (i2c->need_stop) {
			clear_rxfl_irq(i2c);
			i2c_send_rcmd(i2c, i2c->rdcmd_len - 1);
			i2c_send_rcmd_with_stop(i2c);
		} else {
			i2c->rx_level = i2c->rd_len;
			i2c_set_rxtl(i2c, i2c->rx_level);
			i2c->read_finsh = 1;
			i2c_send_rcmd(i2c, i2c->rd_len);
		}
	}
	return 0;
}

static int handle_txemp_noirq(struct jz_i2c *i2c)
{
	if (i2c->write_finsh) {
		i2c_writew(i2c, I2C_INTM, 0);
		i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
		return 1;
	} else if (i2c->wt_len > TRANS_UNIT) {
		i2c_write_data(i2c, TRANS_UNIT);
	} else {
		if (i2c->need_stop) {
			clear_txemp_irq(i2c);
			i2c_write_data(i2c, i2c->wt_len - 1);
			i2c_write_data_with_stop(i2c);
		} else {
			i2c_set_txtl(i2c, 0);
			i2c->write_finsh = 1;
			i2c_write_data(i2c, i2c->wt_len);
		}
	}
	return 0;
}

static int xfer_read_noirq(struct jz_i2c *i2c, unsigned char * const buf,
			const int len)
{
	int ret = 0;
	int timeout = 50;
	unsigned short pending;

	i2c->rd_len = len;
	i2c->rdcmd_len = len;
	i2c->rbuf = buf;

	i2c->operate = M_READ;
	i2c->read_finsh = 0;

	i2c_writew(i2c, I2C_INTM, I2C_INTM_MTXABT | I2C_INTM_MISTP);

	if (i2c->rd_len > I2C_FIFO_LEN) {
		i2c_read_more_buflen(i2c);
	} else {
		i2c_read_less_buflen(i2c);
	}

	udelay(20);

	while (1) {
		pending = i2c_readw(i2c, I2C_INTST);

		if (likely(pending & I2C_INTST_ISTP)) {
			ret = handle_stop_noirq(i2c);
		} else if (pending & I2C_INTST_RXFL) {
			ret = handle_rxfl_noirq(i2c);
		} else if (pending & I2C_INTST_TXEMP) {
			ret = handle_txemp_noirq(i2c);
		} else if (pending & I2C_INTST_TXABT) {
			ret = handle_txabt_noirq(i2c);
		} else {
			if (--timeout == 0) {
				ret = -ETIMEDOUT;
				i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
				dev_err(&(i2c->adap.dev), "--I2C irq read timeout\n");
#ifdef CONFIG_I2C_DEBUG
				dev_err(&(i2c->adap.dev), "--I2C send cmd count:%d	%d\n",
						i2c->cmd, i2c->cmd_buf[i2c->cmd]);
				dev_err(&(i2c->adap.dev), "--I2C receive data count:%d	%d\n",
						i2c->cmd, i2c->data_buf[i2c->cmd]);
				jz_dump_i2c_regs(i2c);
#endif
				break;
			}
		}

		timeout = 50;
		if (ret)
			break;

		mdelay(1);
	}

	ret = i2c_check_error(i2c);
	if (ret < 0) {
		jz_i2c_reset(i2c);
	}

	return ret;
}

static int xfer_write_noirq(struct jz_i2c *i2c, unsigned char * const buf,
			const int len)
{
	int ret = 0;
	int timeout = 50;
	unsigned short pending;

	i2c->wt_len = len;
	i2c->wbuf = buf;
	i2c->operate = M_WRITE;
	i2c->write_finsh = 0;

	i2c_writew(i2c, I2C_INTM, I2C_INTM_MTXABT | I2C_INTM_MISTP);

	if (i2c->wt_len > I2C_FIFO_LEN) {
		i2c_write_more_buflen(i2c);
	} else {
		i2c_write_less_buflen(i2c);
	}

	udelay(20);

	while (1) {
		pending = i2c_readw(i2c, I2C_INTST);

		if (likely(pending & I2C_INTST_ISTP)) {
			ret = handle_stop_noirq(i2c);
		} else if (pending & I2C_INTST_RXFL) {
			ret = handle_rxfl_noirq(i2c);
		} else if (pending & I2C_INTST_TXEMP) {
			ret = handle_txemp_noirq(i2c);
		} else if (pending & I2C_INTST_TXABT) {
			ret = handle_txabt_noirq(i2c);
		} else {
			if (--timeout == 0) {
				ret = -ETIMEDOUT;
				i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
				dev_err(&(i2c->adap.dev), "--I2C irq write timeout\n");
#ifdef CONFIG_I2C_DEBUG
				dev_err(&(i2c->adap.dev), "--I2C send cmd count:%d	%d\n",
						i2c->cmd, i2c->cmd_buf[i2c->cmd]);
				dev_err(&(i2c->adap.dev), "--I2C receive data count:%d	%d\n",
						i2c->cmd, i2c->data_buf[i2c->cmd]);
				jz_dump_i2c_regs(i2c);
#endif
				break;
			}
		}

		timeout = 50;
		if (ret)
			break;

		mdelay(1);
	}

	ret = i2c_check_error(i2c);
	if (ret < 0) {
		jz_i2c_reset(i2c);
	}

	return ret;
}


static int xfer_read(struct jz_i2c *i2c, unsigned char * const buf,
			const int len)
{
	int ret = 0;
	long timeout;
	unsigned int wait_complete_timeout_ms;

	wait_complete_timeout_ms = len * 1000 * 9 * 2 / i2c->bus_speed
			 + CONFIG_I2C_JZ4775_WAIT_MS;

	i2c->rd_len = len;
	i2c->rdcmd_len = len;
	i2c->rbuf = buf;
	
	i2c->operate = M_READ;
	i2c->read_finsh = 0;

	i2c_writew(i2c, I2C_INTM, I2C_INTM_MTXABT | I2C_INTM_MISTP);

	if (i2c->rd_len > I2C_FIFO_LEN) {
		i2c_read_more_buflen(i2c);
	} else {
		i2c_read_less_buflen(i2c);
	}

	timeout = wait_for_completion_timeout(&i2c->r_complete,
			msecs_to_jiffies(wait_complete_timeout_ms));
	if (!timeout) {
		dev_err(&(i2c->adap.dev), "--I2C irq read timeout\n");
#ifdef CONFIG_I2C_DEBUG
		dev_err(&(i2c->adap.dev), "--I2C send cmd count:%d	%d\n",
				i2c->cmd, i2c->cmd_buf[i2c->cmd]);
		dev_err(&(i2c->adap.dev), "--I2C receive data count:%d	%d\n",
				i2c->cmd, i2c->data_buf[i2c->cmd]);
		jz_dump_i2c_regs(i2c);
#endif
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = i2c_check_error(i2c);
	if (ret < 0) {
		jz_i2c_reset(i2c);
	}
out:
	return ret;
}

static int xfer_write(struct jz_i2c *i2c, unsigned char * const buf,
			const int len)
{
	int ret = 0;
	long timeout = TIMEOUT;
	unsigned int wait_complete_timeout_ms;

	wait_complete_timeout_ms = len * 1000 * 9 * 2 / i2c->bus_speed
				+ CONFIG_I2C_JZ4775_WAIT_MS;

	i2c->wt_len = len;
	i2c->wbuf = buf;
	i2c->operate = M_WRITE;
	i2c->write_finsh = 0;

	i2c_writew(i2c, I2C_INTM, I2C_INTM_MTXABT | I2C_INTM_MISTP);

	if (i2c->wt_len > I2C_FIFO_LEN) {
		i2c_write_more_buflen(i2c);
	} else {
		i2c_write_less_buflen(i2c);
	}

	timeout = wait_for_completion_timeout(&i2c->w_complete,
			msecs_to_jiffies(wait_complete_timeout_ms));
	if (!timeout) {
		dev_err(&(i2c->adap.dev), "--I2C irq write timeout\n");
#ifdef CONFIG_I2C_DEBUG
		dev_err(&(i2c->adap.dev), "--I2C send cmd count:%d	%d\n",
				i2c->cmd, i2c->cmd_buf[i2c->cmd]);
		dev_err(&(i2c->adap.dev), "--I2C receive data count:%d	%d\n",
				i2c->cmd, i2c->data_buf[i2c->cmd]);
		jz_dump_i2c_regs(i2c);
#endif
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = i2c_check_error(i2c);
	if (ret < 0) {
		jz_i2c_reset(i2c);
	}

out:
	return ret;
}

static int disable_i2c_clk(struct jz_i2c *i2c)
{
	int ret = 0;
	int timeout = 10;
	unsigned short tmp;

	tmp = i2c_readb(i2c, I2C_STA);
	while ((tmp & I2C_STA_MSTACT) && (--timeout > 0)) {
		udelay(90);
		tmp = i2c_readb(i2c, I2C_STA);
	}

	if (timeout != 10) {
		dev_info(&(i2c->adap.dev),"timeout = %d\n", timeout);
	}

	if (timeout > 0) {
		clk_disable(i2c->clk);
	} else {
		dev_err(&(i2c->adap.dev),
			"--I2C disable clk timeout, I2C_STA = 0x%x\n", tmp);
		jz_i2c_reset(i2c);
		clk_disable(i2c->clk);
		ret = -ETIMEDOUT;
	}

	return ret;
}

static void set_master_addr_type(struct jz_i2c *i2c,
				enum i2c_address_type address_type)
{
	unsigned short tmp;

	tmp = i2c_readw(i2c, I2C_TAR);
	if (address_type == ADDR_7BIT) {
		tmp &= ~I2C_TAR_MATP;
	} else {
		tmp |= I2C_TAR_MATP;
	}
	i2c_writew(i2c, I2C_TAR, tmp);
}

static void set_slave_addr_type(struct jz_i2c *i2c,
				enum i2c_address_type address_type)
{
	unsigned short tmp;

	tmp = i2c_readb(i2c, I2C_CTRL);
	if (address_type == ADDR_7BIT) {
		tmp &= ~I2C_CTRL_SATP;
	} else {
		tmp |= I2C_CTRL_SATP;
	}
	i2c_writeb(i2c, I2C_CTRL, tmp);
}

#define I2C_ADDR_MASK	0x3FF	/* 10-bit address */
static void set_target_addr(struct jz_i2c *i2c, struct i2c_msg *msg)
{
	unsigned short tmp;

	tmp = i2c_readw(i2c, I2C_TAR);
	if (msg->addr != (tmp & I2C_ADDR_MASK)) {
		tmp &= ~I2C_ADDR_MASK;
		tmp |= msg->addr;
		i2c_writew(i2c, I2C_TAR, tmp);
	}
}

static int i2c_jz_xfer(struct i2c_adapter *adap, struct i2c_msg *msg, int count)
{
	int i, ret = 0;
	struct jz_i2c *i2c = adap->algo_data;

	mutex_lock(&i2c->lock);

	/**
	 * In suspend       i2c->is_suspend = 1;         i2c xfer ops will be change to noirq way.
	 * In suspend_noirq i2c->block_other_thread = 1; i2c xfer will block all non-suspend-thread's access.
	 * In resume_noirq  i2c->block_other_thread = 0; non-suspend-thread can do i2c xfer.
	 * In reume         i2c->is_suspend = 0;         i2c xfer ops will be change to irq way.
	 *
	 * 1 - avoid i2c time out erorr, it cause i2c controller can not reset by call jz_i2c_reset().
	 * after dpm_suspend() be executed, suspend_device_irqs() will be executed,
	 * then i2c's irq is disabled. so we use is_suspend to ctrl the xfer way (irq/noirq),
	 * to avoid the irq be disabled when i2c is xfering (it cause i2c xfer timeout)
	 *
	 * 2 - avoid half-baked i2c xfer, or i2c is still xfering when cpu entering suspend.
	 * after dpm_suspend_noirq() be executed, arch_suspend_disable_irqs() will be executed,
	 * then only suspend thread is running, so we use block_other_thread to block non-suspend-thread's i2c
	 * access, to avoid the thread be stoped when i2c is xfering.
	 *
	 */
	if (i2c->block_other_thread && (task_pid_nr(current) != i2c->suspend_thread_pid)) {
		while (1) {
			pr_info("!!!!%s: [%d] is not the suspend thread [%d], waiting for resume!!!\n",
					i2c->adap.name, task_pid_nr(current) , i2c->suspend_thread_pid);
			mutex_unlock(&i2c->lock);
			msleep(10);
			mutex_lock(&i2c->lock);
			if (!i2c->block_other_thread) {
				pr_info("!!!%s: now can do xfer!!!\n", i2c->adap.name);
				break;
			}
		}
	}

	clk_enable(i2c->clk);

	if (msg->flags == I2C_M_TEN) {
		set_master_addr_type(i2c, ADDR_10BIT);
	}

	set_target_addr(i2c, msg);
	
	if (i2c_readw(i2c, I2C_RAW_INTST) & I2C_INTST_ISTP) {
		dev_info(&(i2c->adap.dev),"warning: i2c%d unexpected stop irq\n",
				i2c->adap.nr);
		clear_stop_intr_flag(i2c);
	}

	for (i = 0; i < count; i++, msg++) {
		if (i == 0) {
			i2c->need_restart = 0;
		} else {
			i2c->need_restart = 1;
		}
		if (i == count - 1) {
			i2c->need_stop = 1;
		} else {
			i2c->need_stop = 0;
		}

		/**
		 * if i2c is suspended, we use noirq operation to xfer data.
		 */
		if (msg->flags & I2C_M_RD) {
			ret = unlikely(i2c->is_suspend) ? xfer_read_noirq(i2c, msg->buf, msg->len)
				: xfer_read(i2c, msg->buf, msg->len);
		} else {
			ret = unlikely(i2c->is_suspend) ? xfer_write_noirq(i2c, msg->buf, msg->len)
				: xfer_write(i2c, msg->buf, msg->len);
		}
		if (ret < 0) {
			clk_disable(i2c->clk);
			goto out;
		}
	}

	if (disable_i2c_clk(i2c) < 0) {
		clk_disable(i2c->clk);
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = i;

out:
	mutex_unlock(&i2c->lock);
	return ret;
}

static u32 i2c_jz_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_jz_algorithm = {
	.master_xfer	= i2c_jz_xfer,
	.functionality	= i2c_jz_functionality,
};

static void i2c_init_controller(struct jz_i2c *i2c, enum i2c_address_type addr_type,
				enum i2c_device_type dev_type)
{
	unsigned int ctrl;

	if (jz_i2c_disable(i2c) < 0) {
		dev_err(&(i2c->adap.dev), "i2c is not disabled\n");
		clk_disable(i2c->clk);
	}

	ctrl = i2c_readb(i2c, I2C_CTRL);
	ctrl |= I2C_CTRL_RESTART;

	if (dev_type == MASTER) {
		ctrl |= I2C_CTRL_MD | I2C_CTRL_SLVDIS;
		i2c_writeb(i2c, I2C_CTRL, ctrl);
		set_master_addr_type(i2c, addr_type);
	} else {
		ctrl &= ~(I2C_CTRL_MD | I2C_CTRL_SLVDIS);
		i2c_writeb(i2c, I2C_CTRL, ctrl);
		set_slave_addr_type(i2c, addr_type);
	}

	i2c_writeb(i2c, I2C_INTM, 0x0);

	i2c_set_txtl(i2c, 0);
	i2c_set_rxtl(i2c, I2C_FIFO_LEN);
	clear_soft_intr_flag(i2c);
}

#define SM_SPEED	( 100 * 1000)	/* Hz, kbits/sec */
#define FM_SPEED	( 400 * 1000)	/* Hz. kbits/sec */
#define HS_SPEED	(3400 * 1000)	/* Hz. kbits/sec */

static int i2c_set_clock(struct jz_i2c *i2c, int bus_speed)
{
	int ret = 0;
	unsigned int tmp;
	unsigned long device_clock, cycle, period;

	if (jz_i2c_disable(i2c) < 0) {
		dev_err(&(i2c->adap.dev), "i2c not disable\n");
		clk_disable(i2c->clk);
	}
	i2c->bus_speed = bus_speed;

	tmp = i2c_readb(i2c, I2C_CTRL);
	if (bus_speed <= SM_SPEED) {
		tmp |= I2C_CTRL_SPDS;
		tmp &= ~I2C_CTRL_SPDF;
	} else if (bus_speed <= FM_SPEED) {
		tmp &= ~I2C_CTRL_SPDS;
		tmp |= I2C_CTRL_SPDF;
	} else {
		dev_err(&(i2c->adap.dev), "unsupport i2c speed mode %d\n",
				bus_speed);
		ret = -EINVAL;
		goto out;
	}
	i2c_writeb(i2c, I2C_CTRL, tmp);

	device_clock = clk_get_rate(i2c->clk); /* Hz. */
	if (device_clock == 0) {
		dev_err(&(i2c->adap.dev), "please check i2c device clock config\n");
		device_clock = 100 * 1000 * 1000;
	}

	tmp = device_clock / bus_speed / 2; /* duty ratio is 1:1*/
	if (bus_speed <= SM_SPEED) {
		i2c_writew(i2c, I2C_SHCNT, I2CSHCNT_ADJUST(tmp));
		i2c_writew(i2c,I2C_SLCNT, I2CSLCNT_ADJUST(tmp));
	} else if (bus_speed <= FM_SPEED) {
		i2c_writew(i2c, I2C_FHCNT, I2CFHCNT_ADJUST(tmp));
		i2c_writew(i2c, I2C_FLCNT, I2CFLCNT_ADJUST(tmp));
	}

	period = 1000000000 / device_clock; /* ns, nanosecond */


#define MIN_SETUP_TIME	250 /* ns, nanosecond */
#define I2C_SDASU_MAX	0xFF

	cycle = MIN_SETUP_TIME / period + 1;
	if (cycle > I2C_SDASU_MAX) {
		cycle = I2C_SDASU_MAX;
	}
	i2c_writeb(i2c, I2C_SDASU, cycle);

#define MIN_HOLD_TIME	300 /* ns, nanosecond */
#define I2C_SDAHD_MAX	0xFFFF

	cycle = MIN_HOLD_TIME / period;
	if (cycle > I2C_SDAHD_MAX) {
		cycle = I2C_SDAHD_MAX;
	}
	i2c_writew(i2c, I2C_SDAHD, cycle);

#define MAX_SPIKE_PULSE		50 /*  ns, nanosecond */
#define TYPICAL_SPIKE_PULSE	(MAX_SPIKE_PULSE/2)

	cycle = TYPICAL_SPIKE_PULSE / period;
	i2c_writeb(i2c, I2C_FLT, cycle);

out:
	return ret;
}

static int i2c_jz_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct jz_i2c *i2c = platform_get_drvdata(pdev);

	mutex_lock(&i2c->lock);
	i2c->is_suspend = 1;
	disable_irq(i2c->irq);
	i2c->suspend_thread_pid = task_pid_nr(current);
	mutex_unlock(&i2c->lock);

	return 0;
}

static int i2c_jz_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct jz_i2c *i2c = platform_get_drvdata(pdev);

	mutex_lock(&i2c->lock);
	i2c->is_suspend = 0;
	enable_irq(i2c->irq);
	mutex_unlock(&i2c->lock);

	return 0;
}

int i2c_jz_suspend_noirq(struct device *dev) {
	struct platform_device *pdev = to_platform_device(dev);
	struct jz_i2c *i2c = platform_get_drvdata(pdev);

	mutex_lock(&i2c->lock);
	i2c->block_other_thread = 1;
	mutex_unlock(&i2c->lock);

	return 0;
}

int i2c_jz_resume_noirq(struct device *dev) {
	struct platform_device *pdev = to_platform_device(dev);
	struct jz_i2c *i2c = platform_get_drvdata(pdev);

	mutex_lock(&i2c->lock);
	i2c->block_other_thread = 0;
	mutex_unlock(&i2c->lock);

	return 0;
}

const struct dev_pm_ops i2c_jz_pm_ops = {
	.suspend = i2c_jz_suspend,
	.resume = i2c_jz_resume,
	.suspend_noirq = i2c_jz_suspend_noirq,
	.resume_noirq = i2c_jz_resume_noirq,
};

static int i2c_jz_probe(struct platform_device *dev)
{
	int ret = 0;
	struct jz_i2c *i2c;
	struct resource *res;

	i2c = kzalloc(sizeof(struct jz_i2c), GFP_KERNEL);
	if (i2c == NULL) {
		ret = -ENOMEM;
		goto no_mem;
	}

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &i2c_jz_algorithm;
	i2c->adap.retries = 5;
	i2c->adap.timeout = 5;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &dev->dev;
	i2c->adap.nr = dev->id;
	mutex_init(&i2c->lock);
	sprintf(i2c->adap.name, "i2c%u", dev->id);

	i2c->clk = clk_get(&dev->dev, i2c->adap.name);
	if (i2c->clk == NULL) {
		ret = -ENODEV;
		goto clk_failed;
	}

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);

	i2c->iomem = ioremap(res->start, resource_size(res));
	if (i2c->iomem == NULL) {
		ret = -ENOMEM;
		goto io_failed;
	}

	init_completion(&i2c->r_complete);
	init_completion(&i2c->w_complete);

	res = platform_get_resource(dev, IORESOURCE_BUS, 0);
	i2c->bus_speed = res->start * 1000;

	clk_enable(i2c->clk);

	i2c_init_controller(i2c, ADDR_7BIT, MASTER);
	i2c_set_clock(i2c, i2c->bus_speed);

	platform_set_drvdata(dev, i2c);

	jz_i2c_enable(i2c);

	i2c->irq = platform_get_irq(dev, 0);
	ret = request_irq(i2c->irq, jz_i2c_irq, IRQF_DISABLED,
			dev_name(&dev->dev), i2c);
	if (ret < 0) {
		goto irq_failed;
	}

	clk_disable(i2c->clk);

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&(i2c->adap.dev), KERN_INFO "I2C: Failed to add bus\n");
		goto adapt_failed;
	}

    if (i2c->adap.nr == 0) {
#ifdef CONFIG_I2C0_INTERNAL_PULL
        set_i2c0_internal_pull();
#endif
    }

    if (i2c->adap.nr == 1) {
#ifdef CONFIG_I2C1_INTERNAL_PULL
        set_i2c1_internal_pull();
#endif
    }

    if (i2c->adap.nr == 2) {
#ifdef CONFIG_I2C2_INTERNAL_PULL
        set_i2c2_internal_pull();
#endif
    }

    if (i2c->adap.nr == 3) {
#ifdef CONFIG_I2C3_INTERNAL_PULL
        set_i2c3_internal_pull();
#endif
    }

#ifdef CONFIG_I2C_DEBUG
	i2c->cmd = 0;
	memset(i2c->cmd_buf, 0, BUFSIZE);
	memset(i2c->data_buf, 0, BUFSIZE);
#endif

	i2c->is_suspend = 0;
	i2c->block_other_thread = 0;

	return ret;

adapt_failed:
	free_irq(i2c->irq, i2c);
irq_failed:
	iounmap(i2c->iomem);
io_failed:
	clk_put(i2c->clk);
clk_failed:
	kfree(i2c);
no_mem:
	return ret;
}

static int i2c_jz_remove(struct platform_device *dev)
{
	struct jz_i2c *i2c = platform_get_drvdata(dev);

	free_irq(i2c->irq, i2c);
	iounmap(i2c->iomem);
	clk_put(i2c->clk);
	i2c_del_adapter(&i2c->adap);
	kfree(i2c);

	return 0;
}

static struct platform_driver jz_i2c_driver = {
	.probe	= i2c_jz_probe,
	.remove	= i2c_jz_remove,
	.driver	= {
		.name = "jz-i2c",
		.pm = &i2c_jz_pm_ops,
	},
};

static int __init jz4775_i2c_init(void)
{
	return platform_driver_register(&jz_i2c_driver);
}

static void __exit jz4775_i2c_exit(void)
{
	platform_driver_unregister(&jz_i2c_driver);
}

subsys_initcall(jz4775_i2c_init);
module_exit(jz4775_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("skzhang<skzhang@ingenic.cn>");
MODULE_DESCRIPTION("i2c driver for JZ47XX SoCs");

static int i2c_read_noirq(struct i2c_client *client, int len, unsigned char *buf)
{
	int i;
	int tmp = 0;
	int rd_len = len;
	struct jz_i2c *i2c = client->adapter->algo_data;
	unsigned int wait_timeout_ms = len * 1000 * 9 * 2 / i2c->bus_speed
					+ CONFIG_I2C_JZ4775_WAIT_MS;
	if (rd_len > 64) {
		dev_info(&(i2c->adap.dev),
			"warning:the length is greater than the read range of function\n");
		tmp = -EMSGSIZE; /*message too long*/
		goto out;
	}

	for (i = 0; i < len; i++) {
		if (i == len - 1)
			i2c_writew(i2c, I2C_DC, I2C_DC_STP | I2C_DC_READ);
		else
			i2c_writew(i2c, I2C_DC, I2C_DC_READ);
	}

	while ((rd_len > 0) && (wait_timeout_ms > 0)) {
		if (i2c_readw(i2c, I2C_RAW_INTST) & I2C_INTST_TXABT) {
			i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
			tmp = i2c_check_error(i2c);
			break;
		}

		if ((i2c_readb(i2c, I2C_STA) & I2C_STA_RFNE)) {
			*buf++ = i2c_readw(i2c, I2C_DC) & 0xff;
			rd_len--;
		} else {
			mdelay(1);
			wait_timeout_ms--;
		}
	}

	if (wait_timeout_ms <= 0) {
		dev_info(&(i2c->adap.dev),"%s read: wait timeout\n",__func__);
		tmp = -ETIMEDOUT;
	}

out:
	return tmp;
}

static int i2c_write_noirq(struct i2c_client *client, int len, unsigned char *buf)
{
	int i;
	int tmp = 0;
	struct jz_i2c *i2c = client->adapter->algo_data;
	unsigned int wait_timeout_ms = len * 1000 * 9 * 2 / i2c->bus_speed
					+ CONFIG_I2C_JZ4775_WAIT_MS;
	if (len > 64) {
		dev_info(&(i2c->adap.dev),
			"warning:the length is greater than the write range of function\n");
		tmp = -EMSGSIZE; /*message too long*/
		goto out;
	}

	for (i = 0; i < len; i++) {
		if (i == len - 1)
			i2c_writew(i2c, I2C_DC, I2C_DC_STP | *buf++);
		else
			i2c_writew(i2c, I2C_DC, *buf++);
	}

	while (!(i2c_readb(i2c, I2C_STA) & I2C_STA_TFE)) {
		if (i2c_readw(i2c, I2C_RAW_INTST) & I2C_INTST_TXABT) {
			i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
			tmp = i2c_check_error(i2c);
			break;
		}
		if (wait_timeout_ms > 0) {
			wait_timeout_ms--;
			mdelay(1);
		} else {
			dev_info(&(i2c->adap.dev),"%s status:0x%x\n",__func__,
					i2c_readb(i2c, I2C_STA));
			tmp = -ETIMEDOUT;
			break;
		}
	}

out:
	return tmp;
}

/**
 *i2c_read_noirq_less_buflen -return error value if occured error
 *@client:the client being refenced
 *@len:length of read data
 *@buf:store the read data
 
 *read data without irq,but len <= 64
 */
int i2c_read_noirq_less_buflen(struct i2c_client *client, int len,
				unsigned char *buf)
{
	int tmp = 0;
	struct jz_i2c *i2c = client->adapter->algo_data;

	mutex_lock(&i2c->lock);
	clk_enable(i2c->clk);

	i2c_writew(i2c, I2C_INTM, 0x0);
	i2c_writew(i2c, I2C_TAR, client->addr);

	tmp = i2c_read_noirq(client, len, buf);

	i2c_readb(i2c, I2C_CSTP);
	disable_i2c_clk(i2c);
	mutex_unlock(&i2c->lock);

	return tmp;
}
EXPORT_SYMBOL_GPL(i2c_read_noirq_less_buflen);

/**
 *i2c_write_noirq_less_buflen -return error value if occured error
 *@client:the client being refenced
 *@len:length of write data
 *@buf:store the wrtie data
 
 *write data without irq,but len <= 64
 */
int i2c_write_noirq_less_buflen(struct i2c_client *client, int len,
				unsigned char *buf)
{
	int tmp = 0;
	struct jz_i2c *i2c = client->adapter->algo_data;

	mutex_lock(&i2c->lock);
	clk_enable(i2c->clk);

	i2c_writew(i2c, I2C_INTM, 0x0);
	i2c_writew(i2c, I2C_TAR, client->addr);

	tmp = i2c_write_noirq(client, len, buf);

	i2c_readb(i2c, I2C_CSTP); /*clear stop irq*/
	disable_i2c_clk(i2c);
	mutex_unlock(&i2c->lock);

	return tmp;
}
EXPORT_SYMBOL_GPL(i2c_write_noirq_less_buflen);

/*func: transfer one or more messeges when no irq,but msg->len < 64*/
/**
 * i2c_transfer_msgs_noirq -return the transmit messges counts or error value
 *@client:the client being refenced
 *@msgs: One or more messages to execute before STOP is issued to
 *		terminate the operation; each message begins with a START.
 *@count: Number of messages to be executed.

 *transfer one or more messeges without irq,but msg->len < 64
 */
int i2c_transfer_msgs_noirq(struct i2c_client *client, struct i2c_msg *msg, int count)
{
	int i;
	int tmp = 0;
	struct jz_i2c *i2c = client->adapter->algo_data;

	mutex_lock(&i2c->lock);
	clk_enable(i2c->clk);
	i2c_writew(i2c, I2C_INTM, 0x0);
	i2c_writew(i2c, I2C_TAR, msg->addr);

	for (i = 0; i < count; msg++, i++) {

		if (msg->flags & I2C_M_RD) {
			tmp = i2c_read_noirq(client, msg->len, msg->buf);
		} else {
			tmp = i2c_write_noirq(client, msg->len, msg->buf);
		}

		if (tmp < 0) {
			goto ERR_OUT;
		}
	}

	tmp = i;

ERR_OUT:
	disable_i2c_clk(i2c);
	mutex_unlock(&i2c->lock);

	return tmp;
}
EXPORT_SYMBOL_GPL(i2c_transfer_msgs_noirq);

/**
 * i2c_read_byte_noirq -return the read value or error value
 *@client:the client being refenced
 *@reg:write data to the reg
 
 *read a byte data to the reg without irq
 */
unsigned char i2c_read_byte_noirq(struct i2c_client *client, unsigned char reg)
{
	int tmp = 0;
	struct jz_i2c *i2c = client->adapter->algo_data;
	int len = 1;
	unsigned int wait_timeout_ms = len * 1000 * 9 * 2 / i2c->bus_speed
					+ CONFIG_I2C_JZ4775_WAIT_MS;

	mutex_lock(&i2c->lock);
	clk_enable(i2c->clk);

	i2c_writew(i2c, I2C_INTM, 0x0);
	i2c_writew(i2c, I2C_TAR, client->addr);

	i2c_writew(i2c, I2C_DC, reg);
	i2c_writew(i2c, I2C_DC, I2C_DC_READ | I2C_DC_STP);

	while (wait_timeout_ms > 0) {
		if (i2c_readw(i2c, I2C_RAW_INTST) & I2C_INTST_TXABT) {
			i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
			tmp = i2c_check_error(i2c);
			break;
		}
		if ((i2c_readb(i2c, I2C_STA) & I2C_STA_RFNE)) {
			tmp = i2c_readw(i2c, I2C_DC) & 0xff;
			break;
		} else {
			mdelay(1);
			wait_timeout_ms--;
		}
	}

	if (wait_timeout_ms <= 0) {
		dev_info(&(i2c->adap.dev),"%s read: wait timeout\n",__func__);
		tmp = -ETIMEDOUT;
	}

	i2c_readb(i2c, I2C_CSTP); /*clear stop irq*/

	disable_i2c_clk(i2c);
	mutex_unlock(&i2c->lock);

	return tmp;
}
EXPORT_SYMBOL_GPL(i2c_read_byte_noirq);

/**
 * i2c_write_byte_noirq -return error value if occured erro
 *@client:the client being refenced
 *@reg:write data to the reg
 *@val:the data write to the reg
 
 *write a byte data to the reg without irq
 */
int i2c_write_byte_noirq(struct i2c_client *client, unsigned char reg,
					unsigned char val)
{
	int ret = 0;
	int len = 1;
	struct jz_i2c *i2c = client->adapter->algo_data;
	unsigned int wait_timeout_ms = len * 1000 * 9 * 2 / i2c->bus_speed
					+ CONFIG_I2C_JZ4775_WAIT_MS;

	mutex_lock(&i2c->lock);
	clk_enable(i2c->clk);

	i2c_writew(i2c, I2C_INTM, 0x0);
	i2c_writew(i2c, I2C_TAR, client->addr);

	i2c_writew(i2c, I2C_DC, reg);
	i2c_writew(i2c, I2C_DC, val | I2C_DC_STP);

	while (!(i2c_readb(i2c, I2C_STA) & I2C_STA_TFE)) {
		if (i2c_readw(i2c, I2C_RAW_INTST) & I2C_INTST_TXABT) {
			i2c->abort_src = i2c_readl(i2c, I2C_TXABRT);
			ret = i2c_check_error(i2c);
			break;
		}
		if (wait_timeout_ms > 0) {
			wait_timeout_ms--;
			mdelay(1);
		} else {
			dev_info(&(i2c->adap.dev), "%s status:0x%x\n",__func__,
					i2c_readb(i2c, I2C_STA));
			ret = -ETIMEDOUT;
			break;
		}
	}
	if (wait_timeout_ms <= 0) {
		dev_err(&(i2c->adap.dev),"%s write: wait timeout\n",__func__);
		ret = -ETIMEDOUT;
	}

	i2c_readb(i2c, I2C_CSTP); /*clear stop irq*/

	disable_i2c_clk(i2c);
	mutex_unlock(&i2c->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(i2c_write_byte_noirq);
