/*
 * include/nand.h
 */
#ifndef __NAND_H__
#define __NAND_H__
#include "common.h"
/*
 * these are base message of nand
 *(start)
 */
#define PNAND_NEMC		(1 << 0)
#define PNAND_DDR		(1 << 1)
#define PNAND_BCH_ENC		(1 << 2)
#define PNAND_BCH_DEC		(1 << 3)
#define PNAND_HALT		(1 << 16)
/*
#define CTRL_READ_DATA		(1 << 0)
#define CTRL_READ_OOB		(1 << 1)
#define CTRL_WRITE_DATA		(1 << 2)
#define CTRL_WRITE_OOB		(1 << 3)
#define CTRL_WRITE_CONFIRM	(1 << 4)
 */
#define NANDTYPE_COMMON		0
#define NANDTYPE_TOGGLE		1

/*
 * Standard NAND flash commands
 */
#define NAND_CMD_READ0			0x00
#define NAND_CMD_RNDOUT			0x05
#define NAND_CMD_STATUS			0x70
#define NAND_CMD_READID			0x90
#define NAND_CMD_2P_READ		0x60
#define NAND_CMD_2P_READSTART	0x30
#define NAND_CMD_READSTART      0x30
#define NAND_CMD_RNDOUTSTART    0xE0

#define NAND_CMD_WRITE			0x80
#define NAND_CMD_RNDIN			0x85
#define NAND_CMD_PROGRAM		0x10
#define NAND_CMD_2P_WRITE1  	0x80
#define NAND_CMD_2P_PROGRAM1 	0x11
#define NAND_CMD_2P_WRITE2      0x81

#define NAND_CMD_ERASE1		0x60
#define NAND_CMD_ERASE2		0xD0
#define NAND_CMD_RESET		0xFF

/* Extended commands for large page devices */
#define NAND_CMD_CACHEDPROG     0x15
#define TWOP_FIRST_OOB            0x00
#define TWOP_SECOND_OOB           0x80


/* Status Bits */
#define NAND_STATUS_FAIL	0x01
#define NAND_STATUS_FAIL_N1	0x02
#define NAND_STATUS_TRUE_READY	0x20
#define NAND_STATUS_READY	0x40
#define NAND_STATUS_WP		0x80

#define NAND_CMD_OFFSET		0x00400000
#define NAND_ADDR_OFFSET	0x00800000

#define __nand_cmd(cmd, port)		(REG8((port) | NAND_CMD_OFFSET) = (cmd))
#define __nand_addr(addr, port)		(REG8((port) | NAND_ADDR_OFFSET) = (addr))
#define __nand_status(state, port)	((state) = REG8(port))
/*
 * these are base message of nand
 *(end)
 */
/*
 * pagesize : physical page size
 * oobsize  : physical oob size
 * rowcycle :
 * ecclevel : eccbit per step
 * eccsize  : data bytes per ECC step
 * eccbytes : ecc parity per step
 * eccpos   : ecc position in oob partition
 * cs[]     : chipselect id
 * rb1     : it is number of gpio,which is used as rb1
 * rb2     : it is number of gpio,which is used as rb2
 * taskmsgaddr : the address of message of task in ddr memory
 */
struct __nand_chip {
	unsigned short pagesize;
	unsigned short oobsize;
	unsigned short eccsize;
	unsigned short eccbytes;
	unsigned short ecclevel;
	unsigned short twhr2;
	unsigned short tcwaw;
	unsigned short tadl;
	unsigned short tcs;
	unsigned short tclh;
	unsigned short tsync;
//	unsigned char eccpos;
	unsigned char buswidth;
	unsigned char  rowcycle;
	char cs[4];
	unsigned char rb0;
	unsigned char rb1;
	unsigned int taskmsgaddr;
	unsigned int fcycle;
};
typedef struct __nand_chip NandChip;


/*
 * these are promises of CPU and MCU
 * (start)
 */

struct msgdata_data{
	unsigned int offset :16;
	unsigned int bytes  :16;
	unsigned int pdata;
};

struct msgdata_cmd{
	unsigned int command   :8;
	unsigned int cmddelay  :9;
	unsigned int addrdelay :9;
	unsigned int offset    :6; // the unit is 512 bytes
	unsigned int pageid;
};

struct msgdata_prepare{
	unsigned int unit      :4;
	unsigned int eccbit    :8;
	unsigned short totaltasknum;
	unsigned short retnum;
};

struct msgdata_ret{
	unsigned int		:16;
	unsigned int bytes	:16;
	unsigned int retaddr;
};

struct msgdata_parity{
	unsigned int offset	:16;
	unsigned int bytes	:16;
	unsigned int parityaddr;
};
struct msgdata_badblock{
	unsigned int planes     :4;
	unsigned int blockid;
};
union taskmsgdata{
	struct msgdata_data data;
	struct msgdata_cmd  cmd;
	struct msgdata_prepare prepare;
	struct msgdata_ret  ret;
	struct msgdata_parity parity;
	struct msgdata_badblock badblock;
};

/* the definition of task_msg.ops */
enum nand_opsstate{
	CHAN1_DATA,
	CHAN1_PARITY,
	BCH_PREPARE,
	BCH_FINISH,
	CHAN2_DATA,
	CHAN2_FINISH,
};
enum nand_opsmodel{
	MCU_NO_RB,
	MCU_WITH_RB,
	MCU_READ_DATA,
	MCU_WRITE_DATA_WAIT,
	MCU_WRITE_DATA,
	MCU_ISBADBLOCK,
	MCU_MARKBADBLOCK,
};
enum nand_opstype{
	MSG_MCU_INIT = 1,
	MSG_MCU_PREPARE,
	MSG_MCU_CMD,
	MSG_MCU_BADBLOCK,
	MSG_MCU_DATA,
	MSG_MCU_RET,
	MSG_MCU_PARITY,
};

struct taskmsghead_bits{
	unsigned int type     :3;
	unsigned int model    :3;
	unsigned int state    :3;
	unsigned int chipsel  :4; // the chip's number,while should be selected.
	unsigned int          :3;
	unsigned int resource :16;
};

union taskmsghead{
	struct taskmsghead_bits bits;
	int flag;
};

struct task_msg{
	union taskmsghead ops;
	union taskmsgdata msgdata;
};

struct taskmsg_init {
	union taskmsghead ops;
	NandChip info;
};

/* the return value of every task */
#define MSG_RET_SUCCESS 0
#define MSG_RET_WP (0x01<<0)
#define MSG_RET_EMPTY (0x01<<1)
#define MSG_RET_MOVE (0x01<<2)
#define MSG_RET_FAIL (0x01<<3)

/* INTC MailBox msg */
#define MCU_MSG_NORMAL          0x1
#define MCU_MSG_INTC            0x2
#define MCU_MSG_RB  	        0x3
#define MCU_MSG_PDMA            0x4

/* PDMA MailBox msg */
#define MB_MCU_DONE           (0x1 << 4)
#define MB_MCU_ERROR          (0x5 << 5)

#endif /*__NAND_H__*/
