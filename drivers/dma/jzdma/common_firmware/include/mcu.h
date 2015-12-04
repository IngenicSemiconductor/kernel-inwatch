/*
 * include/mcu.h
 */
#ifndef __MCU_H__
#define __MCU_H__

#include "nand.h"

#define MCU_TASK_NUM  2
#define MCU_TMPP  16    /* the max number of taskmsg per messagepipe */
/*  status of task in mcu */
enum task_status{
	TASK_WAIT = 1,
	TASK_FINISH,
	TASK_HANDLE,
};

enum mcu_pipe_status{
	PIPE_BUSY,
	PIPE_FREE,
};
enum nemc_chip_status{
	NAND_CHIP_ENABLE,
	NAND_CHIP_DISABLE,
};

typedef int (*TASKFUNC)(struct task_msg *,int);

struct __tasknode{
	TASKFUNC func;
	struct task_msg msg;
};
typedef struct __tasknode TaskNode;

struct __pipenode {
	unsigned char *pipe_data;
	unsigned char *pipe_par;
};
typedef struct __pipenode PipeNode;
/* some constant parameter after mcu init */
/*
 * msgdata_start : the start address of a message's pipe of task.
 * msgdata_end   : the end address of a message's pipe of task.
 */

struct __task_msgpipe{
	struct task_msg *msgdata_start;
	struct task_msg *msgdata_end;
	struct __task_msgpipe *next;
};

typedef struct __task_msgpipe TaskMsgPipe;
/*
 * current_cs   : current chipsel
 * nand_io      : current address of nand io
 * chan_descriptor: the transmission of channel(31~0) does't be interrupted,
 that is similar to the mode of channel's descriptor
 * totaltasknum : the total number of task during the operation
 * completetasknum : the complete number of task during the operation
 * taskmsg_cpipe   : the pointer of current taskmsgpipe
 * taskmsgpipe  : the array of pipe of taskmessage
 * taskmsg_index: the pointer of current taskmsg
 */
struct mcu_common_msg{
	int current_cs;
	unsigned int nand_io;
	volatile unsigned int chan_descriptor;
	unsigned short totaltasknum;
	unsigned short completetasknum;
	TaskMsgPipe *taskmsg_cpipe;
	TaskMsgPipe taskmsgpipe[2];
	struct task_msg *taskmsg_index;
	unsigned char *ret;
	unsigned short ret_index;
};
#define SET_CHAN_DESCRIPTOR(desc,chan)     ((desc) |= (0x1<<(chan)))
#define CLEAR_CHAN_DESCRIPTOR(desc,chan)   ((desc) &= ~(0x1<<(chan)))
struct nand_mcu_ops{
	NandChip nand_info;
	PipeNode pipe[2];
	struct mcu_common_msg common;
	TaskNode *wait_task;
};

/* all resource of firmware operation
 *  0 : the resource is unavailable
 *  1 : the resource is available
 --------------------------------------
 * rb0 : nand chip 0
 * rb1 : nand chip 1
 * bch : bch controller
 * chan0 : special 0, the channel moves datas between tcsm and bch
 * chan1 : special 1, the channel moves datas between tcsm and nemc
 * chan2 : the channel moves datas between tcsm and ddr
 * chan3 : the channel moves message between tcsm and ddr
 * pipe0 : it is a bank in tcsm,that is used as storing data and parity
 * pipe1 : it is a bank in tcsm,that is used as storing data and parity
 * msgflag : it is the counter of available message;
 */

struct __resource{
	unsigned int rb0: 1;
	unsigned int rb1: 1;
	unsigned int nemc: 1;
	unsigned int chan0: 1;
	unsigned int chan1: 1;
	unsigned int chan2: 1;
	unsigned int pipe0: 1;
	unsigned int pipe1: 1;
	unsigned int msgflag: 2;
	unsigned int setrb0: 1;
	unsigned int setrb1: 1;
};
union ops_resource{
	struct __resource bits;
	unsigned int flag;
};
typedef union ops_resource OpsResource;

enum __resource_state{
	REDUCE = -1,
	UNAVAILABLE,
	AVAILABLE,
	INCREASE = 1,
	GOTRB = 0,
	WAITRB = 1,
};
typedef enum __resource_state Rstate;

enum __resource_bit{
	R_RB0,
	R_RB1,
	R_NEMC,
	R_CHAN0,
	R_CHAN1,
	R_CHAN2,
	R_PIPE0,
	R_PIPE1,
	R_MSGFLAG,
	R_SETRB0,
	R_SETRB1,
	R_ALLBITS,
};
typedef enum __resource_bit Rbit;

/* if res satisfy req , return 1; else return 0*/
#define __resource_satisfy(res,req) (!(((res) ^ (req)) & (req)))

/* the model of bch channel */
#define CHANNEL_FREE       0x0000
#define CHANNEL_BUSY       0x0001

#define NAND_DATA_PORT1         0x1B000000      /* read-write area in static bank 1 */
#define NAND_DATA_PORT2         0x1A000000      /* read-write area in static bank 2 */
#define NAND_DATA_PORT3         0x19000000      /* read-write area in static bank 3 */
#define NAND_DATA_PORT4         0x18000000      /* read-write area in static bank 4 */
#define NAND_DATA_PORT5         0x17000000
#define NAND_DATA_PORT6         0x16000000

#if 0
/* PDMA GPIOA interrupt */
#define PDMA_RB0_MASK      		(0x01<<20)
#define PDMA_RB1_MASK      		(0x01<<20)
#define PDMA_GPIOA_FLAG    		(0x10010050)
#define PDMA_GPIOA_CFLAG    	(0x10010058)
#define __pdma_rb0_pending()    (*(volatile unsigned int *)PDMA_GPIOA_FLAG & PDMA_RB0_MASK)
#define __pdma_rb0_clear_flag()  (*(volatile unsigned int *)PDMA_GPIOA_CFLAG &= PDMA_RB0_MASK)
#define __pdma_rb1_pending()    (*(volatile unsigned int *)PDMA_GPIOA_FLAG & PDMA_RB1_MASK)
#define __pdma_rb1_clear_flag()  (*(volatile unsigned int *)PDMA_GPIOA_CFLAG &= PDMA_RB1_MASK)
#endif
unsigned int get_resource(void);
void mcu_nand_reset(struct nand_mcu_ops *mcu, unsigned short msg);
unsigned int mcu_taskmanager(struct nand_mcu_ops *mcu);

void nand_enable(NandChip *nandinfo, int cs);
void nand_disable(NandChip *nandinfo);
unsigned int send_nand_command(struct nand_mcu_ops *mcu, struct msgdata_cmd *cmd);
void send_read_random(struct nand_mcu_ops *mcu, int offset);
void send_prog_random(struct nand_mcu_ops *mcu, int offset);

void pdma_bch_encode_prepare(NandChip *nand_info, PipeNode *pipe);
void pdma_bch_decode_prepare(NandChip *nand_info, PipeNode *pipe);
void bch_encode_complete(NandChip *nand_info, PipeNode *pipe);
void bch_decode_complete(NandChip *nand, unsigned char *data_buf,unsigned char *err_buf, unsigned char *report);

#endif /*__MCU_H_*/
