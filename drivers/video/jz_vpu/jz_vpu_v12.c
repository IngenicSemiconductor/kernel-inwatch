/*
 * linux/drivers/misc/jz_vpu_v12.c - Ingenic VPU driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: Large Dipper <ykli@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//#define DEBUG

#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/syscalls.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pfn.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <mach/jzcpm_pwc.h>
#ifdef CONFIG_SOC_M200
#include <mach/libdmmu.h>
#endif

#include "jz_vpu_v12.h"

#define MAX_LOCK_DEPTH		999
#define CMD_VPU_CACHE 100
#define CMD_VPU_PHY   101
#define CMD_VPU_DMA_NOTLB 102
#define CMD_VPU_DMA_TLB 103
#define CMD_VPU_CLEAN_WAIT_FLAG 104
#define CMD_VPU_RESET 105
#define CMD_VPU_SET_REG 106
#define CMD_VPU_DMMU_MAP 107
#define CMD_VPU_DMMU_UNMAP 108
#define CMD_VPU_DMMU_UNMAP_ALL 109

/* we add them to wait for vpu end before suspend */
#define VPU_NEED_WAIT_END_FLAG 0x80000000
#define VPU_WAIT_OK 0x40000000
#define VPU_END 0x1

struct jz_vpu {
	struct device		*dev;
	spinlock_t		lock;
	int			irq;
	void __iomem		*iomem;
	struct miscdevice	mdev;
	struct clk		*clk;
	struct clk		*clk_gate;

	struct wake_lock	wake_lock;
	struct completion	done;
	int			use_count;
	struct mutex		mutex;
	pid_t			owner_pid;
	unsigned int		status;
};

static int vpu_reset(struct jz_vpu *vpu)
{
	int timeout = 0xffffff;
	unsigned int srbc = cpm_inl(CPM_SRBC);

	cpm_set_bit(30, CPM_SRBC);
	while (!(cpm_inl(CPM_SRBC) & (1 << 29)) && --timeout);

	if (timeout == 0) {
		dev_warn(vpu->dev, "[%d:%d] wait stop ack timeout\n",
			 current->tgid, current->pid);
		cpm_outl(srbc, CPM_SRBC);
		return -1;
	} else {
		cpm_outl(srbc | (1 << 31), CPM_SRBC);
		cpm_outl(srbc, CPM_SRBC);
	}

	return 0;
}

static int vpu_on(struct jz_vpu *vpu)
{
	if (cpm_inl(CPM_OPCR) & OPCR_IDLE)
		return -EBUSY;

	clk_enable(vpu->clk);
	clk_enable(vpu->clk_gate);
	__asm__ __volatile__ (
		"mfc0  $2, $16,  7   \n\t"
		"ori   $2, $2, 0x340 \n\t"
		"andi  $2, $2, 0x3ff \n\t"
		"mtc0  $2, $16,  7  \n\t"
		"nop                  \n\t");
	vpu_reset(vpu);
	enable_irq(vpu->irq);
	wake_lock(&vpu->wake_lock);
	dev_dbg(vpu->dev, "[%d:%d] on\n", current->tgid, current->pid);

	return 0;
}

static long vpu_off(struct jz_vpu *vpu)
{
	disable_irq_nosync(vpu->irq);

	__asm__ __volatile__ (
		"mfc0  $2, $16,  7   \n\t"
		"andi  $2, $2, 0xbf \n\t"
		"mtc0  $2, $16,  7  \n\t"
		"nop                  \n\t");

	cpm_clear_bit(31,CPM_OPCR);
	clk_disable(vpu->clk_gate);
	clk_disable(vpu->clk);
	/* Clear completion use_count here to avoid a unhandled irq after vpu off */
	vpu->done.done = 0;
	wake_unlock(&vpu->wake_lock);
	dev_dbg(vpu->dev, "[%d:%d] off\n", current->tgid, current->pid);

	return 0;
}

static int vpu_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_vpu *vpu = container_of(dev, struct jz_vpu, mdev);
	int dec_count = MAX_LOCK_DEPTH;

	dev_warn(vpu->dev, "[%d:%d] open\n", current->tgid, current->pid);

	spin_lock(&vpu->lock);

	if (vpu->use_count == 0) {
		vpu_on(vpu);
		while (mutex_is_locked(&vpu->mutex) && --dec_count) {
			dev_warn(vpu->dev, "[%d:%d] mutex locked, dec by 1\n",
				 current->tgid, current->pid);
			mutex_unlock(&vpu->mutex);
		}
		if (!dec_count) {
			dev_err(vpu->dev, "[%d:%d] lock depth > %d\n",
				current->tgid, current->pid, MAX_LOCK_DEPTH);
			WARN_ON(1);
		}

	} else {
		dev_dbg(vpu->dev, "[%d:%d] already on\n",
			current->tgid, current->pid);
	}
	vpu->use_count++;
	spin_unlock(&vpu->lock);

	return 0;
}

static int vpu_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_vpu *vpu = container_of(dev, struct jz_vpu, mdev);

	spin_lock(&vpu->lock);
	vpu->use_count--;

	if (vpu->use_count == 0) {
		vpu_off(vpu);
	} else {
		dev_dbg(vpu->dev, "[%d:%d] someone else used, leave on\n",
			current->tgid, current->pid);
	}
	spin_unlock(&vpu->lock);
	dev_warn(vpu->dev, "[%d:%d] close\n", current->tgid, current->pid);

	return 0;
}

static ssize_t vpu_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	return -1;
}

static ssize_t vpu_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	return -1;
}
unsigned long jz_tcsm_start;
static long vpu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_vpu *vpu = container_of(dev, struct jz_vpu, mdev);
	struct flush_cache_info info;
	int ret = 0;
	unsigned int status = 0;

	volatile unsigned long flags;
	unsigned int addr, size;
	unsigned int * arg_r;
	int i,num;

	switch (cmd) {
	case WAIT_COMPLETE:
		ret = wait_for_completion_interruptible_timeout(
			&vpu->done, msecs_to_jiffies(200));
		if (ret > 0) {
			status = vpu->status;
		} else {
			dev_warn(vpu->dev, "[%d:%d] wait_for_completion timeout\n",
				 current->tgid, current->pid);
			if (vpu_reset(vpu) < 0)
				status = 0;
			vpu->done.done = 0;
		}
		if (copy_to_user((void *)arg, &status, sizeof(status)))
			ret = -EFAULT;
		break;
	case LOCK:
		if (vpu->owner_pid == current->pid) {
			dev_err(vpu->dev, "[%d:%d] dead lock\n",
				current->tgid, current->pid);
			ret = -EINVAL;
			break;
		}

		if (mutex_lock_interruptible(&vpu->mutex) != 0) {
			dev_err(vpu->dev, "[%d:%d] lock error!\n",
				current->tgid, current->pid);
			ret = -EIO;
			break;
		}
		vpu->owner_pid = current->pid;
		dev_dbg(vpu->dev, "[%d:%d] lock\n", current->tgid, current->pid);
		break;
	case UNLOCK:
		mutex_unlock(&vpu->mutex);
		vpu->owner_pid = 0;
		dev_dbg(vpu->dev, "[%d:%d] unlock\n", current->tgid, current->pid);
		break;
	case FLUSH_CACHE:
		if (copy_from_user(&info, (void *)arg, sizeof(info))) {
			ret = -EFAULT;
			break;
		}
		dma_cache_sync(NULL, (void *)info.addr, info.len, info.dir);
		dev_dbg(vpu->dev, "[%d:%d] flush cache\n", current->tgid, current->pid);
		break;
	case CMD_VPU_PHY:
		arg_r = (unsigned int *)arg;
		*arg_r = (0x1fffffff) & jz_tcsm_start;
		break;
	case CMD_VPU_CACHE:
		arg_r = (unsigned int *)arg;
		addr = (unsigned int)arg_r[0];
		size = arg_r[1];
		if(size == 0) {
			printk("ERROR:%s:size = %d, addr = 0x%x\n", __func__, size, addr);
			force_sig(SIGSEGV, current);
		}else {
			dma_cache_wback_inv(addr, size);
		}
		break;
	case CMD_VPU_DMA_NOTLB:
		local_irq_save(flags);
		arg_r = (unsigned int *)arg;
		REG_VPU_LOCK |= VPU_NEED_WAIT_END_FLAG;
		for(i = 0;i < 4; i += 2){
			*(unsigned int *)(arg_r[i]) = arg_r[i+1];
			printk("arg[%d]=%x arg[%d]=%d",i,arg_r[i],i+1,arg_r[i+1]);
		}
		local_irq_restore(flags);
		break;
	case CMD_VPU_DMA_TLB:
		local_irq_save(flags);
		arg_r = (unsigned int *)arg;
		REG_VPU_LOCK |= VPU_NEED_WAIT_END_FLAG;
		for(i = 0;i < 10;i += 2)
			*(unsigned int *)(arg_r[i]) = arg_r[i+1];
		local_irq_restore(flags);
		break;
	case CMD_VPU_CLEAN_WAIT_FLAG:
		local_irq_save(flags);
		while( !(( REG_VPU_LOCK &(VPU_WAIT_OK)) ||( REG_VPU_STATUS&(VPU_END))) )
			;
		REG_VPU_LOCK &= ~(VPU_NEED_WAIT_END_FLAG);
		if(REG_VPU_LOCK & VPU_WAIT_OK)
			REG_VPU_LOCK &= ~(VPU_WAIT_OK);
		local_irq_restore(flags);
		break;
	case  CMD_VPU_RESET:
		local_irq_save(flags);
		REG_CPM_VPU_SWRST |= CPM_VPU_STP;
		while(!(REG_CPM_VPU_SWRST & CPM_VPU_ACK))
			;
		REG_CPM_VPU_SWRST = ((REG_CPM_VPU_SWRST | CPM_VPU_SR) & ~CPM_VPU_STP);
		REG_CPM_VPU_SWRST = (REG_CPM_VPU_SWRST & ~CPM_VPU_SR & ~CPM_VPU_STP);
		REG_VPU_LOCK = 0;
		local_irq_restore(flags);
		break;
	case   CMD_VPU_SET_REG:
		local_irq_save(flags);
		num = *(unsigned int*)arg;
		arg += 4;
		arg_r = (unsigned int *)arg;
		REG_VPU_LOCK |= VPU_NEED_WAIT_END_FLAG;
		for(i = 0;i < num; i += 2)
			*(unsigned int *)(arg_r[i]) = arg_r[i+1];
		local_irq_restore(flags);
		break;
#ifdef CONFIG_SOC_M200
	case CMD_VPU_DMMU_MAP:
		{
			struct vpu_dmmu_map_info di;
			if (copy_from_user(&di, (void *)arg, sizeof(di))) {
				ret = -EFAULT;
				break;
			}
			return dmmu_map(dev->this_device,di.addr,di.len);
		}
		break;
	case CMD_VPU_DMMU_UNMAP:
		{
			struct vpu_dmmu_map_info di;
			if (copy_from_user(&di, (void *)arg, sizeof(di))) {
				ret = -EFAULT;
				break;
			}
			return dmmu_unmap(dev->this_device,di.addr,di.len);
		}
		break;
	case CMD_VPU_DMMU_UNMAP_ALL:
		dmmu_unmap_all(dev->this_device);
		break;
#endif
	default:
		break;
	}

	return ret;
}

static int vpu_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if( PFN_PHYS(vma->vm_pgoff) < 0x13200000 || PFN_PHYS(vma->vm_pgoff) >= 0x13300000){
		printk("phy addr err ,range is 0x13200000 - 13300000");
		return -EAGAIN;
	}
	if (io_remap_pfn_range(vma,vma->vm_start,
			       vma->vm_pgoff,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static irqreturn_t vpu_interrupt(int irq, void *dev)
{
	struct jz_vpu *vpu = dev;
	unsigned int vpu_stat;

	vpu_stat = vpu_readl(vpu,REG_VPU_STAT);

#define CLEAR_VPU_BIT(vpu,offset,bm)				\
	do {							\
		unsigned int stat;				\
		stat = vpu_readl(vpu,offset);			\
		vpu_writel(vpu,offset,stat & ~(bm));		\
	} while(0)

#define check_vpu_status(STAT, fmt, args...) do {	\
		if(vpu_stat & STAT)			\
			dev_err(vpu->dev, fmt, ##args);	\
	}while(0)

	if(vpu_stat) {
		if(vpu_stat & VPU_STAT_ENDF) {
			if(vpu_stat & VPU_STAT_JPGEND) {
				dev_dbg(vpu->dev, "JPG successfully done!\n");
				vpu_stat = vpu_readl(vpu,REG_VPU_JPGC_STAT);
				CLEAR_VPU_BIT(vpu,REG_VPU_JPGC_STAT,JPGC_STAT_ENDF);
			} else {
				dev_dbg(vpu->dev, "SCH successfully done!\n");
				CLEAR_VPU_BIT(vpu,REG_VPU_SDE_STAT,SDE_STAT_BSEND);
				CLEAR_VPU_BIT(vpu,REG_VPU_DBLK_STAT,DBLK_STAT_DOEND);
			}
		} else {
			check_vpu_status(VPU_STAT_SLDERR, "SHLD error!\n");
			check_vpu_status(VPU_STAT_TLBERR, "TLB error! Addr is 0x%08x\n",
					 vpu_readl(vpu,REG_VPU_STAT));
			check_vpu_status(VPU_STAT_BSERR, "BS error!\n");
			check_vpu_status(VPU_STAT_ACFGERR, "ACFG error!\n");
			check_vpu_status(VPU_STAT_TIMEOUT, "TIMEOUT error!\n");
			CLEAR_VPU_BIT(vpu,REG_VPU_GLBC,
				      (VPU_INTE_ACFGERR |
				       VPU_INTE_TLBERR |
				       VPU_INTE_BSERR |
				       VPU_INTE_ENDF
					      ));
		}
	} else {
		if(vpu_readl(vpu,REG_VPU_AUX_STAT) & AUX_STAT_MIRQP) {
			dev_dbg(vpu->dev, "AUX successfully done!\n");
			CLEAR_VPU_BIT(vpu,REG_VPU_AUX_STAT,AUX_STAT_MIRQP);
		} else {
			dev_dbg(vpu->dev, "illegal interrupt happened!\n");
			return IRQ_HANDLED;
		}
	}

	vpu->status = vpu_stat;
	complete(&vpu->done);

	return IRQ_HANDLED;
}

static struct file_operations vpu_misc_fops = {
	.open		= vpu_open,
	.release	= vpu_release,
	.read		= vpu_read,
	.write		= vpu_write,
	.unlocked_ioctl	= vpu_ioctl,
	.mmap		= vpu_mmap,
//	.mmap       =  jz_tcsm_mmap,
};
struct jz_vpu *vpu;
static int vpu_probe(struct platform_device *pdev)
{
	int ret;
	struct resource			*regs;

	vpu = kzalloc(sizeof(struct jz_vpu), GFP_KERNEL);
	if (!vpu)
		ret = -ENOMEM;

	vpu->irq = platform_get_irq(pdev, 0);
	if(vpu->irq < 0) {
		dev_err(&pdev->dev, "get irq failed\n");
		ret = vpu->irq;
		goto err_get_mem;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "No iomem resource\n");
		ret = -ENXIO;
		goto err_get_mem;
	}

	vpu->iomem = ioremap(regs->start, resource_size(regs));
	if (!vpu->iomem) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENXIO;
		goto err_get_mem;
	}

	vpu->clk_gate = clk_get(&pdev->dev, "vpu");
	if (IS_ERR(vpu->clk_gate)) {
		ret = PTR_ERR(vpu->clk_gate);
		goto err_get_clk_gate;
	}

	vpu->clk = clk_get(vpu->dev,"cgu_vpu");
	if (IS_ERR(vpu->clk)) {
		ret = PTR_ERR(vpu->clk);
		goto err_get_clk_cgu;
	}
	/*
	 * for jz4775, when vpu freq is set over 300M, the decode process
	 * of vpu may be error some times which can led to graphic abnomal
	 */

#if defined(CONFIG_SOC_4775)
	clk_set_rate(vpu->clk,250000000);
#else
	clk_set_rate(vpu->clk,300000000);
#endif

	vpu->dev = &pdev->dev;
	vpu->mdev.minor = MISC_DYNAMIC_MINOR;
	vpu->mdev.name =  "jz-vpu";
	vpu->mdev.fops = &vpu_misc_fops;

	spin_lock_init(&vpu->lock);

	ret = misc_register(&vpu->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "misc_register failed\n");
		goto err_registe_misc;
	}
	platform_set_drvdata(pdev, vpu);

	wake_lock_init(&vpu->wake_lock, WAKE_LOCK_SUSPEND, "vpu");

	mutex_init(&vpu->mutex);

	init_completion(&vpu->done);
	ret = request_irq(vpu->irq, vpu_interrupt, IRQF_DISABLED,
			  "vpu",vpu);
	if (ret < 0) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto err_request_irq;
	}

	disable_irq_nosync(vpu->irq);

	return 0;

err_request_irq:
	misc_deregister(&vpu->mdev);
err_registe_misc:
	clk_put(vpu->clk);
err_get_clk_cgu:
	clk_put(vpu->clk_gate);
err_get_clk_gate:
	iounmap(vpu->iomem);
err_get_mem:
	kfree(vpu);
	return ret;
}

static int vpu_remove(struct platform_device *dev)
{
	struct jz_vpu *vpu = platform_get_drvdata(dev);

	misc_deregister(&vpu->mdev);
	wake_lock_destroy(&vpu->wake_lock);
	clk_put(vpu->clk);
	clk_put(vpu->clk_gate);
	free_irq(vpu->irq, vpu);
	iounmap(vpu->iomem);
	kfree(vpu);

	return 0;
}
static int vpu_suspend(struct platform_device * pdev, pm_message_t state)
{
	struct jz_vpu *vpu = platform_get_drvdata(pdev);
	int timeout = 0xffff;

	if( vpu->use_count != 0 ) {
		if( REG_VPU_LOCK & VPU_NEED_WAIT_END_FLAG) {
			while(!( REG_VPU_STATUS & (0x1) ) && timeout--)
				;
			if(!timeout){
				printk("vpu -------- timeout\n");
				return 0;
			}
			REG_VPU_LOCK |= VPU_WAIT_OK;
			REG_VPU_LOCK &= ~VPU_NEED_WAIT_END_FLAG;
		}
		clk_disable(vpu->clk);
		clk_disable(vpu->clk_gate);
	}
	return 0;
}
static int vpu_resume(struct platform_device * pdev)
{
	struct jz_vpu *vpu = platform_get_drvdata(pdev);

	if(vpu->use_count != 0) {
		clk_set_rate(vpu->clk,300000000);
		clk_enable(vpu->clk);
		clk_enable(vpu->clk_gate);
	}
	return 0;
}
static struct platform_driver jz_vpu_driver = {
	.probe		= vpu_probe,
	.remove		= vpu_remove,
	.driver		= {
		.name	= "jz-vpu",
	},
	.suspend    = vpu_suspend,
	.resume     = vpu_resume,
};

static int __init vpu_init(void)
{
	return platform_driver_register(&jz_vpu_driver);
}

static void __exit vpu_exit(void)
{
	platform_driver_unregister(&jz_vpu_driver);
}

module_init(vpu_init);
module_exit(vpu_exit);

MODULE_DESCRIPTION("JZ M200 v12 VPU driver");
MODULE_AUTHOR("Large Dipper <ykli@ingenic.cn>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20120925");
