#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <soc/gpio.h>
#include <asm/atomic.h>
#include <linux/i2c/i2c_power_manager.h>

#define PAH8001
#ifndef PAH8001
#define PAJ3007			//choose PAH8001 or PAJ3007
#endif

#include <linux/input/pah8001.h>

#ifdef PAH8001
#define OFN_SUPPORT 0		//must be 0
#else
#define OFN_SUPPORT 1		//could be 0 or 1
#endif

#if (OFN_SUPPORT == 1)
#define OFN_MOUSE	1	// 1 for mouse, 0 for keyboard
#endif

#if (OFN_SUPPORT == 1)
#if (OFN_MOUSE == 1)
#define ABS_MOUSE 0		//1 for abs mouse, 0 for relative mouse
#endif
#endif

#if (OFN_SUPPORT == 1)
#if (OFN_MOUSE == 1)
#if (ABS_MOUSE == 1)
#define X_MAX 1920
#define Y_MAX 1080
#endif
#endif
#endif

#ifdef PAH8001
#define INTERRUPT_MODE 0	//could be 0 or 1
#else
#define INTERRUPT_MODE 0	//must be 0
#endif

#define RAW_TO_HAL	0	//could be 0 or 1

#define ofn_name "pixart_ofn"

#define PIXART_IOC_MAGIC 'h'
#define PIXART_IOCTL_ENABLE_DISABLE (0)
#define PIXART_IOC_MAXNR 1

#define PIXART_IOCTL_SENSOR_ENABLE_DISABLE \
            _IOW(PIXART_IOC_MAGIC, PIXART_IOCTL_ENABLE_DISABLE, int)

#define OFN_REGITER_BANK_SEL 127
#define OFN_BANK0 0
#define OFN_BANK1 1
#define FIFO_SIZE 32
#define FIFO_SIZE_M1 (FIFO_SIZE-1)
typedef struct {
	struct i2c_client *client;
	struct pah8001_platform_data *pdata;
	struct input_dev *pah8001_input_dev;
	struct input_dev *mouse_input_dev;
	struct input_dev *keyboard_input_dev;
	bank_e bank;
	struct delayed_work work;
	struct delayed_work x_work;	//for PPR, HRV
	struct delayed_work resume_work;
	struct wake_lock work_wake_lock;
	struct kobject *kobj;
	struct class *ofn_class;
	dev_t ofn_dev;
	struct device *ofn_device;
	unsigned int major_id;
	bool run_hrd;
	bool run_ms;
	bool run_kbd;
	int hr;
	int irq;
	int abs_x;
	int abs_y;
	int enabled;
} ofn_data_t;

typedef struct {
	struct i2c_client *client;
} mems_data_t;

static ofn_data_t ofndata;
// static  mems_data_t memsdata;  //for gsenosor LIS3DH
typedef struct {
	u8 HRD_Data[13];
	u32 MEMS_Data[3];
} ppg_mems_data_t;

static ppg_mems_data_t _ppg_mems_data[FIFO_SIZE];

static int _read_index = 0;
static int _write_index = 0;
static atomic_t device_working_flag = ATOMIC_INIT(0);
static atomic_t devices_configed = ATOMIC_INIT(0);

static struct i2c_power_device *device = NULL;
static struct i2c_control_operations func;

unsigned char ofn_write_reg(unsigned char addr, unsigned char data);
unsigned char ofn_read_reg(unsigned char addr, unsigned char *data);
static void ofn_ppg(void);

static ssize_t ofn_read(struct file *filp, char *buf, size_t count,
			loff_t * l);
static ssize_t ofn_write(struct file *filp, const char *buf, size_t count,
			 loff_t * f_ops);
static long ofn_ioctl(struct file *file, unsigned int cmd,
		      unsigned long arg);
static int ofn_open(struct inode *inode, struct file *filp);
static int ofn_release(struct inode *inode, struct file *filp);
static void resume_work_func(struct work_struct *work);
static void open_device(void);
static void resume_open_device(void);

static struct file_operations ofn_fops = {
	.owner = THIS_MODULE,
	.read = ofn_read,
	.write = ofn_write,
//	.ioctl        =       ofn_ioctl,
	.unlocked_ioctl = ofn_ioctl,
	.open = ofn_open,
	.release = ofn_release,
};

/////******** LED Conttol ********************
#define __write_reg(a, b) ofn_write_reg(a, b)
#define __read_reg(a, b) ofn_read_reg(a, b)

void PAJ3007_led_ctrl(uint8_t touch)
{
	if (touch == 0x80) {
		__write_reg(0x05, 0xA8);
		__write_reg(0x7F, 0x01);
		__write_reg(0x38, 0xFC);
		__write_reg(0x42, 0xA6);
	} else {
		__write_reg(0x05, 0xB8);
		__write_reg(0x7F, 0x01);
		__write_reg(0x38, 0xF2);
		__write_reg(0x42, 0xA0);
	}
}

void PAH8001_led_ctrl(uint8_t touch)
{
	if (touch == 0x80) {
		__write_reg(0x05, 0x98);
		__write_reg(0x7f, 0x01);	//for bank1
		__write_reg(0x42, 0xA4);

	} else {
		__write_reg(0x05, 0xB8);
		__write_reg(0x7f, 0x01);	//for bank1
		__write_reg(0x42, 0xA0);
	}
}


static int ofn_i2c_write(u8 reg, u8 * data, int len)
{
	u8 buf[20];
	int rc;
	int ret = 0;
	int i;

	buf[0] = reg;
	if (len >= 20) {
		printk
		    ("%s (%d) : FAILED: buffer size is limitted(20) %d\n",
		     __func__, __LINE__, len);
		dev_err(&ofndata.client->dev,
			"ofn_i2c_write FAILED: buffer size is limitted(20)\n");
		return -1;
	}

	for (i = 0; i < len; i++) {
		buf[i + 1] = data[i];
	}

	rc = i2c_master_send(ofndata.client, buf, len + 1);	// Returns negative errno, or else the number of bytes written.

	if (rc != len + 1) {
		printk("%s (%d) : FAILED: writing to reg 0x%x\n", __func__,
		       __LINE__, reg);
		ret = -1;
	}

	return ret;
}

static int ofn_i2c_burst_read(u8 reg, u8 * data, u8 len)
{
	u8 buf[256];
	int rc;

	buf[0] = reg;

	rc = i2c_master_send(ofndata.client, buf, 1);	// Returns negative errno, or else the number of bytes written.
	if (rc != 1) {
		printk("%s (%d) : FAILED: writing to address 0x%x\n",
		       __func__, __LINE__, reg);
		return -1;
	}

	rc = i2c_master_recv(ofndata.client, data, len);
	if (rc != len) {
		printk("%s (%d) : FAILED: reading data %d\n", __func__,
		       __LINE__, rc);
		return -1;
	}

	return 0;
}

static int ofn_i2c_read(u8 reg, u8 * data)
{
	u8 buf[20];
	int rc;

	buf[0] = reg;

	rc = i2c_master_send(ofndata.client, buf, 1);	//If everything went ok (i.e. 1 msg transmitted), return #bytes  transmitted, else error code.   thus if transmit is ok  return value 1
	if (rc != 1) {
		printk("%s (%d) : FAILED: writing to address 0x%x\n",
		       __func__, __LINE__, reg);
		return -1;
	}

	rc = i2c_master_recv(ofndata.client, buf, 1);	// returns negative errno, or else the number of bytes read
	if (rc != 1) {
		printk("%s (%d) : FAILED: reading data\n", __func__,
		       __LINE__);
		return -1;
	}

	*data = buf[0];
	return 0;
}

unsigned char ofn_write_reg(unsigned char addr, unsigned char data)
{
	int ret = ofn_i2c_write(addr, &data, 1);

	if (ret != 0)
		return false;
	else
		return true;
}

unsigned char ofn_read_reg(unsigned char addr, unsigned char *data)
{
	int ret = ofn_i2c_read(addr, data);

	if (ret != 0)
		return false;
	else
		return true;
}

unsigned char ofn_burst_read_reg(unsigned char addr, unsigned char *data,
				 unsigned int length)
{
	int ret = ofn_i2c_burst_read(addr, data, length);

	if (ret != 0)
		return false;
	else
		return true;
}

static int ofn_set_reg(u8 reg, u8 data)
{
	int ret = ofn_i2c_write(reg, &data, 1);

	if (ret != 0) {
		printk("%s (%d) : FAILED:  ofn register setting \n",
		       __func__, __LINE__);
		return -1;
	}

	return ret;
}

static int ofn_bank_select(bank_e bank)
{
    int ret = 0;
	// printk("%s (%d) : ofn bank selection\n", __func__, __LINE__);

	//if(ofndata.bank != bank)
	//{
	switch (bank) {
	case BANK0:
		ret = ofn_set_reg(OFN_REGITER_BANK_SEL, OFN_BANK0);	// OFN_REGITER_BANK_SEL = OX7F
		if (ret == -1)
		    return -1;
		break;
	case BANK1:
		ret = ofn_set_reg(OFN_REGITER_BANK_SEL, OFN_BANK1);
		if (ret == -1)
		    return -1;
		break;
	default:
		break;
		//      }

		ofndata.bank = bank;
	}
	return 0;
}

static long _read_addr = 0x01;
static ssize_t write_reg_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	char s[256];
	char *p = s;
	printk("%s (%d) : write register\n", __func__, __LINE__);

	memcpy(s, buf, size);

	*(s + 1) = '\0';
	*(s + 4) = '\0';
	*(s + 7) = '\0';
	//example(in console): echo w 12 34 > rw_reg
	if (*p == 'w') {
		long write_addr, write_data;

		p += 2;
		if (!kstrtol(p, 16, &write_addr)) {
			p += 3;
			if (!kstrtol(p, 16, &write_data)) {
				printk("w 0x%x 0x%x\n",
				       (unsigned int) write_addr,
				       (unsigned int) write_data);
				ofn_set_reg((u8) write_addr,
					    (u8) write_data);
			}
		}
	}
	//example(in console): echo r 12 > rw_reg
	else if (*p == 'r') {
		p += 2;

		if (!kstrtol(p, 16, &_read_addr)) {
			u8 data = 0;
			if (!ofn_i2c_read((u8) _read_addr, &data)) {
				printk("r 0x%x 0x%x\n",
				       (unsigned int) _read_addr, data);
			}
		}
	}
	return size;
}

static ssize_t read_reg_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int ret = -1;
	char *s = buf;
	u8 data = 0;

	printk("%s (%d) : read register\n", __func__, __LINE__);
	ret = ofn_i2c_read(_read_addr, &data);
	if (ret)
		s += sprintf(s, "Error\n");
	else
		s += sprintf(s, "Addr 0x%x, Data 0x%x\n",
			     (unsigned int) _read_addr, data);
	return (s - buf);
}

static DEVICE_ATTR(rw_reg, 0666, read_reg_show, write_reg_store);

static struct attribute *rw_reg_sysfs_attrs[] = {
	&dev_attr_rw_reg.attr,
	NULL
};

static struct attribute_group rw_reg_attribute_group = {
	.attrs = rw_reg_sysfs_attrs,
};

#ifdef PAH8001
static int ppg_1000_init(void)
{
	int i = 0;
	int ret = -1;
	int bank = 0;
	u8 data = 0;

	for (i = 0; i < INIT_PPG_REG_ARRAY_SIZE_1000; i++) {
		if (init_ppg_register_array_1000[i][0] == 0x7F)
			bank = init_ppg_register_array_1000[i][1];

		if ((bank == 0)
		    && (init_ppg_register_array_1000[i][0] == 0x17)) {
			//read and write bit7=1
			ret = ofn_i2c_read(0x17, &data);
			if (ret == 0) {
				data |= 0x80;
				ret = ofn_set_reg(0x17, data);
			}
		} else {
			ret = ofn_set_reg(init_ppg_register_array_1000[i][0],
					init_ppg_register_array_1000[i][1]);
		}
		if (ret)
			break;
	}

	return ret;
}

#define PIXART_PAH8001_INT_GIO 122	//GPIO_PD(26)

#if (INTERRUPT_MODE == 1)
irqreturn_t pixart_8001_irq_thread_fn(int irq, void *data)
{
	int ret;
	ret = gpio_get_value(PIXART_PAH8001_INT_GIO);
	//printk("%s (%d) : gpio value = %d\n", __func__, __LINE__, ret);
	ofn_ppg();
	//printk("%s (%d) \n", __func__, __LINE__);
	return IRQ_HANDLED;
}

static int pixart_pah8001_init_interrupt(void)
{
	int result;

	printk("%s (%d) : ofn init interrupt\n", __func__, __LINE__);
	result =
	    gpio_request(PIXART_PAH8001_INT_GIO,
			 "pixart_pah8001_int_gpio");
	if (result != 0) {
		printk("pixart_proximity interrupt request failed!\n");
		goto rtn;
	}

	result = gpio_direction_input(PIXART_PAH8001_INT_GIO);
	if (result != 0) {
		printk("pixart_proximity direction failed!\n");
		goto rtn;
	}

	ofndata.irq = gpio_to_irq(PIXART_PAH8001_INT_GIO);

	result = request_threaded_irq(ofndata.irq, NULL, pixart_8001_irq_thread_fn, IRQF_TRIGGER_FALLING,	// trugger -> level change
				      "pixart_proximity_irq", &ofndata);

	disable_irq(ofndata.irq);

	if (result != 0) {
		printk("pixart_proximity request_irq failed \n");
		goto rtn;
	}
	enable_irq(ofndata.irq);

      rtn:
	printk("%s (%d) : gpio %d, irq %d, ret (%d)\n", __func__, __LINE__,
	       PIXART_PAH8001_INT_GIO, ofndata.irq, result);

	return result;

}
#endif

#else
static int paj3007_init(void)
{
	int i = 0;
	int ret = -1;

	for (i = 0; i < INIT_PPG_REG_ARRAY_SIZE; i++) {
		ret =
		    ofn_set_reg(init_ppg_register_array[i][0],
				init_ppg_register_array[i][1]);
		if (ret) {
			return ret;
		}
	}

	return ret;
}
#endif

static int ofn_init_reg(void)
{
	//Near_normal_mode_V5_6.15mm_121017 for 940nm
	u8 data0 = 0, data1 = 0;
	int ret = -1;

	ret = ofn_bank_select(BANK0);
	if (ret < 0) {
	    printk("%s : ofn_bank_select error\n", __func__);
	}
	ret = ofn_i2c_read(0, &data0);
	if (ret) {
		return ret;
	}

	ret = ofn_i2c_read(1, &data1);
	if (ret) {
		return ret;
	}

//	printk("%s (%d) : ADDR0 = 0x%x, ADDR1 = 0x%x.\n", __func__, __LINE__, data0, data1);	// ADDR0 = 0x30, ADDR1= 0xd3
	if ((data0 != 0x30) || ((data1 & 0xF0) != 0xD0)) {
		return -1;
	}
#ifdef PAH8001
	ret = ppg_1000_init();
#else
	ret = paj3007_init();
#endif
	if (ret) {
		return ret;
	}

//	printk("%s (%d) : ofn initialize register.\n", __func__, __LINE__);

	return 0;
}

static void ofn_ppg(void)
{
	static unsigned long volatile start_jiffies = 0, end_jiffies;
	static u8 Frame_Count = 0;
	u8 touch_flag = 0;
	u8 data;
	int ret = 0;

	if (ofndata.run_hrd) {
		ret = ofn_bank_select(BANK0);
		if (ret < 0) {
		    printk("%s : ofn_bank_select error\n", __func__);
		}
		ofn_i2c_read(0x59, &touch_flag);

		//	Try not to print the message when formal use,
		//	cause it maybe extend the time to communicate with IC
/*		printk(">>>>>> into : %s , %d , touch_flag : 0x%x ------------\n",
		__func__, __LINE__, touch_flag);*/

		touch_flag &= 0x80;
#ifdef PAJ3007
		PAJ3007_led_ctrl(touch_flag);
#else
#ifdef PAH8001
		PAH8001_led_ctrl(touch_flag);
#endif
#endif

		ret = ofn_bank_select(BANK1);
		if (ret < 0) {
		    printk("%s : ofn_bank_select error\n", __func__);
		}
		ofn_i2c_read(0x68, &data);
		_ppg_mems_data[_write_index].HRD_Data[0] = data & 0x0f;

		if (_ppg_mems_data[_write_index].HRD_Data[0] == 0) {
			ret = ofn_bank_select(BANK0);
			if (ret < 0) {
			    printk("%s : ofn_bank_select error\n", __func__);
			}
			msleep(10);
		} else {
#if (RAW_TO_HAL == 0)
			int tmp = 0;
#endif
			ofn_i2c_burst_read(0x64,
					   &(_ppg_mems_data[_write_index].
					     HRD_Data[1]), 4);
			ofn_i2c_burst_read(0x1A,
					   &(_ppg_mems_data[_write_index].
					     HRD_Data[5]), 3);
			_ppg_mems_data[_write_index].HRD_Data[8] =
			    Frame_Count++;
			end_jiffies = jiffies;
			_ppg_mems_data[_write_index].HRD_Data[9] =
			    jiffies_to_msecs(end_jiffies - start_jiffies);
			start_jiffies = end_jiffies;
			_ppg_mems_data[_write_index].HRD_Data[10] = 0;
			_ppg_mems_data[_write_index].HRD_Data[11] =
			    touch_flag;
			_ppg_mems_data[_write_index].HRD_Data[12] =
			    _ppg_mems_data[_write_index].HRD_Data[6];

#if (RAW_TO_HAL == 1)
			//printk(">>>>>> into : %s , %d -----\n", __func__, __LINE__);
			input_report_abs(ofndata.pah8001_input_dev, ABS_X,
					 *(uint32_t
					   *) (_ppg_mems_data
					       [_write_index].HRD_Data));
			input_report_abs(ofndata.pah8001_input_dev, ABS_Y,
					 *(uint32_t
					   *) (_ppg_mems_data
					       [_write_index].HRD_Data +
					       4));
			input_report_abs(ofndata.pah8001_input_dev, ABS_Z,
					 *(uint32_t
					   *) (_ppg_mems_data
					       [_write_index].HRD_Data +
					       8));
			input_sync(ofndata.pah8001_input_dev);
//                      printk(">>>%s (%x)(%x)(%x) \n", __func__,*(uint32_t *)(_ppg_mems_data[_write_index].HRD_Data), *(uint32_t *)(_ppg_mems_data[_write_index].HRD_Data + 4), *(uint32_t *)(_ppg_mems_data[_write_index].HRD_Data + 8));
			printk(">>>%s (%d)(%d)(%d)(%d) \n", __func__,
			       _ppg_mems_data[_write_index].HRD_Data[0],
			       _ppg_mems_data[_write_index].HRD_Data[1],
			       _ppg_mems_data[_write_index].HRD_Data[2],
			       _ppg_mems_data[_write_index].HRD_Data[3]);
//                      printk(">>>%s (%d)(%d)(%d)(%d) \n", __func__,_ppg_mems_data[_write_index].HRD_Data[4], _ppg_mems_data[_write_index].HRD_Data[5], _ppg_mems_data[_write_index].HRD_Data[6], _ppg_mems_data[_write_index].HRD_Data[7]);
//                      printk(">>>%s (%d)(%d)(%d)(%d) \n", __func__,_ppg_mems_data[_write_index].HRD_Data[8], _ppg_mems_data[_write_index].HRD_Data[9], _ppg_mems_data[_write_index].HRD_Data[10], _ppg_mems_data[_write_index].HRD_Data[11]);

#else
//                      mems_i2c_read(0x28, _ppg_mems_data[_write_index].MEMS_Data, 6);
			memset(_ppg_mems_data[_write_index].MEMS_Data, 0,
			       sizeof(_ppg_mems_data[_write_index].
				      MEMS_Data));
			tmp = _write_index + 1;
			tmp &= FIFO_SIZE_M1;

			if (tmp == _read_index) {
				printk("Buffer over flow!!!\n");
			} else
				_write_index = tmp;
#endif
		}
	}
}

#if (OFN_SUPPORT == 0)
#if (INTERRUPT_MODE == 0)
static void ofn_x_work_func(struct work_struct *work)	//for PPR, HRV      ofn_open \C7Լ\F6 \BA\BC\B0\CD
{
	printk(">>>%s (%d)\n", __func__, __LINE__);
//	printk("%s : run_hrd=%d\n", __func__, ofndata.run_hrd);
	while (ofndata.run_hrd) {
		ofn_ppg();
	}

	printk("<<< %s (%d)\n", __func__, __LINE__);
}
#endif

#ifdef PAH8001
#if (RAW_TO_HAL == 1)
static int pah8001_open(struct input_dev *dev)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
	_read_index = 0;
	_write_index = 0;
	ofndata.hr = 0;
	ofndata.run_hrd = true;
#if (OFN_SUPPORT == 0)
#if (INTERRUPT_MODE == 0)
	printk(">>> %s (%d) \n", __func__, __LINE__);
	schedule_delayed_work(&ofndata.x_work, msecs_to_jiffies(100));
#endif


#endif
	return 0;
}

static void pah8001_close(struct input_dev *dev)
{
	ofndata.run_hrd = false;
	printk(">>> %s (%d) \n", __func__, __LINE__);

}

static int pah8001_init_input_data(void)
{
	int ret = 0;

	printk("%s (%d) : initialize data\n", __func__, __LINE__);

	ofndata.pah8001_input_dev = input_allocate_device();

	if (!ofndata.pah8001_input_dev) {
		printk("%s (%d) : could not allocate mouse input device\n",
		       __func__, __LINE__);
		return -ENOMEM;
	}
	ofndata.pah8001_input_dev->evbit[0] = BIT_MASK(EV_ABS);
	ofndata.pah8001_input_dev->absbit[0] =
	    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) | BIT_MASK(ABS_Z) |
	    BIT_MASK(ABS_RX) | BIT_MASK(ABS_RY);

	input_abs_set_max(ofndata.pah8001_input_dev, ABS_X, 0xffffffff);
	input_abs_set_max(ofndata.pah8001_input_dev, ABS_Y, 0xffffffff);
	input_abs_set_max(ofndata.pah8001_input_dev, ABS_Z, 0xffffffff);
	input_abs_set_max(ofndata.pah8001_input_dev, ABS_RX, 0xffffffff);
	input_abs_set_max(ofndata.pah8001_input_dev, ABS_RY, 0xffffffff);

	input_set_drvdata(ofndata.pah8001_input_dev, &ofndata);
	ofndata.pah8001_input_dev->name = "Pixart PPG";

	ofndata.pah8001_input_dev->open = pah8001_open;
	ofndata.pah8001_input_dev->close = pah8001_close;

	ret = input_register_device(ofndata.pah8001_input_dev);
	if (ret < 0) {
		input_free_device(ofndata.pah8001_input_dev);
		printk("%s (%d) : could not register input device\n",
		       __func__, __LINE__);
		return ret;
	}

	return 0;
}
#endif				//#if (RAW_TO_HAL == 1)
#endif				//#ifdef PAH8001
#endif

#if (OFN_SUPPORT == 1)

#if (OFN_MOUSE == 0)
static void ofn_work_func_kb(struct work_struct *work)
{
	int ret;
	unsigned int keyCode = KEY_RESERVED;
	uint8_t data = 0;
	uint8_t ui8Reg52 = 0, ui8Reg52Click = 0;
	uint8_t single_click = 0;
	unsigned long single_click_ts = 0;
	//uint32_t i = 0;

	while (ofndata.run_kbd) {
		//printk("[ofn_work_function] %s (%d)\n", __func__, __LINE__);
		ofn_ppg();
		ofn_bank_select(OFN_BANK0);
		ret = ofn_i2c_read(2, &data);
		//printk("(%d)(%d) \n",__LINE__ ,data);
		if (ret) {
			continue;
		}

		ret = ofn_i2c_read(3, &data);
		//printk("(%d)(%d) \n",__LINE__ ,data);
		if (ret) {
			continue;
		}
		ret = ofn_i2c_read(4, &data);
		//printk("(%d)(%d) \n",__LINE__ ,data);
		if (ret) {
			continue;
		}
		ret = ofn_i2c_read(52, &ui8Reg52);
		//printk("(%d)(%d) \n",__LINE__ ,ui8Reg52);
		if (ret) {
			continue;
		}

		keyCode = KEY_RESERVED;
		switch (ui8Reg52) {
		case 1:	//Up
			keyCode = KEY_UP;
			break;
		case 2:	//down
			keyCode = KEY_DOWN;
			break;
		case 3:	//left
			keyCode = KEY_RIGHT;
			break;
		case 4:	//right
			keyCode = KEY_LEFT;
			break;
		}

		if (keyCode != KEY_RESERVED) {
			input_report_key(ofndata.keyboard_input_dev,
					 keyCode, 1);
			input_sync(ofndata.keyboard_input_dev);
			input_report_key(ofndata.keyboard_input_dev,
					 keyCode, 0);
			input_sync(ofndata.keyboard_input_dev);
		}

		ui8Reg52Click = ui8Reg52 & 0x30;
		if (ui8Reg52Click == 0x20) {
			printk("double click \n");
			//double click
			input_report_key(ofndata.keyboard_input_dev,
					 KEY_BACK, 1);
			input_sync(ofndata.keyboard_input_dev);
			input_report_key(ofndata.keyboard_input_dev,
					 KEY_BACK, 0);
			input_sync(ofndata.keyboard_input_dev);
			single_click = 0;
		} else if (ui8Reg52Click == 0x10) {
			printk("single click trigger\n");
			single_click = 1;
			single_click_ts = jiffies;

		}

		if (single_click) {
			int t =
			    jiffies_to_msecs(jiffies - single_click_ts);
			if (t > 500)	//more than 500ms
			{
				//click
				printk("single click\n");
				single_click = 0;
				input_report_key(ofndata.
						 keyboard_input_dev,
						 KEY_ENTER, 1);
				input_sync(ofndata.keyboard_input_dev);
				input_report_key(ofndata.
						 keyboard_input_dev,
						 KEY_ENTER, 0);
				input_sync(ofndata.keyboard_input_dev);
			}
		}
	}
	//schedule_delayed_work(&ofndata.work, msecs_to_jiffies(1));
}

static int ofn_kb_open(struct input_dev *dev)
{
	ofndata.run_kbd = true;
	schedule_delayed_work(&ofndata.work, msecs_to_jiffies(1000));

	return 0;
}

static void ofn_kb_close(struct input_dev *dev)
{
	cancel_delayed_work_sync(&ofndata.work);
}

static int ofn_init_keyboard_data(void)
{
	int ret = 0;

	ofndata.keyboard_input_dev = input_allocate_device();

	if (!ofndata.keyboard_input_dev) {
		printk
		    ("%s (%d) : could not allocate keyboard input device\n",
		     __func__, __LINE__);
		return -ENOMEM;
	}

	input_set_drvdata(ofndata.keyboard_input_dev, &ofndata);
	ofndata.keyboard_input_dev->name = "ofn_sensor";

	input_set_capability(ofndata.keyboard_input_dev, EV_KEY, KEY_UP);
	input_set_capability(ofndata.keyboard_input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(ofndata.keyboard_input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(ofndata.keyboard_input_dev, EV_KEY,
			     KEY_RIGHT);
	input_set_capability(ofndata.keyboard_input_dev, EV_KEY,
			     KEY_ENTER);
	input_set_capability(ofndata.keyboard_input_dev, EV_KEY, KEY_BACK);

	ofndata.keyboard_input_dev->open = ofn_kb_open;
	ofndata.keyboard_input_dev->close = ofn_kb_close;

	ret = input_register_device(ofndata.keyboard_input_dev);
	if (ret < 0) {
		input_free_device(ofndata.keyboard_input_dev);
		printk("%s (%d) : could not register input device\n",
		       __func__, __LINE__);
		return ret;
	}

	return 0;
}
#else
static void ofn_work_func_ms(struct work_struct *work)
{
	int ret;
	uint8_t data;		// error message mixed declarations and code
	uint8_t x, y;
	int rep_x, rep_y;
	uint8_t ui8Reg52 = 0, ui8Reg52Click = 0;
	uint8_t single_click = 0;
	unsigned long single_click_ts = 0;

	while (ofndata.run_ms) {
		ofn_ppg();

		ofn_bank_select(OFN_BANK0);

		ret = ofn_i2c_read(2, &data);
		//printk("(%d)(%d) \n",__LINE__ ,data);
		if (ret) {
			continue;
		}
		if ((data & 0x80) == 0)
			goto CLICK;

		ret = ofn_i2c_read(3, &x);
		//printk("(%d)(%d) \n",__LINE__ ,data);
		if (ret) {
			continue;
		}
		ret = ofn_i2c_read(4, &y);
		//printk("(%d)(%d) \n",__LINE__ ,data);
		if (ret) {
			continue;
		}

		rep_x = x;
		rep_y = y;
		rep_x <<= 24;
		rep_x >>= 24;
		rep_y <<= 24;
		rep_y >>= 24;

		ofn_write_reg(0x12, 0x25);
		printk(" [register] (%d)(%d), (%d)(%d) \n", x, rep_x, y,
		       rep_y);

#if (ABS_MOUSE == 1)
		ofndata.abs_x += rep_x;
		ofndata.abs_y += rep_y;

		if (ofndata.abs_x < 0)
			ofndata.abs_x = 0;
		if (ofndata.abs_y < 0)
			ofndata.abs_y = 0;
		if (ofndata.abs_x >= X_MAX)
			ofndata.abs_x = (X_MAX - 1);
		if (ofndata.abs_y >= Y_MAX)
			ofndata.abs_y = (Y_MAX - 1);

		//printk("(%d)(%d), (%d)(%d) \n", rep_x, ofndata.abs_x, rep_y, ofndata.abs_y);
		input_report_abs(ofndata.mouse_input_dev, ABS_X,
				 ofndata.abs_x);
		input_report_abs(ofndata.mouse_input_dev, ABS_Y,
				 ofndata.abs_y);
		input_sync(ofndata.mouse_input_dev);

#else
		printk("(%d)(%d), (%d)(%d) \n", x, rep_x, y, rep_y);
		input_report_rel(ofndata.mouse_input_dev, REL_X, rep_x);
		input_report_rel(ofndata.mouse_input_dev, REL_Y, rep_y);
		input_sync(ofndata.mouse_input_dev);
#endif

	      CLICK:
		ret = ofn_i2c_read(52, &ui8Reg52);
		//printk("(%d)(%d) \n",__LINE__ ,ui8Reg52);
		if (ret) {
			continue;
		}

		ui8Reg52Click = ui8Reg52 & 0x30;
		if (ui8Reg52Click == 0x20) {
			printk("double click \n");
			//double click
			input_report_key(ofndata.mouse_input_dev, KEY_BACK,
					 1);
			input_sync(ofndata.mouse_input_dev);
			input_report_key(ofndata.mouse_input_dev, KEY_BACK,
					 0);
			input_sync(ofndata.mouse_input_dev);
			single_click = 0;
		} else if (ui8Reg52Click == 0x10) {
			printk("single click trigger\n");
			single_click = 1;
			single_click_ts = jiffies;
		}

		if (single_click) {
			int t =
			    jiffies_to_msecs(jiffies - single_click_ts);
			if (t > 500)	//more than 500ms
			{
				//click
				printk("single click\n");
				single_click = 0;
				input_report_key(ofndata.mouse_input_dev,
						 BTN_LEFT, 1);
				input_sync(ofndata.mouse_input_dev);
				input_report_key(ofndata.mouse_input_dev,
						 BTN_LEFT, 0);
				input_sync(ofndata.mouse_input_dev);
			}
		}
	}
	//schedule_delayed_work(&ofndata.work, msecs_to_jiffies(1));
}

static int ofn_ms_open(struct input_dev *dev)
{
	ofndata.run_ms = true;
	schedule_delayed_work(&ofndata.work, msecs_to_jiffies(1000));

	return 0;
}

static void ofn_ms_close(struct input_dev *dev)
{
	cancel_delayed_work_sync(&ofndata.work);
}

static int ofn_init_mouse_data(void)
{
	int ret = 0;
	ofndata.mouse_input_dev = input_allocate_device();

	if (!ofndata.mouse_input_dev) {
		printk("%s (%d) : could not allocate mouse input device\n",
		       __func__, __LINE__);
		return -ENOMEM;
	}
#if (ABS_MOUSE == 1)
	ofndata.mouse_input_dev->evbit[0] =
	    BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ofndata.mouse_input_dev->absbit[0] =
	    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y);
	ofndata.mouse_input_dev->keybit[BIT_WORD(BTN_MOUSE)] =
	    BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT);

	input_abs_set_max(ofndata.mouse_input_dev, ABS_X, X_MAX);
	input_abs_set_max(ofndata.mouse_input_dev, ABS_Y, Y_MAX);
	input_set_capability(ofndata.mouse_input_dev, EV_KEY, KEY_BACK);
#else

	ofndata.mouse_input_dev->evbit[0] =
	    BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
	ofndata.mouse_input_dev->relbit[0] =
	    BIT_MASK(REL_X) | BIT_MASK(REL_Y);
	ofndata.mouse_input_dev->keybit[BIT_WORD(BTN_MOUSE)] =
	    BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT);
	input_set_capability(ofndata.mouse_input_dev, EV_KEY, KEY_BACK);
#endif

	input_set_drvdata(ofndata.mouse_input_dev, &ofndata);
	ofndata.mouse_input_dev->name = "ofn_sensor";

	ofndata.mouse_input_dev->open = ofn_ms_open;
	ofndata.mouse_input_dev->close = ofn_ms_close;

	ret = input_register_device(ofndata.mouse_input_dev);
	if (ret < 0) {
		input_free_device(ofndata.mouse_input_dev);
		printk("%s (%d) : could not register input device\n",
		       __func__, __LINE__);
		return ret;
	}

	return 0;
}
#endif				//if (OFN_MOUSE == 0)
#endif

static void ofn_power_on(struct pah8001_platform_data *pdata)
{
    if (pdata->power_on)
        pdata->power_on();
}

static void ofn_power_off(struct pah8001_platform_data *pdata)
{
    if (pdata->power_off)
        pdata->power_off();
}

static int ofn_init_chip(struct pah8001_platform_data *pdata)
{
    int err = 0;
    u8 buf = 0;

    //  Test I2C communication
    err = __read_reg(0x0, &buf);
    if (err == false) {
        printk("pah8001: Failed to read register 0x0!!!\n");
        err = -EIO;
        return err;
    }

    gpio_set_value(pdata->gpio_reset, 1);

    gpio_direction_input(pdata->gpio_int);

    if (pdata->gpio_pd >= 0) {
        gpio_set_value(pdata->gpio_pd, 1);
        msleep(10);
    }

    gpio_set_value(pdata->gpio_reset, 0);
    msleep(10);
    gpio_set_value(pdata->gpio_reset, 1);
    msleep(10);

    //Software Power Down Mode
    ofn_bank_select(BANK0);
    if (err < 0) {
        printk("%s : ofn_bank_select error\n", __func__);
        return err;
    }

    err = __write_reg(0x06, 0x0a);
    if (err < 0) {
        printk("ofn write reg error\n");
        return err;
    }
    msleep(5);

    err = ofn_init_reg();
    if (err < 0) {
        printk("ofn_init_reg error\n");
        return err;
    }

    return 0;
}

static int ofn_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	struct pah8001_platform_data *pdata = NULL;
	unsigned char buf = 0;

	pdata = (struct pah8001_platform_data *) client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		return err;
	}

	ofndata.client = client;
	/**********************************  add  *********************************************/
	ofndata.pdata = pdata;

    if (pdata->board_init) {
        err = pdata->board_init(&client->dev);
        if (err < 0) {
            pr_err("board_init failed ! errno = %d\n", err);
            return err;
        }
    }

    err = gpio_request(pdata->gpio_reset, "gpio reset");
    if (err < 0) {
        printk("Unable to request GPIO : %d reset\n", pdata->gpio_reset);
        goto free_gpio_1;
    } else {
        gpio_direction_output(pdata->gpio_reset, 1);
    }

    err = gpio_request(pdata->gpio_int, "gpio int");
    if (err < 0) {
        printk("Unable to request GPIO : %d int\n", pdata->gpio_int);
        goto free_gpio_2;
    } else {
        gpio_direction_input(pdata->gpio_int);
    }

    if (pdata->gpio_pd >= 0) {
        err = gpio_request(pdata->gpio_pd, "gpio power down");
        if (err < 0) {
            printk(KERN_INFO "Unable to request GPIO : %d pd\n", pdata->gpio_pd);
            goto free_gpio;
        } else {
            gpio_direction_output(pdata->gpio_pd, 1);
            msleep(10);
        }
    }

    func.power_on = ofn_power_on;
    func.power_off = ofn_power_off;
    func.init_chip = ofn_init_chip;

    device = register_i2c_power_device(i2c_adapter_id(client->adapter), &func, pdata);
    if (!device) {
        err = -1;
        goto board_exit;
    }

    err = i2c_power_device_on(device);

    //  Test I2C communication
    err = __read_reg(0x0, &buf);
    if (err == false) {
        err = -EIO;
        goto power_off;
    }

	err = register_chrdev(0, ofn_name, &ofn_fops);	//\B9\AE\C0\DA \C0\E5ġ return value\B4\C2 major number \C7Ҵ\E7
	if (err < 0) {
		printk(KERN_WARNING "Can't get major\n");
		goto free_gpio;
	} else {
#if (OFN_SUPPORT == 1)
#if (OFN_MOUSE == 1)
#if (ABS_MOUSE == 1)
		ofndata.abs_x = (X_MAX >> 1);
		ofndata.abs_y = (Y_MAX >> 1);
#endif
#endif
#endif
		ofndata.major_id = err;	//235 \B6\F3\B4\C2 \C1ֹ\F8ȣ\B8\A6 get
		printk(KERN_WARNING "pixart_ofn : Succeed to register character device %d\n", ofndata.major_id);	// \C7\F6\C0\E7  ofndata.major_id =235
		ofndata.ofn_class = class_create(THIS_MODULE, ofn_name);
		if (IS_ERR(ofndata.ofn_class)) {
			printk(KERN_ERR "class_create() failed for ofn_class\n");
			err = -1;
			goto free_chrdev;
		}
		ofndata.ofn_device =
		    device_create(ofndata.ofn_class, NULL,
				  MKDEV(ofndata.major_id, 0), NULL,
				  ofn_name);

//		insmod \BF\A1 \C0\C7\C7\D8 \B5\EE\B7\CF\C7\D1 \B5\F0\B9\D9\C0̽\BA \B5\E5\B6\F3\C0̹\F6\B8\A6 Ŀ\B3ο\A1 \B5\EE\B7\CF\C7϶\F3\B4\C2 \C0ǹ\CC
//		sys_mknod("/dev/pixart_ofn",S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH, MKDEV(ofndata.major_id, 0));
//		sys_chmod("/dev/pixart_ofn",S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);

	}

#if (INTERRUPT_MODE	== 1)
	printk("call pixart_pah8001_init_interrupt \n");
	err = pixart_pah8001_init_interrupt();
	if (err < 0) {
	    goto free_class_and_device;
	}
#endif

#if (OFN_SUPPORT == 1)
#if (OFN_MOUSE == 1)
	printk("call ofn_work_func_ms\n");
	INIT_DELAYED_WORK(&ofndata.work, ofn_work_func_ms);
#else
	printk("call ofn_work_func_kb\n");
	INIT_DELAYED_WORK(&ofndata.work, ofn_work_func_kb);
#endif				//#if (OFN_MOUSE == 1)
#else				//if (OFN_SUPPORT == 1)

#if (INTERRUPT_MODE == 0)
	INIT_DELAYED_WORK(&ofndata.x_work, ofn_x_work_func);
#endif

#endif				//if (OFN_SUPPORT == 1)


#ifdef PAH8001

#if (RAW_TO_HAL == 1)
	err = pah8001_init_input_data();
#endif

#else				// #ifdef PAH8001
#if (OFN_SUPPORT == 1)
#if (OFN_MOUSE == 1)
	err = ofn_init_mouse_data();
#else
	err = ofn_init_keyboard_data();
#endif
#endif

	if (err < 0) {
#if (INTERRUPT_MODE == 1)
        goto free_int_gio;
#else
        goto free_class_and_device;
#endif
	}
#endif				//#ifdef PAH8001

	ofndata.kobj = kobject_create_and_add("pixart_ofn", kernel_kobj);
	if (!ofndata.kobj) {
		err = ENOMEM;
#if (INTERRUPT_MODE == 1)
		goto free_int_gio;
#else
		goto free_class_and_device;
#endif
	}

	err = sysfs_create_group(ofndata.kobj, &rw_reg_attribute_group);
	if (err) {
		kobject_put(ofndata.kobj);
#if (INTERRUPT_MODE == 1)
        goto free_int_gio;
#else
        goto free_class_and_device;
#endif
	}
//	gpio_direction_output(GPIO_FOR_TIME, 0);

	INIT_DELAYED_WORK(&ofndata.resume_work, resume_work_func);
	ofndata.run_hrd = false;
	atomic_set(&devices_configed, 1);

#ifndef CONFIG_SENSORS_PIXART_PAH8001_RESUME_RECONFIG
	wake_lock_init(&ofndata.work_wake_lock, WAKE_LOCK_SUSPEND, "pixart_ofn");
#endif
	return err;

#if (INTERRUPT_MODE == 1)
free_int_gpio:
    gpio_free(PIXART_PAH8001_INT_GIO);
#endif
free_class_and_device:
    device_destroy(ofndata.ofn_class, MKDEV(ofndata.major_id, 0));
    class_destroy(ofndata.ofn_class);
free_chrdev:
    unregister_chrdev(ofndata.major_id, ofn_name);
power_off:
    i2c_power_device_off(device);
    unregister_i2c_power_device(device);
board_exit:
    pdata->board_exit(&client->dev);
free_gpio:
    if (pdata->gpio_pd >= 0) {
        gpio_free(pdata->gpio_pd);
    }
free_gpio_2:
    gpio_free(pdata->gpio_int);
free_gpio_1:
    gpio_free(pdata->gpio_reset);
    return err;
}

static int ofn_i2c_remove(struct i2c_client *client)
{
//      cancel_delayed_work_sync(&ofndata.work);

//      sysfs_remove_group(&ofndata.mouse_input_dev->dev.kobj,
//                         &rw_reg_attribute_group);
#ifndef CONFIG_SENSORS_PIXART_PAH8001_RESUME_RECONFIG
    wake_lock_destroy(&ofndata.work_wake_lock);
#endif
	return 0;
}

static int ofn_suspend(struct device *dev)
{
    struct pah8001_platform_data *pdata = NULL;

    ofndata.run_hrd = false;
    pdata = ofndata.pdata;

    cancel_delayed_work_sync(&ofndata.x_work);
    cancel_delayed_work_sync(&ofndata.resume_work);
    flush_scheduled_work();

    if (pdata->gpio_reset > 0)
        gpio_direction_input(pdata->gpio_reset);

    if (pdata->gpio_pd > 0) {
        gpio_direction_input(pdata->gpio_pd);
    }

    if (pdata->gpio_int > 0) {
        gpio_direction_input(pdata->gpio_int);
    }

    i2c_power_device_off(device);

    atomic_set(&devices_configed, 0);

//    printk("%s (%d) : ofn suspend \n", __func__, __LINE__);
    return 0;
}

/**
 * resume_work_func : To reinitialize PAH8001 when resume.
 */
static void resume_work_func(struct work_struct *work)
{
    struct pah8001_platform_data *pdata = NULL;
    int err = 0;

    pdata = ofndata.pdata;

    i2c_power_device_on(device);

    if (pdata->gpio_reset > 0) {
        gpio_direction_output(pdata->gpio_reset, 1);
    }

    if (pdata->gpio_pd > 0) {
        gpio_direction_output(pdata->gpio_pd, 1);
        msleep(10);
    }

    if (pdata->gpio_reset > 0) {
        gpio_direction_output(pdata->gpio_reset, 0);
        msleep(10);
        gpio_direction_output(pdata->gpio_reset, 1);
        msleep(10);
    }

    //Software Power Down Mode
    err = ofn_bank_select(BANK0);
    if (err < 0) {
        printk("%s : ofn_bank_select error\n", __func__);
    }
    err = __write_reg(0x06, 0x0a);
    if (err < 0) {
        printk("ofn write reg error\n");
        return;
    }
    msleep(5);

    err = ofn_init_reg();
    if (err < 0) {
        printk("ofn_init_reg error\n");
        return;
    }

    atomic_set(&devices_configed, 1);

    if (atomic_read(&device_working_flag)) {
        resume_open_device();
    }
}

static int ofn_resume(struct device *dev)
{
#ifdef CONFIG_SENSORS_PIXART_PAH8001_RESUME_RECONFIG
    cancel_delayed_work(&ofndata.resume_work);
    schedule_delayed_work(&ofndata.resume_work, msecs_to_jiffies(5));
#endif
//    printk("%s (%d) : ofn resume \n", __func__, __LINE__);
    return 0;
}

static const struct i2c_device_id ofn_device_id[] = {
	{"pixart_ofn", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ofn_device_id);

static const struct dev_pm_ops ofn_pm_ops = {
	.suspend = ofn_suspend,
	.resume = ofn_resume
};

static struct i2c_driver ofn_i2c_driver = {
	.driver = {
		.name = "pixart_ofn",
		.owner = THIS_MODULE,
		.pm = &ofn_pm_ops},
		.probe = ofn_i2c_probe,
		.remove = ofn_i2c_remove,
		.id_table = ofn_device_id,
};

static ssize_t ofn_read(struct file *filp, char *buf, size_t count,
			loff_t * l)
{
	if (_write_index == _read_index)
		return 0;

	if (count != sizeof(ppg_mems_data_t))
		return 0;

	memcpy(buf, &_ppg_mems_data[_read_index], count);

	_read_index++;
	_read_index &= FIFO_SIZE_M1;

	return 1;
}

static ssize_t ofn_write(struct file *filp, const char *buf, size_t count,
			 loff_t * f_ops)
{
	return count;
}

static void resume_open_device(void)
{
    _read_index = 0;
    _write_index = 0;
    ofndata.hr = 0;
    ofndata.run_hrd = true;

    ofn_bank_select(BANK0);
    __write_reg(0x06, 0x02);
    if (ofndata.pdata->gpio_pd >= 0)
        gpio_direction_output(ofndata.pdata->gpio_pd, 0);
    msleep(5);

#if (OFN_SUPPORT == 0)
#if (INTERRUPT_MODE == 0)
    schedule_delayed_work(&ofndata.x_work, msecs_to_jiffies(100));
#endif
#endif
}

static void open_device(void) {
    unsigned long timeout = 0;

    atomic_inc(&device_working_flag);
    timeout = jiffies + (HZ);

#ifndef CONFIG_SENSORS_PIXART_PAH8001_RESUME_RECONFIG
    if (!atomic_read(&devices_configed)) {
        cancel_delayed_work(&ofndata.resume_work);
        schedule_delayed_work(&ofndata.resume_work, msecs_to_jiffies(5));
    }
#endif

    while(time_before(jiffies, timeout) && !atomic_read(&devices_configed));

    _read_index = 0;
    _write_index = 0;
    ofndata.hr = 0;
    ofndata.run_hrd = true;

    ofn_bank_select(BANK0);
    __write_reg(0x06, 0x02);
    if (ofndata.pdata->gpio_pd >= 0)
        gpio_direction_output(ofndata.pdata->gpio_pd, 0);
    msleep(5);

#if (OFN_SUPPORT == 0)
#if (INTERRUPT_MODE == 0)
    schedule_delayed_work(&ofndata.x_work, msecs_to_jiffies(100));
#endif
#endif
}

static void close_device(void) {
    unsigned char val = 0;
    unsigned long timeout = 0;

    atomic_dec(&device_working_flag);
    timeout = jiffies + (HZ / 4);
    while(time_before(jiffies, timeout) && !atomic_read(&devices_configed));

    ofn_bank_select(BANK0);
    do {
        __write_reg(0x06, 0x0a);
        __read_reg(0x06, &val);
    } while (val != 0x0a);
    if (ofndata.pdata->gpio_pd >= 0)
        gpio_direction_output(ofndata.pdata->gpio_pd, 1);
    msleep(5);
    ofndata.run_hrd = false;
}

static long ofn_ioctl(struct file *file, unsigned int cmd,
		      unsigned long arg)
{
    int retval = 0;

    if (_IOC_TYPE(cmd) != PIXART_IOC_MAGIC)
        return -EINVAL;

    if (_IOC_NR(cmd) > PIXART_IOC_MAXNR)
        return -EINVAL;

    if (_IOC_DIR(cmd) & _IOC_READ)
        retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (retval)
        return -EFAULT;

    switch(cmd) {
    case PIXART_IOCTL_SENSOR_ENABLE_DISABLE:
        if (get_user(ofndata.enabled, (int __user *)arg))
            return -EFAULT;

        if (ofndata.enabled) {
            dev_info(&ofndata.client->dev, "set enable\n");
            open_device();
#ifndef CONFIG_SENSORS_PIXART_PAH8001_RESUME_RECONFIG
            wake_lock(&ofndata.work_wake_lock);
#endif
        } else {
            dev_info(&ofndata.client->dev, "set disable\n");
            close_device();
#ifndef CONFIG_SENSORS_PIXART_PAH8001_RESUME_RECONFIG
            wake_unlock(&ofndata.work_wake_lock);
#endif
        }
        break;

    default:
       dev_err(&ofndata.client->dev, "Unsupport IO command\n");
       return -EINVAL;
    }

    return retval;
}

static int ofn_open(struct inode *inode, struct file *filp)
{
    dev_info(&ofndata.client->dev, "%s\n", __FUNCTION__);
    return 0;
}

static int ofn_release(struct inode *inode, struct file *filp)
{
    dev_info(&ofndata.client->dev, "%s\n", __FUNCTION__);
	return 0;
}

static int __init ofn_init(void)
{
	int retval;

/*	retval = i2c_add_driver(&mems_i2c_driver);   //for gsenosor LIS3DH
	if(retval < 0)
	{
		printk(KERN_WARNING "Can't add 8001 driver\n");
		return retval;
	}
*/

	retval = i2c_add_driver(&ofn_i2c_driver);
	if (retval) {
		printk
		    ("****************ofn_name: %s, add ofn i2c driver failed ***************\n",
		     ofn_name);
	}

	return retval;
}

static void __exit ofn_exit(void)
{
	device_destroy(ofndata.ofn_class, MKDEV(ofndata.major_id, 0));	//delete device node under /dev
	class_destroy(ofndata.ofn_class);	//delete class created by us
	unregister_chrdev(ofndata.major_id, ofn_name);
	i2c_del_driver(&ofn_i2c_driver);
//	i2c_del_driver(&mems_i2c_driver);       //for gsenosor LIS3DH
}

//////******************************************************************************************************************///////////////////
/*                                                               //for gsenosor LIS3DH
static int mems_i2c_write(u8 reg, u8 *data, int len)
{
	u8  buf[20];
	int rc;
	int ret = 0;
	int i;

	buf[0] = reg;
	if (len >= 20) {
		printk("%s (%d) : FAILED: buffer size is limitted(20) %d\n", __func__, __LINE__, len);
		dev_err(&memsdata.client->dev, "mems_i2c_write FAILED: buffer size is limitted(20)\n");
		return -1;
	}

	for(i = 0; i < len; i++ ) {
		buf[i+1] = data[i];
	}

	rc = i2c_master_send(memsdata.client, buf, len+1);

	if (rc != len+1) {
		printk("%s (%d) : FAILED: writing to reg 0x%x\n", __func__, __LINE__, reg);
		ret = -1;
	}

	return ret;
}

static int mems_i2c_read(u8 reg, u8 *data, u8 len)
{
	u8  buf[256];
	int rc;

	if(len>1)
		reg |= 0x80;

	buf[0] = reg;

	rc = i2c_master_send(memsdata.client, buf, 1);    // Returns negative errno, or else the number of bytes written.
	if (rc != 1) {
		printk("%s (%d) : FAILED: writing to address 0x%x\n", __func__, __LINE__, reg);
		return -1;
	}

	rc = i2c_master_recv(memsdata.client, data, len);
	if (rc != len) {
		printk("%s (%d) : FAILED: reading data %d\n", __func__, __LINE__, rc);
		return -1;
	}

	return 0;
}

static int mems_init_reg(void)
{
	u8 who_am_i = 0;
	u8 data;
	int ret = -1;

	printk("%s (%d) : \n", __func__, __LINE__);

	ret = mems_i2c_read(0x0F, &who_am_i, 1);
	if(ret) {
		return ret;
	}

	printk("who_ami_i = 0x%x\n", who_am_i);
	if(who_am_i != 0x33) {
		return -1;
	}

	data = 0x67;
	ret = mems_i2c_write(0x20, &data, 1);
	if(ret) {
		return ret;
	}

	data = 0x10;
	ret = mems_i2c_write(0x22, &data, 1);
	if(ret) {
		return ret;
	}

	printk("%s (%d) \n", __func__, __LINE__);

	return 0;
}

static int mems_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	printk("%s (%d) : probe module\n", __func__, __LINE__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		return err;
	}

	memsdata.client = client;

	err = mems_init_reg();
	if (err < 0) {
		return err;
	}

	return err;
}

static int mems_i2c_remove(struct i2c_client *client)
{
	printk("%s (%d) : \n", __func__, __LINE__);
	return 0;
}

static int mems_suspend(struct device *dev)
{
	printk("%s (%d) : \n", __func__, __LINE__);
	return 0;
}

static int mems_resume(struct device *dev)
{
	printk("%s (%d) :  \n", __func__, __LINE__);
	return 0;
}


static const struct i2c_device_id mems_device_id[] = {
	{"st_mems", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mems_device_id);

static const struct dev_pm_ops mems_pm_ops = {
	.suspend = mems_suspend,
	.resume = mems_resume
};

static struct i2c_driver mems_i2c_driver = {
	.driver = {
		.name = "st_mems",
		.owner = THIS_MODULE,
		.pm = &mems_pm_ops},
		.probe = mems_i2c_probe,
		.remove = mems_i2c_remove,
		.id_table = mems_device_id,
};
*/

module_init(ofn_init);
module_exit(ofn_exit);
MODULE_AUTHOR("pixart");
MODULE_DESCRIPTION("pixart ofn driver");
MODULE_LICENSE("GPL");
