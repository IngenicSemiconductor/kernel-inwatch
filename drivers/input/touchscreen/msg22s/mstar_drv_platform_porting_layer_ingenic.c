////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_platform_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_interface.h"

#include <linux/tsc.h>

/*=============================================================*/
// EXTREN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct kset *g_TouchKSet;
extern struct kobject *g_TouchKObj;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_FaceClosingTp;
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

extern struct regulator *g_ReguVdd;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

struct mutex g_Mutex;
static struct work_struct _gFingerTouchWork;

static struct early_suspend _gEarlySuspend;

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
const int g_TpVirtualKey[] = {TOUCH_KEY_MENU, TOUCH_KEY_HOME, TOUCH_KEY_BACK, TOUCH_KEY_SEARCH};
#endif //CONFIG_TP_HAVE_KEY

struct input_dev *g_InputDevice = NULL;
static int _gIrq = -1;

static int g_gpio_irq = 0;
static int g_gpio_rst = 0;
/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/

/* read data through I2C then report data to input sub-system when interrupt occurred */
static void _DrvPlatformLyrFingerTouchDoWork(struct work_struct *pWork)
{
    DBG("*** %s() ***\n", __func__);

    DrvIcFwLyrHandleFingerTouch(NULL, 0);

//    enable_irq(MS_TS_MSG_IC_GPIO_INT);
    enable_irq(_gIrq);
}

/* The interrupt service routine will be triggered when interrupt occurred */
static irqreturn_t _DrvPlatformLyrFingerTouchInterruptHandler(s32 nIrq, void *pDeviceId)
{
    DBG("*** %s() ***\n", __func__);

//    disable_irq_nosync(MS_TS_MSG_IC_GPIO_INT);
    disable_irq_nosync(_gIrq);
    schedule_work(&_gFingerTouchWork);

    return IRQ_HANDLED;
}

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

void DrvPlatformLyrTouchDeviceRegulatorPowerOn(void)
{
//    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

//    nRetVal = regulator_set_voltage(g_ReguVdd, 2800000, 2800000); // For specific SPRD BB chip(ex. SC7715) or QCOM BB chip(ex. MSM8610), need to enable this function call for correctly power on Touch IC.

//    if (nRetVal)
//    {
//        DBG("Could not set to 2800mv.\n");
//    }
    regulator_enable(g_ReguVdd);

    mdelay(20);
}

void DrvPlatformLyrTouchDevicePowerOn(void)
{
	DBG("*** %s() ***\n", __func__);
    gpio_direction_output(g_gpio_rst, 1);
//    gpio_set_value(g_gpio_rst, 1);
    udelay(100);
    gpio_set_value(g_gpio_rst, 0);
    udelay(100);
    gpio_set_value(g_gpio_rst, 1);
    mdelay(25);
}

void DrvPlatformLyrTouchDevicePowerOff(void)
{
    DBG("*** %s() ***\n", __func__);

    DrvIcFwLyrOptimizeCurrentConsumption();

//    gpio_direction_output(g_gpio_rst, 0);
    gpio_set_value(g_gpio_rst, 0);
}

void DrvPlatformLyrTouchDeviceResetHw(void)
{
    DBG("*** %s() ***\n", __func__);

    gpio_direction_output(g_gpio_rst, 1);
//    gpio_set_value(g_gpio_rst, 1);
    gpio_set_value(g_gpio_rst, 0);
    mdelay(100);
    gpio_set_value(g_gpio_rst, 1);
    mdelay(100);
}

void DrvPlatformLyrDisableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

//    disable_irq(MS_TS_MSG_IC_GPIO_RST);
    disable_irq(_gIrq);
}

void DrvPlatformLyrEnableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

//    enable_irq(MS_TS_MSG_IC_GPIO_RST);
    enable_irq(_gIrq);
}

void DrvPlatformLyrFingerTouchPressed(s32 nX, s32 nY, s32 nPressure, s32 nId)
{

/****************************************** ingenic add **************************************/
#if defined(CONFIG_TSC_SWAP_X) || defined(CONFIG_TSC_SWAP_Y)
        struct msg22s_platform_data *pdata = (struct msg22s_platform_data *)
													g_InputDevice->dev.parent->platform_data;
#endif
/****************************************** ingenic add **************************************/
    DBG("*** %s() ***\n", __func__);
    DBG("point touch pressed\n");

#ifdef CONFIG_TSC_SWAP_XY
		tsc_swap_xy((u16 *)&nX, (u16 *)&nY);
#endif

#ifdef CONFIG_TSC_SWAP_X
		tsc_swap_x((u16 *)&nX, pdata->touch_screen_x_max);
#endif

#ifdef CONFIG_TSC_SWAP_Y
		tsc_swap_y((u16 *)&nY, pdata->touch_screen_y_max);
#endif

    input_report_key(g_InputDevice, BTN_TOUCH, 1);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, nId);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_report_abs(g_InputDevice, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_WIDTH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_X, nX);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_Y, nY);

    input_mt_sync(g_InputDevice);
}

void DrvPlatformLyrFingerTouchReleased(s32 nX, s32 nY)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch released\n");

    input_report_key(g_InputDevice, BTN_TOUCH, 0);
    input_mt_sync(g_InputDevice);
}

s32 DrvPlatformLyrInputDeviceInitialize(struct i2c_client *pClient)
{
    s32 nRetVal = 0;
    struct msg22s_platform_data *pdata =
			(struct msg22s_platform_data *)(pClient->dev.platform_data);

    DBG("*** %s() ***\n", __func__);

    mutex_init(&g_Mutex);

    /* allocate an input device */
    g_InputDevice = input_allocate_device();
    if (g_InputDevice == NULL)
    {
        DBG("*** input device allocation failed ***\n");
        return -ENOMEM;
    }

    g_InputDevice->name = pClient->name;
    g_InputDevice->phys = "I2C";
    g_InputDevice->dev.parent = &pClient->dev;
    g_InputDevice->id.bustype = BUS_I2C;

    g_gpio_irq = pdata->irq;
    g_gpio_rst = pdata->reset;

    /* set the supported event type for input device */
    set_bit(EV_ABS, g_InputDevice->evbit);
    set_bit(EV_SYN, g_InputDevice->evbit);
    set_bit(EV_KEY, g_InputDevice->evbit);
    set_bit(BTN_TOUCH, g_InputDevice->keybit);
    set_bit(INPUT_PROP_DIRECT, g_InputDevice->propbit);

#ifdef CONFIG_TP_HAVE_KEY
    {
        u32 i;
        for (i = 0; i < MAX_KEY_NUM; i ++)
        {
            input_set_capability(g_InputDevice, EV_KEY, g_TpVirtualKey[i]);
        }
    }
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(g_InputDevice, EV_KEY, KEY_POWER);
    input_set_capability(g_InputDevice, EV_KEY, KEY_UP);
    input_set_capability(g_InputDevice, EV_KEY, KEY_DOWN);
    input_set_capability(g_InputDevice, EV_KEY, KEY_LEFT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_RIGHT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_W);
    input_set_capability(g_InputDevice, EV_KEY, KEY_Z);
    input_set_capability(g_InputDevice, EV_KEY, KEY_V);
    input_set_capability(g_InputDevice, EV_KEY, KEY_O);
    input_set_capability(g_InputDevice, EV_KEY, KEY_M);
    input_set_capability(g_InputDevice, EV_KEY, KEY_C);
    input_set_capability(g_InputDevice, EV_KEY, KEY_E);
    input_set_capability(g_InputDevice, EV_KEY, KEY_S);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*
#ifdef CONFIG_TP_HAVE_KEY
    set_bit(TOUCH_KEY_MENU, g_InputDevice->keybit); //Menu
    set_bit(TOUCH_KEY_HOME, g_InputDevice->keybit); //Home
    set_bit(TOUCH_KEY_BACK, g_InputDevice->keybit); //Back
    set_bit(TOUCH_KEY_SEARCH, g_InputDevice->keybit); //Search
#endif
*/

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_set_abs_params(g_InputDevice, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_set_abs_params(g_InputDevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_X, pdata->touch_screen_x_min, pdata->touch_screen_x_max/* TOUCH_SCREEN_X_MAX */, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_Y, pdata->touch_screen_y_min, pdata->touch_screen_y_max/* TOUCH_SCREEN_Y_MAX */, 0, 0);

    /* register the input device to input sub-system */
    nRetVal = input_register_device(g_InputDevice);
    if (nRetVal < 0)
    {
        DBG("*** Unable to register touch input device ***\n");
    }

    return nRetVal;
}

s32 DrvPlatformLyrTouchDeviceRequestGPIO(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    nRetVal = gpio_request(g_gpio_rst, "C_TP_RST");
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", g_gpio_rst, nRetVal);
    }

    nRetVal = gpio_request(g_gpio_irq, "C_TP_INT");
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", g_gpio_irq, nRetVal);
    }

    return nRetVal;
}

s32 DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    if (DrvIcFwLyrIsRegisterFingerTouchInterruptHandler())
    {
        /* initialize the finger touch work queue */
        INIT_WORK(&_gFingerTouchWork, _DrvPlatformLyrFingerTouchDoWork);

        _gIrq = gpio_to_irq(g_gpio_irq);

        /* request an irq and register the isr */
        nRetVal = request_irq(_gIrq/*MS_TS_MSG_IC_GPIO_INT*/, _DrvPlatformLyrFingerTouchInterruptHandler,
                      IRQF_TRIGGER_RISING /* | IRQF_NO_SUSPEND *//* IRQF_TRIGGER_FALLING */,
                      "msg2xxx", NULL);
        if (nRetVal != 0)
        {
            DBG("*** Unable to claim irq %d; error %d ***\n", g_gpio_irq, nRetVal);
        }
    }

    return nRetVal;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
void DrvPlatformLyrTouchDeviceRegisterEarlySuspend(void)
{
    DBG("*** %s() ***\n", __func__);

    _gEarlySuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    _gEarlySuspend.suspend = MsDrvInterfaceTouchDeviceSuspend;
    _gEarlySuspend.resume = MsDrvInterfaceTouchDeviceResume;
    register_early_suspend(&_gEarlySuspend);
}
#endif
/* remove function is triggered when the input device is removed from input sub-system */
s32 DrvPlatformLyrTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

//    free_irq(MS_TS_MSG_IC_GPIO_INT, g_InputDevice);
    free_irq(_gIrq, g_InputDevice);
    gpio_free(g_gpio_irq);
    gpio_free(g_gpio_rst);
    input_unregister_device(g_InputDevice);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    kset_unregister(g_TouchKSet);
    kobject_put(g_TouchKObj);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    return 0;
}

void DrvPlatformLyrSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() nIicDataRate = %d ***\n", __func__, nIicDataRate);

    // TODO : Please FAE colleague to confirm with customer device driver engineer for how to set i2c data rate on SPRD platform
//    sprd_i2c_ctl_chg_clk(pClient->adapter->nr, nIicDataRate);
//    mdelay(100);
}

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION

int DrvPlatformLyrGetTpPsData(void)
{
    DBG("*** %s() g_FaceClosingTp = %d ***\n", __func__, g_FaceClosingTp);

    return g_FaceClosingTp;
}

void DrvPlatformLyrTpPsEnable(int nEnable)
{
    DBG("*** %s() nEnable = %d ***\n", __func__, nEnable);

    if (nEnable)
    {
        DrvIcFwLyrEnableProximity();
    }
    else
    {
        DrvIcFwLyrDisableProximity();
    }
}

#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

//------------------------------------------------------------------------------//

