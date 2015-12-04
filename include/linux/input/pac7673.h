#ifndef __PAC7673_H__
#define __PAC7673_H__

#include <linux/ioctl.h>

struct pac7673_platform_data {
    int gpio_int;
    int (*board_init)(struct device *dev);
    int (*board_exit)(struct device *dev);
    int (*power_on)(void);
    int (*power_off)(void);
};

//mofidy at 21020420 by zhu end
//SYSTEM
#define ALPS_PAC7673_DEVICE_ID                  0x00
#define ALPS_PAC7673_OPCON                      0x01
#define ALPS_PAC7673_INT_FLAG                   0x02
#define ALPS_PAC7673_ALS_WAIT_TIME              0x03
#define ALPS_PAC7673_PS_WAIT_TIME               0x04
#define ALPS_PAC7673_ALS_ANA_WAKEUP_TIME        0x05
#define ALPS_PAC7673_PS_ANA_WAKEUP_TIME         0x06

// AMBIENT LIGHT SENSOR
#define ALPS_PAC7673_ALS_CON                    0x08
#define ALPS_PAC7673_ALS_SATUS                  0x09
#define ALPS_PAC7673_ALS_CNT_H                  0x0A
#define ALPS_PAC7673_ALS_CNT_M                  0x0B
#define ALPS_PAC7673_ALS_CNT_L                  0x0C
#define ALPS_PAC7673_ALS_IHTH                   0x0D
#define ALPS_PAC7673_ALS_IHTL                   0x0E
#define ALPS_PAC7673_ALS_ILTH                   0x0F
#define ALPS_PAC7673_ALS_ILTL                   0x10
#define ALPS_PAC7673_ALS_INTG                   0x11
#define ALPS_PAC7673_ALS_STEPH                  0x12
#define ALPS_PAC7673_ALS_STEPL                  0x13
#define ALPS_PAC7673_ALS_ANA_CON2               0x15

// PROXIMITY SENSOR
#define ALPS_PAC7673_PS_CON                     0x19
#define ALPS_PAC7673_PS_BIT_CHECK               0x1A
#define ALPS_PAC7673_PS_CNT_STEP                0x1B
#define ALPS_PAC7673_PS_MTIME                   0x1C
#define ALPS_PAC7673_PS_CYC_THR                 0x1D
#define ALPS_PAC7673_PS_LED_DET                 0x1E
#define ALPS_PAC7673_PS_LED_REL                 0x1F
#define ALPS_PAC7673_PS_LED_PAT                 0x20
#define ALPS_PAC7673_PS_DET_REV                 0x21
#define ALPS_PAC7673_PS_ANA_CON1                0x22
#define ALPS_PAC7673_PS_ANA_CON2                0x23
#define ALPS_PAC7673_PS_ANA_CON3                0x24


#define PAC7673_SUCCESS                         0
#define PAC7673_ERR_I2C                         -1
#define PAC7673_ERR_STATUS                      -3
#define PAC7673_ERR_SETUP_FAILURE               -4
#define PAC7673_ERR_GETGSENSORDATA              -5
#define PAC7673_ERR_IDENTIFICATION              -6

struct sensor_device_attribute{
    struct device_attribute dev_attr;
    int index;
};

#define SENSOR_ATTR(_name, _mode, _show, _store, _index)    \
    { .dev_attr = __ATTR(_name, _mode, _show, _store),  \
      .index = _index }

#define SENSOR_DEVICE_ATTR(_name, _mode, _show, _store, _index) \
struct sensor_device_attribute sensor_dev_attr_##_name      \
    = SENSOR_ATTR(_name, _mode, _show, _store, _index)

#endif
