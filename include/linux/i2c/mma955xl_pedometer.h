/*
 * mma955xl_pedometer.h
 *
 *  Created on: Aug 27, 2015
 *      Author: stupid
 */
#ifndef __MMA955XL_PEDOMETER_H__
#define __MMA955XL_PEDOMETER_H__

struct mma955xl_platform_data {
        unsigned int sensor_int0_n;
        unsigned int ap_wakeup_mcu;
        unsigned int mcu_wakeup_ap;
};

#endif
