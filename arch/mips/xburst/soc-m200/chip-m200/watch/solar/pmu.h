
#ifndef __PMU_H__
#define __PMU_H__
#ifdef CONFIG_REGULATOR_RICOH619

#define PMU_I2C_BUSNUM  0

/* ****************************PMU DC/LDO NAME******************************* */
#define DC1_NAME       "cpu_core"
#define DC2_NAME       "cpu_vmema"
#define DC3_NAME       "cpu_mem12"
#define DC4_NAME       "cpu_vddio"
#define DC5_NAME       "DC5"        // speaker motor
#define LDO1_NAME      "blvss"      // ELVSS-3.4V
#define LDO2_NAME      "emmc_vcc"
#define LDO3_NAME      "mpu9255"
#define LDO4_NAME      "lcd_1v8"
#define LDO5_NAME      "cpu_2v5"    //always on
#define LDO6_NAME      "lcd_2v8"
#define LDO7_NAME      "sensor3v3"
#define LDO8_NAME      "sensor1v8"
#define LDO9_NAME      "tp1v8"
#define LDO10_NAME     "tp2v8"
#define LDORTC1_NAME   "rtc_1v8"
#define LDORTC2_NAME   "rtc_1v1"

/* ****************************PMU DC/LDO NAME END*************************** */

/* ****************************PMU DC/LDO DEFAULT V************************** */
#define DC1_INIT_UV     1175
#define DC2_INIT_UV     1200
#define DC3_INIT_UV     3300
#define DC4_INIT_UV     1800
#define DC5_INIT_UV     3300
#define LDO1_INIT_UV    1800
#define LDO2_INIT_UV    3300
#define LDO3_INIT_UV    2500
#define LDO4_INIT_UV    1800
#define LDO5_INIT_UV    2500
#define LDO6_INIT_UV    2800
#define LDO7_INIT_UV    3000
#define LDO8_INIT_UV    1800
#define LDO9_INIT_UV    1800
#define LDO10_INIT_UV   2800
#define LDORTC1_INIT_UV 1800
#define LDORTC2_INIT_UV 1100
/* ****************************PMU DC/LDO DEFAULT V END********************** */

/* ****************************PMU DC/LDO ALWAYS ON************************** */
#define DC1_ALWAYS_ON     1
#define DC2_ALWAYS_ON     1
#define DC3_ALWAYS_ON     0
#define DC4_ALWAYS_ON     1
#ifdef CONFIG_SLPT
#define DC5_ALWAYS_ON     1
#define LDO4_ALWAYS_ON    1
#define LDO6_ALWAYS_ON    1
#else
#define DC5_ALWAYS_ON     0
#define LDO4_ALWAYS_ON    0
#define LDO6_ALWAYS_ON    0
#endif
#define LDO1_ALWAYS_ON    1
#define LDO2_ALWAYS_ON    1
#define LDO3_ALWAYS_ON    1
#define LDO5_ALWAYS_ON    1
#define LDO7_ALWAYS_ON    0
#define LDO8_ALWAYS_ON    0
#define LDO9_ALWAYS_ON    0
#define LDO10_ALWAYS_ON   0
#define LDORTC1_ALWAYS_ON 1
#define LDORTC2_ALWAYS_ON 1
/* ****************************PMU DC/LDO ALWAYS ON END********************** */

/* ****************************PMU DC/LDO BOOT ON**************************** */
#define DC1_BOOT_ON     1
#define DC2_BOOT_ON     1
#define DC3_BOOT_ON     0
#define DC4_BOOT_ON     1
#define DC5_BOOT_ON     1
#define LDO1_BOOT_ON    1
#define LDO2_BOOT_ON    1
#define LDO3_BOOT_ON    0
#define LDO4_BOOT_ON    1
#define LDO5_BOOT_ON    0
#define LDO6_BOOT_ON    1
#define LDO7_BOOT_ON    0
#define LDO8_BOOT_ON    0
#define LDO9_BOOT_ON    0
#define LDO10_BOOT_ON   0
#define LDORTC1_BOOT_ON 1
#define LDORTC2_BOOT_ON 1
/* ****************************PMU DC/LDO BOOT ON END************************ */

/* ****************************PMU DC/LDO INIT ENABLE************************ */
#define DC1_INIT_ENABLE     DC1_BOOT_ON
#define DC2_INIT_ENABLE     DC2_BOOT_ON
#define DC3_INIT_ENABLE     DC3_BOOT_ON
#define DC4_INIT_ENABLE     DC4_BOOT_ON
#define DC5_INIT_ENABLE     DC5_BOOT_ON
#define LDO1_INIT_ENABLE    LDO1_BOOT_ON
#define LDO2_INIT_ENABLE    LDO2_BOOT_ON
#define LDO3_INIT_ENABLE    LDO3_BOOT_ON
#define LDO4_INIT_ENABLE    LDO4_BOOT_ON
#define LDO5_INIT_ENABLE    LDO5_BOOT_ON
#define LDO6_INIT_ENABLE    LDO6_BOOT_ON
#define LDO7_INIT_ENABLE    LDO7_BOOT_ON
#define LDO8_INIT_ENABLE    LDO8_BOOT_ON
#define LDO9_INIT_ENABLE    LDO9_BOOT_ON
#define LDO10_INIT_ENABLE   LDO10_BOOT_ON
#define LDORTC1_INIT_ENABLE LDORTC1_BOOT_ON
#define LDORTC2_INIT_ENABLE LDORTC2_BOOT_ON
/* ****************************PMU DC/LDO INIT ENABLE END******************** */
#endif	/* CONFIG_REGULATOR_RICOH619 */
#endif
