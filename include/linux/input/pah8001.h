#ifndef __PAHEARTRATE__
#define __PAHEARTRATE__

struct pah8001_platform_data {
    int gpio_int;
    int gpio_reset;
    int gpio_pd;
    int (*board_init)(struct device *dev);
    int (*board_exit)(struct device *dev);
    int (*power_on)(void);
    int (*power_off)(void);
};

typedef enum {
	BANK0 = 0,
	BANK1,
} bank_e;


#ifdef PAH8001
static const unsigned char init_ppg_register_array_1000[][2] = {
{0x7F,0x00},
{0x09,0x5A},
{0x05,0x99},
{0x17,0xA2}, //read and write bit7=1
{0x27,0xFF},
{0x28,0xFA},
{0x29,0x0A},
{0x2A,0xC8},
{0x2B,0xA0},
{0x2C,0x8C},
{0x2D,0x64},
{0x42,0x20},
{0x4D,0x18},
{0x7A,0xB5}, //default
{0x7F,0x01},
{0x07,0x48},
{0x2E,0x48},
{0x38,0xE4},
{0x42,0xA4},
{0x43,0x41},
{0x44,0x41},
{0x45,0x3F},
{0x46,0x00},
//{0x52,0x05}, // -- 200Hz
{0x52,0x32},	// -- 20Hz
{0x53,0x28},
{0x56,0x60},
{0x57,0x28},
{0x6D,0x02},
{0x0F,0xC8}, //default
{0x7F,0x00},
{0x5D,0x81},
};

#define INIT_PPG_REG_ARRAY_SIZE_1000 (sizeof(init_ppg_register_array_1000)/sizeof(init_ppg_register_array_1000[0]))

#else
#ifdef PAJ3007
unsigned char init_ofn_register_array[][2] = {	// initial
{0x7F, 0x00},
//{0x06, 0x82},//Soft Reset, ****Need Delay****
{0x09, 0x5A},
{0x0D, 0x0A},
{0x4D, 0x17},
{0x7F, 0x01},
{0x2E, 0x48},
{0x7F, 0x00},
};

#define INIT_OFN_REG_ARRAY_SIZE (sizeof(init_ofn_register_array)/sizeof(init_ofn_register_array[0]))

//for ofn & ppg
static const unsigned char init_ppg_register_array[][2] = {
{0x09,0x5A},
{0x05,0xA9},
{0x0D,0x0A},
{0x1D,0x1B},
{0x27,0xFF},
{0x28,0xFA},
{0x29,0x0A},
{0x2A,0xC8},
{0x2B,0xA0},
{0x2C,0x8C},
{0x2D,0x64},
{0x42,0x20},
{0x4D,0x17},
{0x4C,0x94},//X=500CPI
{0x4F,0x14},//Y=500CPI
{0x7A,0x35},
{0x7F,0x01},
{0x27,0x2A},
{0x2E,0x48},
{0x38,0xFC},
{0x42,0xA6},
{0x45,0x26},
{0x46,0xC0},
//{0x52 , 0x0F}, // -- 200Hz
{0x52,0x96},	// -- 20Hz
{0x53,0x28},
{0x56,0x60},
{0x57,0x28},
{0x0F,0xE8},
{0x7F,0x00},
{0x5D,0x81},
};

#define INIT_PPG_REG_ARRAY_SIZE (sizeof(init_ppg_register_array)/sizeof(init_ppg_register_array[0]))
#endif
#endif

#endif //__PAHEARTRATE__
