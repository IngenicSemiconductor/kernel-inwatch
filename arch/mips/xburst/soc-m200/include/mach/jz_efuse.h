#ifndef __JZ_EFUSE_H__
#define __JZ_EFUSE_H__

struct jz_efuse_platform_data {
	int gpio_vddq_en;	/* supply 2.5V to VDDQ */
	int gpio_vddq_en_level;
};

#ifdef CONFIG_JZ_EFUSE_V12
int read_jz_efuse(uint32_t xaddr, uint32_t xlen, void *buf);
int read_jz_efuse_chip_id(void *buf);
int read_jz_efuse_chip_num(void *buf);
#else
static inline int read_jz_efuse(uint32_t xaddr, uint32_t xlen, void *buf)
{
	return -ENODEV;
}
static inline int read_jz_efuse_chip_id(void *buf)
{
	return -ENODEV;
}
static inline int read_jz_efuse_chip_num(void *buf)
{
	return -ENODEV;
}

#endif
#endif
