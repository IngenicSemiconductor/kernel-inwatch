#include <gpio.h>
__initdata int gpio_ss_table[][2] = {
	/* GPIO Group - A */
	{32*0+0,	GSS_INPUT_PULL		},	/* NC */
	{32*0+1,	GSS_INPUT_PULL		},	/* NC */
	{32*0+2,	GSS_INPUT_PULL		},	/* NC */
	{32*0+3,	GSS_INPUT_PULL		},	/* NC */
	{32*0+4,	GSS_INPUT_NOPULL	},	/* MSC0_D4 */
	{32*0+5,	GSS_INPUT_NOPULL	},	/* MSC0_D5 */
	{32*0+6,	GSS_INPUT_NOPULL	},	/* MSC0_D6 */
	{32*0+7,	GSS_INPUT_NOPULL	},	/* MSC0_D7 */
	{32*0+8,	GSS_INPUT_PULL		},	/* NC */
	{32*0+9,	GSS_INPUT_PULL		},	/* NC */
	{32*0+10,	GSS_INPUT_PULL		},	/* NC */
	{32*0+11,	GSS_INPUT_PULL		},	/* NC */
	{32*0+12,	GSS_INPUT_PULL		},	/* NC */
	{32*0+13,	GSS_INPUT_PULL		},	/* NC */
	{32*0+14,	GSS_INPUT_PULL		},	/* NC */
	{32*0+15,	GSS_INPUT_PULL 		},	/* NC */
	{32*0+18,	GSS_IGNORE		},	/* MSC0_CLK */
	{32*0+19,	GSS_INPUT_NOPULL	},	/* MSC0_CMD */
	{32*0+20,	GSS_INPUT_NOPULL	},	/* MSC0_D0 */
	{32*0+21,	GSS_INPUT_NOPULL	},	/* MSC0_D1 */
	{32*0+22,	GSS_INPUT_NOPULL	},	/* MSC0_D2 */
	{32*0+23,	GSS_INPUT_NOPULL	},	/* MSC0_D3 */
	{32*0+29,	GSS_IGNORE		},	/* NC */
	{32*0+30,	GSS_IGNORE		},	/* WKUP_N */
	{32*0+31,	GSS_IGNORE		},	/* NC */

	/* GPIO Group - B */
	{32*1+0,	GSS_INPUT_PULL		},	/* NC */
	{32*1+1,	GSS_INPUT_PULL		},	/* NC */
	{32*1+7,	GSS_INPUT_NOPULL	},	/* SMB3_SDA */
	{32*1+8,	GSS_INPUT_NOPULL	},	/* SMB3_SCK */
	{32*1+20,   	GSS_INPUT_PULL		},  	/* NC */
	{32*1+21,   	GSS_INPUT_PULL    	},  	/* NC */
    	{32*1+28,	GSS_INPUT_PULL    	},  	/* NC */
    	{32*1+29,	GSS_INPUT_PULL    	},  	/* NC */
    	{32*1+30,	GSS_INPUT_PULL    	},  	/* NC */
    	{32*1+31,	GSS_INPUT_PULL    	},  	/* NC */

	/* GPIO Group - C */
	{32*2+2,	GSS_INPUT_PULL		},	/* NC */
	{32*2+3,	GSS_INPUT_PULL 		},	/* NC */
	{32*2+4,	GSS_INPUT_PULL		},	/* NC */
	{32*2+5,	GSS_INPUT_PULL		},	/* NC */
	{32*2+6,	GSS_INPUT_PULL		},	/* NC */
	{32*2+7,	GSS_INPUT_PULL		},	/* NC */
	{32*2+8,	GSS_INPUT_PULL		},	/* NC */
	{32*2+9,	GSS_INPUT_PULL		},	/* NC */
	{32*2+12,	GSS_INPUT_PULL		},	/* SENSOR_INT0_N */
	{32*2+13,	GSS_OUTPUT_HIGH		},	/* TP_WKUP/RST */
	{32*2+14,	GSS_OUTPUT_HIGH		},	/* TP_IRQ_N	*/
	{32*2+15,	GSS_IGNORE		},	/* PMU_SLEEP */
	{32*2+16,	GSS_IGNORE		},	/* HUB_RESPONSE_HOST */
	{32*2+17,	GSS_IGNORE		},	/* PMU_INTB */
	{32*2+18,	GSS_IGNORE		},	/* Vibra_PWREN */
#ifdef CONFIG_SLPT
	{32*2+19,	GSS_IGNORE		},	/* AMOLED_RST	@@@*/
#else
	{32*2+19,	GSS_OUTPUT_LOW		},	/* SPK_EN */
#endif
	{32*2+22,	GSS_INPUT_PULL		},	/* NC */
	{32*2+23,	GSS_INPUT_PULL		},	/* NC */
	{32*2+24,	GSS_INPUT_PULL		},	/* NC */
	{32*2+25,	GSS_INPUT_PULL		},	/* NC */
	{32*2+26,	GSS_INPUT_PULL		},	/* NC */
	{32*2+27,	GSS_INPUT_PULL		},	/* NC */

	/* GPIO Group - D */
	{32*3+0, 	GSS_IGNORE		},	/* LCD_PWR_ON */
	{32*3+1,	GSS_IGNORE		},	/* Health_RST_N */
	{32*3+2,	GSS_IGNORE		},	/* Health_INT_N */
	{32*3+3,	GSS_IGNORE		},	/* LCD_RST */
	{32*3+4,	GSS_IGNORE		},	/* LCD_TE */
	{32*3+5,	GSS_IGNORE		},	/* Camera_nRST */
	{32*3+6,	GSS_IGNORE		},	/* PRE-TEST */
	{32*3+7,	GSS_IGNORE		},	/* USB_DET_N */
	{32*3+8,	GSS_IGNORE		},	/* HOST_WAKEUP_BT */
	{32*3+9,	GSS_IGNORE		},	/* BT_nRST/REGEN */
	{32*3+10,	GSS_IGNORE		},	/* BT_WAKEUP_HOST */
	{32*3+11,	GSS_IGNORE		},	/*  */
	{32*3+12,	GSS_OUTPUT_LOW		},	/* WL_WAKEUP_HOST */
	{32*3+13,	GSS_OUTPUT_LOW		},	/* WL_REG_EN */
	{32*3+14,	GSS_INPUT_NOPULL	},	/* CLK32K */
	{32*3+17,	GSS_INPUT_NOPULL	},	/* BOOT */
	{32*3+18,	GSS_INPUT_NOPULL	},	/* BOOT_SEL1 */
	{32*3+19,	GSS_INPUT_NOPULL	},	/* BOOT */
	{32*3+26,	GSS_IGNORE		},	/* UART1_RXD_BT */
	{32*3+27,	GSS_IGNORE		},	/* UART1_CTS_BT */
	{32*3+28,	GSS_IGNORE		},	/* UART1_RTS_BT */
	{32*3+29,	GSS_IGNORE		},	/* UART1_TXD_BT */
	{32*3+30,	GSS_OUTPUT_HIGH		},	/* SMB0_SDA_PMU */
	{32*3+31,	GSS_OUTPUT_HIGH		},	/* SMB0_SCK_PMU */

	/* GPIO Group - E */
	{32*4+0,	GSS_INPUT_NOPULL	},	/* SMB2_SDA */
	{32*4+1, 	GSS_OUTPUT_LOW		},	/* PWM_LCD */
	{32*4+2, 	GSS_IGNORE			},	/* Vibra_IN/TRIG */
	{32*4+3,	GSS_INPUT_NOPULL	},	/* SMB2_SCK */
	{32*4+10,	GSS_IGNORE		},	/* USB_DETE 	@@@*/
	{32*4+20,	GSS_IGNORE		},	/* SDIO_D0_WIFI */
	{32*4+21,	GSS_IGNORE		},	/* SDIO_D1_WIFI */
	{32*4+22,	GSS_IGNORE		},	/* SDIO_D2_WIFI */
	{32*4+23,	GSS_IGNORE		},	/* SDIO_D3_WIFI */
	{32*4+28,	GSS_IGNORE		},	/* SDIO_CLK_WIFI @@@*/
	{32*4+29,	GSS_IGNORE		},	/* SDIO_CMD_WIFI */
	{32*4+30,	GSS_INPUT_NOPULL	},	/* SMB1_SDA */
	{32*4+31,	GSS_INPUT_NOPULL	},	/* SMB1_SCK */

	/* GPIO Group - F */
	{32*5+0,	GSS_IGNORE		},	/* UART0_RXD_HUB */
	{32*5+1,	GSS_IGNORE		},	/* UART0_CTS_HUB */
	{32*5+2,	GSS_IGNORE		},	/* UART0_RTS_HUB */
	{32*5+3,	GSS_IGNORE		},	/* UART0_TXD_HUB */
#ifdef CONFIG_JZ_DMIC_WAKEUP
	{32*5+6,	GSS_IGNORE		},	/* DMIC_CLK 	@@@*/
	{32*5+7,	GSS_IGNORE		},	/* DMIC_DOUT 	@@@*/
#else
	{32*5+6,	GSS_OUTPUT_LOW		},	/* DMIC_CLK 	@@@*/
	{32*5+7,	GSS_INPUT_PULL		},	/* DMIC_DOUT 	@@@*/
#endif
	{32*5+12,	GSS_IGNORE		},	/* BT_PCM_DO */
	{32*5+13,	GSS_IGNORE		},	/* BT_PCM_CLK */
	{32*5+14,	GSS_IGNORE		},	/* BT_PCM_SYN */
	{32*5+15,	GSS_IGNORE		},	/* BT_PCM_DI */
	{GSS_TABLET_END, GSS_TABLET_END		}
};
