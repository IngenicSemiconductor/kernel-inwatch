#include <gpio.h>

#if 1
__initdata int gpio_ss_table[][2] = {
    /* GPIO Group - A */
    { 32 * 0 + 0, GSS_INPUT_NOPULL }, /* NC */
    { 32 * 0 + 1, GSS_IGNORE }, /* BT_PWR_EN */
    { 32 * 0 + 2, GSS_IGNORE }, /* HOST_WAKE_NFC */
    { 32 * 0 + 3, GSS_IGNORE }, /* BT_REG_EN */
    { 32 * 0 + 4, GSS_IGNORE }, /* MSC0_D4 */
    { 32 * 0 + 5, GSS_IGNORE }, /* MSC0_D5 */
    { 32 * 0 + 6, GSS_IGNORE }, /* MSC0_D6 */
    { 32 * 0 + 7, GSS_IGNORE }, /* MSC0_D7 */

    { 32 * 0 + 8, GSS_IGNORE }, /* HOST_WAKE_WL */
    { 32 * 0 + 9, GSS_IGNORE }, /* WL_REG_EN */
    { 32 * 0 + 10, GSS_IGNORE }, /* HOST_BT_RST */

    { 32 * 0 + 11, GSS_IGNORE }, /* CTP_WAKE_UP */
    { 32 * 0 + 12, GSS_IGNORE }, /* CTP_IRQ */
    { 32 * 0 + 13, GSS_IGNORE }, /* DMIC_LR */
    { 32 * 0 + 14, GSS_IGNORE }, /* CHARGEN_S*/
    { 32 * 0 + 15, GSS_IGNORE }, /* PMU_SLEEP */

    { 32 * 0 + 18, GSS_IGNORE }, /* MSC0_CLK */
    { 32 * 0 + 19, GSS_IGNORE }, /* MSC0_CMD */
    { 32 * 0 + 20, GSS_IGNORE }, /* MSC0_D0 */
    { 32 * 0 + 21, GSS_IGNORE }, /* MSC0_D1 */
    { 32 * 0 + 22, GSS_IGNORE }, /* MSC0_D2 */
    { 32 * 0 + 23, GSS_IGNORE }, /* MSC0_D3 */
    { 32 * 0 + 29, GSS_IGNORE }, /* USB_DETE */
    { 32 * 0 + 30, GSS_IGNORE }, /* WAKE_UP*/
    { 32 * 0 + 31, GSS_IGNORE }, /* NC */

    /* GPIO Group - B */
    { 32 * 1 + 0, GSS_IGNORE }, /* MSC0_RST_N */
    { 32 * 1 + 1, GSS_IGNORE }, /* SP_SHUTDOWN_N */

    { 32 * 1 + 7, GSS_INPUT_NOPULL }, /* CIM_SMB3_SDA */
    { 32 * 1 + 8, GSS_INPUT_NOPULL }, /* CIM_SMB3_SDA */

    /* GPIO Group - C */
    { 32 * 2 + 2, GSS_OUTPUT_LOW }, /* SLCD_D0 */
    { 32 * 2 + 3, GSS_OUTPUT_LOW }, /* SLCD_D1*/
    { 32 * 2 + 4, GSS_OUTPUT_LOW }, /* SLCD_D2*/
    { 32 * 2 + 5, GSS_OUTPUT_LOW }, /* SLCD_D3*/
    { 32 * 2 + 6, GSS_OUTPUT_LOW }, /* SLCD_D4*/
    { 32 * 2 + 7, GSS_OUTPUT_LOW }, /* SLCD_D5*/
    { 32 * 2 + 8, GSS_OUTPUT_LOW }, /* SLCD_D6*/
#if defined(CONFIG_LCD_X163) && defined(CONFIG_SLPT)
    { 32 * 2 + 9, GSS_IGNORE }, /* SLCD_D7*/
#else
    { 32 * 2 + 9, GSS_OUTPUT_LOW }, /* SLCD_D7*/
#endif
    { 32 * 2 + 12, GSS_IGNORE }, /* WL_WAKE_HOST */
    { 32 * 2 + 13, GSS_IGNORE }, /* HOST_WAKE_BT */
    { 32 * 2 + 14, GSS_IGNORE }, /* BT_WAKE_HOST */
    { 32 * 2 + 15, GSS_IGNORE }, /* WL_PWR_EN */

    { 32 * 2 + 16, GSS_IGNORE }, /* LCD_RESET */
    { 32 * 2 + 17, GSS_IGNORE }, /* LCD_INT */
    { 32 * 2 + 18, GSS_IGNORE }, /* LCD_HSYN */
    { 32 * 2 + 19, GSS_IGNORE }, /* LCD_VSYN */
    { 32 * 2 + 22, GSS_IGNORE }, /* ISP_CLK */
    { 32 * 2 + 23, GSS_IGNORE }, /* ISP_PWD */
    { 32 * 2 + 24, GSS_IGNORE }, /* ISP_RST */

    { 32 * 2 + 25, GSS_INPUT_NOPULL }, /* SLCD_WR */
    { 32 * 2 + 26, GSS_INPUT_NOPULL }, /* SLCD_DC */
    { 32 * 2 + 27, GSS_INPUT_NOPULL }, /* SLCD_TE */

    /* GPIO Group - D */
    { 32 * 3 + 14, GSS_IGNORE }, /* CLK32K */
    { 32 * 3 + 17, GSS_INPUT_NOPULL }, /* NC */
    { 32 * 3 + 18, GSS_INPUT_NOPULL }, /* BOOT_SEL1 */
    { 32 * 3 + 19, GSS_INPUT_NOPULL }, /* NC*/

    { 32 * 3 + 26, GSS_IGNORE }, /* UART1_RXD */
    { 32 * 3 + 27, GSS_IGNORE }, /* UART1_CTS */
    { 32 * 3 + 28, GSS_IGNORE }, /* UART1_RTS */
    { 32 * 3 + 29, GSS_IGNORE }, /* UART1_TXD */

    { 32 * 3 + 30, GSS_INPUT_NOPULL }, /* SMB0_SDA */
    { 32 * 3 + 31, GSS_INPUT_NOPULL }, /* SMB0_CLK */

    /* GPIO Group - E */
    { 32 * 4 + 0, GSS_INPUT_NOPULL }, /* SMB2_SDA */
    { 32 * 4 + 1, GSS_INPUT_NOPULL }, /* LCD_PWM */
    { 32 * 4 + 2, GSS_INPUT_NOPULL }, /* nc */
    { 32 * 4 + 3, GSS_INPUT_NOPULL }, /* SMB2_SCK */
    { 32 * 4 + 10, GSS_IGNORE }, /* DRV_VBUS */
    { 32 * 4 + 20, GSS_IGNORE }, /* SDIO_D0 */
    { 32 * 4 + 21, GSS_IGNORE }, /* SDIO_D1 */
    { 32 * 4 + 22, GSS_IGNORE }, /* SDIO_D2 */
    { 32 * 4 + 23, GSS_IGNORE }, /* SDIO_D3 */
    { 32 * 4 + 28, GSS_IGNORE }, /* SDIO_CLK */
    { 32 * 4 + 29, GSS_IGNORE }, /* SDIO_CMD */
    { 32 * 4 + 30, GSS_INPUT_NOPULL }, /* SMB1_SDA */
    { 32 * 4 + 31, GSS_INPUT_NOPULL }, /* SMB1_SCK */

    /* GPIO Group - F */
    { 32 * 5 + 0, GSS_IGNORE }, /* UART0_RXD */
    { 32 * 5 + 1, GSS_IGNORE }, /* UART0_CTS */
    { 32 * 5 + 2, GSS_IGNORE }, /* UART0_RTS */
    { 32 * 5 + 3, GSS_IGNORE }, /* UART0_TXD */

    { 32 * 5 + 6, GSS_OUTPUT_LOW }, /* DMIC_CLK */
    { 32 * 5 + 7, GSS_INPUT_PULL }, /* DMIC_DOUT */

    { 32 * 5 + 12, GSS_OUTPUT_LOW }, /* BT_PCM_DO */
    { 32 * 5 + 13, GSS_OUTPUT_LOW }, /* BT_PCM_CLK */
    { 32 * 5 + 14, GSS_OUTPUT_LOW }, /* BT_PCM_SYN */
    { 32 * 5 + 15, GSS_OUTPUT_LOW }, /* BT_PCM_DI */
    { GSS_TABLET_END, GSS_TABLET_END }
};
#endif
