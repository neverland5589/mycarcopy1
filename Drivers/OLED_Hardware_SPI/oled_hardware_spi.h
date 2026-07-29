/*
 * ST7735S 128 x 160 RGB TFT driver.
 *
 * Tianmengxing display connector:
 *   SCK = PB23, MOSI/PICO = PB22, MISO/POCI = PB21,
 *   LCD_CS = PB20/SPI1_CS0, spare CS = PB27/SPI1_CS1,
 *   RES = PB10, DC = PB11, BLK = PB14.
 *
 * The original OLED_* function names and parameters are deliberately kept so
 * existing application calls do not need to change. The y argument used by
 * the text functions keeps the original page convention: one unit is 8 TFT
 * pixels.
 */

#ifndef __OLED_HARDWARE_SPI_H
#define __OLED_HARDWARE_SPI_H

#include "ti_msp_dl_config.h"

#ifndef GPIO_OLED_PIN_OLED_RES_PORT
#define GPIO_OLED_PIN_OLED_RES_PORT GPIO_OLED_PORT
#endif
#ifndef GPIO_OLED_PIN_OLED_DC_PORT
#define GPIO_OLED_PIN_OLED_DC_PORT GPIO_OLED_PORT
#endif
#ifndef GPIO_OLED_PIN_OLED_BLK_PORT
#define GPIO_OLED_PIN_OLED_BLK_PORT GPIO_OLED_PORT
#endif

#define OLED_CMD   0U
#define OLED_DATA  1U

#define OLED_WIDTH   128U
#define OLED_HEIGHT  160U

/* RGB565 colors. */
#define OLED_BLACK    0x0000U
#define OLED_BLUE     0x001FU
#define OLED_RED      0xF800U
#define OLED_GREEN    0x07E0U
#define OLED_CYAN     0x07FFU
#define OLED_MAGENTA  0xF81FU
#define OLED_YELLOW   0xFFE0U
#define OLED_WHITE    0xFFFFU

#define OLED_RES_Set()  DL_GPIO_setPins(GPIO_OLED_PIN_OLED_RES_PORT, GPIO_OLED_PIN_OLED_RES_PIN)
#define OLED_RES_Clr()  DL_GPIO_clearPins(GPIO_OLED_PIN_OLED_RES_PORT, GPIO_OLED_PIN_OLED_RES_PIN)
#define OLED_DC_Set()   DL_GPIO_setPins(GPIO_OLED_PIN_OLED_DC_PORT, GPIO_OLED_PIN_OLED_DC_PIN)
#define OLED_DC_Clr()   DL_GPIO_clearPins(GPIO_OLED_PIN_OLED_DC_PORT, GPIO_OLED_PIN_OLED_DC_PIN)
#define OLED_BLK_Set()  DL_GPIO_setPins(GPIO_OLED_PIN_OLED_BLK_PORT, GPIO_OLED_PIN_OLED_BLK_PIN)
#define OLED_BLK_Clr()  DL_GPIO_clearPins(GPIO_OLED_PIN_OLED_BLK_PORT, GPIO_OLED_PIN_OLED_BLK_PIN)

void delay_ms(uint32_t ms);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);
uint32_t oled_pow(uint8_t m, uint8_t n);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey);
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[]);
void OLED_Init(void);

#endif
