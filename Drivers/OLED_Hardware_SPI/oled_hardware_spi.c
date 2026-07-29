#include "oled_hardware_spi.h"
#include "oledfont.h"
#include "clock.h"

/* Default text colors used by the original OLED_* drawing interface. */
static uint16_t text_color = OLED_GREEN;
static uint16_t back_color = OLED_BLACK;

void delay_ms(uint32_t ms)
{
    mspm0_delay_ms(ms);
}

/* Wait until every queued SPI bit has physically left the controller. */
static void lcd_wait_idle(void)
{
    while (DL_SPI_isBusy(SPI_OLED_INST)) {
    }
}

/* Start one command or data transaction. ST7735S uses DC to distinguish it. */
static void lcd_begin_write(uint8_t is_data)
{
    lcd_wait_idle();

    if (is_data != 0U) {
        OLED_DC_Set();
    } else {
        OLED_DC_Clr();
    }
}

/* Queue one byte without releasing CS, allowing fast pixel streaming. */
static void lcd_write_stream(uint8_t value)
{
    while (DL_SPI_isTXFIFOFull(SPI_OLED_INST)) {
    }
    DL_SPI_transmitData8(SPI_OLED_INST, value);
}

static void lcd_end_write(void)
{
    lcd_wait_idle();
}

static void lcd_write(uint8_t value, uint8_t is_data)
{
    lcd_begin_write(is_data);
    lcd_write_stream(value);
    lcd_end_write();
}

/* Compatibility function: cmd=OLED_CMD sends a command, cmd=OLED_DATA sends data. */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    lcd_write(dat, cmd);
}

static void lcd_command(uint8_t command)
{
    lcd_write(command, OLED_CMD);
}

static void lcd_data(uint8_t data)
{
    lcd_write(data, OLED_DATA);
}

/* Select an inclusive RGB565 drawing rectangle in ST7735S display RAM. */
static void lcd_set_window(uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1)
{
    lcd_command(0x2AU); /* CASET: column address */
    lcd_data((uint8_t)(x0 >> 8));
    lcd_data((uint8_t)x0);
    lcd_data((uint8_t)(x1 >> 8));
    lcd_data((uint8_t)x1);

    lcd_command(0x2BU); /* RASET: row address */
    lcd_data((uint8_t)(y0 >> 8));
    lcd_data((uint8_t)y0);
    lcd_data((uint8_t)(y1 >> 8));
    lcd_data((uint8_t)y1);

    lcd_command(0x2CU); /* RAMWR: following bytes are pixel data */
}

static void lcd_write_color_stream(uint16_t color)
{
    /* ST7735S expects RGB565 most-significant byte first. */
    lcd_write_stream((uint8_t)(color >> 8));
    lcd_write_stream((uint8_t)color);
}

static void lcd_fill(uint16_t x, uint16_t y,
                     uint16_t width, uint16_t height,
                     uint16_t color)
{
    uint32_t pixels;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT) ||
        (width == 0U) || (height == 0U)) {
        return;
    }
    if ((x + width) > OLED_WIDTH) {
        width = OLED_WIDTH - x;
    }
    if ((y + height) > OLED_HEIGHT) {
        height = OLED_HEIGHT - y;
    }

    lcd_set_window(x, y, x + width - 1U, y + height - 1U);
    pixels = (uint32_t)width * height;

    lcd_begin_write(OLED_DATA);
    while (pixels-- != 0U) {
        lcd_write_color_stream(color);
    }
    lcd_end_write();
}

/* Keep the old API: 0=normal colors, nonzero=inverted colors. */
void OLED_ColorTurn(uint8_t i)
{
    lcd_command((i != 0U) ? 0x21U : 0x20U); /* INVON / INVOFF */
}

/* Keep the old API: 0=portrait, nonzero=portrait rotated by 180 degrees. */
void OLED_DisplayTurn(uint8_t i)
{
    lcd_command(0x36U); /* MADCTL */
    lcd_data((i != 0U) ? 0x08U : 0xC8U);
}

/*
 * Compatibility position function. The old OLED code treated y as an 8-pixel
 * page, so the same convention is retained for existing application calls.
 */
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    uint16_t pixel_y = (uint16_t)y * 8U;

    if ((x < OLED_WIDTH) && (pixel_y < OLED_HEIGHT)) {
        lcd_set_window(x, pixel_y, OLED_WIDTH - 1U, OLED_HEIGHT - 1U);
    }
}

void OLED_Display_On(void)
{
    lcd_command(0x11U); /* SLPOUT */
    delay_ms(120U);
    lcd_command(0x29U); /* DISPON */
    OLED_BLK_Set();
}

void OLED_Display_Off(void)
{
    OLED_BLK_Clr();
    lcd_command(0x28U); /* DISPOFF */
    lcd_command(0x10U); /* SLPIN */
}

void OLED_Clear(void)
{
    lcd_fill(0U, 0U, OLED_WIDTH, OLED_HEIGHT, back_color);
}

/* Read one pixel from the original monochrome 6x8 or 8x16 font tables. */
static uint8_t glyph_pixel(uint8_t chr, uint8_t sizey,
                           uint8_t x, uint8_t y)
{
    uint8_t index = (uint8_t)(chr - (uint8_t)' ');

    if (sizey == 8U) {
        return (uint8_t)((asc2_0806[index][x] >> y) & 0x01U);
    }

    return (uint8_t)((asc2_1608[index][x + ((y >= 8U) ? 8U : 0U)] >>
                      (y & 7U)) & 0x01U);
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t px;
    uint8_t py;
    uint8_t width;
    uint16_t screen_y = (uint16_t)y * 8U;

    if ((chr < (uint8_t)' ') || (chr > (uint8_t)'~')) {
        chr = (uint8_t)'?';
    }
    if ((sizey != 8U) && (sizey != 16U)) {
        return;
    }

    width = (sizey == 8U) ? 6U : 8U;
    if (((uint16_t)x + width > OLED_WIDTH) ||
        (screen_y + sizey > OLED_HEIGHT)) {
        return;
    }

    lcd_set_window(x, screen_y, (uint16_t)x + width - 1U,
                   screen_y + sizey - 1U);
    lcd_begin_write(OLED_DATA);
    for (py = 0U; py < sizey; py++) {
        for (px = 0U; px < width; px++) {
            lcd_write_color_stream((glyph_pixel(chr, sizey, px, py) != 0U) ?
                                   text_color : back_color);
        }
    }
    lcd_end_write();
}

uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1U;

    while (n-- != 0U) {
        result *= m;
    }
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num,
                  uint8_t len, uint8_t sizey)
{
    uint8_t i;
    uint8_t digit;
    uint8_t started = 0U;
    uint8_t width = (sizey == 8U) ? 6U : 8U;

    for (i = 0U; i < len; i++) {
        digit = (uint8_t)((num / oled_pow(10U, len - i - 1U)) % 10U);
        if ((started == 0U) && (i < (len - 1U)) && (digit == 0U)) {
            OLED_ShowChar(x + width * i, y, (uint8_t)' ', sizey);
        } else {
            started = 1U;
            OLED_ShowChar(x + width * i, y, digit + (uint8_t)'0', sizey);
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey)
{
    uint8_t width = (sizey == 8U) ? 6U : 8U;

    while (*chr != 0U) {
        if (((uint16_t)x + width) > OLED_WIDTH) {
            break;
        }
        OLED_ShowChar(x, y, *chr++, sizey);
        x = (uint8_t)(x + width);
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey)
{
    uint8_t px;
    uint8_t py;
    uint16_t screen_y = (uint16_t)y * 8U;

    if (sizey != 16U) {
        return;
    }
    if (((uint16_t)x + 16U > OLED_WIDTH) ||
        (screen_y + 16U > OLED_HEIGHT)) {
        return;
    }

    lcd_set_window(x, screen_y, (uint16_t)x + 15U, screen_y + 15U);
    lcd_begin_write(OLED_DATA);
    for (py = 0U; py < 16U; py++) {
        for (px = 0U; px < 16U; px++) {
            uint8_t column = Hzk[no][px + ((py >= 8U) ? 16U : 0U)];
            lcd_write_color_stream(((column >> (py & 7U)) & 1U) ?
                                   text_color : back_color);
        }
    }
    lcd_end_write();
}

/* Draw the original 1-bit page-format bitmap using RGB565 foreground/background. */
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex,
                  uint8_t sizey, uint8_t BMP[])
{
    uint16_t px;
    uint16_t py;

    if (((uint16_t)x + sizex > OLED_WIDTH) ||
        ((uint16_t)y + sizey > OLED_HEIGHT) ||
        (sizex == 0U) || (sizey == 0U)) {
        return;
    }

    lcd_set_window(x, y, (uint16_t)x + sizex - 1U,
                   (uint16_t)y + sizey - 1U);
    lcd_begin_write(OLED_DATA);
    for (py = 0U; py < sizey; py++) {
        for (px = 0U; px < sizex; px++) {
            uint8_t bits = BMP[(py / 8U) * sizex + px];
            lcd_write_color_stream(((bits >> (py & 7U)) & 1U) ?
                                   text_color : back_color);
        }
    }
    lcd_end_write();
}

void OLED_Init(void)
{
    /*
     * 屏幕使用SPI1_CS0（PB20）。
     * SPI1_CS1（PB27）已经在SysConfig中配置，留给第二个SPI设备。
     */
    DL_SPI_setChipSelect(SPI_OLED_INST, DL_SPI_CHIP_SELECT_0);

    /* Keep the backlight off until the controller and display RAM are ready. */
    OLED_DC_Set();
    OLED_BLK_Clr();

    /* Hardware reset required by ST7735S after the power rail is stable. */
    OLED_RES_Set();
    delay_ms(10U);
    OLED_RES_Clr();
    delay_ms(20U);
    OLED_RES_Set();
    delay_ms(120U);

    lcd_command(0x01U); /* SWRESET */
    delay_ms(150U);
    lcd_command(0x11U); /* SLPOUT */
    delay_ms(120U);

    /* ST7735S frame-rate configuration. */
    lcd_command(0xB1U);
    lcd_data(0x01U);
    lcd_data(0x2CU);
    lcd_data(0x2DU);
    lcd_command(0xB2U);
    lcd_data(0x01U);
    lcd_data(0x2CU);
    lcd_data(0x2DU);
    lcd_command(0xB3U);
    lcd_data(0x01U);
    lcd_data(0x2CU);
    lcd_data(0x2DU);
    lcd_data(0x01U);
    lcd_data(0x2CU);
    lcd_data(0x2DU);
    lcd_command(0xB4U);
    lcd_data(0x07U);

    /* ST7735S power and VCOM settings. */
    lcd_command(0xC0U);
    lcd_data(0xA2U);
    lcd_data(0x02U);
    lcd_data(0x84U);
    lcd_command(0xC1U);
    lcd_data(0xC5U);
    lcd_command(0xC2U);
    lcd_data(0x0AU);
    lcd_data(0x00U);
    lcd_command(0xC3U);
    lcd_data(0x8AU);
    lcd_data(0x2AU);
    lcd_command(0xC4U);
    lcd_data(0x8AU);
    lcd_data(0xEEU);
    lcd_command(0xC5U);
    lcd_data(0x0EU);

    lcd_command(0x36U); /* MADCTL: portrait orientation and BGR order */
    lcd_data(0xC8U);
    lcd_command(0x3AU); /* COLMOD: RGB565, 16 bits per pixel */
    lcd_data(0x05U);
    lcd_command(0x20U); /* INVOFF */
    lcd_command(0x13U); /* NORON */
    lcd_command(0x29U); /* DISPON */
    delay_ms(20U);

    OLED_Clear();
    OLED_BLK_Set();
}
