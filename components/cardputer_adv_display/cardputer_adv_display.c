/*
 * cardputer_adv_display.c
 *
 * ST7789V2 display driver for M5Stack Cardputer ADV.
 * Uses ESP-IDF 5.x esp_lcd component + SPI master.
 *
 * Pin mapping (from Sch_M5CardputerAdv_v1.0):
 *   G35 -> MOSI (DAT)
 *   G36 -> SCK
 *   G37 -> CS
 *   G34 -> DC  (RS)
 *   G33 -> RST
 *   G38 -> BL  (backlight, active-high)
 *
 * Panel: ST7789V2, 240x135 px, landscape
 */

#include "cardputer_adv_display.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"

static const char *TAG = "adv_disp";

/* ── Pins ──────────────────────────────────────────────────────── */
#define ADV_LCD_MOSI    35
#define ADV_LCD_SCLK    36
#define ADV_LCD_CS      37
#define ADV_LCD_DC      34
#define ADV_LCD_RST     33
#define ADV_LCD_BL      38

/* ── Panel ─────────────────────────────────────────────────────── */
#define ADV_LCD_H_RES   240
#define ADV_LCD_V_RES   135
#define ADV_LCD_COL_OFF 40
#define ADV_LCD_ROW_OFF 53
#define ADV_LCD_SPI_HOST SPI2_HOST
#define ADV_LCD_SPI_HZ  (40 * 1000 * 1000)

/* ── Colours RGB565 ────────────────────────────────────────────── */
#define COL_BLACK   0x0000u
#define COL_WHITE   0xFFFFu
#define COL_ACCENT  0x3EFFu
#define COL_GREEN   0x07E0u
#define COL_RED     0xF9A0u
#define COL_YELLOW  0xFFE0u
#define COL_GREY    0x8410u

static esp_lcd_panel_handle_t s_panel = NULL;
static bool s_initialised = false;

/* ── 5x7 font (ASCII 0x20-0x7E, 5 bytes per glyph, LSB=top) ───── */
static const uint8_t FONT5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x08,0x14,0x54,0x54,0x3C},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x00,0x7F,0x10,0x28,0x44},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x40,0x3C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x08,0x08,0x2A,0x1C,0x08},
};

/* ── Internal helpers (also used by debug_shell.c) ─────────────── */
void _fill_rect(int x, int y, int w, int h, uint16_t colour)
{
    if (!s_panel || w <= 0 || h <= 0) return;
    uint16_t *row = malloc(w * sizeof(uint16_t));
    if (!row) return;
    uint16_t be = (colour >> 8) | (colour << 8);
    for (int i = 0; i < w; i++) row[i] = be;
    for (int ry = y; ry < y + h; ry++)
        esp_lcd_panel_draw_bitmap(s_panel, x, ry, x + w, ry + 1, row);
    free(row);
}

void _draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *g = FONT5x7[(uint8_t)(c - 0x20)];
    int cw = 5 * scale, ch = 7 * scale;
    uint16_t *buf = malloc(cw * ch * sizeof(uint16_t));
    if (!buf) return;
    uint16_t fbe = (fg >> 8) | (fg << 8);
    uint16_t bbe = (bg >> 8) | (bg << 8);
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            uint16_t pix = (bits & (1 << row)) ? fbe : bbe;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++)
                    buf[(row * scale + sy) * cw + col * scale + sx] = pix;
        }
    }
    esp_lcd_panel_draw_bitmap(s_panel, x, y, x + cw, y + ch, buf);
    free(buf);
}

void _draw_string(int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale)
{
    if (!s) return;
    while (*s) { _draw_char(x, y, *s, fg, bg, scale); x += (5 + 1) * scale; s++; }
}

/* ── Public API ────────────────────────────────────────────────── */

esp_err_t display_adv_init(void)
{
    if (s_initialised) return ESP_OK;

    spi_bus_config_t bus = {
        .mosi_io_num     = ADV_LCD_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = ADV_LCD_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = ADV_LCD_H_RES * ADV_LCD_V_RES * 2 + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ADV_LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num       = ADV_LCD_DC,
        .cs_gpio_num       = ADV_LCD_CS,
        .pclk_hz           = ADV_LCD_SPI_HZ,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .spi_mode          = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)ADV_LCD_SPI_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = ADV_LCD_RST,
        .rgb_endian     = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel));

    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_mirror(s_panel, true, false);
    esp_lcd_panel_swap_xy(s_panel, true);
    esp_lcd_panel_set_gap(s_panel, ADV_LCD_COL_OFF, ADV_LCD_ROW_OFF);
    esp_lcd_panel_disp_on_off(s_panel, true);

    gpio_config_t bl = { .pin_bit_mask = (1ULL << ADV_LCD_BL), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&bl);
    gpio_set_level(ADV_LCD_BL, 1);

    s_initialised = true;

    /* Splash */
    _fill_rect(0, 0, ADV_LCD_H_RES, ADV_LCD_V_RES, COL_BLACK);
    _fill_rect(0, 0, ADV_LCD_H_RES, 20, COL_ACCENT);
    _draw_string(4, 6, "USBane", COL_BLACK, COL_ACCENT, 2);

    ESP_LOGI(TAG, "Display ready");
    return ESP_OK;
}

void display_adv_update_status(display_wifi_status_t wifi_status,
                               const char *ip_str,
                               display_usb_status_t usb_status,
                               const char *usb_desc)
{
    if (!s_initialised) return;
    _fill_rect(0, 22, ADV_LCD_H_RES, ADV_LCD_V_RES - 22, COL_BLACK);

    /* WiFi row */
    uint16_t wc;
    const char *wl;
    switch (wifi_status) {
        case DISPLAY_WIFI_AP_ACTIVE:      wc = COL_GREEN;  wl = "AP Active";  break;
        case DISPLAY_WIFI_STA_CONNECTED:  wc = COL_GREEN;  wl = "Connected";  break;
        case DISPLAY_WIFI_STA_CONNECTING: wc = COL_YELLOW; wl = "Connecting"; break;
        case DISPLAY_WIFI_STA_FAILED:     wc = COL_RED;    wl = "Failed";     break;
        default:                          wc = COL_GREY;   wl = "Starting";   break;
    }
    _draw_string(4, 28, "WiFi:", COL_GREY, COL_BLACK, 1);
    _draw_string(46, 28, wl, wc, COL_BLACK, 1);
    if (ip_str && *ip_str) _draw_string(4, 38, ip_str, COL_WHITE, COL_BLACK, 1);

    /* USB row */
    uint16_t uc;
    const char *ul;
    switch (usb_status) {
        case DISPLAY_USB_CONNECTED: uc = COL_GREEN; ul = "Connected"; break;
        case DISPLAY_USB_ERROR:     uc = COL_RED;   ul = "Error";     break;
        default:                    uc = COL_GREY;  ul = "Idle";      break;
    }
    _draw_string(4, 52, "USB: ", COL_GREY, COL_BLACK, 1);
    _draw_string(46, 52, ul, uc, COL_BLACK, 1);
    if (usb_desc && *usb_desc) _draw_string(4, 62, usb_desc, COL_WHITE, COL_BLACK, 1);

    /* Footer */
    _fill_rect(0, ADV_LCD_V_RES - 12, ADV_LCD_H_RES, 1, COL_GREY);
    _draw_string(4, ADV_LCD_V_RES - 10, "http://192.168.4.1", COL_GREY, COL_BLACK, 1);
}

void display_adv_message(const char *line1, const char *line2)
{
    if (!s_initialised) return;
    _fill_rect(0, 22, ADV_LCD_H_RES, ADV_LCD_V_RES - 22, COL_BLACK);
    if (line1) _draw_string(4, 40, line1, COL_WHITE, COL_BLACK, 1);
    if (line2) _draw_string(4, 56, line2, COL_GREY,  COL_BLACK, 1);
}

void display_adv_deinit(void)
{
    if (!s_initialised) return;
    gpio_set_level(ADV_LCD_BL, 0);
    esp_lcd_panel_del(s_panel);
    spi_bus_free(ADV_LCD_SPI_HOST);
    s_panel = NULL;
    s_initialised = false;
}
