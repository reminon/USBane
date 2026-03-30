#include "tca8418.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "tca8418";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static tca8418_key_cb_t s_cb = NULL;
static void *s_cb_arg = NULL;
static SemaphoreHandle_t s_sem = NULL;
static TaskHandle_t s_task = NULL;
static uint8_t s_cfg_readback = 0xFF;


/* 4x14 keymap matching Cardputer-ADV physical layout */
static const char s_keymap_4x14[4][14] = {
    /* row 0 */ {'`','1','2','3','4','5','6','7','8','9','0','-','=','\b'},
    /* row 1 */ {'\t','q','w','e','r','t','y','u','i','o','p','[',']','\\'},
    /* row 2 */ {1,2,'a','s','d','f','g','h','j','k','l',';','\'','\r'},
    /* row 3 */ {3,4,5,'z','x','c','v','b','n','m',',','.','/',' '},
};

static esp_err_t _write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, pdMS_TO_TICKS(50));
}

static esp_err_t _read_reg(uint8_t reg, uint8_t *out) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, 1, pdMS_TO_TICKS(50));
}

static uint8_t mask_low_bits(uint8_t n) {
    if (n == 0) return 0;
    if (n >= 8) return 0xFF;
    return (uint8_t)((1u << n) - 1u);
}

static void IRAM_ATTR _int_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

static void _kbd_task(void *arg) {
    while (1) {
        xSemaphoreTake(s_sem, pdMS_TO_TICKS(50));

        uint8_t stat = 0;
        _read_reg(TCA8418_REG_INT_STAT, &stat);

        uint8_t count = 0;
        _read_reg(TCA8418_REG_KEY_LCK_EC, &count);
        count &= 0x0F;

        for (uint8_t i = 0; i < count; i++) {
            uint8_t evt = 0;
            _read_reg(TCA8418_REG_KEY_EVENT_A, &evt);
            if (evt == 0) break;

            bool pressed = (evt & 0x80) != 0;
            uint8_t raw_code = evt & 0x7F;
            uint8_t keynum = (raw_code > 0) ? (raw_code - 1) : 0;
            uint8_t tca_row = keynum / 10;
            uint8_t tca_col = keynum % 10;
            uint8_t out_col = tca_row * 2 + (tca_col > 3 ? 1 : 0);
            uint8_t out_row = (tca_col + 4) % 4;
            char ascii = (out_row < 4 && out_col < 14) ? s_keymap_4x14[out_row][out_col] : 0;

            tca8418_key_event_t ke = {
                .ascii = ascii,
                .keycode = raw_code,
                .pressed = pressed,
            };
            if (!ascii) ESP_LOGI(TAG, "unmapped raw=%d orow=%d ocol=%d", raw_code, out_row, out_col);
            if (s_cb) s_cb(&ke, s_cb_arg);
        }

        /* Clear interrupts */
        uint8_t tmp = 0;
        _read_reg(0x11, &tmp);
        _read_reg(0x12, &tmp);
        _read_reg(0x13, &tmp);
        _write_reg(TCA8418_REG_INT_STAT, 0x03);
    }
}

esp_err_t tca8418_init(tca8418_key_cb_t cb, void *arg) {
    s_cb = cb;
    s_cb_arg = arg;

    i2c_master_bus_config_t bus_cfg = {
        .trans_queue_depth = 0,
        .i2c_port          = TCA8418_I2C_PORT,
        .sda_io_num        = TCA8418_SDA_PIN,
        .scl_io_num        = TCA8418_SCL_PIN,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TCA8418_I2C_ADDR,
        .scl_speed_hz    = TCA8418_I2C_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    /* GPIO direction: all inputs */
    _write_reg(0x23, 0x00);
    _write_reg(0x24, 0x00);
    _write_reg(0x25, 0x00);
    /* GPI event mode: all enabled */
    _write_reg(0x20, 0xFF);
    _write_reg(0x21, 0xFF);
    _write_reg(0x22, 0xFF);
    /* GPIO interrupt level: falling edge */
    _write_reg(0x26, 0x00);
    _write_reg(0x27, 0x00);
    _write_reg(0x28, 0x00);
    /* GPIO interrupt enable */
    _write_reg(0x1A, 0xFF);
    _write_reg(0x1B, 0xFF);
    _write_reg(0x1C, 0xFF);
    /* Keypad matrix: 7 rows in KP_GPIO1, 8 cols in KP_GPIO2 */
    _write_reg(TCA8418_REG_KP_GPIO_1, mask_low_bits(7));
    _write_reg(TCA8418_REG_KP_GPIO_2, mask_low_bits(8));
    /* Flush pending events */
    { uint8_t tmp = 0; for (int i=0; i<10; i++) { _read_reg(TCA8418_REG_KEY_EVENT_A, &tmp); if (!tmp) break; } }
    _write_reg(0x11, 0xFF);
    _write_reg(0x12, 0xFF);
    _write_reg(0x13, 0xFF);
    _write_reg(TCA8418_REG_INT_STAT, 0x03);
    /* Enable key event + GPI interrupts */
    _write_reg(TCA8418_REG_CFG, TCA8418_CFG_KE_IEN | TCA8418_CFG_GPI_IEN | TCA8418_CFG_INT_CFG);

    _read_reg(TCA8418_REG_CFG, &s_cfg_readback);
    ESP_LOGI(TAG, "TCA8418 ready: CFG=0x%02x", s_cfg_readback);

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << TCA8418_INT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&int_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TCA8418_INT_PIN, _int_isr, NULL);

    s_sem = xSemaphoreCreateBinary();
    xTaskCreate(_kbd_task, "tca8418", 2048, NULL, 5, &s_task);

    return ESP_OK;
}

void tca8418_deinit(void) {
    if (s_task) { vTaskDelete(s_task); s_task = NULL; }
    gpio_isr_handler_remove(TCA8418_INT_PIN);
    if (s_dev) { i2c_master_bus_rm_device(s_dev); s_dev = NULL; }
    if (s_bus) { i2c_del_master_bus(s_bus); s_bus = NULL; }
}

uint8_t tca8418_get_cfg_readback(void) { return s_cfg_readback; }
