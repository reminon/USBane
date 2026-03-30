#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#define TCA8418_I2C_ADDR   0x34
#define TCA8418_SDA_PIN    8
#define TCA8418_SCL_PIN    9
#define TCA8418_INT_PIN    11
#define TCA8418_I2C_PORT   I2C_NUM_0
#define TCA8418_I2C_HZ     400000

/* Registers */
#define TCA8418_REG_CFG            0x01
#define TCA8418_REG_INT_STAT       0x02
#define TCA8418_REG_KEY_LCK_EC     0x03
#define TCA8418_REG_KEY_EVENT_A    0x04
#define TCA8418_REG_GPIO_INT_EN_1  0x1A
#define TCA8418_REG_GPIO_INT_EN_2  0x1B
#define TCA8418_REG_GPIO_INT_EN_3  0x1C
#define TCA8418_REG_KP_GPIO_1      0x1D
#define TCA8418_REG_KP_GPIO_2      0x1E
#define TCA8418_REG_KP_GPIO_3      0x1F
#define TCA8418_REG_GPI_EM_1       0x20
#define TCA8418_REG_GPI_EM_2       0x21
#define TCA8418_REG_GPI_EM_3       0x22
#define TCA8418_REG_GPIO_DIR_1     0x23
#define TCA8418_REG_GPIO_DIR_2     0x24
#define TCA8418_REG_GPIO_DIR_3     0x25
#define TCA8418_REG_GPIO_INT_LVL_1 0x26
#define TCA8418_REG_GPIO_INT_LVL_2 0x27
#define TCA8418_REG_GPIO_INT_LVL_3 0x28

/* CFG bits */
#define TCA8418_CFG_INT_CFG  (1 << 4)
#define TCA8418_CFG_GPI_IEN  (1 << 1)
#define TCA8418_CFG_KE_IEN   (1 << 0)

typedef struct {
    char    ascii;
    uint8_t keycode;
    bool    pressed;
} tca8418_key_event_t;

typedef void (*tca8418_key_cb_t)(const tca8418_key_event_t *e, void *arg);

esp_err_t tca8418_init(tca8418_key_cb_t cb, void *arg);
void      tca8418_deinit(void);
uint8_t   tca8418_get_cfg_readback(void);
