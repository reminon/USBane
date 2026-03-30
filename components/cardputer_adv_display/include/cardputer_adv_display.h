#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DISPLAY_WIFI_STARTING,
    DISPLAY_WIFI_AP_ACTIVE,
    DISPLAY_WIFI_STA_CONNECTING,
    DISPLAY_WIFI_STA_CONNECTED,
    DISPLAY_WIFI_STA_FAILED,
} display_wifi_status_t;

typedef enum {
    DISPLAY_USB_IDLE,
    DISPLAY_USB_CONNECTED,
    DISPLAY_USB_ERROR,
} display_usb_status_t;

/**
 * Initialise ST7789V2 on Cardputer ADV and show splash screen.
 * Must be called before any other display function.
 */
esp_err_t display_adv_init(void);

/**
 * Redraw the status area (WiFi + USB state, IP address).
 * Safe to call from any task.
 */
void display_adv_update_status(display_wifi_status_t wifi_status,
                               const char *ip_str,
                               display_usb_status_t usb_status,
                               const char *usb_desc);

/**
 * Show a temporary full-screen message (cleared on next update_status call).
 */
void display_adv_message(const char *line1, const char *line2);

/**
 * Turn off backlight and free resources.
 */
void display_adv_deinit(void);

#ifdef __cplusplus
}
#endif
