#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "cardputer_adv_display.h"

void wifi_ap_init(void);
bool wifi_is_sta_connected(void);
esp_err_t wifi_get_sta_ip(char *ip_str, size_t len);

/* Display state accessors — used by main.c monitor loop */
display_wifi_status_t wifi_adv_current_status(void);
const char *wifi_adv_current_ip(void);
