/*
 * USBane - ESP32-S3 USB Security Research Tool
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "usbane.h"
#include "dwc2_backend.h"
#include "wifi_ap.h"
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "driver/spi_master.h"
#include "sdmmc_cmd.h"
#include <dirent.h>
#include "esp_spiffs.h"
#include "web_interface.h"
#include "cardputer_adv_display.h"
#include "debug_shell.h"
#include "tca8418.h"

static const char *TAG = "USBANE";

void app_main(void)
{
    // Display first — all subsequent ESP_LOGx calls appear on screen
    display_adv_init();
    debug_shell_init();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  _   _ ____  ____                   ");
    ESP_LOGI(TAG, " | | | / ___|| __ )  __ _ _ __   ___ ");
    ESP_LOGI(TAG, " | | | \\___ \\|  _ \\ / _` | '_ \\ / _ \\");
    ESP_LOGI(TAG, " | |_| |___) | |_) | (_| | | | |  __/");
    ESP_LOGI(TAG, "  \\___/|____/|____/ \\__,_|_| |_|\\___|");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  USB Security Research Tool v%s", CONFIG_USBANE_VERSION);
    
    // Initialize WiFi AP
    ESP_LOGI(TAG, "Starting WiFi...");
    display_adv_update_status(DISPLAY_WIFI_STARTING, NULL, DISPLAY_USB_IDLE, NULL);
    wifi_ap_init();
    vTaskDelay(pdMS_TO_TICKS(3000));  /* pause to read boot log */
    debug_shell_enable_log_hook();
    ESP_LOGI(TAG, "KBD CFG readback: 0x%02x", tca8418_get_cfg_readback());
    
    // Mount SPIFFS
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",
        .max_files = 10,
        .format_if_mount_failed = true,
    };
    esp_err_t spiffs_ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (spiffs_ret != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS mount failed: %s", esp_err_to_name(spiffs_ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted at /spiffs");
        // Create chains directory if it doesn't exist
        mkdir("/spiffs/chains", 0755);
    }

    // Mount SD card
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 14,
        .miso_io_num = 39,
        .sclk_io_num = 40,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI3_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = 12;
    slot_cfg.host_id = SPI3_HOST;
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
    };
    sdmmc_card_t *card = NULL;
    esp_err_t sd_ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_cfg, &mount_cfg, &card);
    if (sd_ret == ESP_OK) {
        ESP_LOGI(TAG, "SD card mounted");
        // Copy chains from SD to SPIFFS
        ESP_LOGI(TAG, "Opening /sdcard/usbane/chains");
        DIR *d = opendir("/sdcard/usbane/chains");
        ESP_LOGI(TAG, "opendir result: %s", d ? "ok" : "failed");
        if (d) {
            struct dirent *e;
            ESP_LOGI(TAG, "Scanning chains dir...");
            while ((e = readdir(d)) != NULL) {
                ESP_LOGI(TAG, "Found: %s", e->d_name);
                size_t len = strlen(e->d_name);
                ESP_LOGI(TAG, "Checking: %s len=%d", e->d_name, (int)len);
                if (len > 4 && (strcmp(e->d_name + len - 4, ".csv") == 0 || strcmp(e->d_name + len - 4, ".CSV") == 0)) {
                    ESP_LOGI(TAG, "Copying: %s", e->d_name);
                    char src[128], dst[128];
                    snprintf(src, sizeof(src), "/sdcard/usbane/chains/%s", e->d_name);
                    snprintf(dst, sizeof(dst), "/spiffs/chains/%s", e->d_name);
                    FILE *in = fopen(src, "r");
                    FILE *out = fopen(dst, "w");
                    if (in && out) {
                        char buf[256];
                        size_t n;
                        while ((n = fread(buf, 1, sizeof(buf), in)) > 0)
                            fwrite(buf, 1, n, out);
                        ESP_LOGI(TAG, "Copied chain: %s", e->d_name);
                    }
                    if (in) fclose(in);
                    if (out) fclose(out);
                }
            }
            closedir(d);
        } else {
            ESP_LOGW(TAG, "No /sdcard/usbane/chains dir");
        }
    } else {
        ESP_LOGW(TAG, "SD mount failed: %s", esp_err_to_name(sd_ret));
    }

    // Start web interface
    ESP_LOGI(TAG, "Starting Web Interface...");
    web_interface_start();
    
    // Wait for network to settle
    ESP_LOGI(TAG, "Waiting 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Start USB Backend Worker on Core 1
    ESP_LOGI(TAG, "Starting USB Backend Worker on Core 1...");
    esp_err_t ret = usb_backend_start_worker();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start USB worker: %s", esp_err_to_name(ret));
        display_adv_message("USB worker failed", esp_err_to_name(ret));
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Let worker start
    
    // Initialize USB Backend (reads config from NVS, inits on Core 1)
    ESP_LOGI(TAG, "Initializing USB Backend...");
    ret = usb_backend_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "USB backend init returned: %s (continuing anyway)", esp_err_to_name(ret));
        // Don't return - keep running so web interface works
    } else {
        ESP_LOGI(TAG, "USB Backend: %s", usb_backend_type_name(usb_backend_get_type()));
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "-------------------------------------------");
    ESP_LOGI(TAG, "USBane READY!");
    ESP_LOGI(TAG, "-------------------------------------------");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "CONNECT TO WEB INTERFACE:");
    ESP_LOGI(TAG, "  1. WiFi: USBane");
    ESP_LOGI(TAG, "  2. Password: usbane123");
    ESP_LOGI(TAG, "  3. Open: http://192.168.4.1");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Monitoring USB connection status...");
    ESP_LOGI(TAG, "");
    
    // Monitor device connection status
    bool last_connected = false;
    
    while (1) {
        bool connected = usb_backend_is_device_connected();
        
        if (connected != last_connected) {
            if (connected) {
                ESP_LOGI(TAG, "USB Device connected!");
                display_adv_update_status(wifi_adv_current_status(),
                                          wifi_adv_current_ip(),
                                          DISPLAY_USB_CONNECTED, NULL);
            } else {
                ESP_LOGW(TAG, "USB Device disconnected!");
                display_adv_update_status(wifi_adv_current_status(),
                                          wifi_adv_current_ip(),
                                          DISPLAY_USB_IDLE, NULL);
            }
            last_connected = connected;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
