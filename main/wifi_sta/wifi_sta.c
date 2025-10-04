/*
 * wifi_sta.c
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */


#include "wifi_sta.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "wifi_sta";

/* EventGroup bits to signal connection state */
static EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int FAIL_BIT = BIT1;

/* keep track of retry count */
static int s_retry_num = 0;

/* Event handler handles WIFI and IP events */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START -> call esp_wifi_connect()");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_connect() failed: %s", esp_err_to_name(err));
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Failed to connect or got disconnected */
        if (s_retry_num < WIFI_STA_MAX_RETRY) {
            ESP_LOGW(TAG, "Disconnected, retrying connection (%d/%d)", s_retry_num+1, WIFI_STA_MAX_RETRY);
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_connect() failed on retry: %s", esp_err_to_name(err));
            }
            s_retry_num++;
        } else {
            ESP_LOGE(TAG, "Failed to connect after %d attempts.", WIFI_STA_MAX_RETRY);
            xEventGroupSetBits(s_wifi_event_group, FAIL_BIT);
        }
        ESP_LOGI(TAG, "Reason: DISCONNECTED event id");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    }
}

/**
 * Initialize NVS (if not already), network stack, wifi driver and start STA mode.
 *
 * Possible returned esp_err_t:
 *   - ESP_OK: initialization started successfully (connection handled asynchronously).
 *   - ESP_ERR_NO_MEM: out of memory.
 *   - ESP_ERR_INVALID_STATE: wifi already init-ed in incompatible state.
 *   - ESP_FAIL: NVS or other initialization failure.
 *
 * All internal errors are logged with esp_log.
 */
esp_err_t wifi_init_sta(void)
{
    esp_err_t ret;

    /* 1) Initialize NVS — required by WiFi */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated and needs erasing */
        ESP_LOGW(TAG, "NVS needs erase, erasing and re-initializing");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "nvs_flash_erase failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2) Initialize TCP/IP stack and default event loop */
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE may mean default loop already created; allow that */
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 3) Create default WiFi station */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "esp_netif_create_default_wifi_sta returned NULL");
        return ESP_FAIL;
    }

    /* 4) WiFi driver init */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 5) Create event group to signal when connected or failed */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    /* 6) Register event handlers for WiFi and IP events */
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_handler_register (WIFI_EVENT) failed: %s", esp_err_to_name(ret));
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
        return ret;
    }

    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_handler_register (IP_EVENT) failed: %s", esp_err_to_name(ret));
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
        return ret;
    }

    /* 7) Configure WiFi as station and set credentials */
    wifi_config_t wifi_config = {
        .sta = {
            /* Copy SSID and password; ensure null-terminated */
        },
    };
    strncpy((char*)wifi_config.sta.ssid, WIFI_STA_SSID, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char*)wifi_config.sta.password, WIFI_STA_PASS, sizeof(wifi_config.sta.password)-1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; /* prefer WPA2 */
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        goto fail_unregister;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(ret));
        goto fail_unregister;
    }

    /* 8) Start WiFi driver — this will trigger WIFI_EVENT_STA_START */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        goto fail_unregister;
    }

    ESP_LOGI(TAG, "wifi_init_sta finished. Connecting to SSID:%s", WIFI_STA_SSID);

    /* Optionally: wait here for connection result (blocking) — minimal implementation */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           CONNECTED_BIT | FAIL_BIT,
                                           pdTRUE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(30000)); /* wait max 10s */

    if (bits & CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP: %s", WIFI_STA_SSID);
        return ESP_OK;
    } else if (bits & FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", WIFI_STA_SSID);
        ret = ESP_FAIL;
        goto fail_unregister;
    } else {
        ESP_LOGW(TAG, "Connection attempt timed out (no IP in 10s). Continue in background.");
        /* Not a fatal error: WiFi still running and will keep trying (per event handler) */
        return ESP_OK;
    }

fail_unregister:
    /* cleanup on failure */
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }
    return ret;
}

/**
 * Stop WiFi and free resources.
 * This stops wifi driver, unregisters event handlers and deletes event group.
 */
void wifi_stop(void)
{
    ESP_LOGI(TAG, "Stopping WiFi");
    esp_err_t err;

    err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "esp_wifi_stop returned %s", esp_err_to_name(err));
    }

    err = esp_wifi_deinit();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "esp_wifi_deinit returned %s", esp_err_to_name(err));
    }

    /* Unregister handlers — safe even if not registered */
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);

    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    ESP_LOGI(TAG, "WiFi stopped and cleaned up");
}


