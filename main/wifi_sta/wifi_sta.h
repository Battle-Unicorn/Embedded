/*
 * wifi_sta.h
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */

#pragma once

#include "esp_err.h"
#include "esp_event.h"

/**
 * Minimal WiFi station header
 *
 * Public API:
 *   - esp_err_t wifi_init_sta(void);
 *       Initialize NVS, network stack and start WiFi in STA mode.
 *
 *   - void wifi_stop(void);
 *       Stop WiFi and unregister event handlers.
 *
 * Error handling:
 *   The functions return esp_err_t. Possible errors (examples):
 *     - ESP_OK: success.
 *     - ESP_ERR_NO_MEM: allocation failure.
 *     - ESP_ERR_INVALID_STATE: attempted operation in invalid state.
 *     - ESP_FAIL: generic failure.
 *   See esp_err.h and esp-idf docs for full meanings.
 */

/* WiFi credentials (user provided) */
#define WIFI_STA_SSID       "WIFI_SIEC_NIEDOSTEPNA"
#define WIFI_STA_PASS       "KotKotKot509#"

/* Optional: maximum retry attempts before giving up */
#define WIFI_STA_MAX_RETRY  5

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize WiFi in station mode and attempt to connect to configured SSID.
 *
 * Returns:
 *   ESP_OK on success (WiFi started and connection attempt in progress).
 *   esp_err_t error code on failure (see esp_err.h).
 *
 * Notes:
 *   - This function will initialize NVS if needed.
 *   - If successful, connection progress is reported via logs.
 */
esp_err_t wifi_init_sta(void);

/**
 * Stop WiFi and release resources created by wifi_init_sta.
 * This is safe to call even if wifi_init_sta failed partially.
 */
void wifi_stop(void);

#ifdef __cplusplus
}
#endif

