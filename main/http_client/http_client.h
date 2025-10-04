/*
 * http_client.c
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */
#pragma once

#include "esp_err.h"

/**
 * Minimal HTTP client for ESP-IDF (POST JSON periodically).
 *
 * Public API:
 *   - esp_err_t http_client_start(void);
 *       Start the HTTP client background task which will POST JSON every 30s.
 *       Returns ESP_OK on successful task creation or an esp_err_t on failure.
 *
 *   - void http_client_stop(void);
 *       Stop the HTTP client background task and release resources.
 *
 * Notes:
 *   - Comments and error messages are in English.
 *   - Returned values follow esp_err_t conventions (ESP_OK, ESP_ERR_NO_MEM, ESP_FAIL, etc).
 */

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t http_client_start(void);
void http_client_stop(void);

#ifdef __cplusplus
}
#endif

