/*
 * http_client.c
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */
#include "http_client.h"

#include <string.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "time.h"

/* Tag used for ESP logging */
static const char *TAG = "http_client";

/* Server configuration */
#define SERVER_IP      "192.168.8.102"
#define SERVER_PORT    8080
#define FLAGS_ENDPOINT    "/embedded/flags"
#define POST_INTERVAL_MS (30000) /* 30 seconds */

extern bool atonia_flag;
extern bool sleep_flag;
extern float temp;


/* Task handle for the background POST task */
static TaskHandle_t s_http_task_handle = NULL;

/* The JSON payload to send */
static char s_payload[2048] = {0};

/* Helper: build server URL string */
static void build_url(char *out, size_t out_len)
{
    /* Format: http://<ip>:<port><path> */
    snprintf(out, out_len, "http://%s:%d%s", SERVER_IP, SERVER_PORT, FLAGS_ENDPOINT);
}

/* The background task that repeatedly POSTs the JSON payload */
static void http_post_task(void *arg)
{
    ESP_LOGI(TAG, "HTTP POST task started");

    char url[128];
    build_url(url, sizeof(url));

    /* Configure the http client */
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000, /* 10 s timeout */
    };

    while (1) {
        /* Create client for each iteration to ensure clean state */
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client == NULL) {
            ESP_LOGE(TAG, "Failed to initialize HTTP client (ESP_ERR_NO_MEM or other).");
            /* If client cannot be created, wait and retry later */
            vTaskDelay(pdMS_TO_TICKS(POST_INTERVAL_MS));
            continue;
        }

        /* Set header and body */
        esp_err_t err = esp_http_client_set_header(client, "Content-Type", "application/json");
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set header: %s", esp_err_to_name(err));
        }
        
        //create JSON payload
		/*draft:{
  "device_id": "Dev_001",
  "timestamp": "2025-09-24T23:45:12Z",
  "flags": {
    "sleep_flag": true,
    "atonia_flag": true
  },
  "system_info": {
    "battery_level": 82,
    "signal_strength": 89,
    "device_temperature": 36.9,
    "last_calibration": "2025-09-24T20:00:00Z"
    time from time.h
  }
}*/
		wifi_ap_record_t ap_info;
		esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
		int8_t rssi = 0;
		if (ret == ESP_OK) {
		    rssi = ap_info.rssi; // RSSI w dBm, np. -45
		    printf("RSSI: %d dBm\n", rssi);
		} else {
		    printf("esp_wifi_sta_get_ap_info failed: %d\n", ret);
		    
		}        
		time_t now;
		time(&now);
		struct tm *timeinfo = gmtime(&now);
		char time_str[25];
		strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", timeinfo);
				snprintf(s_payload, sizeof(s_payload),
				 "{\"device_id\":\"Dev_001\",\"timestamp\":\"%s\",\"flags\":{\"sleep_flag\":%s,\"atonia_flag\":%s},"
				 "\"system_info\":{\"battery_level\":82,\"signal_strength\":%d,\"body_temperature\":%.1f,\"last_calibration\":\"2025-09-24T20:00:00Z\"}}",
				 time_str,
				 sleep_flag ? "true" : "false",
				 atonia_flag ? "true" : "false",
				 rssi,
				 temp);

        err = esp_http_client_set_post_field(client, s_payload, strlen(s_payload));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set POST field: %s", esp_err_to_name(err));
        }

        /* Perform the POST */
        err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int status = esp_http_client_get_status_code(client);
            int content_len = esp_http_client_get_content_length(client);
            ESP_LOGI(TAG, "POST succeeded, status=%d, content_length=%d", status, content_len);
            /* Optionally you could read response body here using esp_http_client_read if needed */
        } else {
            /* Common errors: ESP_ERR_HTTP_CONN, ESP_ERR_HTTP_FETCH_HEADER, ESP_ERR_HTTP_EAGAIN */
            ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
        }

        esp_http_client_cleanup(client);

        /* Sleep for configured interval */
        vTaskDelay(pdMS_TO_TICKS(POST_INTERVAL_MS));
    }

    /* Should never reach here, but delete task if it does */
    vTaskDelete(NULL);
}

/**
 * Start the HTTP client background task.
 *
 * Returns:
 *   - ESP_OK: task created successfully.
 *   - ESP_ERR_NO_MEM: not enough memory to create task.
 *   - ESP_ERR_INVALID_STATE: already started.
 *   - ESP_FAIL: other failure.
 *
 * Note:
 *   The HTTP client task will run indefinitely until http_client_stop() is called.
 */
esp_err_t http_client_start(void)
{
    if (s_http_task_handle != NULL) {
        ESP_LOGW(TAG, "HTTP client task already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t r = xTaskCreatePinnedToCore(
        http_post_task,        /* task function */
        "http_post_task",      /* name */
        8192,                  /* stack size in bytes */
        NULL,                  /* param */
        tskIDLE_PRIORITY + 5,  /* priority */
        &s_http_task_handle,   /* task handle */
        tskNO_AFFINITY         /* run on any core */
    );

    if (r != pdPASS) {
        s_http_task_handle = NULL;
        ESP_LOGE(TAG, "Failed to create HTTP client task (out of memory?)");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "HTTP client task created");
    return ESP_OK;
}

/**
 * Stop the HTTP client background task and cleanup.
 *
 * This will delete the task and set the handle to NULL. Safe to call even if not started.
 */
void http_client_stop(void)
{
    if (s_http_task_handle == NULL) {
        ESP_LOGW(TAG, "HTTP client task not running");
        return;
    }

    /* Delete the task */
    vTaskDelete(s_http_task_handle);
    s_http_task_handle = NULL;
    ESP_LOGI(TAG, "HTTP client task stopped");
}
