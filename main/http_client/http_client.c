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
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "time.h"
#include "cJSON.h"

/* Tag used for ESP logging */
static const char *TAG = "http_client";

/* Task handle for the background POST task */
static TaskHandle_t s_http_task_handle = NULL;

/* Queue for EMG samples */
static QueueHandle_t s_emg_queue = NULL;
#define EMG_QUEUE_SIZE 50
#define MAX_EMG_SAMPLES_PER_POST 30  /* Send max 30 samples per POST */

/* The JSON payload buffers */
static char s_flags_payload[1024] = {0};
static char s_emg_payload[4096] = {0}; /* Larger buffer for EMG data */

/* External variables */
extern bool atonia_flag;
extern bool sleep_flag;
extern float temp;
extern int steps;

/* Helper: build server URL string for different endpoints */
static void build_url(char *out, size_t out_len, const char *endpoint)
{
    /* Format: http://<ip>:<port><path> */
    snprintf(out, out_len, "http://%s:%d%s", SERVER_IP, SERVER_PORT, endpoint);
}

/* Build flags JSON payload */
static void build_flags_payload(void)
{
    /* Get current timestamp */
    time_t now;
    time(&now);
    struct tm *timeinfo = gmtime(&now);
    char time_str[25];
    strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", timeinfo);

    /* Get WiFi RSSI */
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    int8_t rssi = 0;
    if (ret == ESP_OK) {
        rssi = ap_info.rssi;
    }

    /* Build JSON payload for flags */
    snprintf(s_flags_payload, sizeof(s_flags_payload),
             "{\"device_id\":\"Dev_001\",\"timestamp\":\"%s\",\"flags\":{\"sleep_flag\":%s,\"atonia_flag\":%s},"
             "\"system_info\":{\"battery_level\":82,\"signal_strength\":%d,\"body_temperature\":%.1f,\"steps\":%d,\"last_calibration\":\"2025-09-24T20:00:00Z\"}}",
             time_str,
             sleep_flag ? "true" : "false",
             atonia_flag ? "true" : "false",
             rssi,
             temp,
             steps);  
}

/* Build EMG JSON payload from samples in queue */
static size_t build_emg_payload(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *emg_obj = cJSON_CreateObject();
    cJSON *samples_array = cJSON_CreateArray();

    if (!root || !emg_obj || !samples_array) {
        ESP_LOGE(TAG, "Failed to create JSON objects");
        if (root) cJSON_Delete(root);
        return 0;
    }

    /* Get current timestamp for the request */
    time_t now;
    time(&now);
    struct tm *timeinfo = gmtime(&now);
    char time_str[25];
    strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", timeinfo);

    /* Add device info */
    cJSON_AddStringToObject(root, "device_id", "Dev_001");
    cJSON_AddStringToObject(root, "timestamp", time_str);

    /* Collect samples from queue (up to MAX_EMG_SAMPLES_PER_POST) */
    emg_sample_t sample;
    int samples_collected = 0;
    
    while (samples_collected < MAX_EMG_SAMPLES_PER_POST && 
           xQueueReceive(s_emg_queue, &sample, 0) == pdTRUE) {
        
        cJSON *sample_obj = cJSON_CreateObject();
        if (sample_obj) {
            cJSON_AddStringToObject(sample_obj, "timestamp", sample.timestamp);
            cJSON_AddNumberToObject(sample_obj, "envelope", sample.envelope);
            cJSON_AddItemToArray(samples_array, sample_obj);
            samples_collected++;
        }
    }

    ESP_LOGI(TAG, "Collected %d EMG samples for POST", samples_collected);

    /* Build the complete EMG object */
    cJSON_AddItemToObject(emg_obj, "samples", samples_array);
    cJSON_AddItemToObject(root, "emg", emg_obj);

    /* Serialize to string */
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        size_t json_len = strlen(json_str);
        if (json_len < sizeof(s_emg_payload)) {
            strncpy(s_emg_payload, json_str, sizeof(s_emg_payload));
            s_emg_payload[sizeof(s_emg_payload) - 1] = '\0';
        } else {
            ESP_LOGW(TAG, "EMG payload too large: %d > %d", json_len, sizeof(s_emg_payload));
            json_len = 0;
        }
        free(json_str);
        cJSON_Delete(root);
        return json_len;
    }

    cJSON_Delete(root);
    return 0;
}

/* Send HTTP POST request */
static esp_err_t send_http_post(const char *url, const char *payload, const char *endpoint_name)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000, /* 10 s timeout */
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client for %s", endpoint_name);
        return ESP_ERR_NO_MEM;
    }

    /* Set headers */
    esp_err_t err = esp_http_client_set_header(client, "Content-Type", "application/json");
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set header for %s: %s", endpoint_name, esp_err_to_name(err));
    }
    
    err = esp_http_client_set_post_field(client, payload, strlen(payload));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set POST field for %s: %s", endpoint_name, esp_err_to_name(err));
    }

    /* Perform the POST */
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "%s POST succeeded, status=%d", endpoint_name, status);
    } else {
        ESP_LOGE(TAG, "%s HTTP POST failed: %s", endpoint_name, esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

/* The background task that repeatedly POSTs data */
static void http_post_task(void *arg)
{
    ESP_LOGI(TAG, "HTTP POST task started");

    char flags_url[128];
    char emg_url[128];
    
    build_url(flags_url, sizeof(flags_url), FLAGS_ENDPOINT);
    build_url(emg_url, sizeof(emg_url), EMG_ENDPOINT);

    while (1) {
        /* Send flags data */
        build_flags_payload();
        send_http_post(flags_url, s_flags_payload, "flags");

        /* Send EMG data if available */
        size_t emg_payload_size = build_emg_payload();
        if (emg_payload_size > 0) {
            send_http_post(emg_url, s_emg_payload, "EMG");
        } else {
            ESP_LOGI(TAG, "No EMG data to send");
        }

        /* Sleep for configured interval */
        vTaskDelay(pdMS_TO_TICKS(POST_INTERVAL_MS));
    }

    vTaskDelete(NULL);
}

/**
 * Initialize EMG data queue
 *
 * Returns:
 *   - ESP_OK: queue created successfully
 *   - ESP_ERR_NO_MEM: not enough memory for queue
 */
esp_err_t http_client_init_emg_queue(void)
{
    if (s_emg_queue != NULL) {
        ESP_LOGW(TAG, "EMG queue already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_emg_queue = xQueueCreate(EMG_QUEUE_SIZE, sizeof(emg_sample_t));
    if (s_emg_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create EMG queue");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "EMG queue created with size %d", EMG_QUEUE_SIZE);
    return ESP_OK;
}

/**
 * Send EMG sample to queue for later transmission
 *
 * Parameters:
 *   - sample: pointer to EMG sample structure
 *
 * Returns:
 *   - pdTRUE: sample sent successfully
 *   - pdFALSE: queue full or error
 */
BaseType_t http_client_send_emg_sample(const emg_sample_t *sample)
{
    if (s_emg_queue == NULL) {
        ESP_LOGE(TAG, "EMG queue not initialized");
        return pdFALSE;
    }

    BaseType_t result = xQueueSend(s_emg_queue, sample, 0); /* Non-blocking */
    if (result != pdTRUE) {
        ESP_LOGW(TAG, "EMG queue full, sample dropped");
    }
    
    return result;
}

/**
 * Start the HTTP client background task
 *
 * Returns:
 *   - ESP_OK: task created successfully
 *   - ESP_ERR_NO_MEM: not enough memory to create task
 *   - ESP_ERR_INVALID_STATE: already started
 */
esp_err_t http_client_start(void)
{
    if (s_http_task_handle != NULL) {
        ESP_LOGW(TAG, "HTTP client task already running");
        return ESP_ERR_INVALID_STATE;
    }

    /* Initialize EMG queue if not already done */
    if (s_emg_queue == NULL) {
        esp_err_t ret = http_client_init_emg_queue();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize EMG queue");
            return ret;
        }
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
        ESP_LOGE(TAG, "Failed to create HTTP client task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "HTTP client task created");
    return ESP_OK;
}

/**
 * Stop the HTTP client background task and cleanup
 */
void http_client_stop(void)
{
    if (s_http_task_handle == NULL) {
        ESP_LOGW(TAG, "HTTP client task not running");
        return;
    }

    vTaskDelete(s_http_task_handle);
    s_http_task_handle = NULL;
    
    if (s_emg_queue != NULL) {
        vQueueDelete(s_emg_queue);
        s_emg_queue = NULL;
    }
    
    ESP_LOGI(TAG, "HTTP client task stopped");
}