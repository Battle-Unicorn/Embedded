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

/* Tag used for ESP logging */
static const char *TAG = "http_client";

/* Server configuration */
#define SERVER_IP      "192.168.8.102"
#define SERVER_PORT    8080
#define SERVER_PATH    "/embedded/data"
#define POST_INTERVAL_MS (30000) /* 30 seconds */

/* Task handle for the background POST task */
static TaskHandle_t s_http_task_handle = NULL;

/* The JSON payload to send (exact payload provided by user).
   Error codes and comments are English. */
static const char s_payload[] =
"{\n"
"  \"device_id\": \"Dev_001\",\n"
"  \"timestamp\": \"2025-09-24T23:45:12Z\",\n"
"  \"sensor_data\": {\n"
"    \"plethysmometer\": [\n"
"      {\"timestamp\": \"2025-09-24T23:44:43Z\", \"heart_rate\": 63, \"spo2\": 98.1},\n"
"      {\"timestamp\": \"2025-09-24T23:44:44Z\", \"heart_rate\": 64, \"spo2\": 97.9},\n"
"      {\"timestamp\": \"2025-09-24T23:44:45Z\", \"heart_rate\": 63, \"spo2\": 98.0},\n"
"      {\"timestamp\": \"2025-09-24T23:44:46Z\", \"heart_rate\": 65, \"spo2\": 97.8},\n"
"      {\"timestamp\": \"2025-09-24T23:44:47Z\", \"heart_rate\": 64, \"spo2\": 98.1},\n"
"      {\"timestamp\": \"2025-09-24T23:44:48Z\", \"heart_rate\": 63, \"spo2\": 98.2},\n"
"      {\"timestamp\": \"2025-09-24T23:44:49Z\", \"heart_rate\": 65, \"spo2\": 97.9},\n"
"      {\"timestamp\": \"2025-09-24T23:44:50Z\", \"heart_rate\": 66, \"spo2\": 98.0},\n"
"      {\"timestamp\": \"2025-09-24T23:44:51Z\", \"heart_rate\": 64, \"spo2\": 98.1},\n"
"      {\"timestamp\": \"2025-09-24T23:44:52Z\", \"heart_rate\": 63, \"spo2\": 98.3},\n"
"      {\"timestamp\": \"2025-09-24T23:44:53Z\", \"heart_rate\": 65, \"spo2\": 97.8},\n"
"      {\"timestamp\": \"2025-09-24T23:44:54Z\", \"heart_rate\": 64, \"spo2\": 98.2},\n"
"      {\"timestamp\": \"2025-09-24T23:44:55Z\", \"heart_rate\": 66, \"spo2\": 97.9},\n"
"      {\"timestamp\": \"2025-09-24T23:44:56Z\", \"heart_rate\": 67, \"spo2\": 98.0},\n"
"      {\"timestamp\": \"2025-09-24T23:44:57Z\", \"heart_rate\": 65, \"spo2\": 98.1},\n"
"      {\"timestamp\": \"2025-09-24T23:44:58Z\", \"heart_rate\": 64, \"spo2\": 97.9},\n"
"      {\"timestamp\": \"2025-09-24T23:44:59Z\", \"heart_rate\": 63, \"spo2\": 98.2},\n"
"      {\"timestamp\": \"2025-09-24T23:45:00Z\", \"heart_rate\": 65, \"spo2\": 98.1},\n"
"      {\"timestamp\": \"2025-09-24T23:45:01Z\", \"heart_rate\": 66, \"spo2\": 97.8},\n"
"      {\"timestamp\": \"2025-09-24T23:45:02Z\", \"heart_rate\": 64, \"spo2\": 98.0},\n"
"      {\"timestamp\": \"2025-09-24T23:45:03Z\", \"heart_rate\": 65, \"spo2\": 98.1},\n"
"      {\"timestamp\": \"2025-09-24T23:45:04Z\", \"heart_rate\": 66, \"spo2\": 97.9},\n"
"      {\"timestamp\": \"2025-09-24T23:45:05Z\", \"heart_rate\": 63, \"spo2\": 98.3},\n"
"      {\"timestamp\": \"2025-09-24T23:45:06Z\", \"heart_rate\": 64, \"spo2\": 98.2},\n"
"      {\"timestamp\": \"2025-09-24T23:45:07Z\", \"heart_rate\": 65, \"spo2\": 98.0},\n"
"      {\"timestamp\": \"2025-09-24T23:45:08Z\", \"heart_rate\": 66, \"spo2\": 98.1},\n"
"      {\"timestamp\": \"2025-09-24T23:45:09Z\", \"heart_rate\": 67, \"spo2\": 97.8},\n"
"      {\"timestamp\": \"2025-09-24T23:45:10Z\", \"heart_rate\": 65, \"spo2\": 98.0},\n"
"      {\"timestamp\": \"2025-09-24T23:45:11Z\", \"heart_rate\": 64, \"spo2\": 98.2},\n"
"      {\"timestamp\": \"2025-09-24T23:45:12Z\", \"heart_rate\": 63, \"spo2\": 98.1}\n"
"    ],\n"
"    \"mpu\": {\n"
"      \"sleep_flag\": false,\n"
"      \"samples\": [\n"
"        {\"timestamp\": \"2025-09-24T23:44:43Z\", \"acceleration\": {\"x\": 0.01, \"y\": -0.02, \"z\": 0.98}, \"rotation\": {\"x\": 0.5, \"y\": 1.2, \"z\": 0.3}, \"temperature\": 37.0},\n"
"        {\"timestamp\": \"2025-09-24T23:44:44Z\", \"acceleration\": {\"x\": 0.02, \"y\": -0.01, \"z\": 0.97}, \"rotation\": {\"x\": 0.6, \"y\": 1.3, \"z\": 0.4}, \"temperature\": 37.1}\n"
"      ]\n"
"    },\n"
"    \"emg\": {\n"
"      \"atonia_flag\": false,\n"
"      \"samples\": [\n"
"        {\"timestamp\": \"2025-09-24T23:44:43Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:44:44Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:44:45Z\", \"muscle_tone\": 12.4},\n"
"        {\"timestamp\": \"2025-09-24T23:44:46Z\", \"muscle_tone\": 12.7},\n"
"        {\"timestamp\": \"2025-09-24T23:44:47Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:44:48Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:44:49Z\", \"muscle_tone\": 12.7},\n"
"        {\"timestamp\": \"2025-09-24T23:44:50Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:44:51Z\", \"muscle_tone\": 12.4},\n"
"        {\"timestamp\": \"2025-09-24T23:44:52Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:44:53Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:44:54Z\", \"muscle_tone\": 12.7},\n"
"        {\"timestamp\": \"2025-09-24T23:44:55Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:44:56Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:44:57Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:44:58Z\", \"muscle_tone\": 12.7},\n"
"        {\"timestamp\": \"2025-09-24T23:44:59Z\", \"muscle_tone\": 12.4},\n"
"        {\"timestamp\": \"2025-09-24T23:45:00Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:45:01Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:45:02Z\", \"muscle_tone\": 12.7},\n"
"        {\"timestamp\": \"2025-09-24T23:45:03Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:45:04Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:45:05Z\", \"muscle_tone\": 12.4},\n"
"        {\"timestamp\": \"2025-09-24T23:45:06Z\", \"muscle_tone\": 12.7},\n"
"        {\"timestamp\": \"2025-09-24T23:45:07Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:45:08Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:45:09Z\", \"muscle_tone\": 12.7},\n"
"        {\"timestamp\": \"2025-09-24T23:45:10Z\", \"muscle_tone\": 12.6},\n"
"        {\"timestamp\": \"2025-09-24T23:45:11Z\", \"muscle_tone\": 12.5},\n"
"        {\"timestamp\": \"2025-09-24T23:45:12Z\", \"muscle_tone\": 12.6}\n"
"      ]\n"
"    }\n"
"  },\n"
"  \"response\": {\n"
"    \"status\": \"received\",\n"
"    \"device_id\": \"Dev_001\",\n"
"    \"samples_received\": 30,\n"
"    \"received_at\": \"2025-09-24T23:45:13Z\"\n"
"  }\n"
"}\n";

/* Helper: build server URL string */
static void build_url(char *out, size_t out_len)
{
    /* Format: http://<ip>:<port><path> */
    snprintf(out, out_len, "http://%s:%d%s", SERVER_IP, SERVER_PORT, SERVER_PATH);
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
