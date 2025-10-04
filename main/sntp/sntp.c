/*
 * sntp.c
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */
#include "sntp.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "string.h"
#include "time.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

static const char *TAG = "SNTP";

static bool time_synced = false;

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized with SNTP server");
    time_synced = true;
}

void sntp_time_init(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");

    // Basic SNTP configuration with Polish NTP server
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("0.pl.pool.ntp.org");
    config.sync_cb = time_sync_notification_cb;
    config.start = true;
    config.server_from_dhcp = false;
    
    esp_netif_sntp_init(&config);

    // Set Poland timezone (CET/CEST)
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    ESP_LOGI(TAG, "SNTP initialized with server: 0.pl.pool.ntp.org");
}

char* sntp_get_time_string(void)
{
    static char time_str[64];
    time_t now;
    struct tm timeinfo;
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return time_str;
}

bool sntp_time_is_synced(void)
{
    return time_synced;
}

bool sntp_wait_for_sync(int timeout_ms)
{
    int retry = 0;
    const int retry_count = timeout_ms / 1000;
    
    while (!sntp_time_is_synced() && retry < retry_count) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
        ESP_LOGI(TAG, "Waiting for time sync... (%d/%d)", retry, retry_count);
    }
    
    return sntp_time_is_synced();
}