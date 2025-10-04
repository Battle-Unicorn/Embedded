

#include "emg/emg.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mpu6050/mpu_utils.h"
#include "wifi_sta/wifi_sta.h"
#include "http_client/http_client.h"

#include "esp_netif.h"
#include "esp_netif_types.h"
#include "sntp/sntp.h"
#include "oled/oled.h"

void app_main(void) {
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("MAIN", "NVS initialized successfully");
    
    // Start sensor tasks
    mpu_task_start();
    emg_task_start();
    
    // Wait for sensors to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Start WiFi
    ESP_LOGI("MAIN", "Starting WiFi...");
    esp_err_t err = wifi_init_sta();
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "wifi_init_sta failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Wait up to 30 seconds for IP address
    ESP_LOGI("MAIN", "Waiting for IP address...");
    for (int i = 0; i < 30; i++) {
        esp_netif_ip_info_t ip_info;
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            if (ip_info.ip.addr != 0) {
                ESP_LOGI("MAIN", "WiFi Connected! IP: " IPSTR, IP2STR(&ip_info.ip));
                
                // Start HTTP client
                ESP_LOGI("MAIN", "Starting HTTP client...");
                err = http_client_start();
                if (err != ESP_OK) {
                    ESP_LOGE("MAIN", "http_client_start failed: %s", esp_err_to_name(err));
                } else {
                    ESP_LOGI("MAIN", "HTTP client started successfully");
                }
                break;
            }
        }
        
        if (i == 29) {
            ESP_LOGE("MAIN", "No IP address after 30 seconds - check router/DHCP");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
    
    // Initialize SNTP
    sntp_time_init();
    if (!sntp_wait_for_sync(10000)) { // Wait up to 10 seconds
		ESP_LOGW("MAIN", "SNTP time sync failed or timed out");
	} else {
		ESP_LOGI("MAIN", "SNTP time synchronized successfully");
	}
	
	//print time
	ESP_LOGI("MAIN", "Current time: %s", sntp_get_time_string());
	oled_task_start();
}