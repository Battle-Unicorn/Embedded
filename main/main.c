

#include "emg/emg.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mpu6050/mpu_utils.h"

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
    
    // Start all tasks
    //max30102_task_start();
    //mpu_task_start();
    // Start EMG task
    emg_task_start();
    
    
}