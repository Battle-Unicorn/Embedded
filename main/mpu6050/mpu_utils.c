/*
 * mpu_utils.c
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */



#include "mpu_utils.h"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <mpu6050.h>

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

// Global variables
int steps = 0;
bool sleep_flag = false;
static const char *TAG = "mpu6050_task";

// Step detection algorithm variables
static time_t last_step_time = 0;
static time_t last_activity_time = 0;
static time_t last_reset_time = 0;
static int activity_counter = 0;
static float accel_magnitude_history[20];
static int history_index = 0;

// Step detection thresholds
#define ACCEL_THRESHOLD_MIN 1.3f    // Minimum acceleration for step detection
#define ACCEL_THRESHOLD_MAX 3.0f    // Maximum acceleration for step detection  
#define STEP_TIMEOUT_MS 300         // Minimum time between steps (ms)
#define SLEEP_TIMEOUT 300           // 5 minutes of inactivity (300 seconds)
#define WAKEUP_ACTIVITY_COUNT 15    // Number of steps in 20 seconds to wake up
#define ACTIVITY_WINDOW 20          // Activity monitoring window (seconds)

static float calculate_accel_magnitude(mpu6050_acceleration_t *accel)
{
    return sqrt(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
}

static void reset_steps_at_midnight(void)
{
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    // Check if it's a new day (00:00) and we haven't reset today
    if (timeinfo.tm_hour == 0 && timeinfo.tm_min == 0 && 
        timeinfo.tm_sec < 5 && difftime(now, last_reset_time) > 86400) {
        
        steps = 0;
        last_reset_time = now;
        ESP_LOGI(TAG, "Step counter reset at midnight - New day started!");
    }
}

static void detect_step(mpu6050_acceleration_t *accel)
{
    time_t current_time = time(NULL);
    float accel_magnitude = calculate_accel_magnitude(accel);
    
    // Store in history for activity monitoring
    accel_magnitude_history[history_index] = accel_magnitude;
    history_index = (history_index + 1) % 20;
    
    // Check if acceleration is within valid step range and timeout has passed
    if (accel_magnitude > ACCEL_THRESHOLD_MIN && 
        accel_magnitude < ACCEL_THRESHOLD_MAX &&
        difftime(current_time * 1000, last_step_time * 1000) > STEP_TIMEOUT_MS) {
        
        // Valid step detected
        steps++;
        last_step_time = current_time;
        last_activity_time = current_time;
        activity_counter++;
        
        ESP_LOGI(TAG, "Step detected! Magnitude: %.3fg, Total steps: %d", accel_magnitude, steps);
    }
}

static void update_sleep_state(void)
{
    time_t current_time = time(NULL);
    float activity_variance = 0.0;
    float mean = 0.0;
    
    // Calculate variance of recent acceleration for micro-movement detection
    for (int i = 0; i < 20; i++) {
        mean += accel_magnitude_history[i];
    }
    mean /= 20.0;
    
    for (int i = 0; i < 20; i++) {
        activity_variance += pow(accel_magnitude_history[i] - mean, 2);
    }
    activity_variance /= 20.0;
    
    // Check inactivity duration
    float inactivity_duration = difftime(current_time, last_activity_time);
    
    // Sleep condition: 5 minutes of inactivity OR low variance (micro-movements only)
    if (inactivity_duration > SLEEP_TIMEOUT || activity_variance < 0.05) {
        if (!sleep_flag) {
            sleep_flag = true;
            activity_counter = 0; // Reset activity counter when entering sleep
            ESP_LOGI(TAG, "Sleep mode activated - Inactivity: %.0fs, Variance: %.4f", 
                    inactivity_duration, activity_variance);
        }
    }
    
    // Wake-up condition: sufficient activity detected while in sleep mode
    if (sleep_flag && activity_counter >= WAKEUP_ACTIVITY_COUNT) {
        sleep_flag = false;
        ESP_LOGI(TAG, "Wake up detected! Activity count: %d", activity_counter);
    }
    
    // Reset activity counter every ACTIVITY_WINDOW seconds for wake-up detection
    if (difftime(current_time, last_activity_time) > ACTIVITY_WINDOW) {
        activity_counter = 0;
    }
}

static void mpu6050_task(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    // Initialize timing variables
    last_activity_time = time(NULL);
    last_reset_time = time(NULL);
    last_step_time = time(NULL);
    
    // Initialize acceleration history
    for (int i = 0; i < 20; i++) {
        accel_magnitude_history[i] = 1.0f; // Start with 1g (gravity)
    }

    ESP_LOGI(TAG, "Step counter and sleep detection initialized");

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        // Process sensor data for step counting and sleep detection
        detect_step(&accel);
        update_sleep_state();
        reset_steps_at_midnight();

        // Debug logging (can be enabled for troubleshooting)
        /*
        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        ESP_LOGI(TAG, "Temperature:  %.1f", temp);
        ESP_LOGI(TAG, "Steps: %d, Sleep: %s", steps, sleep_flag ? "YES" : "NO");
        */

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sampling rate
    }
}

void mpu_task_start(void)
{
    xTaskCreate(mpu6050_task, "mpu6050_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}