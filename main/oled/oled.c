/*
 * oled.c
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ssd1306.h"
#include "font8x8_basic.h"
#include "oled.h"

// Pin definitions
#define MOSI 5
#define SCLK 18
#define CS   17
#define DC   21
#define RESET 19

#define TAG "SSD1306"

extern int steps;

// Mock data for heart rate and steps
static int heart_rate = 72;

static void display_current_data(SSD1306_t *dev)
{
    char buffer[20];
    time_t now;
    struct tm timeinfo;
    
    // Get current time
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // Clear the display area for data (lines 2-7)
    for(int line = 2; line < 8; line++) {
        ssd1306_clear_line(dev, line, false);
    }
    
    // Display time
    strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);
    ssd1306_display_text(dev, 2, "Time:", 5, false);
    ssd1306_display_text(dev, 3, buffer, strlen(buffer), false);
    
    // Display heart rate
    snprintf(buffer, sizeof(buffer), "HR: %d bpm", heart_rate);
    ssd1306_display_text(dev, 4, buffer, strlen(buffer), false);
    
    // Display steps
    snprintf(buffer, sizeof(buffer), "Steps: %d", steps);
    ssd1306_display_text(dev, 5, buffer, strlen(buffer), false);
    
    // Display date
    strftime(buffer, sizeof(buffer), "%Y-%m-%d", &timeinfo);
    ssd1306_display_text(dev, 6, buffer, strlen(buffer), false);
    
    // Increment mock data for demonstration
    heart_rate += (rand() % 3) - 1; // +/- 1 bpm
    if (heart_rate < 60) heart_rate = 60;
    if (heart_rate > 100) heart_rate = 100;
    
}

static void oled_display_task(void *pvParameters)
{
    SSD1306_t dev;
    
    ESP_LOGI(TAG, "INTERFACE is SPI");
    ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", MOSI);
    ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", SCLK);
    ESP_LOGI(TAG, "CONFIG_CS_GPIO=%d", CS);
    ESP_LOGI(TAG, "CONFIG_DC_GPIO=%d", DC);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", RESET);
    
    // Initialize SPI master
    spi_master_init(&dev, MOSI, SCLK, CS, DC, RESET);

    ESP_LOGI(TAG, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);

    // Clear screen and display initial "Hello" message
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Clear screen and start main display loop
    ssd1306_clear_screen(&dev, false);
    
    // Display header
    ssd1306_display_text(&dev, 0, "SMART SLEEP   ", 15, false);
    ssd1306_display_text(&dev, 1, "-----------------", 14, false);
    
    while(1) {
        // Update and display current data
        display_current_data(&dev);
        
        // Update every second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void oled_task_start(void)
{
    // Create task for OLED display
    xTaskCreate(oled_display_task, "oled_display_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "OLED display task started");
}