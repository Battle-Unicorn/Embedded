/*
 * max_30102_utils.c
 *
 * Updated to use i2cdev driver for compatibility with other I2C devices
 * Compatible with ESP-IDF 5
 *
 * Author: majorBien
 */

#include "max30102_utils.h"
#include <stdio.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <i2cdev.h>
#include <string.h>
#include "algorithm.h"

#define TAG "MAX30102"

// Global variables
int heart_rate = 0;
float temperature = 0.0f;
double spo2_value = 0.0;

// Data buffers
int32_t red_data_buffer[BUFFER_SIZE];
int32_t ir_data_buffer[BUFFER_SIZE];
double auto_correlationated_data[BUFFER_SIZE];

// I2C device handle
static i2c_dev_t max30102_dev;

// MAX30102 I2C address
#define MAX30102_I2C_ADDRESS 0x57

// MAX30102 registers
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C // Red LED pulse amplitude register
#define REG_LED2_PA         0x0D // IR LED pulse amplitude register
#define REG_PILOT_PA        0x10 // Pilot LED pulse amplitude (if available on your variant)
#define REG_MULTI_LED_CTRL1 0x11 // Multi-LED mode control (slots 1/2)
#define REG_MULTI_LED_CTRL2 0x12 // Multi-LED mode control (slots 3/4)
#define REG_TEMP_INTR       0x1F
#define REG_TEMP_FRAC       0x20
#define REG_TEMP_CONFIG     0x21
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF

// Sampling delay (in milliseconds)
#define SAMPLE_DELAY_MS 10

// I2C settings (should match your hardware configuration)
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_SDA_GPIO 25
#define I2C_MASTER_SCL_GPIO 26

// --- Low level I2C helpers using i2cdev ---

static esp_err_t max30102_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    return i2c_dev_write_reg(&max30102_dev, reg_addr, &data, 1);
}

static esp_err_t max30102_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_dev_read_reg(&max30102_dev, reg_addr, data, len);
}

static esp_err_t max30102_reset(void)
{
    ESP_LOGI(TAG, "Resetting MAX30102...");
    esp_err_t ret = max30102_register_write_byte(REG_MODE_CONFIG, 0x40);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset command failed");
        return ret;
    }

    uint8_t mode = 0;
    const TickType_t timeout = pdMS_TO_TICKS(500);
    TickType_t start = xTaskGetTickCount();
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (max30102_register_read(REG_MODE_CONFIG, &mode, 1) != ESP_OK) {
            return ESP_FAIL;
        }
        if ((xTaskGetTickCount() - start) > timeout) {
            ESP_LOGW(TAG, "Reset timeout, proceeding anyway");
            break;
        }
    } while (mode & 0x40);

    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static esp_err_t max30102_init(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initialising MAX30102...");

    memset(&max30102_dev, 0, sizeof(i2c_dev_t));
    max30102_dev.port = I2C_MASTER_NUM;
    max30102_dev.addr = MAX30102_I2C_ADDRESS;
    max30102_dev.cfg.sda_io_num = I2C_MASTER_SDA_GPIO;
    max30102_dev.cfg.scl_io_num = I2C_MASTER_SCL_GPIO;
    max30102_dev.cfg.master.clk_speed = 200000;
    max30102_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    max30102_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;

    if ((ret = i2c_dev_probe(&max30102_dev, I2C_DEV_WRITE)) != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 not found");
        return ret;
    }

    uint8_t part_id = 0;
    if ((ret = max30102_register_read(REG_PART_ID, &part_id, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PART ID");
        return ret;
    }
    ESP_LOGI(TAG, "PART ID: 0x%02x", part_id);

    // Clear FIFO pointers
    ESP_LOGI(TAG, "Clearing FIFO pointers...");
    if ((ret = max30102_register_write_byte(REG_FIFO_WR_PTR, 0x00)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_OVF_COUNTER, 0x00)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_FIFO_RD_PTR, 0x00)) != ESP_OK) return ret;

    // FIFO config: no averaging, rollover off, A_FULL = 0 (fastest raw stream)
    ESP_LOGI(TAG, "Configuring FIFO (no averaging)...");
    if ((ret = max30102_register_write_byte(REG_FIFO_CONFIG, 0x00)) != ESP_OK) return ret;

    // --- Mode settings: Multi-LED mode (MODE[2:0] = 111)
    // Multi-LED mode allows explicit assignment of slots (here: SLOT1 = RED, SLOT2 = IR)
    ESP_LOGI(TAG, "Setting Multi-LED mode (RED+IR)...");
    if ((ret = max30102_register_write_byte(REG_MODE_CONFIG, 0x07)) != ESP_OK) return ret;

    // SPO2 config: ADC range = 11 (largest), SR = 010 (200 sps), PW = 10 (215 us)
    // bits: [6:5]=11, [4:2]=010, [1:0]=10  => 0x6A
    ESP_LOGI(TAG, "Configuring SPO2 (ADC range=max, 200sps, PW=215us)...");
    if ((ret = max30102_register_write_byte(REG_SPO2_CONFIG, 0x6A)) != ESP_OK) return ret;

    // LED currents -> MAX (0xFF ≈ 51 mA typical)
    ESP_LOGI(TAG, "Setting LED currents to MAX (0xFF)...");
    if ((ret = max30102_register_write_byte(REG_LED1_PA, 0xFF)) != ESP_OK) return ret; // RED
    if ((ret = max30102_register_write_byte(REG_LED2_PA, 0xFF)) != ESP_OK) return ret; // IR
    // if your hardware exposes REG_PILOT_PA you can also set it, but the MAX30102 datasheet does not always expose it
    // if ((ret = max30102_register_write_byte(REG_PILOT_PA, 0xFF)) != ESP_OK) return ret;

    // Multi-LED slot mapping:
    // REG_MULTI_LED_CTRL1 (0x11): [7:5]=SLOT2, [2:0]=SLOT1
    // SLOT1 = 001 (LED1/RED), SLOT2 = 010 (LED2/IR) => 0b010 000 001 = 0x41
    ESP_LOGI(TAG, "Configuring multi-LED slots (SLOT1=RED, SLOT2=IR)...");
    if ((ret = max30102_register_write_byte(REG_MULTI_LED_CTRL1, 0x41)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_MULTI_LED_CTRL2, 0x00)) != ESP_OK) return ret; // SLOT3/4 disabled

    // Interrupts: enable PPG_RDY (bit6) and optionally A_FULL (bit7) if you want FIFO notifications
    ESP_LOGI(TAG, "Configuring interrupts...");
    if ((ret = max30102_register_write_byte(REG_INTR_ENABLE_1, 0xC0)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_INTR_ENABLE_2, 0x00)) != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(50));

    // optional register verification (as in your original)...
    ESP_LOGI(TAG, "MAX30102 initialised in FULL-POWER mode");
    return ESP_OK;
}


// Read one FIFO sample (red + ir)
static esp_err_t max30102_read_fifo_once(int32_t *red, int32_t *ir)
{
    if (!red || !ir) return ESP_ERR_INVALID_ARG;

    uint8_t raw[6] = {0};
    esp_err_t ret = max30102_register_read(REG_FIFO_DATA, raw, 6);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO data (err 0x%x)", ret);
        *red = 0;
        *ir = 0;
        return ret;
    }

    uint32_t r = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2];
    uint32_t i = ((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 8) | raw[5];

    r &= 0x3FFFF;
    i &= 0x3FFFF;

    *red = (int32_t)r;
    *ir = (int32_t)i;
    return ESP_OK;
}

// Read temperature from MAX30102
static float max30102_read_temp(void)
{
    uint8_t int_temp = 0;
    uint8_t decimal_temp = 0;
    float temp = 0.0f;

    // Start temperature measurement
    if (max30102_register_write_byte(REG_TEMP_CONFIG, 0x01) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start temperature measurement");
        return 0.0f;
    }

    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(40));

    // Read integer and fractional parts
    if (max30102_register_read(REG_TEMP_INTR, &int_temp, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature integer part");
        return 0.0f;
    }
    if (max30102_register_read(REG_TEMP_FRAC, &decimal_temp, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature fractional part");
        return 0.0f;
    }

    temp = (int8_t)int_temp + ((float)decimal_temp * 0.0625f); // Fractional part is in 1/16th increments
    return temp;
}

static void max30102_diagnostics(void)
{
    uint8_t int_status1 = 0, int_status2 = 0, fifo_wr = 0, fifo_rd = 0, fifo_ovf = 0;

    if (max30102_register_read(REG_INTR_STATUS_1, &int_status1, 1) == ESP_OK) {
        ESP_LOGI(TAG, "INTR_STATUS_1: 0x%02x", int_status1);
    }
    if (max30102_register_read(REG_INTR_STATUS_2, &int_status2, 1) == ESP_OK) {
        ESP_LOGI(TAG, "INTR_STATUS_2: 0x%02x", int_status2);
    }
    if (max30102_register_read(REG_FIFO_WR_PTR, &fifo_wr, 1) == ESP_OK) {
        ESP_LOGI(TAG, "FIFO_WR_PTR: 0x%02x", fifo_wr);
    }
    if (max30102_register_read(REG_FIFO_RD_PTR, &fifo_rd, 1) == ESP_OK) {
        ESP_LOGI(TAG, "FIFO_RD_PTR: 0x%02x", fifo_rd);
    }
    if (max30102_register_read(REG_OVF_COUNTER, &fifo_ovf, 1) == ESP_OK) {
        ESP_LOGI(TAG, "FIFO_OVF: 0x%02x", fifo_ovf);
    }
}

// Fill data buffers with samples
static void fill_buffers_data(void)
{
    for (int i = 0; i < BUFFER_SIZE; i++) {
        int32_t red = 0, ir = 0;
        if (max30102_read_fifo_once(&red, &ir) == ESP_OK) {
            red_data_buffer[i] = red;
            ir_data_buffer[i] = ir;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}

static void max30102_processing_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MAX30102 processing task starting...");

    uint64_t ir_mean = 0;
    uint64_t red_mean = 0;
    double r0_autocorrelation = 0.0;

    // Initialize time array for algorithms
    init_time_array();

    while (1) {
        // Fill data buffers with fresh samples
        fill_buffers_data();

        // Read temperature
        temperature = max30102_read_temp();

        // Remove DC component
        remove_dc_part(ir_data_buffer, red_data_buffer, &ir_mean, &red_mean);

        // Remove slow trend (baseline drift)
        remove_trend_line(ir_data_buffer);
        remove_trend_line(red_data_buffer);

        // Compute correlation between IR and RED signals
        double pearson_correlation = correlation_datay_datax(red_data_buffer, ir_data_buffer);
        if (isnan(pearson_correlation)) {
            pearson_correlation = 0.0;
        }

        // Compute heart rate from IR signal
        heart_rate = calculate_heart_rate(ir_data_buffer, &r0_autocorrelation, auto_correlationated_data);

        // Log sensor data
        ESP_LOGI(TAG, "HR: %d bpm, Corr: %.3f, Temp: %.1f C",
                heart_rate, pearson_correlation, temperature);

        // If IR and RED signals are strongly correlated, compute SpO2
        if (pearson_correlation >= 0.7) {
            spo2_value = spo2_measurement(ir_data_buffer, red_data_buffer, ir_mean, red_mean);
            ESP_LOGI(TAG, "SpO2: %.1f%%", spo2_value);
        } else {
            spo2_value = 0.0;
            ESP_LOGW(TAG, "SpO2: skipped (weak correlation: %.3f)", pearson_correlation);
        }

        // Diagnostics every 100 cycles
        static int cycle_count = 0;
        if (++cycle_count % 100 == 0) {
            max30102_diagnostics();
        }
    }
}

static void max30102_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MAX30102 task starting...");

    // Wait a bit to ensure I2C is initialized by main application
    vTaskDelay(pdMS_TO_TICKS(100));

    if (max30102_init() != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 initialization failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "MAX30102 task running successfully");

    // Start processing task
    xTaskCreate(max30102_processing_task, "max30102_processing", 8192, NULL, 5, NULL);
    
    // Keep this task alive but not doing much
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void max_task_start(void)
{
    // Note: i2cdev_init() should be called once in main application
    //xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 9, NULL);
    xTaskCreatePinnedToCore(max30102_task, "max30102_task", 8192, NULL, 9, NULL, 1);
    ESP_LOGI(TAG, "MAX30102 task created");
}

// Public getter functions
int get_heart_rate(void) {
    return heart_rate;
}

float get_temperature(void) {
    return temperature;
}

double get_spo2(void) {
    return spo2_value;
}
