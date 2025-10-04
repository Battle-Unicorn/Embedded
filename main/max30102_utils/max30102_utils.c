/*
 * max_30102_utils.c
 *
 * Updated with configuration from working driver and advanced algorithms
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
#include <driver/i2c.h>
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

// I2C settings
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 200000
#define I2C_MASTER_SDA_GPIO 25
#define I2C_MASTER_SCL_GPIO 26

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
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PILOT_PA        0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR       0x1F
#define REG_TEMP_FRAC       0x20
#define REG_TEMP_CONFIG     0x21
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF

// Sampling delay (in milliseconds)
#define SAMPLE_DELAY_MS 10

// --- Low level I2C helpers (combined transactions) ---

static esp_err_t max30102_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed reg 0x%02x (err 0x%x)", reg_addr, ret);
    }
    return ret;
}

static esp_err_t max30102_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (len == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed reg 0x%02x (err 0x%x)", reg_addr, ret);
    }
    return ret;
}

// Init I2C
static esp_err_t max30102_init_i2c(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: 0x%x", ret);
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialised successfully");
    return ESP_OK;
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

    uint8_t part_id = 0;
    ret = max30102_register_read(REG_PART_ID, &part_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PART ID");
        return ret;
    }
    ESP_LOGI(TAG, "PART ID: 0x%02x", part_id);

    if (part_id == 0xFF) {
        ESP_LOGW(TAG, "PART ID is 0xFF, performing reset...");
        ret = max30102_reset();
        if (ret != ESP_OK) return ret;

        ret = max30102_register_read(REG_PART_ID, &part_id, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read PART ID after reset");
            return ret;
        }
        ESP_LOGI(TAG, "PART ID after reset: 0x%02x", part_id);
    }

    // Clear FIFO pointers
    ESP_LOGI(TAG, "Clearing FIFO pointers...");
    if ((ret = max30102_register_write_byte(REG_FIFO_WR_PTR, 0x00)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_OVF_COUNTER, 0x00)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_FIFO_RD_PTR, 0x00)) != ESP_OK) return ret;

    // FIFO config: SMP_AVE = 0b010 (4 samples average), FIFO_ROLLOVER = 1, A_FULL = 0
    ESP_LOGI(TAG, "Configuring FIFO...");
    if ((ret = max30102_register_write_byte(REG_FIFO_CONFIG, 0x50)) != ESP_OK) return ret;

    // Set mode to SPO2 (0x03)
    ESP_LOGI(TAG, "Setting SPO2 mode...");
    if ((ret = max30102_register_write_byte(REG_MODE_CONFIG, 0x03)) != ESP_OK) return ret;

    // SPO2 config: ADC range = 01, SR = 001 (200 samples/sec), PW = 10 (215µs)
    ESP_LOGI(TAG, "Configuring SPO2...");
    if ((ret = max30102_register_write_byte(REG_SPO2_CONFIG, 0x26)) != ESP_OK) return ret;

    // LED currents - zgodnie z working driver
    ESP_LOGI(TAG, "Configuring LED current...");
    if ((ret = max30102_register_write_byte(REG_LED1_PA, 0x24)) != ESP_OK) return ret; // RED - 25.4mA
    if ((ret = max30102_register_write_byte(REG_LED2_PA, 0x24)) != ESP_OK) return ret; // IR - 25.4mA
    if ((ret = max30102_register_write_byte(REG_PILOT_PA, 0x7F)) != ESP_OK) return ret;

    // Multi-LED control: ALL SLOTS DISABLED (0x00) - zgodnie z working driver
    ESP_LOGI(TAG, "Configuring multi-LED control...");
    if ((ret = max30102_register_write_byte(REG_MULTI_LED_CTRL1, 0x00)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_MULTI_LED_CTRL2, 0x00)) != ESP_OK) return ret;

    // Interrupt enable: A_FULL (bit7) + PPG_RDY (bit6)
    ESP_LOGI(TAG, "Configuring interrupts...");
    if ((ret = max30102_register_write_byte(REG_INTR_ENABLE_1, 0xC0)) != ESP_OK) return ret;
    if ((ret = max30102_register_write_byte(REG_INTR_ENABLE_2, 0x00)) != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify configuration
    uint8_t mode_reg = 0, spo2_reg = 0, led1_reg = 0, led2_reg = 0, multi_led_reg = 0, fifo_conf = 0, intr_en1 = 0;
    max30102_register_read(REG_MODE_CONFIG, &mode_reg, 1);
    max30102_register_read(REG_SPO2_CONFIG, &spo2_reg, 1);
    max30102_register_read(REG_LED1_PA, &led1_reg, 1);
    max30102_register_read(REG_LED2_PA, &led2_reg, 1);
    max30102_register_read(REG_MULTI_LED_CTRL1, &multi_led_reg, 1);
    max30102_register_read(REG_FIFO_CONFIG, &fifo_conf, 1);
    max30102_register_read(REG_INTR_ENABLE_1, &intr_en1, 1);

    ESP_LOGI(TAG, "Configuration verification:");
    ESP_LOGI(TAG, "MODE_CONFIG: 0x%02x", mode_reg);
    ESP_LOGI(TAG, "SPO2_CONFIG: 0x%02x", spo2_reg);
    ESP_LOGI(TAG, "LED1_PA: 0x%02x", led1_reg);
    ESP_LOGI(TAG, "LED2_PA: 0x%02x", led2_reg);
    ESP_LOGI(TAG, "MULTI_LED_CTRL1: 0x%02x", multi_led_reg);
    ESP_LOGI(TAG, "FIFO_CONFIG: 0x%02x", fifo_conf);
    ESP_LOGI(TAG, "INTR_ENABLE_1: 0x%02x", intr_en1);

    ESP_LOGI(TAG, "MAX30102 initialised successfully");
    return ESP_OK;
}

// Read one FIFO sample (red + ir)
static esp_err_t max30102_read_fifo_once(int32_t *red, int32_t *ir)
{
    if (!red || !ir) return ESP_ERR_INVALID_ARG;

    uint8_t raw[6] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, raw + 5, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
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

    if (max30102_init_i2c() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        vTaskDelete(NULL);
        return;
    }

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
    xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 5, NULL);
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