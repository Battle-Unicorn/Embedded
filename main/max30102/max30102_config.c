/*
 * max30102_config.c
 *
 *  Created on: 4 paź 2025
 *      Author: lenovo
 */




#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "max30102_config.h"
#include "max30102_api.h"
#include "algorithm.h"
#include "i2c_api.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h> // for isnan()

// =============== TASK HANDLES ===============
TaskHandle_t sensor_reader_handle = NULL;

// MAX30102 configuration structure
max_config max30102_configuration = {
    .INT_EN_1.A_FULL_EN         = 1,
    .INT_EN_1.PPG_RDY_EN        = 1,
    .INT_EN_1.ALC_OVF_EN        = 0,
    .INT_EN_1.PROX_INT_EN       = 0,

    .INT_EN_2.DIE_TEMP_RDY_EN   = 0,

    .FIFO_WRITE_PTR.FIFO_WR_PTR = 0,

    .OVEF_COUNTER.OVF_COUNTER   = 0,

    .FIFO_READ_PTR.FIFO_RD_PTR  = 0,

    .FIFO_CONF.SMP_AVE          = 0b010,  // Average of 4 values
    .FIFO_CONF.FIFO_ROLLOVER_EN = 1,      // FIFO rollover enable
    .FIFO_CONF.FIFO_A_FULL      = 0,      // 0

    .MODE_CONF.SHDN             = 0,
    .MODE_CONF.RESET            = 0,
    .MODE_CONF.MODE             = 0b011,  // SPO2 mode

    .SPO2_CONF.SPO2_ADC_RGE     = 0b01,   // 16384 nA (DAC scale)
    .SPO2_CONF.SPO2_SR          = 0b001,  // 200 samples per second
    .SPO2_CONF.LED_PW           = 0b10,   // LED pulse of 215 µs

    .LED1_PULSE_AMP.LED1_PA     = 0x24,   // LED1 current 25.4mA
    .LED2_PULSE_AMP.LED2_PA     = 0x24,   // LED2 current 25.4mA

    .PROX_LED_PULS_AMP.PILOT_PA = 0X7F,

    .MULTI_LED_CONTROL1.SLOT2   = 0,      // Disabled
    .MULTI_LED_CONTROL1.SLOT1   = 0,      // Disabled

    .MULTI_LED_CONTROL2.SLOT4   = 0,      // Disabled
    .MULTI_LED_CONTROL2.SLOT3   = 0,      // Disabled
};


// =============== GLOBAL DATA BUFFERS ===============
int32_t red_data = 0;
int32_t ir_data = 0;

int32_t red_data_buffer[BUFFER_SIZE];
int32_t ir_data_buffer[BUFFER_SIZE];
double auto_correlationated_data[BUFFER_SIZE];

// Sampling delay (in milliseconds)
#define SAMPLE_DELAY_MS 10

#define DEBUG true

// =============== FUNCTION DECLARATIONS ===============
void sensor_data_reader(void *pvParameters);
void fill_buffers_data(void);

// =====================================================
//                 SENSOR READER TASK
// =====================================================
void sensor_data_reader(void *pvParameters)
{
    // ---- Initialize I2C and MAX30102 ----
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    max30102_init(&max30102_configuration);
    init_time_array();

    uint64_t ir_mean = 0;
    uint64_t red_mean = 0;
    float temperature = 0.0f;
    double r0_autocorrelation = 0.0;

    for (;;)
    {
        // Fill data buffers with fresh samples from the sensor
        fill_buffers_data();

        // Read temperature (optional, may be 0.0 if not supported)
        temperature = get_max30102_temp();

        // Remove DC component (mean value)
        remove_dc_part(ir_data_buffer, red_data_buffer, &ir_mean, &red_mean);

        // Remove slow trend (baseline drift)
        remove_trend_line(ir_data_buffer);
        remove_trend_line(red_data_buffer);

        // Compute correlation between IR and RED signals
        double pearson_correlation = correlation_datay_datax(red_data_buffer, ir_data_buffer);
        if (isnan(pearson_correlation))
            pearson_correlation = 0.0;

        // Compute heart rate from IR signal
        int heart_rate_ir = calculate_heart_rate(ir_data_buffer, &r0_autocorrelation, auto_correlationated_data);

        // ---- Print data summary ----
        printf("\n");
        printf("HEART_RATE: %d bpm\n", heart_rate_ir);
        printf("Correlation: %f\n", pearson_correlation);
        printf("Temperature: %f\n", temperature);

        // If IR and RED signals are strongly correlated, compute SpO2
        if (pearson_correlation >= 0.7)
        {
            double spo2 = spo2_measurement(ir_data_buffer, red_data_buffer, ir_mean, red_mean);
            printf("SpO2: %f%%\n", spo2);
        }
        else
        {
            printf("SpO2: skipped (weak correlation)\n");
        }

#if DEBUG
        // Print debug data for analysis
        printf("\n--- DEBUG SIGNAL DATA ---\n");
        for (int i = 0; i < BUFFER_SIZE; i++)
        {
            printf("%d ", (int)ir_data_buffer[i]);
            printf("%d ", (int)red_data_buffer[i]);
            printf("%f\n", auto_correlationated_data[i]);
        }
#endif
    }
}

// =====================================================
//                  FILL BUFFER FUNCTION
// =====================================================
void fill_buffers_data()
{
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        // Read one sample from MAX30102 FIFO
        read_max30102_fifo(&red_data, &ir_data);

        // Store data into buffers
        ir_data_buffer[i] = ir_data;
        red_data_buffer[i] = red_data;

        // Small delay between samples (defines sampling rate)
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}

// =====================================================
//              START SENSOR READER TASK
// =====================================================
void max30102_task_start()
{
    xTaskCreatePinnedToCore(
        sensor_data_reader,
        "DataReader",
        10240,
        NULL,
        2,
        &sensor_reader_handle,
        1
    );
}
