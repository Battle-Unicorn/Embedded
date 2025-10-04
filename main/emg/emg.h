#ifndef EMG_SENSOR_H
#define EMG_SENSOR_H

#include "esp_err.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ADC configuration
#define EMG_ADC_CHANNEL      ADC_CHANNEL_6
#define EMG_ADC_UNIT         ADC_UNIT_1
#define EMG_ADC_ATTEN        ADC_ATTEN_DB_12
#define EMG_ADC_WIDTH        ADC_WIDTH_BIT_12

#define SAMPLE_RATE          50 // 50 Hz sampling rate
#define MOVING_AVG_SIZE      4  // Reduced for faster response

void emg_task_start(void);
int read_emg_sample(void);
float remove_dc_offset(float input);
float mild_highpass_filter(float input);
float bandpass_filter(float input);
float calculate_envelope(float input);
float moving_average(float new_sample);

#endif