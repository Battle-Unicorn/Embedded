#include "emg.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "EMG_SENSOR";

// Global variables
static TaskHandle_t emg_task_handle = NULL;
static esp_adc_cal_characteristics_t adc_chars;
static bool adc_calibrated = false;

// Moving average buffer
static float moving_avg_buffer[MOVING_AVG_SIZE] = {0};
static int moving_avg_index = 0;

// DC offset removal
static float dc_offset = 0.0f;
static float alpha_dc = 0.01f;

// Envelope tracking
static float envelope = 0.0f;
bool atonia_flag = false;

int read_emg_sample(void) {
    int raw = adc1_get_raw(EMG_ADC_CHANNEL);
    if (raw < 0) {
        ESP_LOGE(TAG, "Error reading ADC: %d", raw);
        return 0;
    }
    return raw;
}

// Simpler DC removal - just subtract the offset
float remove_dc_offset(float input) {
    // Update DC offset slowly
    dc_offset = alpha_dc * input + (1 - alpha_dc) * dc_offset;
    return input - dc_offset;
}

// Very mild highpass to remove only the slowest drifts
float mild_highpass_filter(float input) {
    static float prev_input = 0.0f;
    static float prev_output = 0.0f;
    // 1Hz cutoff - very mild
    float alpha = 0.03f;
    
    float output = alpha * prev_output + alpha * (input - prev_input);
    prev_input = input;
    prev_output = output;
    
    return output;
}

// Bandpass filter - combination of mild highpass and lowpass
float bandpass_filter(float input) {
    static float hp_prev_input = 0.0f;
    static float hp_prev_output = 0.0f;
    static float lp_prev_output = 0.0f;
    
    // Highpass: 10Hz cutoff
    float hp_alpha = 0.1f;
    float hp_output = hp_alpha * hp_prev_output + hp_alpha * (input - hp_prev_input);
    hp_prev_input = input;
    hp_prev_output = hp_output;
    
    // Lowpass: 100Hz cutoff  
    float lp_alpha = 0.3f;
    float lp_output = lp_alpha * hp_output + (1 - lp_alpha) * lp_prev_output;
    lp_prev_output = lp_output;
    
    return lp_output;
}

// Better envelope detection
float calculate_envelope(float input) {
    float abs_input = fabsf(input);
    
    // Fast attack when signal increases, slow release when decreases
    float attack_alpha = 0.2f;  // Fast response to increases
    float release_alpha = 0.02f; // Slow decay
    
    if (abs_input > envelope) {
        envelope = attack_alpha * abs_input + (1 - attack_alpha) * envelope;
    } else {
        envelope = release_alpha * abs_input + (1 - release_alpha) * envelope;
    }
    
    return envelope;
}

float moving_average(float new_sample) {
    moving_avg_buffer[moving_avg_index] = new_sample;
    moving_avg_index = (moving_avg_index + 1) % MOVING_AVG_SIZE;
    
    float sum = 0.0f;
    for (int i = 0; i < MOVING_AVG_SIZE; i++) {
        sum += moving_avg_buffer[i];
    }
    
    return sum / MOVING_AVG_SIZE;
}

float raw_to_mv(int raw_value) {
    return raw_value * (3300.0f / 4095.0f);  
}

static void emg_task(void *pvParameters) {
    ESP_LOGI(TAG, "EMG task started");
    
    // Initialize ADC1
    adc1_config_width(EMG_ADC_WIDTH);
    adc1_config_channel_atten(EMG_ADC_CHANNEL, EMG_ADC_ATTEN);
    
    // ADC calibration
    esp_adc_cal_characterize(EMG_ADC_UNIT, EMG_ADC_ATTEN, EMG_ADC_WIDTH, 0, &adc_chars);
    adc_calibrated = true;
    
    // Calculate initial DC offset
    ESP_LOGI(TAG, "Calibrating DC offset...");
    float sum = 0.0f;
    for (int i = 0; i < 100; i++) {
        int raw = read_emg_sample();
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    dc_offset = sum / 100.0f;
    ESP_LOGI(TAG, "DC offset calibrated: %.2f", dc_offset);
    
    const TickType_t xDelay = pdMS_TO_TICKS(10); // 100Hz sampling
    
    uint32_t sample_count = 0;
    uint32_t last_log_time = xTaskGetTickCount();
    
    while (1) {
        vTaskDelay(xDelay);
        
        // Read raw value
        int raw_sample = read_emg_sample();
        
        // Process signal with less aggressive filtering
        float dc_removed = remove_dc_offset((float)raw_sample);
        float bandpassed = bandpass_filter(dc_removed);
        float filtered = moving_average(bandpassed);
        
        // Calculate envelope
        float current_envelope = calculate_envelope(filtered);
        
        // Display values every second
        sample_count++;
        if (xTaskGetTickCount() - last_log_time >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "Raw: %d, DC_removed: %.1f, Filtered: %.2f, Envelope: %.2f, mV: %.2f",
					 raw_sample, dc_removed, filtered, current_envelope, raw_to_mv(current_envelope));
            last_log_time = xTaskGetTickCount();
        }
    }
}

void emg_task_start(void) {
    // Create task
    xTaskCreate(emg_task, "emg_task", 4096, NULL, 5, &emg_task_handle);
    
    if (emg_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create EMG task");
        return;
    }
    
    ESP_LOGI(TAG, "EMG task created successfully");
}