#include "emg.h"
#include "esp_log.h"
#include "math.h"
#include "time.h"

static const char *TAG = "EMG_SENSOR";

// --- Global variables ---
static TaskHandle_t emg_task_handle = NULL;
static esp_adc_cal_characteristics_t adc_chars;
static bool adc_calibrated = false;

// EMG signal processing
static float moving_avg_buffer[MOVING_AVG_SIZE] = {0};
static int moving_avg_index = 0;
static float dc_offset = 0.0f;
static float alpha_dc = 0.01f;
static float envelope = 0.0f;

// --- Calibration state ---
static bool calibration_done = false;
static bool calibration_in_progress = false;
static float mvc_value_envelope = 0.0f;   // 100% MVC value based on envelope
bool atonia_flag = false;    // Atonia detection flag

// --- Helper functions ---

// Read one EMG ADC sample
int read_emg_sample(void) {
    int raw = adc1_get_raw(EMG_ADC_CHANNEL);
    if (raw < 0) {
        ESP_LOGE(TAG, "Error reading ADC: %d", raw);
        return 0;
    }
    return raw;
}

// Simple DC offset removal
float remove_dc_offset(float input) {
    dc_offset = alpha_dc * input + (1 - alpha_dc) * dc_offset;
    return input - dc_offset;
}

// Bandpass filter (approx. 10–100 Hz)
float bandpass_filter(float input) {
    static float hp_prev_input = 0.0f;
    static float hp_prev_output = 0.0f;
    static float lp_prev_output = 0.0f;

    // High-pass filter
    float hp_alpha = 0.1f;
    float hp_output = hp_alpha * hp_prev_output + hp_alpha * (input - hp_prev_input);
    hp_prev_input = input;
    hp_prev_output = hp_output;

    // Low-pass filter
    float lp_alpha = 0.3f;
    float lp_output = lp_alpha * hp_output + (1 - lp_alpha) * lp_prev_output;
    lp_prev_output = lp_output;

    return lp_output;
}

// Envelope extraction (fast attack, slow release)
float calculate_envelope(float input) {
    float abs_input = fabsf(input);
    float attack_alpha = 0.2f;
    float release_alpha = 0.02f;

    if (abs_input > envelope) {
        envelope = attack_alpha * abs_input + (1 - attack_alpha) * envelope;
    } else {
        envelope = release_alpha * abs_input + (1 - release_alpha) * envelope;
    }
    return envelope;
}

// Moving average for smoothing
float moving_average(float new_sample) {
    moving_avg_buffer[moving_avg_index] = new_sample;
    moving_avg_index = (moving_avg_index + 1) % MOVING_AVG_SIZE;

    float sum = 0.0f;
    for (int i = 0; i < MOVING_AVG_SIZE; i++) sum += moving_avg_buffer[i];
    return sum / MOVING_AVG_SIZE;
}

// Send EMG sample to HTTP client queue
static void send_emg_sample_to_queue(float envelope_value) {
    emg_sample_t sample;
    
    // Get current timestamp in ISO 8601 format
    time_t now;
    time(&now);
    struct tm *timeinfo = gmtime(&now);
    strftime(sample.timestamp, sizeof(sample.timestamp), "%Y-%m-%dT%H:%M:%SZ", timeinfo);
    
    sample.envelope = envelope_value;
    
    BaseType_t result = http_client_send_emg_sample(&sample);
    if (result != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send EMG sample to queue");
    }
}

// --- Simple MVC calibration routine ---
static void perform_mvc_calibration(void) {
    calibration_in_progress = true;
    ESP_LOGI(TAG, "Starting MVC calibration for %d seconds...", MVC_CALIBRATION_TIME_S);
    ESP_LOGI(TAG, "Please contract and relax your tibial muscle repeatedly");

    float max_envelope = 0.0f;
    float second_max = 0.0f;
    float third_max = 0.0f;
    
    TickType_t start_time = xTaskGetTickCount();
    TickType_t calibration_ticks = pdMS_TO_TICKS(MVC_CALIBRATION_TIME_S * 1000);
    int samples_collected = 0;

    while ((xTaskGetTickCount() - start_time) < calibration_ticks) {
        int raw = read_emg_sample();
        
        // Process signal to get envelope
        float dc_removed = remove_dc_offset((float)raw);
        float bandpassed = bandpass_filter(dc_removed);
        float filtered = moving_average(bandpassed);
        float current_envelope = calculate_envelope(filtered);

        // Simple peak tracking - find top 3 values
        if (current_envelope > max_envelope) {
            third_max = second_max;
            second_max = max_envelope;
            max_envelope = current_envelope;
        } else if (current_envelope > second_max) {
            third_max = second_max;
            second_max = current_envelope;
        } else if (current_envelope > third_max) {
            third_max = current_envelope;
        }

        samples_collected++;
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));  // 50 Hz sampling
    }

    // Use average of top 3 peaks as MVC value
    mvc_value_envelope = (max_envelope + second_max + third_max) / 3.0f;
    calibration_done = true;
    calibration_in_progress = false;

    ESP_LOGI(TAG, "Calibration complete. Samples: %d", samples_collected);
    ESP_LOGI(TAG, "MVC (100%%) = %.3f", mvc_value_envelope);
    ESP_LOGI(TAG, "Atonia threshold = %.4f (0.9%% of MVC)", mvc_value_envelope * ATONIA_THRESHOLD_PCT);
}

// --- Main EMG processing task ---
static void emg_task(void *pvParameters) {
    ESP_LOGI(TAG, "EMG task started");

    // Initialize ADC
    adc1_config_width(EMG_ADC_WIDTH);
    adc1_config_channel_atten(EMG_ADC_CHANNEL, EMG_ADC_ATTEN);
    esp_adc_cal_characterize(EMG_ADC_UNIT, EMG_ADC_ATTEN, EMG_ADC_WIDTH, 0, &adc_chars);
    adc_calibrated = true;

    // DC offset calibration
    ESP_LOGI(TAG, "Calibrating DC offset...");
    float sum = 0.0f;
    for (int i = 0; i < 100; i++) {
        int raw = read_emg_sample();
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
    dc_offset = sum / 100.0f;
    ESP_LOGI(TAG, "DC offset calibrated: %.2f", dc_offset);

    const TickType_t xDelay = pdMS_TO_TICKS(1000 / SAMPLE_RATE);  // 50 Hz sampling
    uint32_t sample_count = 0;
    uint32_t last_log_time = xTaskGetTickCount();

    while (1) {
        vTaskDelay(xDelay);
        int raw_sample = read_emg_sample();
        if (raw_sample == 0) continue;

        // Automatically start calibration
        if (!calibration_done && !calibration_in_progress) {
            perform_mvc_calibration();
        }

        // EMG signal processing
        float dc_removed = remove_dc_offset((float)raw_sample);
        float bandpassed = bandpass_filter(dc_removed);
        float filtered = moving_average(bandpassed);
        float current_envelope = calculate_envelope(filtered);

        // --- Send sample to HTTP queue every second ---
        sample_count++;
        if (xTaskGetTickCount() - last_log_time >= pdMS_TO_TICKS(1000)) {
            send_emg_sample_to_queue(current_envelope);
            
            // --- Atonia detection ---
            if (calibration_done && mvc_value_envelope > 0) {
                float atonia_threshold = mvc_value_envelope * ATONIA_THRESHOLD_PCT;
                atonia_flag = (current_envelope <= atonia_threshold);
            }

            float envelope_percent = 0.0f;
            if (calibration_done && mvc_value_envelope > 0) {
                envelope_percent = (current_envelope / mvc_value_envelope) * 100.0f;
            }

 /*           ESP_LOGI(TAG, "Envelope: %.3f (%.2f%%), Atonia: %s", 
                     current_envelope,
                     envelope_percent,
                     atonia_flag ? "YES" : "NO");*/
            
            last_log_time = xTaskGetTickCount();
            sample_count = 0;
        }
    }
}

void emg_task_start(void) {
    xTaskCreate(emg_task, "emg_task", 4096, NULL, 7, &emg_task_handle);
    if (emg_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create EMG task");
        return;
    }
    ESP_LOGI(TAG, "EMG task created successfully");
}

// Get current envelope value
float emg_get_current_envelope(void) {
    int raw_sample = read_emg_sample();
    float dc_removed = remove_dc_offset((float)raw_sample);
    float bandpassed = bandpass_filter(dc_removed);
    float filtered = moving_average(bandpassed);
    return calculate_envelope(filtered);
}

// Get muscle activity as percentage of MVC based on envelope
float emg_get_muscle_activity_percent(void) {
    if (!calibration_done || mvc_value_envelope <= 0) return 0.0f;
    
    float current_envelope = emg_get_current_envelope();
    return (current_envelope / mvc_value_envelope) * 100.0f;
}

// Get atonia status
bool emg_get_atonia_status(void) {
    return atonia_flag;
}

// Check if calibration is complete
bool emg_is_calibration_complete(void) {
    return calibration_done;
}