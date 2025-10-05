/*
 * algorithm.c
 *
 *  Created on: 5 Oct 2025
 *      Author: majorBien
 */

#include "algorithm.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

double time_array[BUFFER_SIZE];

#define DEBUG true
#define MINIMUM_RATIO 0.3
#define EPSILON 1e-9   // Small constant to prevent division by zero

// Initializes the time array
void init_time_array()
{
    double time = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        time_array[i] = time;
        time += DELAY_AMOSTRAGEM / 1000.0;
    }
}

// Removes DC component (mean value)
void remove_dc_part(int32_t *ir_buffer, int32_t *red_buffer, uint64_t *ir_mean, uint64_t *red_mean)
{
    *ir_mean = 0;
    *red_mean = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        *ir_mean += ir_buffer[i];
        *red_mean += red_buffer[i];
    }

    *ir_mean = *ir_mean / BUFFER_SIZE;
    *red_mean = *red_mean / BUFFER_SIZE;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        red_buffer[i] -= *red_mean;
        ir_buffer[i] -= *ir_mean;
    }
}

// Removes linear trend from the signal
void remove_trend_line(int32_t *buffer)
{
    double a = 0, b = 0;
    calculate_linear_regression(&a, &b, buffer);

    double time = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer[i] = ((buffer[i] - a * time) - b);
        time += DELAY_AMOSTRAGEM / 1000.0;
    }
}

// Linear regression calculation (y = ax + b)
void calculate_linear_regression(double *a, double *b, int32_t *data)
{
    int64_t sum_of_y = sum_of_elements(data);
    double sum_of_x = 325.12; // Should be computed dynamically later
    double sum_of_x2 = somatoria_x2();
    double sum_of_xy = sum_of_xy_elements(data);
    double sum_of_x_squared = (sum_of_x * sum_of_x);

    double temp = (sum_of_xy - (sum_of_x * sum_of_y) / BUFFER_SIZE);
    double temp2 = (sum_of_x2 - (sum_of_x_squared / BUFFER_SIZE));
    if (fabs(temp2) < EPSILON) temp2 = EPSILON; // Prevent divide by zero

    *a = temp / temp2;
    *b = ((sum_of_y / BUFFER_SIZE) - (*a * (sum_of_x / BUFFER_SIZE)));
}

// Pearson correlation coefficient
double correlation_datay_datax(int32_t *data_red, int32_t *data_ir)
{
    double sum_x = 0, sum_y = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum_x += data_red[i];
        sum_y += data_ir[i];
    }
    double x_mean = sum_x / BUFFER_SIZE;
    double y_mean = sum_y / BUFFER_SIZE;

    double sum_x_diff2 = 0, sum_y_diff2 = 0, covar_xy = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        double dx = data_red[i] - x_mean;
        double dy = data_ir[i] - y_mean;
        sum_x_diff2 += dx * dx;
        sum_y_diff2 += dy * dy;
        covar_xy += dx * dy;
    }

    double sx = sqrt(sum_x_diff2 / BUFFER_SIZE);
    double sy = sqrt(sum_y_diff2 / BUFFER_SIZE);
    if (sx < EPSILON || sy < EPSILON) return 0.0; // Prevent NaN

    double corr = (covar_xy / BUFFER_SIZE) / (sx * sy);
    return corr;
}

// SpO2 estimation
double spo2_measurement(int32_t *ir_data, int32_t *red_data, uint64_t ir_mean, uint64_t red_mean)
{
    double ir_rms = rms_value(ir_data);
    double red_rms = rms_value(red_data);

    if (ir_mean == 0 || red_mean == 0 || ir_rms == 0 || red_rms == 0)
        return 0.0; // avoid divide-by-zero

    double Z = (red_rms / red_mean) / (ir_rms / ir_mean);
    printf("red_rms %f, ir_rms %f, Z %f\n", red_rms, ir_rms, Z);

    double SpO2 = (49.7 * Z);
    return SpO2;
}

// Heart rate estimation using autocorrelation
int calculate_heart_rate(int32_t *ir_data, double *r0, double *auto_corr)
{
    double r0_val = auto_correlation_function(ir_data, 0);
    if (fabs(r0_val) < EPSILON) return 0; // avoid divide-by-zero
    *r0 = r0_val;

    printf("R0 %f\n", *r0);
    double max_val = 0;
    int max_idx = 0;

    for (int lag = 0; lag < 125; lag++) {
        double r = auto_correlation_function(ir_data, lag);
        double ratio = r / r0_val;
        auto_corr[lag] = ratio;

        if (lag > 10 && ratio > MINIMUM_RATIO) {
            if (ratio > max_val) {
                max_val = ratio;
                max_idx = lag;
            }
        }
    }

    if (max_idx == 0) return 0; // avoid divide by zero
    double hr = (60.0 / (max_idx * (DELAY_AMOSTRAGEM / 1000.0)));
    return (int)hr;
}

// Autocorrelation helper
double auto_correlation_function(int32_t *data, int32_t lag)
{
    double sum = 0;
    for (int i = 0; i < (BUFFER_SIZE - lag); i++)
        sum += (double)data[i] * data[i + lag];
    return sum / BUFFER_SIZE;
}

// Utility math functions
int64_t sum_of_elements(int32_t *data)
{
    int64_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
        sum += data[i];
    return sum;
}

double sum_of_xy_elements(int32_t *data)
{
    double sum_xy = 0, time = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum_xy += data[i] * time;
        time += DELAY_AMOSTRAGEM / 1000.0;
    }
    return sum_xy;
}

double somatoria_x2()
{
    double result = 0.0, time = 0.0;
    double inc = (DELAY_AMOSTRAGEM / 1000.0);
    for (int i = 0; i < BUFFER_SIZE; i++) {
        result += time * time;
        time += inc;
    }
    return result;
}

// RMS computation (safe)
double rms_value(int32_t *data)
{
    double sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
        sum += ((double)data[i] * data[i]);
    return sqrt(sum / BUFFER_SIZE);
}
