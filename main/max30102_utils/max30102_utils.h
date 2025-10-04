/*
 * max30102_utils.h
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */
#ifndef MAX30102_UTILS_H
#define MAX30102_UTILS_H

#include <stdint.h>

#define BUFFER_SIZE 100

// Public functions
void max_task_start(void);
int get_heart_rate(void);
float get_temperature(void);
double get_spo2(void);

// Algorithm functions (you need to implement these or link with existing algorithm library)
void init_time_array(void);
void remove_dc_part(int32_t *ir_buffer, int32_t *red_buffer, uint64_t *ir_mean, uint64_t *red_mean);
void remove_trend_line(int32_t *buffer);
double correlation_datay_datax(int32_t *buffer_x, int32_t *buffer_y);
int calculate_heart_rate(int32_t *ir_buffer, double *r0_autocorrelation, double *auto_correlationated_data);
double spo2_measurement(int32_t *ir_buffer, int32_t *red_buffer, uint64_t ir_mean, uint64_t red_mean);

#endif