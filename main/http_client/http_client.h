/*
 * http_client.c
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */

#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/* Server configuration */
#define SERVER_IP      "192.168.8.102"
#define SERVER_PORT    8080
#define FLAGS_ENDPOINT    "/embedded/flags"
#define EMG_ENDPOINT      "/embedded/emg"
#define PLETH_ENDPOINT    "/embedded/plethysmometer" 
#define MPU_ENDPOINT      "/embedded/mpu6050"
#define POST_INTERVAL_MS  (30000) /* 30 seconds */

/* EMG sample structure for queue */
typedef struct {
    char timestamp[25];  // ISO 8601 format
    float envelope;      // EMG envelope value
} emg_sample_t;

/* Function prototypes */
esp_err_t http_client_start(void);
void http_client_stop(void);

/* Queue functions for EMG data */
esp_err_t http_client_init_emg_queue(void);
BaseType_t http_client_send_emg_sample(const emg_sample_t *sample);

#endif /* HTTP_CLIENT_H */