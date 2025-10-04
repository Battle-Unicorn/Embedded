/*
 * sntp.h
 *
 *  Created on: 4 paź 2025
 *      Author: lenovo
 */

#ifndef SNTP_TIME_H
#define SNTP_TIME_H

#include <stdbool.h>

/**
 * @brief Initialize SNTP with Polish timezone
 */
void sntp_time_init(void);

/**
 * @brief Get current timestamp in ISO 8601 format
 * @return Formatted time string
 */
char* sntp_get_time_string(void);

/**
 * @brief Check if time is synchronized
 * @return true if time is synced
 */
bool sntp_time_is_synced(void);

/**
 * @brief Wait for time synchronization
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if synchronized, false if timeout
 */
bool sntp_wait_for_sync(int timeout_ms);

#endif