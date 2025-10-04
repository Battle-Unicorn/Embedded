/*
 * oled.h
 *
 *  Created on: 4 paź 2025
 *      Author: majorBien
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "ssd1306.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the OLED display task
 * 
 * This function initializes the SPI interface and starts displaying
 * content on the OLED screen. First shows "Hello" message, then
 * continuously displays system time, heart rate, and steps.
 */
void oled_task_start(void);

#ifdef __cplusplus
}
#endif

#endif // OLED_DISPLAY_H