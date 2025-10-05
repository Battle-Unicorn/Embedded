# Lucid Sleep Planner - Embedded System Documentation

## 🎯 Project Overview

**Lucid Sleep Planner** is an intelligent sleep optimization and lucid dreaming assistant that combines biometric sensing, AI-driven personalization, and real-time data analysis. Our embedded system runs on ESP32 and integrates multiple sensors to detect sleep phases and trigger personalized audio cues for enhanced sleep experiences.

## 🔬 Technology Stack

### Hardware Components
- **ESP32** - Main microcontroller
- **MPU6050** - Accelerometer & gyroscope for movement detection
- **MAX30102** - Heart rate and SpO2 sensor
- **EMG Sensor** - Muscle activity monitoring
- **OLED Display** - SSD1306

### Software Architecture
- **FreeRTOS** - Real-time operating system
- **ESP-IDF** - ESP32 development framework
- **cJSON** - JSON data handling
- **Custom sensor drivers** - 
  - EMG data processing - we detect muscle atonia
  - HR and SpO2 data processing (note: currently not fully accurate, values are being tweaked) - we based on this example https://github.com/Gabriel-Gardin/max30102_esp32_oximeter/tree/master
- **SNTP** - Time synchronization
- **HTTP client & WiFi** - For backend communication
- **Libraries used:**
  - [esp-idf-lib](https://github.com/UncleRus/esp-idf-lib)
  - [esp-idf-ssd1306](https://github.com/nopnop2002/esp-idf-ssd1306)

## 🚀 Quick Start Guide

### Prerequisites

1. **Hardware Setup**
   - ESP32 development board
   - MPU6050 sensor connected via I2C
   - EMG sensor connected to an analog input
   - MAX30102 sensor (optional, for MVP)
   - USB cable for programming and debugging

2. **Software Requirements**
   - ESP-IDF v4.4 or later
   - VS Code with ESP-IDF extension (recommended)
   - Python 3.8+
   - Required components: `mpu6050`, 

### Installation Steps

1. **Clone the Repository**
2. Build the Project

idf.py build
Flash and Monitor
Replace COMX with your actual port.

idf.py -p COMX flash monitor

<img width="1536" height="2048" alt="image" src="https://github.com/user-attachments/assets/00cb4c77-988a-432f-a041-04734aa6f28b" />
<img width="281" height="497" alt="image" src="https://github.com/user-attachments/assets/236d47e9-7a0e-498c-880d-dd2fb0b4aaec" />

   ```bash
   git clone <repository-url>
   cd lucid-sleep-planner/embedded
