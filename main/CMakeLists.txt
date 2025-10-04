# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )
set(COMPONENT_ADD_INCLUDEDIRS "")

   set(COMPONENT_SRCS "main.c" 
                      "max30102/i2c_api.c" 
                      "max30102/max30102_api.c" 
                      "max30102/algorithm.c"
                      "max30102/max30102_config.c"
                      "mpu6050/mpu_utils.c"
                      "wifi_sta/wifi_sta.c"
                      "http_client/http_client.c"
                      "emg/emg.c"
                      "sntp/sntp.c"
                      "oled/oled.c"
                      INCLUDE_DIRS "."
                      REQUIRES mpu6050)


register_component()
