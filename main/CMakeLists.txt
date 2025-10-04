# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )
set(COMPONENT_ADD_INCLUDEDIRS "")

   set(COMPONENT_SRCS "main.c" 
                      "max30102_utils/algorithm.c"
                      "mpu6050/mpu_utils.c"
                      "wifi_sta/wifi_sta.c"
                      "http_client/http_client.c"
                      "emg/emg.c"
                      "sntp/sntp.c"
                      "oled/oled.c"
                      "max30102_utils/max30102_utils.c"
                      INCLUDE_DIRS "."
                      REQUIRES mpu6050)
register_component()
