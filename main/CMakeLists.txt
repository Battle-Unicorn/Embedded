# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

   set(COMPONENT_SRCS "main.c" 
                      "max30102/i2c_api.c" 
                      "max30102/max30102_api.c" 
                      "max30102/algorithm.c"
                      "max30102/max30102_config.c")


register_component()
