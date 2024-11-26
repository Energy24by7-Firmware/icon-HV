idf_component_register(SRCS "Relay_Control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver  esp32s2_pac1952  esp32s2_i2c_protocol  esp32s2_fatal_err  gpio_control)

