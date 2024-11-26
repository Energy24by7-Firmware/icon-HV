idf_component_register(SRCS "samples_read.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash  driver esp32s2_pac1952  esp32s2_i2c_protocol  esp32s2_fatal_err  calibration  gpio_control)

