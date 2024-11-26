idf_component_register(SRCS "Server_buffer.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver nvs_flash wifi_mqtt esp-mqtt  json spiffs  esp_wifi  json  Relay_Control  samples_read  esp32s2_pac1952  esp32s2_i2c_protocol  esp32s2_fatal_err)

