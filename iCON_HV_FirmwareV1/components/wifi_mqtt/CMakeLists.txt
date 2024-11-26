idf_component_register(SRCS "wifi_mqtt.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver nvs_flash  esp-mqtt  json spiffs  esp_wifi   )

