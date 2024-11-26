# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c         # list the source files of this component
    INCLUDE_DIRS "."       # optional, add here public include directories
    PRIV_INCLUDE_DIRS "."   # optional, add here private include directories
    REQUIRES  driver mqtt esp_wifi nvs_flash json spiffs  esp_wifi  json  bt       # optional, list the public requirements (component names)
    PRIV_REQUIRES    ble_server  gpio_control  samples_read  Relay_Control  esp32s2_pac1952  esp32s2_i2c_protocol  esp32s2_fatal_err  esp-mqtt  Server_buffer wifi_mqtt switch_detector ota# optional, list the private requirements
)

