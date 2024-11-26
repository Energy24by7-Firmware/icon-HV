idf_component_register(
    SRCS "ota.c" "ota"
    INCLUDE_DIRS "include"
    EMBED_TXTFILES "server_certs/chain.pem"
    REQUIRES esp_event app_update driver esp_http_client esp_https_ota nvs_flash protocol_examples_common esp_netif esp_wifi wifi_mqtt
)


