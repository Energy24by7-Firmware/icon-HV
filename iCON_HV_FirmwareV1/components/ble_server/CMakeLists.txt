idf_component_register(SRCS "src/ble_server.c"
                       INCLUDE_DIRS "include"
                       
		       PRIV_REQUIRES esp_event nvs_flash
			REQUIRES bt wifi_mqtt)
