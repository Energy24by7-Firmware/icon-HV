#define MQTT_BROKER_URI "mqtt://3.109.234.14:1883" // MQTT broker URI

#include "wifi_mqtt.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include "nvs_flash.h"
#include "cJSON.h"
// Global flags for Wi-Fi and MQTT connection status
bool wifi_connected = false;
bool mqtt_connected = false;


char wifi_ssid[32];              // Definition
char wifi_password[64];  



// Task handle for the monitor task
TaskHandle_t monitor_task_handle = NULL;

// Logging tag
static const char *TAG = "WIFI_MQTT";

// MQTT client handle
esp_mqtt_client_handle_t mqtt_client = NULL;  // Declare globally for accessing other components and main.c

// Wi-Fi initialization function (station mode)
void wifi_init_sta(void) {
    // Initialize the netif and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default station network interface
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Declare variables to store SSID and password from NVS
    size_t ssid_len = sizeof(wifi_ssid);
    size_t pass_len = sizeof(wifi_password);

    // Try to read SSID and password from NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        if (nvs_get_str(nvs_handle, "wifi_ssid", wifi_ssid, &ssid_len) == ESP_OK &&
            nvs_get_str(nvs_handle, "wifi_password", wifi_password, &pass_len) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi credentials read from NVS: SSID=%s", wifi_ssid);
        } else {
            // If reading fails, use default SSID and password
            strncpy(wifi_ssid, DEFAULT_WIFI_SSID, sizeof(wifi_ssid));
            strncpy(wifi_password, DEFAULT_WIFI_PASSWORD, sizeof(wifi_password));
            ESP_LOGW(TAG, "Using default WiFi credentials: SSID=%s", wifi_ssid);
        }
        nvs_close(nvs_handle);
    } else {
        // If NVS open fails, use default values
        strncpy(wifi_ssid, DEFAULT_WIFI_SSID, sizeof(wifi_ssid));
        strncpy(wifi_password, DEFAULT_WIFI_PASSWORD, sizeof(wifi_password));
        ESP_LOGW(TAG, "Failed to open NVS, using default WiFi credentials: SSID=%s", wifi_ssid);
    }

    // Configure the WiFi connection
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };

    // Copy the retrieved or default SSID and password
    strncpy((char *)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));

    // Set WiFi mode and apply the configuration
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Connect to the specified WiFi network
    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", wifi_ssid);
    ESP_ERROR_CHECK(esp_wifi_connect());
}


bool message_received_flag = false; // Initialize the flag
char ota_url[256] = "";  // Adjust size based on expected URL length



// MQTT event handler callback
void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data; // Cast event_data to the correct type
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to MQTT broker");
            mqtt_connected = true;

            // Subscribe to the desired topic after connecting
            int msg_id = esp_mqtt_client_subscribe(event->client, "da/iconhv123456", 0);
            ESP_LOGI(TAG, "Subscribed to topic da/iconhv123456, msg_id=%d", msg_id);
        
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected from MQTT broker");
            mqtt_connected = false;
            break;

        case MQTT_EVENT_DATA:
        // Parse JSON data
        cJSON *json = cJSON_ParseWithLength(event->data, event->data_len);
        if (json == NULL) {
            ESP_LOGE(TAG, "Failed to parse JSON");
            break;
        }

        // Extract fields from JSON
        cJSON *status = cJSON_GetObjectItem(json, "status");
        cJSON *ota_array = cJSON_GetObjectItem(json, "ota");
        cJSON *product_ota_version = cJSON_GetObjectItem(json, "product_ota_version");
        cJSON *status_code = cJSON_GetObjectItem(json, "status_code");
        cJSON *packet_type = cJSON_GetObjectItem(json, "packet_type");

        // OLD SECTION CONDITION CHECK :
        if (cJSON_IsBool(status) && cJSON_IsArray(ota_array) && cJSON_IsNumber(status_code) && cJSON_IsString(packet_type)){

        // Check if JSON has all expected fields and they are of the correct types(CHAT GPT GIVEN CONDITION)
        //if (cJSON_IsBool(status) && cJSON_IsArray(ota_array) && cJSON_IsNumber(status_code) &&
        //cJSON_IsString(packet_type) && (cJSON_IsString(product_ota_version) || cJSON_IsNumber(product_ota_version))) {
        


        // Set the flag to true only if JSON is as expected
        message_received_flag = true;

        // Print the full JSON message
        char *json_string = cJSON_PrintUnformatted(json);
        if (json_string) {
            ESP_LOGI(TAG, "Received JSON Message: %s", json_string);
            free(json_string);  // Free the allocated memory for JSON string
        }
        
        // Extract and log individual fields
        bool status_value = cJSON_IsTrue(status);
        ESP_LOGI(TAG, "Status: %s", status_value ? "true" : "false");
        ESP_LOGI(TAG, "Status Code: %d", status_code->valueint);
        ESP_LOGI(TAG, "Packet Type: %s", packet_type->valuestring);

        // Handle product_ota_version if it's a string or number
        int product_ota_version_value = 0;
        if (cJSON_IsString(product_ota_version)) {
            product_ota_version_value = atoi(product_ota_version->valuestring);
        } else if (cJSON_IsNumber(product_ota_version)) {
            product_ota_version_value = product_ota_version->valueint;
        }
        ESP_LOGI(TAG, "Product OTA Version: %d", product_ota_version_value);

        // Parse OTA array and retrieve the first entry's URL
        cJSON *ota_entry = cJSON_GetArrayItem(ota_array, 0);
        if (ota_entry) {
            cJSON *url = cJSON_GetObjectItem(ota_entry, "url");
            if (cJSON_IsString(url)) {
                ESP_LOGI(TAG, "OTA URL: %s", url->valuestring);
                strncpy(ota_url, url->valuestring, sizeof(ota_url) - 1);  // Store URL globally
            }
        }
    } else {
        //ESP_LOGW(TAG, "Received JSON does not match expected format, skipping log.");
    }

    // Clean up cJSON object
    cJSON_Delete(json);
    break;
    }
}

/*
// MQTT event handler callback
void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to MQTT broker");
            mqtt_connected = true;
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected from MQTT broker");
            mqtt_connected = false;
            break;

        default:
            break;
    }
}*/



// MQTT initialization and start
/*void mqtt_app_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
            
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, NULL);
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "Connecting to MQTT broker...");
    int mqqt_connection_count = 0;
    
    // Monitor MQTT connection status
    while(!mqtt_connected) {
        ESP_LOGI(TAG, "Waiting for MQTT connection...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every 1 second
        mqqt_connection_count ++ ;
        if(mqqt_connection_count>=20)
        {
            ESP_LOGW(TAG, "MQTT connection attempt timed out after 20 seconds");
            break ;
        }
    }
}*/

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD
    };

    ESP_LOGI(TAG, "Initializing MQTT client...");
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    ESP_LOGI(TAG, "Registering MQTT event handler...");
    esp_err_t err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Starting MQTT client...");
    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Connecting to MQTT broker...");
    int mqtt_connection_count = 0;

    // Monitor MQTT connection status
    while (!mqtt_connected) {
        ESP_LOGI(TAG, "Waiting for MQTT connection...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every 1 second
        mqtt_connection_count++;
        if (mqtt_connection_count >= 20) {
            ESP_LOGW(TAG, "MQTT connection attempt timed out after 20 seconds");
            break;
        }
    }

    if (mqtt_connected) {
        ESP_LOGI(TAG, "MQTT client connected successfully.");
    } else {
        ESP_LOGE(TAG, "MQTT client failed to connect within the specified time.");
    }
}



// Function to initialize both Wi-Fi and MQTT
void wifi_initialization(void) {
    wifi_init_sta();  // Initialize Wi-Fi
    vTaskDelay(pdMS_TO_TICKS(2000));  // Add delay to ensure Wi-Fi connection is established
    mqtt_app_start();  // Initialize MQTT and connect to the broker
}

// Task to monitor Wi-Fi and MQTT connection status
void monitor_connections_task(void *pvParameters) {
    wifi_ap_record_t wifidata;

    while (1) {
        if (esp_wifi_sta_get_ap_info(&wifidata) == ESP_OK) {
            wifi_connected = true;
        } else {
            wifi_connected = false;
        }

        if (!wifi_connected) {
            ESP_LOGI(TAG, "Attempting to reconnect to Wi-Fi...");
            esp_wifi_connect(); // Attempt to reconnect
        }

        if (!mqtt_connected) {
            ESP_LOGI(TAG, "MQTT disconnected, trying to reconnect...");
            esp_mqtt_client_reconnect(mqtt_client);  // Attempt to reconnect MQTT
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
}
