#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_system.h" // Assosiated with restart

#include "ble_server.h" // Include the component header

//include the other components headers 
#include "wifi_mqtt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
TaskHandle_t bleTaskHandle = NULL ;

// Declaration of the task function
void checkAndEnableBLETask(void *pvParameters);

const char *TAG = "BLE-Server";
uint8_t ble_addr_type;

bool ATE_Config_flag = true ;//when switch pressed this flag becomes false to connect to ble 
bool APP1_flag = false;     // indicating client connected or not
bool Config_Mode = false;   // indication switch to configuration mode
bool config_flag = false;   // indication configuration is sucess 

char wifi_ssid[32];              // Definition
char wifi_password[64];  


// Function prototypes
void store_config_in_nvs(const char *key, const char *value);
void send_response_to_app(const char *message);
void process_config_command(const char *key, const char *value);
int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
int ble_gap_event(struct ble_gap_event *event, void *arg);


//////////////////////          NVS SECTION          //////////////////////////////

// Configuration storage to nvs 
void store_config_in_nvs(const char *key, const char *value) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_str(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing to NVS: %s", esp_err_to_name(err));
    } else {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Configuration saved: %s = %s", key, value);
    }

    nvs_close(nvs_handle);
}


// Save config_flag as a string in NVS
void save_config_flag_to_nvs() {
    if (config_flag) {
        store_config_in_nvs("config_flag", "true");
    } else {
        store_config_in_nvs("config_flag", "false");
    }
}


// Function to read config_flag from NVS
bool read_config_from_nvs(const char *key) {
    nvs_handle_t nvs_handle;
    char stored_value[6] = {0}; // To store "true" or "false"
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);

    if (err == ESP_OK) {
        size_t required_size;
        err = nvs_get_str(nvs_handle, key, NULL, &required_size);
        if (err == ESP_OK && required_size <= sizeof(stored_value)) {
            err = nvs_get_str(nvs_handle, key, stored_value, &required_size);
            if (err == ESP_OK) {
                config_flag = (strcmp(stored_value, "true") == 0);
                ESP_LOGI(TAG, "Config flag loaded: %s", stored_value);
                nvs_close(nvs_handle);
                return config_flag;
            }
        }
        nvs_close(nvs_handle);
    }
    ESP_LOGE(TAG, "Failed to load config flag from NVS");
    return false;
}

// Call this function during initialization to load the config_flag from NVS
void load_config_flag() {
    config_flag = read_config_from_nvs("config_flag");
}








/////////////////////////        BLE SECTION         //////////////////////////////

// Define the BLE connection
//This function sets up and starts the advertising process for the BLE device. Advertising is how a BLE device makes itself known to nearby BLE-capable devices.
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    
}


//This function is called to synchronize the BLE host with the controller and start advertising once the BLE stack is ready.
void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

//This function runs the BLE host stack’s main task, enabling the BLE stack to process events and handle BLE operations.
void host_task(void *param) {
    nimble_port_run(); // This function runs the BLE host stack task
    //nimble_port_freertos_deinit(); // Deinitialize after task completes
}


// BLE event handling
// For checking ble connection established.client connected or not
int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Event for connection
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI("GAP", "Client connected.");
            printf("Client connected.\n");
            APP1_flag = true;  // Set the flag to true when connected
            ESP_LOGI("APP1_flag", "ATE_Config_flag is %s", APP1_flag ? "true" : "false");
        } else {
            ESP_LOGI("GAP", "Client connection failed.");
            printf("Client connection failed.\n");
            APP1_flag = false; // Ensure flag is false if connection fails
            ESP_LOGI("APP1_flag", "ATE_Config_flag is %s", APP1_flag ? "true" : "false");
            // If connection failed, start advertising again
            ble_app_advertise();
        }
        break;
    
    // Event for disconnection
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "Client disconnected.");
        printf("Client disconnected.\n");
        APP1_flag = false;  // Set the flag to false when disconnected
        ESP_LOGI("APP1_flag", "ATE_Config_flag is %s", APP1_flag ? "true" : "false");
        // Start advertising again after disconnection
        ble_app_advertise();
        break;
    
    // Advertising complete event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "Advertising completed.");
        printf("Advertising completed.\n");
        // Restart advertising after completion
        ble_app_advertise();
        break;
    
    default:
        break;
    }
    return 0;
}

////////////////////////////////     CONFIG VIA APP SECTION      /////////////////////////////////////


void send_response_to_app(const char *message) {
    ESP_LOGI(TAG, "Response to APP: %s", message);
}

// Configuration processing and validation function
void process_config_command(const char *key, const char *value) {
    if (strcmp(key, "SSID") == 0) {
        strncpy(wifi_ssid, value, sizeof(wifi_ssid));
        store_config_in_nvs("wifi_ssid", value);
        send_response_to_app("SSID set successfully");
    } else if (strcmp(key, "PASSWORD") == 0) {
        strncpy(wifi_password, value, sizeof(wifi_password));
        store_config_in_nvs("wifi_password", value);
        send_response_to_app("Password set successfully");        
    } else if (strcmp(key, "ENTERPRISE_WIFI") == 0 || strcmp(key, "BATTERY_TYPE") == 0 ||
               strcmp(key, "MQTT_BROKER_URI") == 0 || strcmp(key, "MQTT_TOPIC") == 0 ||
               strcmp(key, "MQTT_USERNAME") == 0 || strcmp(key, "MQTT_PASSWORD") == 0){
        // Store other string configurations
        store_config_in_nvs(key, value);
        ESP_LOGI(TAG, "%s set to: %s", key, value);
        send_response_to_app("CONFIG SET SUCCESSFULLY");
    } else if (strcmp(key, "DC_OVERLOAD") == 0 || strcmp(key, "AC_OVERLOAD") == 0 ||
         strcmp(key, "BAT_AGE") == 0 || strcmp(key, "BAT_AH_CAPACITY") == 0 ||
         strcmp(key, "BAT_MAX_THR") == 0 || strcmp(key, "BAT_LOW_THR") == 0 ||
         strcmp(key, "SOL_CAPACITY") == 0 || strcmp(key, "SOL_MAX_THR") == 0 ||
         strcmp(key, "SOL_LOW_H_THR") == 0 || strcmp(key, "SOL_LOW_L_THR") == 0 ||
         strcmp(key, "FRC_TRIP_EXIT_VOLTAGE") == 0) {
        // Validate integer parameters   
        int value_int = atoi(value);
        if (value_int > 0) {
            store_config_in_nvs(key, value);
            ESP_LOGI(TAG, "%s set to: %d", key, value_int);
            send_response_to_app("CONFIG SET SUCCESSFULLY");
        } else {
            ESP_LOGE(TAG, "Invalid value for %s: %d", key, value_int);
            send_response_to_app("INVALID CONFIG VALUE");
            config_flag = false;
        }
    } else {
        ESP_LOGE(TAG, "Unknown configuration parameter: %s", key);
        send_response_to_app("Unknown configuration parameter");
        config_flag = false; // Configuration failed
    }
}



//  WRITE FUNCTION FOR  BLE GATT CHARACTERISTICS

/*uint16_t conn_handle: The handle identifying the connection.
uint16_t attr_handle: The attribute being written.
struct ble_gatt_access_ctxt *ctxt: Context containing the data being written.
void *arg: Optional argument.*/

/*int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    char received_data[100] = {0};
    snprintf(received_data, ctxt->om->om_len + 1, "%.*s", ctxt->om->om_len, ctxt->om->om_data);

    if (strcmp(received_data, "CONFIG_START") == 0) {
        Config_Mode = true;
        config_flag = false;

        ESP_LOGI(TAG, "Configuration mode started.");
        send_response_to_app("CONFIGURATION MODE STARTED");
    } else if (strcmp(received_data, "CONFIG_END") == 0) {
        config_flag = true;
        Config_Mode = false;
        ATE_Config_flag = true; //Makes true to preload configuration values from 
        // Save config_flag to NVS for persistence
        save_config_flag_to_nvs();

        ESP_LOGI(TAG, "Configuration mode ended. Configuration flag set to true.");
        send_response_to_app("CONFIGURATION ENDED");

        ESP_LOGI(TAG, " Going to RESTART TO RETRIVE LATEST CONFIGURATED DATA");
        esp_restart();

    } else if (APP1_flag && Config_Mode) {
        char *key = strtok(received_data, "=");
        char *value = strtok(NULL, "=");

        if (key && value) {
            process_config_command(key, value);
        } else {
            ESP_LOGI(TAG, "Invalid configuration data: %s", received_data);
            send_response_to_app("INVALID CONFIG DATA");
            config_flag = false;
        }
    } else {
        ESP_LOGI(TAG, "Received data (not in config mode): %s", received_data);
        send_response_to_app("NOT IN CONFIG MODE");
    }

    return 0;
}*/


int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {  
    char received_data[100] = {0};

    // Copy received data to avoid potential overflow
    snprintf(received_data, sizeof(received_data), "%.*s", ctxt->om->om_len, ctxt->om->om_data);
    
    // Log received data and flag statuses for debugging
    ESP_LOGI(TAG, "Received data: %s", received_data);
    ESP_LOGI(TAG, "Config_Mode: %s, APP1_flag: %s, config_flag: %s", 
             Config_Mode ? "true" : "false", APP1_flag ? "true" : "false", config_flag ? "true" : "false");

    // Check if the received data is "CONFIG_START"
    if (strcmp(received_data, "CONFIG_START") == 0) {
        Config_Mode = true;
        config_flag = false;

        ESP_LOGI(TAG, "Configuration mode started.");
        send_response_to_app("CONFIGURATION MODE STARTED");

    } else if (strcmp(received_data, "CONFIG_END") == 0) {
        config_flag = true;
        Config_Mode = false;
        ATE_Config_flag = true; // Makes true to preload configuration values

        // Save config_flag to NVS for persistence
        save_config_flag_to_nvs();

        ESP_LOGI(TAG, "Configuration mode ended. Configuration flag set to true.");
        send_response_to_app("CONFIGURATION ENDED");

        ESP_LOGI(TAG, "Going to RESTART TO RETRIEVE LATEST CONFIGURATED DATA");
        esp_restart();

    } else if (APP1_flag && Config_Mode) {
        char *key = strtok(received_data, "=");
        char *value = strtok(NULL, "=");

        if (key && value) {
            process_config_command(key, value);
        } else {
            ESP_LOGI(TAG, "Invalid configuration data: %s", received_data);
            send_response_to_app("INVALID CONFIG DATA");
            config_flag = false;
        }
    } else {
        ESP_LOGI(TAG, "Received data (not in config mode): %s", received_data);
        send_response_to_app("NOT IN CONFIG MODE");
    }

    return 0;
}



//Handles reading operations by sending a response back to the client.
/*uint16_t con_handle: Handle for the connection.
uint16_t attr_handle: Attribute handle.
struct ble_gatt_access_ctxt *ctxt: Context for read access, containing the buffer for the outgoing data.
void *arg: Optional argument.*/
int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    os_mbuf_append(ctxt->om, "Data from the server", strlen("Data from the server"));
    return 0;
}


/*This code defines a BLE GATT (Generic Attribute Profile) service structure in an array, gatt_svcs, which configures the characteristics and services offered by a BLE peripheral device. 
The structure is organized according to the BLE GATT profile and declares the services and characteristics that a client device (like a phone or other BLE-enabled device) can interact with.*/
const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4), .flags = BLE_GATT_CHR_F_READ, .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD), .flags = BLE_GATT_CHR_F_WRITE, .access_cb = device_write},
         {0}}}, 
    {0}
};

// Initialization of the BLE server
void ble_server_init(void) {

    

    //esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set("BLE-Server"); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
    
    // Start advertising
    ble_app_advertise();
    
    ESP_LOGI(TAG, "BLE server initialized successfully");
}



void checkAndEnableBLETask(void *pvParameters) {
    while (true) {
        if (!ATE_Config_flag) {
            // Start BLE initialization when ATE_Config_flag is false
            ble_server_init();
            ATE_Config_flag = true;

            // Log the flag's condition
            ESP_LOGI("ATE_FLAG", "BLE initialized. ATE_Config_flag is now %s", ATE_Config_flag ? "true" : "false");

            // Delete the task after initializing
            vTaskDelete(NULL);
        }
        
        // Log the flag's condition using a ternary operator
        ESP_LOGI("ATE_FLAG", "ATE_Config_flag is %s", ATE_Config_flag ? "true" : "false");
        vTaskDelay(pdMS_TO_TICKS(10000));  // Delay to avoid rapid flag checks
    }
}
