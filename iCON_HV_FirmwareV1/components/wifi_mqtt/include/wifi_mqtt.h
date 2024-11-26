#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"   // For esp_mqtt_client_handle_t and esp_mqtt_client functions
#include "esp_event.h"     // For esp_event_base_t




// WiFi configuration section 
#define DEFAULT_WIFI_SSID "msNet"
#define DEFAULT_WIFI_PASSWORD "mystic123"

//#define DEFAULT_WIFI_SSID "Renkube"
//#define DEFAULT_WIFI_PASSWORD "Himalayan6"



// Define server-related constants
//#define MQTT_BROKER_URI "mqtt://3.109.234.14:1883" // "mqtt://your_broker_address:port"

#define MQTT_BROKER_URI "mqtt://64.227.135.103:1883" // "mqtt://your_broker_address:port"

//Mqqt username and password 
#define MQTT_USERNAME "nyquest"
#define MQTT_PASSWORD "svk0XV3VLQWjy"

//Mqqt topic name 
#define MQTT_TOPIC "dd/iconhv123456"

extern bool message_received_flag; 
// Declare the OTA URL variable
extern char ota_url[256];
// Declare external flags
extern bool wifi_connected;
extern bool mqtt_connected;
extern char wifi_ssid[32];
extern char wifi_password[64];
// Task handle for monitor_connections_task
extern TaskHandle_t monitor_task_handle;

extern esp_mqtt_client_handle_t mqtt_client;  // Declare as extern


// Function to initialize WiFi and MQTT
void wifi_init_sta(void); // wifi intialise and connect
//Connect to mqqt broker
void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data); 
void mqtt_app_start(void); // initializes and starts the MQTT

void wifi_initialization(void);
void monitor_connections_task(void *pvParameters);


#endif // WIFI_MQTT_H
