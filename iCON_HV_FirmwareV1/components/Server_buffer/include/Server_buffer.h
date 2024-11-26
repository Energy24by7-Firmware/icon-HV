#ifndef SERVER_BUFFER_H
#define SERVER_BUFFER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"




// Task Handles declarations

extern TaskHandle_t collectDataTaskHandle;
extern TaskHandle_t device_control_task_handle;
extern TaskHandle_t x4SecTaskHandle;
extern TaskHandle_t x1MinTaskHandle;
extern TaskHandle_t x15MinTaskHandle;
extern TaskHandle_t xStateChangeTaskHandle;
extern TaskHandle_t monitor_time_and_mqtt_taskHandle;
extern TaskHandle_t timeTaskHandle;

// Constants for buffer sizes
#define BUFFER_SIZE_200MS 20
#define BUFFER_SIZE_4S 15
#define BUFFER_SIZE_1MIN 15
#define BUFFER_SIZE_15MIN 4
#define BUFFER_SIZE_1HOUR 1

////   Sensor data structure  //////
typedef struct {
    float battery_voltage;
    float solar_voltage_adc;
    float solar_voltage_pac;
    float solar_charging_current;
    int solar_heat_sink;
    int internal_temperature;
    int SSR_temperature;
    float power_supply_voltage;
} sensor_data_t;

// Declare state_change_flag as external 
extern int state_change_flag;

// Declare the semaphore
extern SemaphoreHandle_t sensor_data_semaphore; 

// Declare the shared sensor data structure
extern sensor_data_t shared_sensor_data;


extern bool data_ready ;

// Buffers declaration
extern sensor_data_t buffer_200ms[BUFFER_SIZE_200MS];
extern sensor_data_t buffer_4s[BUFFER_SIZE_4S];
extern sensor_data_t buffer_1min[BUFFER_SIZE_1MIN];
extern sensor_data_t buffer_15min[BUFFER_SIZE_15MIN];
extern sensor_data_t buffer_1hour[BUFFER_SIZE_1HOUR];

// Buffer index declaration
extern int buffer_200ms_index;
extern int buffer_4s_index;
extern int buffer_1min_index;
extern int buffer_15min_index;
extern int buffer_1hour_index;

extern const long gmtOffset_sec; // IST is UTC +5:30
extern const int daylightOffset_sec; // IST does not observe daylight saving time



// FUNCTION DECLARATION //
 
//sensor_data_t collect_sensor_data(void); // Function to collect sensor data
sensor_data_t collect_sensor_data(float reference_voltage);//Function to collect sensor data with reference
float generate_random_float(float min, float max); // Generate float random
int generate_random_int(int min, int max); //generate int random

//sensor_data_t collect_sensor_data(float reference_voltage);  // Function to collect data

sensor_data_t calculate_average(sensor_data_t* buffer, int size);// Function to calculate average from a buffer


void wifi_init_sta(void); // wifi intialise and connect
void time_sync_notification_cb(struct timeval *tv); //Time synchronizing
void initialize_sntp(void); //initialise SNTP 
void get_compact_date_time(char* buffer); //Function to get current date and time 


//Connect to mqqt broker
void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data); 
void mqtt_app_start(void); // initializes and starts the MQTT

//Json formate and MQTT publish
bool publish_200ms_data_as_json(int index);
bool publish_4s_data_as_json(int index);
bool publish_1min_data_as_json(int index);
bool publish_15min_data_as_json(int index);
bool publish_1hour_data_as_json(int index);








/////////////     SPIFFS SECTION      ////////////
void init_spiffs();
//void save_buffers_to_spiffs() ;


// RTOS TASKS ///
void collect_data_task(void* arg); // Task for collecting data every 200ms
void device_control_task(void *pvParameter);

void avg_4s_task(void* arg);
void avg_1min_task(void* arg);
void avg_15min_task(void* arg);
void detect_state_change(void* arg);

void monitor_time_and_mqtt_task(void *pvParameters);
void time_task(void* pvParameter);


void all_initialization();



#endif // SERVER_BUFFER_H
