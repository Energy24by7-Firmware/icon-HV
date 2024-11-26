#include <stdio.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include <stdbool.h>
#include "esp_mac.h"



#include "Server_buffer.h"

#define TAG "DATA_MANAGEMENT"
#define BASE_PATH "/spiffs"

//  Access other components   //
#include "esp32s2_pac1952.h"
#include "esp32s2_i2c_protocol.h"
#include "esp32s2_fatal_err.h"

#include "samples_read.h"
#include "wifi_mqtt.h"
#include "Relay_Control.h"


// Initialize the semaphore and shared data
SemaphoreHandle_t sensor_data_semaphore = NULL;
sensor_data_t shared_sensor_data = {0};

/// Task Handles //

TaskHandle_t collectDataTaskHandle = NULL;
TaskHandle_t device_control_task_handle;
TaskHandle_t x4SecTaskHandle = NULL;
TaskHandle_t x1MinTaskHandle = NULL;
TaskHandle_t x15MinTaskHandle = NULL;
TaskHandle_t xStateChangeTaskHandle = NULL; 
TaskHandle_t monitor_time_and_mqtt_taskHandle = NULL;
TaskHandle_t timeTaskHandle = NULL;



int state_change_flag = 0;

#define BUFFER_SIZE_200MS 20
#define BUFFER_SIZE_4S 15
#define BUFFER_SIZE_1MIN 15
#define BUFFER_SIZE_15MIN 4
#define BUFFER_SIZE_1HOUR 1  // Size of the 1-hour buffer
// Buffers
sensor_data_t buffer_200ms[BUFFER_SIZE_200MS];
sensor_data_t buffer_4s[BUFFER_SIZE_4S];
sensor_data_t buffer_1min[BUFFER_SIZE_1MIN];
sensor_data_t buffer_15min[BUFFER_SIZE_15MIN];
sensor_data_t buffer_1hour[BUFFER_SIZE_1HOUR];  // 1-hour buffer

// Time section constants
const long gmtOffset_sec = 19800; // IST is UTC +5:30
const int daylightOffset_sec = 0; // IST does not observe daylight saving time


/// declaration of variables //
int current_index = 0; // Initialize appropriately based on your logic
int buffer_200ms_index = 0;
int buffer_4s_index = 0;
int buffer_1min_index = 0;
int buffer_15min_index = 0;
int buffer_1hour_index = 0;



//////////////////////////////////    FUNCTION SECTION     //////////////////////////////////



///////////  MAC ADDRESS SECTION /////////


uint8_t mac[6];
char mac_str[18] = {0};
size_t mac_str_len = sizeof(mac_str);


void save_mac_to_nvs(const char* mac_str) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        // Write MAC address to NVS
        err = nvs_set_str(my_handle, "mac_addr", mac_str);
        if (err == ESP_OK) {
            // Commit written value to NVS
            nvs_commit(my_handle);
            ESP_LOGI(TAG, "MAC address saved to NVS");
        } else {
            ESP_LOGE(TAG, "Failed to save MAC address to NVS: %s", esp_err_to_name(err));
        }

        // Close the NVS handle
        nvs_close(my_handle);
    }
}

esp_err_t read_mac_from_nvs(char* mac_str, size_t* mac_str_len) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    // Read MAC address from NVS
    err = nvs_get_str(my_handle, "mac_addr", mac_str, mac_str_len);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MAC address read from NVS: %s", mac_str);
    } else {
        ESP_LOGE(TAG, "Failed to read MAC address from NVS: %s", esp_err_to_name(err));
    }

    // Close the NVS handle
    nvs_close(my_handle);
    return err;
}









///////////   TIME SECTION   //////////

void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Got time adjustment from NTP!");
}



void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    //setenv("TZ", "IST-5:30", 1); // Time zone for IST
    setenv("TZ", "UTC", 1); // Use UTC time zone
    //setenv("TZ", "IST", 1);

    tzset();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

// Function to get the current date and time in the format YYYYMMDDHHMM
void get_date_time(char* buffer, size_t size) {
    if (size < 13) {
        // Ensure the buffer is large enough
        return;
    }

    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);

    // Format the date-time string as "YYYYMMDDHHMM"
    snprintf(buffer, size, "%04d%02d%02d%02d%02d",
             tm_info->tm_year + 1900,  // Year (4 digits)
             tm_info->tm_mon + 1,      // Month (1-12, 2 digits)
             tm_info->tm_mday,         // Day (1-31, 2 digits)
             tm_info->tm_hour,         // Hour (0-23, 2 digits)
             tm_info->tm_min);         // Minute (0-59, 2 digits)
}
// Function to get the current date and time in a 6-byte format
void get_compact_date_time(char* buffer) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);

    buffer[0] = (char)(tm_info->tm_year - 100);  // Year since 2000
    buffer[1] = (char)(tm_info->tm_mon + 1);     // Month (1-12)
    buffer[2] = (char)tm_info->tm_mday;          // Day (1-31)
    buffer[3] = (char)tm_info->tm_hour;          // Hour (0-23)
    buffer[4] = (char)tm_info->tm_min;           // Minute (0-59)
    buffer[5] = (char)tm_info->tm_sec;           // Second (0-59)
}

// Function to convert UTC time to IST (UTC + 5:30)
void convert_utc_to_ist(struct tm* timeinfo) {
    // Adjust the time by adding 5 hours and 30 minutes to UTC
    timeinfo->tm_hour += 5;
    timeinfo->tm_min += 30;

    // Handle overflow for minutes and hours
    if (timeinfo->tm_min >= 60) {
        timeinfo->tm_min -= 60;
        timeinfo->tm_hour += 1;
    }

    if (timeinfo->tm_hour >= 24) {
        timeinfo->tm_hour -= 24;
        timeinfo->tm_mday += 1; // Adjust the day (just a simple logic for now)
    }
}

// Function to get and print the time in IST format
void print_ist_time() {
    time_t now = time(NULL);
    struct tm timeinfo = { 0 };

    // Convert to local time (UTC initially)
    localtime_r(&now, &timeinfo);

    // Convert UTC time to IST
    convert_utc_to_ist(&timeinfo);

    // Print the time in IST
    ESP_LOGI("IST Time", "Current IST Time: %04d/%02d/%02d %02d:%02d:%02d",
             timeinfo.tm_year + 1900,  // Year (4 digits)
             timeinfo.tm_mon + 1,      // Month (1-12, 2 digits)
             timeinfo.tm_mday,         // Day (1-31, 2 digits)
             timeinfo.tm_hour,         // Hour (0-23, 2 digits)
             timeinfo.tm_min,          // Minute (0-59, 2 digits)
             timeinfo.tm_sec);         // Second (0-59, 2 digits)
}








/////// SENSOR READING SAVE TO STRUCT SECTION  ////////////////

// Function to collect sensor data
sensor_data_t collect_sensor_data(float reference_voltage) 
{
    //sensor_data_t data;
    // Initialize all fields of the struct to 0 or default values
    sensor_data_t data = {
        .battery_voltage = 0.0f,
        .solar_voltage_adc = 0.0f,
        .solar_voltage_pac = 0.0f,
        .solar_charging_current = 0.0f,
        .solar_heat_sink = 0,
        .internal_temperature = 0,
        .SSR_temperature = 0,
        .power_supply_voltage = 0.0f
    };

    //Read battery voltage adc
    float Battery_adc = get_battery_voltage(reference_voltage);
    ESP_LOGI(TAG, "Battery Voltage adc: %.2f V", Battery_adc);

     //Read solar voltage adc
    data.solar_voltage_adc = get_solar_voltage(reference_voltage);
    ESP_LOGI(TAG, "Solar Voltage adc: %.2f V", data.solar_voltage_adc);

    // Read power supply voltage  from adc
    data.power_supply_voltage = get_power_supply_voltage(reference_voltage);
    ESP_LOGI(TAG,"power_supply_voltage: %.2f V",  data.power_supply_voltage);

    // Read temp1 from adc
    data.solar_heat_sink = get_solar_heat_sink_temperature(reference_voltage);
    //ESP_LOGI(TAG, "solar_heat_sink: %d °C", data.solar_heat_sink);
    ESP_LOGI(TAG, "SSR_temperature(Temp1): %d °C", data.solar_heat_sink);

    // Read temp2 from adc
    data.internal_temperature = get_internal_temperature(reference_voltage);
    ESP_LOGI(TAG, "internal_temperature(Temp2): %d °C", data.internal_temperature);

    // Read temp3 from adc
    data.SSR_temperature = get_ssr_temperature(reference_voltage);
    ESP_LOGI(TAG, "solar_heat_sink(Temp3): %d °C", data.SSR_temperature);

    

    // Call the read_adc_all function to get all ADC data into the buffer
    if (read_adc_all(adc_all_buff) != 0) {
        ESP_LOGE(TAG, "Error Reading PAC1952 Sensor Data");
        return data; // Return initialized struct with default values
    }

    // Read battery voltage from pac
    data.battery_voltage = avrg_bat_volt;
    //ESP_LOGI(TAG, "Battery Voltage pac: %.2f V", data.battery_voltage);


    // Read solar voltage pac
    data.solar_voltage_pac = avrg_sol_volt;
    //ESP_LOGI(TAG,"Solar_Voltage pac: %.2f V", data.solar_voltage_pac);


    // Read solar charging current pac
    data.solar_charging_current = avrg_sol_amp;
    //ESP_LOGI(TAG,"Solar_current pac: %.2f A",  data.solar_charging_current);

    

    return data;
}




// Function to calculate the average of a buffer
sensor_data_t calculate_average(sensor_data_t* buffer, int size) {
    sensor_data_t avg = {0};

    for (int i = 0; i < size; i++) {
        avg.battery_voltage += buffer[i].battery_voltage;
        avg.solar_voltage_adc += buffer[i].solar_voltage_adc;
        avg.solar_voltage_pac += buffer[i].solar_voltage_pac;
        avg.solar_charging_current += buffer[i].solar_charging_current;
        avg.solar_heat_sink += buffer[i].solar_heat_sink;
        avg.internal_temperature += buffer[i].internal_temperature;
        avg.SSR_temperature += buffer[i].SSR_temperature;
        avg.power_supply_voltage += buffer[i].power_supply_voltage;
    }

    // Divide by the number of samples to get the average
    avg.battery_voltage /= size;
    avg.solar_voltage_adc /= size;
    avg.solar_voltage_pac /= size;
    avg.solar_charging_current /= size;
    avg.solar_heat_sink /= size;
    avg.internal_temperature /= size;
    avg.SSR_temperature /= size;
    avg.power_supply_voltage /= size;

    return avg;
}









///////  FAILED BUFFER INITIALIZATION SECTION   ////////
#define BUFFER_SIZE 1024 // Define buffer size for failed messages
// Extended struct to include buffer name and timestamp
typedef struct {
    char buffer_name[20];        // Name of the buffer (e.g., "200ms", "4s", etc.)
    char timestamp[14];          // Timestamp in string format (e.g., "20241008083000")
    sensor_data_t sensor_data;   // Original sensor data
} failed_sensor_data_t;

// Global buffer to hold failed sensor data with metadata
failed_sensor_data_t failed_publish_buffer[BUFFER_SIZE];

// Global index for the buffer
static int buffer_index_faileddata = 0;


static bool is_failed_buffer_full = false;
static bool spiffs_data_saved = false; // Flag to indicate data saved to SPIFFS




///////  SPIFFS SECTION   ////////

#define MAX_LINES 100
#define MAX_FILENAME_LENGTH 30
#define MAX_FILES 10
#define BASE_FILENAME "data_"
nvs_handle_t my_handle; 

static char current_file_name[MAX_FILENAME_LENGTH]; // To store the current file name


// Function to initialize SPIFFS 
void init_spiffs() { 
    esp_vfs_spiffs_conf_t conf = { 
        .base_path = "/spiffs", 
        .partition_label = NULL, 
        .max_files = 5, 
        .format_if_mount_failed = true 
    }; 
 
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf)); 
 
    size_t total = 0, used = 0; 
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used); 
    if (ret != ESP_OK) { 
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information"); 
    } else { 
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used); 
    } 
} 

// Function to get the current line count in the specified file
//This function opens a file, reads it line by line, and counts the total number of lines in the file. It returns the total line count, or -1 if there is an error (such as if the file cannot be opened).
int get_line_count(const char *file_name) {
    FILE *file = fopen(file_name, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading line count");
        return -1; // Error reading the file
    }

    int line_count = 0;
    char buffer[BUFFER_SIZE];
    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        line_count++;
    }

    fclose(file);
    return line_count;
}

void save_to_nvs(const char *key, int32_t value) {
    //nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_i32(my_handle, key, value);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    } else {
        ESP_LOGE(TAG, "Error opening NVS to save key %s", key);
    }
}

void read_from_nvs(const char *key, int32_t *value) {
    //nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_get_i32(my_handle, key, value);
        nvs_close(my_handle);
    } else {
        ESP_LOGE(TAG, "Error opening NVS to read key %s", key);
    }
}

//Check datas are saved in the file ; call by directly passing the argument : read_file_from_spiffs(0);
void read_file_from_spiffs(int read_num) {
    char filename[30];
    snprintf(filename, sizeof(filename), "/spiffs/data_%d.txt", read_num);

    FILE *f = fopen(filename, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", filename);
        return;
    }

    char line[128];
    ESP_LOGI(TAG, "Reading file: %s", filename);
    int line_count = 0; // Keep track of lines read
    while (fgets(line, sizeof(line), f) != NULL) {
        ESP_LOGI(TAG, "%s", line);
        line_count++;
    }

    fclose(f);
    ESP_LOGI(TAG, "File closed: %s. Total lines read: %d", filename, line_count);
}
void copy_buffer_to_spiffs() {
    if (buffer_index_faileddata == 0) {
        ESP_LOGI(TAG, "No data to write to SPIFFS");
        return; // No data to write
    }

    char file_name[MAX_FILENAME_LENGTH];
    static int last_file_num = 1;

    // Read the last file number from NVS
    read_from_nvs("last_Wfile_num", (int32_t *)&last_file_num);

    // Get the current line count for the file
    snprintf(file_name, sizeof(file_name), "/spiffs/data_%d.txt", last_file_num);
    int line_count = get_line_count(file_name);
    // If line_count is -1, the file couldn't be opened, so start fresh with line_count = 0
    if (line_count == -1) {
        line_count = 0;
    }

    FILE *f = fopen(file_name, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    // Write data to the file
    for (int i = 0; i < buffer_index_faileddata; i++) {
        line_count++;

        // If line count reaches MAX_LINES, switch to the next file
        if (line_count > MAX_LINES) {
            fclose(f); // Close the current file
            last_file_num++;
            
            // If the file number exceeds the maximum, reset it to 1
            if (last_file_num > MAX_FILES) {
                last_file_num = 1;
            }

            // Update file name for the new file
            snprintf(file_name, sizeof(file_name), "/spiffs/data_%d.txt", last_file_num);
            
            // Open the new file for writing
            f = fopen(file_name, "w"); // Start fresh for the new file
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open new file %s for writing", file_name);
                return;
            }

            line_count = 1; // Reset line count for the new file, as this is the first line
        }
        fprintf(f, "%s, %s, %.2f, %.2f, %.2f, %.2f, %d, %d, %d, %.2f\n",
                failed_publish_buffer[i].buffer_name,
                failed_publish_buffer[i].timestamp,
                failed_publish_buffer[i].sensor_data.battery_voltage,
                failed_publish_buffer[i].sensor_data.solar_voltage_adc,
                failed_publish_buffer[i].sensor_data.solar_voltage_pac,
                failed_publish_buffer[i].sensor_data.solar_charging_current,
                failed_publish_buffer[i].sensor_data.solar_heat_sink,
                failed_publish_buffer[i].sensor_data.internal_temperature,
                failed_publish_buffer[i].sensor_data.SSR_temperature,
                failed_publish_buffer[i].sensor_data.power_supply_voltage);
    }

    fclose(f);
    ESP_LOGI(TAG, "Copied %d entries from buffer to %s", buffer_index_faileddata, file_name);

    // Save the last file number to NVS
    save_to_nvs("last_Wfile_num", last_file_num);
    buffer_index_faileddata = 0;
}

int find_next_file_with_data(int start_file_num) {
    int file_num = start_file_num;
    int attempts = 0; // Add attempts counter to avoid looping forever

    for (int i = 0; i < MAX_FILES; i++) {
        char file_name[MAX_FILENAME_LENGTH];
        snprintf(file_name, sizeof(file_name), "/spiffs/data_%d.txt", file_num);

        FILE *f = fopen(file_name, "r");
        if (f != NULL) {
            char line[256];
            if (fgets(line, sizeof(line), f) != NULL) {
                fclose(f);
                ESP_LOGI("SPIFFS", "Found data in file: %s", file_name);
                return file_num; // Return the file number
            }
            fclose(f); // Close if empty
        }

        // Move to the next file and avoid wrapping around indefinitely
        file_num = (file_num + 1) % MAX_FILES;
        attempts++;
        if (attempts >= MAX_FILES) {
            break; // Prevent looping over the same files indefinitely
        }
    }
    return -1; // No files with data found
}

 

static bool sent_full_data = false;
void read_data_from_spiff_and_publish() {
   
    //bool sent_full_data = false; // Reset to false each time function is called
    int sent_line = 0; 
    int current_sent_file_num = 0;

    // Check if all data has already been sent
    if (!sent_full_data) {
        read_from_nvs("last_sent_file_num", (int32_t *)&current_sent_file_num);
        read_from_nvs("sent_line_num", (int32_t *)&sent_line);
        ESP_LOGI("DEBUG", "Starting from file: %d, line: %d", current_sent_file_num, sent_line);
        
        while (true) 
        {
            // Find the next file that has data starting from current_sent_file_num
            int next_file_with_data = find_next_file_with_data(current_sent_file_num);
            
            if (next_file_with_data == -1) {
                ESP_LOGI("SPIFFS", "No more files with data found");
                sent_full_data = true;
                return;
            }

            current_sent_file_num = next_file_with_data;
            ESP_LOGI("SPIFFS", "Resuming from file: %d, line: %d", current_sent_file_num, sent_line);

            char file_name[MAX_FILENAME_LENGTH];
            snprintf(file_name, sizeof(file_name), "/spiffs/data_%d.txt", current_sent_file_num);

            FILE *f = fopen(file_name, "r");
            if (f == NULL) {
                ESP_LOGE("SPIFFS", "Failed to open file %s for reading", file_name);
                // Move to the next file if the current one cannot be opened
                current_sent_file_num = (current_sent_file_num + 1) % MAX_FILES;
                continue;
            }

            char line[256];  
            ESP_LOGI("SPIFFS", "Processing file %s", file_name);
            ESP_LOGI("SPIFFS", "Resuming from line number: %d", sent_line);

            // Properly skip lines that have already been processed
            int line_skip_count = 0;
            while (line_skip_count < sent_line && fgets(line, sizeof(line), f) != NULL) {
                line_skip_count++;
            }

            if (line_skip_count != sent_line) {
                ESP_LOGI("SPIFFS", "Reached end of file %s before sending all lines", file_name);
                fclose(f); 
                sent_line = 0;
                current_sent_file_num = (current_sent_file_num + 1) % MAX_FILES;
                save_to_nvs("last_sent_file_num", current_sent_file_num);
                save_to_nvs("sent_line_num", sent_line);
                continue;
            }

            int lines_read = sent_line;
            while (fgets(line, sizeof(line), f) != NULL && lines_read < MAX_LINES) {
                ESP_LOGI("DEBUG", "Reading line %d from file %s: %s", lines_read, file_name, line);

                // Create JSON object and publish to MQTT
                cJSON *root = cJSON_CreateObject();
                if (!root) {
                    ESP_LOGE("JSON", "Failed to create JSON object");
                    break;
                }

                // Parsing logic
                char *token = strtok(line, ",");
                if (token) cJSON_AddStringToObject(root, "lt", token);
                token = strtok(NULL, ",");
                if (token) cJSON_AddStringToObject(root, "dt", token);
                token = strtok(NULL, ",");
                if (token) cJSON_AddStringToObject(root, "bv", token);
                token = strtok(NULL, ",");
                if (token) cJSON_AddStringToObject(root, "sv_adc", token);
                token = strtok(NULL, ",");
                if (token) cJSON_AddNumberToObject(root, "sv_pac", atoi(token));
                token = strtok(NULL, ",");
                if (token) cJSON_AddNumberToObject(root, "scc", atoi(token));
                token = strtok(NULL, ",");
                if (token) cJSON_AddNumberToObject(root, "t1", atoi(token));
                token = strtok(NULL, ",");
                if (token) cJSON_AddStringToObject(root, "t2", token);
                token = strtok(NULL, ",");
                if (token) cJSON_AddStringToObject(root, "t3", token);
                token = strtok(NULL, ",");
                if (token) cJSON_AddStringToObject(root, "pv", token);

                            
                cJSON_AddStringToObject(root, "scr", "1");
                cJSON_AddStringToObject(root, "er", "1");
                cJSON_AddStringToObject(root, "rf", "0");
                cJSON_AddStringToObject(root, "mac", mac_str);
                //cJSON_AddStringToObject(root, "did", "iconhv123456");
                cJSON_AddStringToObject(root, "did", "iconhv789");
                cJSON_AddStringToObject(root, "fr", "v1.2.3a");
                cJSON_AddStringToObject(root, "fb", "additional_data_here");

                char *payload = cJSON_PrintUnformatted(root);
                if (!payload) {
                    ESP_LOGE("JSON", "Failed to create JSON string");
                    cJSON_Delete(root);
                    break;
                }

                const char* mqtt_topic = MQTT_TOPIC; 
                esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 1, 0);
                ESP_LOGI("MQTT", "Published SPIFFS data to %s: %s", mqtt_topic, payload);

                cJSON_Delete(root);
                free(payload);

                lines_read++;
                sent_line++;
                save_to_nvs("sent_line_num", sent_line);
            }

            fclose(f);
            save_to_nvs("last_sent_file_num", current_sent_file_num);

            // If we have processed all lines in the file, move to the next file
            if (lines_read >= MAX_LINES) {
                // Clear the file
                if (remove(file_name) == 0) {
                    ESP_LOGI("SPIFFS", "File %s cleared successfully", file_name);
                } else {
                    ESP_LOGE("SPIFFS", "Failed to clear file %s", file_name);
                }
                sent_line = 0; 
                current_sent_file_num = (current_sent_file_num + 1) % MAX_FILES;
                save_to_nvs("last_sent_file_num", current_sent_file_num);
                save_to_nvs("sent_line_num", sent_line);
                ESP_LOGI("SPIFFS", "Reached end of file %s, moving to next file", file_name);
            } else {
                save_to_nvs("last_sent_file_num", current_sent_file_num);
                save_to_nvs("sent_line_num", sent_line);
                sent_full_data = true;
                ESP_LOGI("SPIFFS", "All files processed and published.");
               

                break;
            }
        }
    }
}








////////////////////////       FAILED BUFFER FUNCTION SECTION       /////////////////////////////////

// Function to save sensor data along with metadata (buffer name and timestamp) to the failed buffer
bool save_failed_buffer_to_buffer(const sensor_data_t *data, const char *buffer_name, int *buffer_index) {
    // Check if buffer is nearing full capacity and offload if necessary
    if (*buffer_index >= (BUFFER_SIZE - 24)) { 
        ESP_LOGW(TAG, "Buffer nearing full capacity. Initiating SPIFFS offload.");
        
        // Copy buffer to SPIFFS before continuing
        copy_buffer_to_spiffs();
        spiffs_data_saved = true;
    }
    if (*buffer_index < BUFFER_SIZE) {
        // Copy the sensor data
        failed_publish_buffer[*buffer_index].sensor_data = *data;
        
        // Copy the buffer name
        strncpy(failed_publish_buffer[*buffer_index].buffer_name, buffer_name, sizeof(failed_publish_buffer[*buffer_index].buffer_name) - 1);
        failed_publish_buffer[*buffer_index].buffer_name[sizeof(failed_publish_buffer[*buffer_index].buffer_name) - 1] = '\0'; // Ensure null-termination
        
        // Get and copy the current timestamp
        get_date_time(failed_publish_buffer[*buffer_index].timestamp, sizeof(failed_publish_buffer[*buffer_index].timestamp));

        ESP_LOGI(TAG, "Saved failed buffer data from %s at index %d with timestamp %s", 
                failed_publish_buffer[*buffer_index].buffer_name, *buffer_index, failed_publish_buffer[*buffer_index].timestamp);

        (*buffer_index)++; // Move to the next position in the buffer
        return true;
    } else{//else if (*buffer_index >= (BUFFER_SIZE)) { 
        ESP_LOGE(TAG, "Failed buffer is full, cannot save more failed data");
        is_failed_buffer_full = true; // Set the flag when the buffer is full
        return false; // Buffer is full
    }
    return false;
}


void print_failed_publish_buffer() {
    ESP_LOGI(TAG, "Contents of the failed_publish_buffer:");

    // Iterate through the buffer and print its contents
    for (int i = 0; i < buffer_index_faileddata; i++) {
        failed_sensor_data_t *entry = &failed_publish_buffer[i];

        ESP_LOGI(TAG, "Entry %d:", i);
        ESP_LOGI(TAG, "  Buffer Name: %s", entry->buffer_name);
        ESP_LOGI(TAG, "  Timestamp: %s", entry->timestamp);
        ESP_LOGI(TAG, "  Battery Voltage: %.2f", entry->sensor_data.battery_voltage);
        ESP_LOGI(TAG, "  Solar Voltage ADC: %.2f", entry->sensor_data.solar_voltage_adc);
        ESP_LOGI(TAG, "  Solar Voltage PAC: %.2f", entry->sensor_data.solar_voltage_pac);
        ESP_LOGI(TAG, "  Solar Charging Current: %.2f", entry->sensor_data.solar_charging_current);
        ESP_LOGI(TAG, "  Solar Heat Sink Temperature: %d", entry->sensor_data.solar_heat_sink);
        ESP_LOGI(TAG, "  Internal Temperature: %d", entry->sensor_data.internal_temperature);
        ESP_LOGI(TAG, "  SSR Temperature: %d", entry->sensor_data.SSR_temperature);
        ESP_LOGI(TAG, "  Power Supply Voltage: %.2f", entry->sensor_data.power_supply_voltage);
        // Add additional fields here as needed
    }
}

bool has_failed_publish_data() {
    // Check if there are any entries in the failed publish buffer
    return (buffer_index_faileddata > 0);
}


void publish_failed_data_to_mqtt() {
    for (int i = 0; i < buffer_index_faileddata; i++) {
        failed_sensor_data_t *entry = &failed_publish_buffer[i];

        // Create a cJSON object for the payload
        cJSON *root = cJSON_CreateObject();

        // Add data to the JSON object
        cJSON_AddStringToObject(root, "lt", entry->buffer_name);          // Buffer name
        //cJSON_AddStringToObject(root, "did", "iconhv123456");
        cJSON_AddStringToObject(root, "did", "iconhv789");
        cJSON_AddStringToObject(root, "scr", "1");                        // State change reason (example)
        cJSON_AddStringToObject(root, "er", "1");                         // Error flag
        cJSON_AddStringToObject(root, "rf", "0");                         // Reset flag
        cJSON_AddStringToObject(root, "mac", mac_str);
        cJSON_AddStringToObject(root, "frr", "v1.2.3a");
        cJSON_AddStringToObject(root, "fb", "additional_data_here");

        // Add buffer values (format floating-point values to 2 decimal places)
        char formatted_value[10];

        // Format and add battery voltage
        snprintf(formatted_value, sizeof(formatted_value), "%.2f", entry->sensor_data.battery_voltage);
        cJSON_AddStringToObject(root, "bv", formatted_value);

        // Format and add solar voltage
        snprintf(formatted_value, sizeof(formatted_value), "%.2f", entry->sensor_data.solar_voltage_adc);
        cJSON_AddStringToObject(root, "sv_adc", formatted_value);

        snprintf(formatted_value, sizeof(formatted_value), "%.2f", entry->sensor_data.solar_voltage_pac);
        cJSON_AddStringToObject(root, "sv_pac", formatted_value);

        // Format and add solar charging current
        snprintf(formatted_value, sizeof(formatted_value), "%.2f", entry->sensor_data.solar_charging_current);
        cJSON_AddStringToObject(root, "scc", formatted_value);

        // Add temperatures
        cJSON_AddNumberToObject(root, "t1", entry->sensor_data.solar_heat_sink);
        cJSON_AddNumberToObject(root, "t2", entry->sensor_data.internal_temperature);
        cJSON_AddNumberToObject(root, "t3", entry->sensor_data.SSR_temperature);

        // Format and add power supply voltage
        snprintf(formatted_value, sizeof(formatted_value), "%.2f", entry->sensor_data.power_supply_voltage);
        cJSON_AddStringToObject(root, "pv", formatted_value);

        // Add timestamp to the JSON object
        cJSON_AddStringToObject(root, "dt", entry->timestamp);

        // Convert the cJSON object to a string
        char *payload = cJSON_PrintUnformatted(root);

        // Determine the correct topic based on the buffer name
        const char* mqtt_topic = NULL;
        if (strcmp(entry->buffer_name, "200ms") == 0) {
            mqtt_topic = MQTT_TOPIC;
        } else if (strcmp(entry->buffer_name, "4s") == 0) {
            mqtt_topic = MQTT_TOPIC;
        } else if (strcmp(entry->buffer_name, "1min") == 0) {
            mqtt_topic = MQTT_TOPIC;
        } else {
            ESP_LOGW(TAG, "Unknown buffer name: %s, skipping publish.", entry->buffer_name);
            cJSON_Delete(root);
            continue; // Skip this entry if the buffer name is unknown
        }

        // Publish the payload to the determined MQTT topic
        esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 1, 0);
        ESP_LOGI(TAG, "Published failed data to %s: %s", mqtt_topic, payload);

        // Free the cJSON object and payload string
        cJSON_Delete(root);
        free(payload);
    }

    // Reset the failed buffer index after publishing
    buffer_index_faileddata = 0;
}







//////////////////    json_FORMATE && MQTT Publish     //////////////


bool publish_200ms_data_as_json(int index) //200 ms JSON FORMATE & PUBLISH TO MQQT SERVER
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return false; // Indicate failure to create JSON
    }
    
    // Add the data to the JSON object
    cJSON_AddStringToObject(root, "lt", "scl_200ms");
    // Add timestamp to the JSON object as a string
    char timestamp_str[14]; 
    get_date_time(timestamp_str ,sizeof(timestamp_str));
    cJSON_AddStringToObject(root, "dt", timestamp_str);
    // Add buffer values
    // Buffer to hold formatted 4-digit float values (e.g., "13.18")
    char formatted_value[10];

    // Format the battery voltage to 2 decimal places and add to JSON
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_200ms[index].battery_voltage);
    cJSON_AddStringToObject(root, "bv", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_200ms[index].solar_voltage_adc);
    cJSON_AddStringToObject(root, "sv_adc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_200ms[index].solar_voltage_pac);
    cJSON_AddStringToObject(root, "sv_pac", formatted_value);
    snprintf(formatted_value, sizeof (formatted_value), "%.2f", buffer_200ms[index].solar_charging_current);
    cJSON_AddStringToObject(root, "scc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_200ms[index].power_supply_voltage);
    cJSON_AddStringToObject(root, "pv", formatted_value);
    cJSON_AddStringToObject(root, "as", "1");
    cJSON_AddNumberToObject(root, "t1", buffer_200ms[index].solar_heat_sink);
    cJSON_AddNumberToObject(root, "t2", buffer_200ms[index].internal_temperature);
    cJSON_AddNumberToObject(root, "t3", buffer_200ms[index].SSR_temperature);
    cJSON_AddStringToObject(root, "scr", "1");
    cJSON_AddStringToObject(root, "er", "1");
    cJSON_AddStringToObject(root, "rf", "0");
    cJSON_AddStringToObject(root, "mac", mac_str);
    //cJSON_AddStringToObject(root, "did", "iconhv123456");
    cJSON_AddStringToObject(root, "did", "iconhv789");
    cJSON_AddStringToObject(root, "fr", "v1.2.3a");
    cJSON_AddStringToObject(root, "fb", "additional_data_here");
    


    // Convert to JSON string
    char *json_data = cJSON_PrintUnformatted(root);
    if (json_data == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON data");
        cJSON_Delete(root);
        return false; // Indicate failure to print JSON data
    }

    
    // Check if MQTT is connected before publishing
    if (mqtt_connected) {

        // Publish JSON data to MQTT broker
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_data, 0, 1, 0);
        if (msg_id >= 0) {
            ESP_LOGI(TAG, "200ms data published to MQTT, msg_id=%d", msg_id);
            
            // Clean up and return true
            cJSON_Delete(root);
            free(json_data);
            return true; // Indicate success
        } else {
            ESP_LOGE(TAG, "Failed to publish 200ms_data to MQTT");

            // Call the buffer-saving function if publish failed
            //The indices are passed by reference (&buffer_index) so the function can update the index value outside the function scope.
           
            // Save failed buffer data
            if (!save_failed_buffer_to_buffer(&buffer_200ms[index], "200ms", &buffer_index_faileddata)) {
                 ESP_LOGE(TAG, "Failed to save 200ms data to failed buffer");
            }

        }
    } else {
        ESP_LOGE(TAG, "MQTT client is not connected, cannot publish 200ms_data");

        // Save the data to the buffer if not connected
        if (!save_failed_buffer_to_buffer(&buffer_200ms[index], "200ms", &buffer_index_faileddata)) {
               ESP_LOGE(TAG, "Failed to save 200ms data to failed buffer");
        }

    }

    // Clean up and return false for failure cases
    cJSON_Delete(root);
    free(json_data);
    return false; // Indicate failure to publish
}
   



bool publish_4s_data_as_json(int index) //4 SECOND JSON FORMATE & PUBLISH TO MQQT SERVER
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return false;
    }

    // Add the data to the JSON object
    cJSON_AddStringToObject(root, "lt", "scl_4s");
    // Add timestamp to the JSON object as a string
    char timestamp_str[14]; 
    get_date_time(timestamp_str ,sizeof(timestamp_str));  
    cJSON_AddStringToObject(root, "dt", timestamp_str);
   

    // Buffer to hold formatted 4-digit float values (e.g., "13.18")
    char formatted_value[10];

    // Format the battery voltage to 2 decimal places and add to JSON
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_4s[index].battery_voltage);
    cJSON_AddStringToObject(root, "bv", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_4s[index].solar_voltage_adc);
    cJSON_AddStringToObject(root, "sv_adc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_4s[index].solar_voltage_pac);
    cJSON_AddStringToObject(root, "sv_pac", formatted_value);
    snprintf(formatted_value, sizeof (formatted_value), "%.2f", buffer_4s[index].solar_charging_current);
    cJSON_AddStringToObject(root, "scc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_4s[index].power_supply_voltage);
    cJSON_AddStringToObject(root, "pv", formatted_value);
    cJSON_AddStringToObject(root, "as", "1");
    cJSON_AddNumberToObject(root, "t1", buffer_4s[index].solar_heat_sink);
    cJSON_AddNumberToObject(root, "t2", buffer_4s[index].internal_temperature);
    cJSON_AddNumberToObject(root, "t3", buffer_4s[index].SSR_temperature);
    
    cJSON_AddStringToObject(root, "scr", "1");
    cJSON_AddStringToObject(root, "er", "1");
    cJSON_AddStringToObject(root, "rf", "0");
    cJSON_AddStringToObject(root, "mac", mac_str);
    //cJSON_AddStringToObject(root, "did", "iconhv123456");
    cJSON_AddStringToObject(root, "did", "iconhv789");
    cJSON_AddStringToObject(root, "fr", "v1.2.3a");
    cJSON_AddStringToObject(root, "fb", "additional_data_here");
    

    char *json_data = cJSON_PrintUnformatted(root);
    if (json_data == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON data");
        cJSON_Delete(root);
        return false;
    }

    if (mqtt_connected) {

        // Publish JSON data to MQTT broker
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_data, 0, 1, 0);
        if (msg_id >= 0) {
            ESP_LOGI(TAG, "4-sec Data published to MQTT, msg_id=%d", msg_id);
            
            // Clean up and return true
            cJSON_Delete(root);
            free(json_data);
            return true; // Indicate success
        } else {
            ESP_LOGE(TAG, "Failed to publish 4s_data to MQTT");

            // Call the buffer-saving function if publish failed
           if (!save_failed_buffer_to_buffer(&buffer_4s[index], "4s", &buffer_index_faileddata)) {
                ESP_LOGE(TAG, "Failed to save 4s data to failed buffer");
            }

        }
    } else {
        ESP_LOGE(TAG, "MQTT client is not connected, cannot publish data");

        // Save the data to the buffer if not connected
        if (!save_failed_buffer_to_buffer(&buffer_4s[index], "4s", &buffer_index_faileddata)) {
           ESP_LOGE(TAG, "Failed to save 4s data to failed buffer");
        }

    }

    // Clean up and return false for failure cases
    cJSON_Delete(root);
    free(json_data);
    return false; // Indicate failure to publish
}


bool publish_1min_data_as_json(int index) //1 MINUTE JSON FORMATE & PUBLISH TO MQQT SERVER
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return false; // Indicate failure to create JSON
    }

    // Add the data to the JSON object
    cJSON_AddStringToObject(root, "lt", "scl_1min");
    // Add timestamp to the JSON object as a string
    char timestamp_str[14]; 
    get_date_time(timestamp_str ,sizeof(timestamp_str));
    
    cJSON_AddStringToObject(root, "dt", timestamp_str);


    // Add buffer values
   // Buffer to hold formatted 4-digit float values (e.g., "13.18")
    char formatted_value[10];

    // Format the battery voltage to 2 decimal places and add to JSON
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1min[index].battery_voltage);
    cJSON_AddStringToObject(root, "bv", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1min[index].solar_voltage_adc);
    cJSON_AddStringToObject(root, "sv_adc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1min[index].solar_voltage_pac);
    cJSON_AddStringToObject(root, "sv_pac", formatted_value);
    snprintf(formatted_value, sizeof (formatted_value), "%.2f", buffer_1min[index].solar_charging_current);
    cJSON_AddStringToObject(root, "scc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1min[index].power_supply_voltage);
    cJSON_AddStringToObject(root, "pv", formatted_value);
    cJSON_AddStringToObject(root, "as", "1");
    cJSON_AddNumberToObject(root, "t1", buffer_1min[index].solar_heat_sink);
    cJSON_AddNumberToObject(root, "t2", buffer_1min[index].internal_temperature);
    cJSON_AddNumberToObject(root, "t3", buffer_1min[index].SSR_temperature);
  
    cJSON_AddStringToObject(root, "scr", "1");
    cJSON_AddStringToObject(root, "er", "1");
    cJSON_AddStringToObject(root, "rf", "0");
    cJSON_AddStringToObject(root, "mac", mac_str);
    //cJSON_AddStringToObject(root, "did", "iconhv123456");
    cJSON_AddStringToObject(root, "did", "iconhv789");
    cJSON_AddStringToObject(root, "fr", "v1.2.3a");
    cJSON_AddStringToObject(root, "fb", "additional_data_here");
    

    
    // Convert to JSON string
    char *json_data = cJSON_PrintUnformatted(root);
    if (json_data == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON data");
        cJSON_Delete(root);
        return false; // Indicate failure to print JSON data
    }

    if (mqtt_connected) {

        // Publish JSON data to MQTT broker
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_data, 0, 1, 0);
        if (msg_id >= 0) {
           ESP_LOGI(TAG, "1-minute Data published to MQTT, msg_id=%d", msg_id);
		   
            // Clean up and return true
            cJSON_Delete(root);
            free(json_data);
            return true; // Indicate success
        } else {
            ESP_LOGE(TAG, "Failed to publish 1m_data to MQTT");

            // Call the buffer-saving function if publish failed
            if (!save_failed_buffer_to_buffer(&buffer_1min[index], "1min", &buffer_index_faileddata)) {
                ESP_LOGE(TAG, "Failed to save 1min data to failed buffer");
            }

        }
    } else {
        ESP_LOGE(TAG, "MQTT client is not connected, cannot publish 1m_data");

        // Save the data to the buffer if not connected
        if (!save_failed_buffer_to_buffer(&buffer_1min[index], "1min", &buffer_index_faileddata)) {
           ESP_LOGE(TAG, "Failed to save 1min data to failed buffer");
        }

    }

    // Clean up and return false for failure cases
    cJSON_Delete(root);
    free(json_data);
    return false; // Indicate failure to publish
}

// Function to publish 15-minute average data as JSON
bool publish_15min_data_as_json(int index) {
    if (index >= BUFFER_SIZE_15MIN) {
        ESP_LOGE(TAG, "Index out of bounds for 15-minute buffer");
        return false; // Indicate failure
    }

    // Retrieve the data from the buffer using the index
    //sensor_data_t data = buffer_15min[index];
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return false; // Indicate failure to create JSON
    }

    // Add the data to the JSON object
    cJSON_AddStringToObject(root, "lt", "15_mins");
    // Add timestamp to the JSON object as a string
    char timestamp_str[14]; 
    get_date_time(timestamp_str ,sizeof(timestamp_str));
    
    cJSON_AddStringToObject(root, "dt", timestamp_str);
    
    // Buffer to hold formatted 4-digit float values (e.g., "13.18")
    char formatted_value[10];

    // Format the battery voltage to 2 decimal places and add to JSON
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_15min[index].battery_voltage);
    cJSON_AddStringToObject(root, "bv", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_15min[index].solar_voltage_adc);
    cJSON_AddStringToObject(root, "sv_adc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_15min[index].solar_voltage_pac);
    cJSON_AddStringToObject(root, "sv_pac", formatted_value);
    snprintf(formatted_value, sizeof (formatted_value), "%.2f", buffer_15min[index].solar_charging_current);
    cJSON_AddStringToObject(root, "scc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_15min[index].power_supply_voltage);
    cJSON_AddStringToObject(root, "pv", formatted_value);
    cJSON_AddStringToObject(root, "as", "1");
    cJSON_AddNumberToObject(root, "t1", buffer_15min[index].solar_heat_sink);
    cJSON_AddNumberToObject(root, "t2", buffer_15min[index].internal_temperature);
    cJSON_AddNumberToObject(root, "t3", buffer_15min[index].SSR_temperature);  
    cJSON_AddStringToObject(root, "scr", "1");
    cJSON_AddStringToObject(root, "er", "1");
    cJSON_AddStringToObject(root, "rf", "0");
    cJSON_AddStringToObject(root, "mac", mac_str);
    //cJSON_AddStringToObject(root, "did", "iconhv123456");
    cJSON_AddStringToObject(root, "did", "iconhv789");
    cJSON_AddStringToObject(root, "fr", "v1.2.3a");
    cJSON_AddStringToObject(root, "fb", "additional_data_here");
    
    

    
    // Convert to JSON string
    char *json_data = cJSON_PrintUnformatted(root);
    if (json_data == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON data");
        cJSON_Delete(root);
        return false; // Indicate failure to print JSON data
    }

     if (mqtt_connected) {

        // Publish JSON data to MQTT broker
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_data, 0, 1, 0);
        if (msg_id >= 0) {
           ESP_LOGI(TAG, "15-minute data published to MQTT, msg_id=%d", msg_id);
		   
            // Clean up and return true
            cJSON_Delete(root);
            free(json_data);
            return true; // Indicate success
        } else {
            ESP_LOGE(TAG, "Failed to publish 15-minute data to MQTT");

            // Call the buffer-saving function if publish failed
            if (!save_failed_buffer_to_buffer(&buffer_15min[index], "15min", &buffer_index_faileddata)) {
                ESP_LOGE(TAG, "Failed to save 15min data to failed buffer");
            }

        }
    } else {
        ESP_LOGE(TAG, "MQTT client is not connected, cannot publish 15-minute data");

        // Save the data to the buffer if not connected
        if (!save_failed_buffer_to_buffer(&buffer_15min[index], "15min", &buffer_index_faileddata)) {
            ESP_LOGE(TAG, "Failed to save 15min data to failed buffer");
        }

    }

    // Clean up and return false for failure cases
    cJSON_Delete(root);
    free(json_data);
    return false; // Indicate failure to publish

}



// Function to publish 1-hour data as JSON
// Function to publish 1-hour average data as JSON
bool publish_1hour_data_as_json( int index) {
    // Create a JSON object
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return false; // Indicate failure to create JSON
    }

    // Add the data to the JSON object
    cJSON_AddStringToObject(root, "lt", "1_hour");
    // Add timestamp to the JSON object as a string
    char timestamp_str[14]; 
    get_date_time(timestamp_str ,sizeof(timestamp_str));
    
    cJSON_AddStringToObject(root, "dt", timestamp_str);
    
    // Buffer to hold formatted 4-digit float values (e.g., "13.18")
    char formatted_value[10];

    // Format the battery voltage to 2 decimal places and add to JSON
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1hour[index].battery_voltage);
    cJSON_AddStringToObject(root, "bv", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1hour[index].solar_voltage_adc);
    cJSON_AddStringToObject(root, "sv_adc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1hour[index].solar_voltage_pac);
    cJSON_AddStringToObject(root, "sv_pac", formatted_value);
    snprintf(formatted_value, sizeof (formatted_value), "%.2f", buffer_1hour[index].solar_charging_current);
    cJSON_AddStringToObject(root, "scc", formatted_value);
    snprintf(formatted_value, sizeof(formatted_value), "%.2f", buffer_1hour[index].power_supply_voltage);
    cJSON_AddStringToObject(root, "pv", formatted_value);
    cJSON_AddNumberToObject(root, "t1", buffer_1hour[index].solar_heat_sink);
    cJSON_AddNumberToObject(root, "t2", buffer_1hour[index].internal_temperature);
    cJSON_AddNumberToObject(root, "t3", buffer_1hour[index].SSR_temperature);
    cJSON_AddStringToObject(root, "as", "1");
    cJSON_AddStringToObject(root, "se", "50.4");
    cJSON_AddStringToObject(root, "ns", "360");
    cJSON_AddStringToObject(root, "scr", "1");
    cJSON_AddStringToObject(root, "er", "1");
    cJSON_AddStringToObject(root, "rf", "0");
    cJSON_AddStringToObject(root, "us", "5.67");
    cJSON_AddStringToObject(root, "mac", mac_str);
    //cJSON_AddStringToObject(root, "did", "iconhv123456");
    cJSON_AddStringToObject(root, "did", "iconhv789");
    cJSON_AddStringToObject(root, "fr", "v1.2.3a");
    cJSON_AddStringToObject(root, "fb", "additional_data_here");

    // Convert to JSON string
    char *json_data = cJSON_PrintUnformatted(root);
    if (json_data == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON data");
        cJSON_Delete(root);
        return false; // Indicate failure to print JSON data
    }

    
    if (mqtt_connected) {

        // Publish JSON data to MQTT broker
         int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_data, 0, 1, 0);
        if (msg_id >= 0) {
           ESP_LOGI(TAG, "1 Hr data published to MQTT, msg_id=%d", msg_id);
		   
            // Clean up and return true
            cJSON_Delete(root);
            free(json_data);
            return true; // Indicate success
        } else {
            ESP_LOGE(TAG, "Failed to publish 1 hr data to MQTT");

            // Call the buffer-saving function if publish failed
            if (!save_failed_buffer_to_buffer(&buffer_1hour[index], "1hour", &buffer_index_faileddata)) {
               ESP_LOGE(TAG, "Failed to save 1hour data to failed buffer");
            }

        }
    } else {
        ESP_LOGE(TAG, "MQTT client is not connected, cannot publish 1 hr data");

        // Save the data to the buffer if not connected
        if (!save_failed_buffer_to_buffer(&buffer_1hour[index], "1hour", &buffer_index_faileddata)) {
            ESP_LOGE(TAG, "Failed to save 1hour data to failed buffer");
        }

    }

    // Clean up and return false for failure cases
    cJSON_Delete(root);
    free(json_data);
    return false; // Indicate failure to publish

}










///////////////////////////       RTOS  TASKS   //////////////////////////////////



bool data_ready = false;  // Global flag to indicate if new data is ready
// Task for collecting data every 200ms and shared data as semaphore
void collect_data_task(void* arg) {

    printf("collect_data_task called\n");
    static float reference_voltage = 0;
    static bool is_reference_voltage_set = false;

    if (!is_reference_voltage_set) {
            reference_voltage = get_reference_voltage();
            save_reference_voltage_to_eeprom(reference_voltage);
            is_reference_voltage_set = true;
        } 
        
        reference_voltage = load_reference_voltage_from_eeprom();
        ESP_LOGI(TAG, "Reference Voltage: %.2f V", reference_voltage);
        

    while (1) {
        
        // Collect the sensor data  
        sensor_data_t sensor_data = collect_sensor_data(reference_voltage);
        

        // Buffer logic (if necessary)
        buffer_200ms[buffer_200ms_index] = sensor_data;
        buffer_200ms_index++;

        // Take the semaphore before writing to shared data
        if (xSemaphoreTake(sensor_data_semaphore, pdMS_TO_TICKS(300)) == pdTRUE) {
            shared_sensor_data = sensor_data;  // Copy the sensor data to shared memory
            data_ready = true;  // Indicate new data is ready
            xSemaphoreGive(sensor_data_semaphore);  // Release the semaphore
        } else {
            ESP_LOGE("collect_data_task", "Failed to take semaphore");
        }

        if (buffer_200ms_index >= BUFFER_SIZE_200MS) {
            buffer_200ms_index = 0;
            xTaskNotifyGive(x4SecTaskHandle);  // Notify 4-second averaging task
        }

        vTaskDelay(pdMS_TO_TICKS(200));  // Delay for 200ms
    }
}

// Task to control the device based on the received sensor data(with semaphore)
void device_control_task(void *pvParameter) {
    static float bat_voltage_saved = 0;
    static float sol_voltage_saved = 0;
    static float sol_current_saved = 0;
    static float ssr_temp_saved = 0;

    while (1) {

        
        // Attempt to take the semaphore to access the shared sensor data
        if (data_ready && xSemaphoreTake(sensor_data_semaphore, pdMS_TO_TICKS(300)) == pdTRUE) {
            /*// Access shared sensor data
            bat_voltage_saved = shared_sensor_data.battery_voltage;
            sol_voltage_saved = shared_sensor_data.solar_voltage_pac;
            sol_current_saved = shared_sensor_data.solar_charging_current;
            //ssr_temp_saved = shared_sensor_data.SSR_temperature; ////Read tem3
            //ssr_temp_saved = shared_sensor_data.internal_temperature ; //Read tem2
            ssr_temp_saved = shared_sensor_data.solar_heat_sink; //Read tem1

             // Log the received sensor data            
            ESP_LOGI(TAG, "Received data from semaphore: Battery Voltage: %.2f V, Solar Voltage: %.2f V, Solar Current: %.2f A, Temp 1: %.2f C", 
                bat_voltage_saved, sol_voltage_saved, sol_current_saved, ssr_temp_saved);


            data_ready = false;  // Reset flag after reading

            // Release the semaphore
            xSemaphoreGive(sensor_data_semaphore);*/

            if (data_ready) 
            {  // Check if new data is available
                bat_voltage_saved = shared_sensor_data.battery_voltage;
                sol_voltage_saved = shared_sensor_data.solar_voltage_pac;
                sol_current_saved = shared_sensor_data.solar_charging_current;
                ssr_temp_saved = shared_sensor_data.solar_heat_sink;

                data_ready = false;  // Reset the flag after reading
                ESP_LOGI(TAG, "Received data from semaphore: Battery Voltage: %.2f V, Solar Voltage: %.2f V, Solar Current: %.2f A, Temp 1: %.2f C", 
                bat_voltage_saved, sol_voltage_saved, sol_current_saved, ssr_temp_saved);

            }
            xSemaphoreGive(sensor_data_semaphore);

           
        } else {
            ESP_LOGW("device_control_task", "Failed to take semaphore. No data received.");
        }

        

            // Update the device state based on the received data
            bool success = update_device_state(bat_voltage_saved, sol_voltage_saved, sol_current_saved ,ssr_temp_saved);
            if (success) {
                ESP_LOGI(TAG,"STATE CHANGE OCCURRED.\n");
                state_change_flag = 1 ;

            } else {
                ESP_LOGI(TAG,"NO STATE CHANGE.\n");
                state_change_flag = 0 ;
            }

        

        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}





// Task for averaging 4-second buffer
void avg_4s_task(void* arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification
        buffer_4s[buffer_4s_index] = calculate_average(buffer_200ms, BUFFER_SIZE_200MS);
        buffer_4s_index++;

        if (buffer_4s_index >= BUFFER_SIZE_4S) {
            printf("4Second Average Taken!\n");
            buffer_4s_index = 0;
            xTaskNotifyGive(x1MinTaskHandle);  // Notify 1-minute averaging task
        }
    }
}


// Task for averaging 1-minute buffer
void avg_1min_task(void* arg) {
    while (1) {
        // Wait for notification that the 4s buffer is full
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Calculate the average of the 4s buffer and store it in the 1min buffer
        buffer_1min[buffer_1min_index] = calculate_average(buffer_4s, BUFFER_SIZE_4S);

        // Print the calculated average for debugging
        printf("1min Average calculated:\n");
        printf("  Battery Voltage: %.2fV\n", buffer_1min[buffer_1min_index].battery_voltage);
        printf("  Solar Voltage_adc: %.2fV\n", buffer_1min[buffer_1min_index].solar_voltage_adc);
        printf("  Solar Voltage_pac: %.2fV\n", buffer_1min[buffer_1min_index].solar_voltage_pac);
        printf("  Solar Charging Current: %.2fA\n", buffer_1min[buffer_1min_index].solar_charging_current);
        printf("  Solar Heat Sink: %d°C\n", buffer_1min[buffer_1min_index].solar_heat_sink);
        printf("  Internal Temperature: %d°C\n", buffer_1min[buffer_1min_index].internal_temperature);
        printf("  SSR Temperature: %d°C\n", buffer_1min[buffer_1min_index].SSR_temperature);
        printf("  Power Supply Voltage: %.2fV\n", buffer_1min[buffer_1min_index].power_supply_voltage);

        buffer_1min_index++;

        // If the 1min buffer is full, notify the 15-minute averaging task
        if (buffer_1min_index >= BUFFER_SIZE_1MIN) {
            buffer_1min_index = 0;
            printf("1min buffer full. Notifying 15-minute task.\n");
            xTaskNotifyGive(x15MinTaskHandle);
        }
    }
}


// Task for averaging 15-minute buffer
void avg_15min_task(void* arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification

        // Calculate the average of the 15-minute buffer
        sensor_data_t avg_15min = calculate_average(buffer_1min, BUFFER_SIZE_1MIN);

        // Print the average of the 15-minute buffer for debugging
        printf("15min Average calculated:\n");
        printf("  Battery Voltage: %.2fV\n", avg_15min.battery_voltage);
        printf("  Solar Voltage_adc: %.2fV\n", avg_15min.solar_voltage_adc);
        printf("  Solar Voltage_pac: %.2fV\n", avg_15min.solar_voltage_pac);
        printf("  Solar Charging Current: %.2fA\n", avg_15min.solar_charging_current);
        printf("  Solar Heat Sink: %d°C\n", avg_15min.solar_heat_sink);
        printf("  Internal Temperature: %d°C\n", avg_15min.internal_temperature);
        printf("  SSR Temperature: %d°C\n", avg_15min.SSR_temperature);
        printf("  Power Supply Voltage: %.2fV\n", avg_15min.power_supply_voltage);
        // Store the calculated average in the 15-minute buffer
        buffer_15min[buffer_15min_index] = avg_15min;
        // Publish the 15-minute average data
        bool publish_success = publish_15min_data_as_json(buffer_15min_index);  // Pass the index
        if (!publish_success) {
            ESP_LOGE(TAG, "Failed to publish 15-minute data");
            // Implement fallback logic if needed
        }
        buffer_15min_index++;

        // If the 15-minute buffer is full, save the average to the 1-hour buffer
        if (buffer_15min_index >= BUFFER_SIZE_15MIN) {
            buffer_15min_index = 0;
            printf("15min buffer full. Saving average to 1-hour buffer.\n");

            // Save the average to the 1-hour buffer
            buffer_1hour[buffer_1hour_index] = avg_15min;
            buffer_1hour_index++;

            // If the 1-hour buffer is full, handle it (e.g., save to flash or transmit)
            if (buffer_1hour_index >= BUFFER_SIZE_1HOUR) {
                buffer_1hour_index = 0;
                printf("1-hour buffer full, publish to mqtt.\n");
                // Implement code to handle the full 1-hour buffer here
                // Publish the 1-hour data
                //bool publish_success = publish_1hour_data_as_json(buffer_1hour[0]); // Assuming one slot
                //bool publish_success = publish_1hour_data_as_json(buffer_1hour[0],buffer_1hour_index); // Assuming one slot
                bool publish_success = publish_1hour_data_as_json(0); // Correct: passing the index 0
                if (!publish_success) {
                    ESP_LOGE(TAG, "Failed to publish 1-hour data");
                    // Implement fallback logic if needed
                }
            }
        }
   }
}


// Task for state change detection
/*This task monitors the state_change_flag and performs actions (publishing data and saving to SPIFFS) when a state change is detected.*/
void detect_state_change(void* arg) {
    bool state_change_occurs = false;

    while (1) {
        if (state_change_flag ==1) { 
            vTaskDelay(pdMS_TO_TICKS(200)); // Debounce delay
            if (!state_change_occurs) { 

                // Set the flag to prevent multiple saves
                state_change_occurs = true;
                // Print statement for state change
                ESP_LOGI("STATE_CHANGE", "State change occurred");

                vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds

                // Publish all 200ms data
                bool all_publish_success = true;
                for (int i = 0; i < BUFFER_SIZE_200MS; i++) {
                    if (!publish_200ms_data_as_json(i)) {
                        all_publish_success = false;
                    }
                }

                // Publish all 4-second data
                for (int i = 0; i < BUFFER_SIZE_4S; i++) {
                    if (!publish_4s_data_as_json(i)) {
                        all_publish_success = false;
                    }
                }

                // Publish all 1-minute data
                for (int i = 0; i < BUFFER_SIZE_1MIN; i++) {
                    if (!publish_1min_data_as_json(i)) {
                        all_publish_success = false;
                    }
                }

                
            }
        }

        // Check if the button is released to reset the flag
        if (state_change_flag == 0) {
            state_change_occurs = false; // Reset the flag
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Polling interval
    }
}




void monitor_time_and_mqtt_task(void *pvParameters) { 
    char current_time[6];  // Holds compact time 
    bool has_checked_failed_data = false; // Flag to avoid multiple checks in a day 
    
 
    while (true) { 

        // Get the current time in a compact format 
        get_compact_date_time(current_time); 
 
        
        // Check for failed data 
        if (has_failed_publish_data()) { 
            // If MQTT is connected, publish the failed data 
            if (mqtt_connected) { 
                ESP_LOGI("TIME", "MQTT is connected, attempting to resend failed publish data..."); 
                publish_failed_data_to_mqtt(); 
                 
                // Mark as processed to avoid repeated attempts 
                has_checked_failed_data = true; 
            }  
            // If MQTT is not connected by a specific time (e.g., 00:00), save data to SPIFFS 
            else if (!has_checked_failed_data && (int)current_time[3] == 0 && (int)current_time[4] == 0) {

                // Print the current time in a human-readable format 
                ESP_LOGI("TIME", "Current time: %02d-%02d-%02d %02d:%02d:%02d",  
                 (int)current_time[0] + 2000,  // Year 
                 (int)current_time[1],         // Month 
                 (int)current_time[2],         // Day 
                 (int)current_time[3],         // Hour 
                 (int)current_time[4],         // Minute 
                 (int)current_time[5]);        // Second 


                ESP_LOGW("TIME", "MQTT IS NOT CONNECTED AT MIDNIGHT.... Saving data to SPIFFS..."); 
                copy_buffer_to_spiffs();
                
                ESP_LOGI("spiff","buffers are copied to spiff");
                spiffs_data_saved = true; // Mark data as saved to SPIFFS
                // Mark as processed to avoid repeated saves 
                has_checked_failed_data = true; 
            } 
            else if (is_failed_buffer_full){
                ESP_LOGW("buffer", "FAILED BUFFER IS FULL.... Saving data to SPIFFS...");
                copy_buffer_to_spiffs();
                
                ESP_LOGI("spiff"," failed buffers are copied to spiff");
                spiffs_data_saved = true; // Mark data as saved to SPIFFS
                is_failed_buffer_full = false; }
        } 
 
        // If MQTT becomes connected after data is saved to SPIFFS, attempt to publish from SPIFFS 
        if (mqtt_connected && spiffs_data_saved) { 
            ESP_LOGI("TIME", "MQTT is now connected, attempting to publish data from SPIFFS");
            read_data_from_spiff_and_publish(); 
            if(sent_full_data)
            {
                spiffs_data_saved = false; // Reset flag after publishing from SPIFFS
                sent_full_data = false;
            }
            
        } 
 
        // Reset the flag at midnight (00:00) for the next day's process 
        if (((int)current_time[3] == 0 && (int)current_time[4] == 0) || ((int)current_time[3] == 0 && (int)current_time[4] < 2)) 
        {  
           if(has_checked_failed_data)
           {
                has_checked_failed_data = false; 
           }
            
        }
 
        // Delay 10 seconds between checks 
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
    } 
} 



// FreeRTOS task to print time every 10 seconds
void time_task(void* pvParameter) {
    while (1) {
        print_ist_time();   // Print IST time
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Wait for 5 seconds
        printf("\n");
    }
}





void all_initialization()
{
    init_spiffs();
    
   
    vTaskDelay(pdMS_TO_TICKS(1000));
   

    // Initialize SNTP
    initialize_sntp();



    // Try to read the MAC address from NVS
    esp_err_t err = read_mac_from_nvs(mac_str, &mac_str_len);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // If MAC address is not found in NVS, read from the device and save it to NVS
        if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            ESP_LOGI(TAG, "MAC address read from device: %s", mac_str);

            // Save MAC address to NVS
            save_mac_to_nvs(mac_str);
        } else {
            ESP_LOGE(TAG, "Failed to read MAC address from device");
        }
    } else if (err == ESP_OK) {
        // MAC address was read from NVS, print it
        ESP_LOGI(TAG, "MAC address from NVS: %s", mac_str);
    } else {
        ESP_LOGE(TAG, "An error occurred: %s", esp_err_to_name(err));
    }

   
    // Create a binary semaphore or mutex semaphore
    sensor_data_semaphore = xSemaphoreCreateMutex();
    if (sensor_data_semaphore == NULL) {
        ESP_LOGE("SERVER_BUFFER", "Failed to create semaphore");
    }else {
        ESP_LOGI(TAG, "Sensor semaphore created.");
    }
    printf("\n");

}



