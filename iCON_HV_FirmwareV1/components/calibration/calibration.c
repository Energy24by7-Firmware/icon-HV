#include "calibration.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "driver/gpio.h"


#define TAG "CALIBRATION"

TaskHandle_t calibrationTaskHandle = NULL;

// Simulated raw sensor data (in real applications, you'd fetch actual sensor readings)
float raw_solar_voltage = 50.0;
float raw_solar_current = 10.0;
float raw_battery_voltage = 12.0;
float raw_temp_sensor = 25.0;


// Calibration constants for each sensor (in a real scenario,it will take from the server)
float solar_voltage_calibration_constant = 164.64;
float solar_current_calibration_constant = 0.01;
float battery_voltage_calibration_constant = 171.42857;
float temp_sensor_calibration_constant = 35.67;


// Variables to store the calibrated values
float calibrated_solar_voltage = 0.0;
float calibrated_solar_current = 0.0;
float calibrated_battery_voltage = 0.0;
float calibrated_temperature = 0.0;

bool recalibration_flag = false;  // Flag to indicate if recalibration is required
bool Calibration_Flag = false ;//Testing : True for checking the nvs data read  portion  ; False for nvs saved section 

// NVS namespace and key for calibration flag
#define NVS_NAMESPACE "storage"
#define CALIBRATION_FLAG_KEY "cal_flag"



// Function to save the calibration flag to NVS
void save_calibration_flag(bool flag) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(err));
        return;
    }

    // Write calibration flag
    err = nvs_set_u8(nvs_handle, CALIBRATION_FLAG_KEY, flag ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) setting calibration flag in NVS", esp_err_to_name(err));
    }

    // Commit changes to NVS
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) committing calibration flag to NVS", esp_err_to_name(err));
    }

    // Close NVS
    nvs_close(nvs_handle);
}


// Function to read calibration flag from NVS
bool read_calibration_flag() {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t flag_value = 0;
    bool calibration_flag = false;

    // Open NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(err));
        return false;
    }

    // Read calibration flag
    err = nvs_get_u8(nvs_handle, CALIBRATION_FLAG_KEY, &flag_value);
    if (err == ESP_OK) {
        calibration_flag = (flag_value == 1);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error (%s) reading calibration flag from NVS", esp_err_to_name(err));
    }

    // Close NVS
    nvs_close(nvs_handle);

    return calibration_flag;
}

void set_recalibration_flag() { // Function to set recalibration_flag to 1
    recalibration_flag = true;
    ESP_LOGI(TAG, "Recalibration flag set to 1.\n");
}



//function to retrive calibration values from the server
/*
#include "esp_http_client.h"
#include "cJSON.h"
void fetch_calibration_constants() {
    // Configuration for the HTTP client
    esp_http_client_config_t config = {
        .url = "http://yourserver.com/api/calibration",  // Replace with your actual server URL
    };
    
    // Initialize the HTTP client
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    // Perform the HTTP GET request
    esp_err_t err = esp_http_client_perform(client);

    // Check if the request was successful
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);  // Get HTTP response status code
        
        // Check if the status code indicates a successful response (HTTP 200 OK)
        if (status_code == 200) {
            // Buffer to hold the server response
            char response_buffer[512];
            
            // Read the response into the buffer
            int content_length = esp_http_client_read(client, response_buffer, sizeof(response_buffer));
            
            // If the response content was read successfully
            if (content_length > 0) {
                response_buffer[content_length] = '\0';  // Null-terminate the string

                // Parse the response as JSON
                cJSON *json_response = cJSON_Parse(response_buffer);
                
                // If the JSON was parsed successfully
                if (json_response != NULL) {
                    // Extract calibration constants from the JSON object
                    float solar_voltage_const = cJSON_GetObjectItem(json_response, "solar_voltage_const")->valuedouble;
                    float solar_current_const = cJSON_GetObjectItem(json_response, "solar_current_const")->valuedouble;
                    float battery_voltage_const = cJSON_GetObjectItem(json_response, "battery_voltage_const")->valuedouble;
                    float temperature_const = cJSON_GetObjectItem(json_response, "temperature_const")->valuedouble;

                    // Store the calibration constants in NVS (non-volatile storage)
                    store_calibration_constants(solar_voltage_const, solar_current_const, battery_voltage_const, temperature_const);
                    
                    // Free the allocated memory for the JSON object
                    cJSON_Delete(json_response);
                }
            }
        }
    } else {
        // Log an error message if the HTTP request failed
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    // Cleanup and free the HTTP client resources
    esp_http_client_cleanup(client);
}
*/


// Function to perform sensor calibration
void perform_calibration() {
    // Apply calibration constants to raw sensor readings
    calibrated_solar_voltage = raw_solar_voltage * solar_voltage_calibration_constant;
    calibrated_solar_current = raw_solar_current * solar_current_calibration_constant;
    calibrated_battery_voltage = raw_battery_voltage * battery_voltage_calibration_constant;
    calibrated_temperature = raw_temp_sensor * temp_sensor_calibration_constant;

    // Log the calibrated values
    ESP_LOGI("Calibration", "Solar Voltage: %.2f V", calibrated_solar_voltage);
    ESP_LOGI("Calibration", "Solar Current: %.2f A", calibrated_solar_current);
    ESP_LOGI("Calibration", "Battery Voltage: %.2f V", calibrated_battery_voltage);
    ESP_LOGI("Calibration", "Temperature: %.2f °C", calibrated_temperature);
}


void store_calibration_constants(float solar_voltage_const, float solar_current_const, float battery_voltage_const, float temperature_const) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }
    // Store the calibration constants as blobs (binary data)
    err = nvs_set_blob(nvs_handle, "sol_volt_const", &solar_voltage_const, sizeof(solar_voltage_const));
    ESP_ERROR_CHECK(err);

    err = nvs_set_blob(nvs_handle, "sol_curr_const", &solar_current_const, sizeof(solar_current_const));
    ESP_ERROR_CHECK(err);

    err = nvs_set_blob(nvs_handle, "bat_volt_const", &battery_voltage_const, sizeof(battery_voltage_const));
    ESP_ERROR_CHECK(err);

    err = nvs_set_blob(nvs_handle, "temp_const", &temperature_const, sizeof(temperature_const));
    ESP_ERROR_CHECK(err);

    err = nvs_commit(nvs_handle);
    ESP_ERROR_CHECK(err);

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Calibration constants stored successfully");
}


void read_calibration_constants(float *solar_voltage_const, float *solar_current_const, float *battery_voltage_const, float *temperature_const) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("calibration", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    // Read solar voltage constant
    size_t required_size = sizeof(*solar_voltage_const);
    err = nvs_get_blob(nvs_handle, "sol_volt_const", solar_voltage_const, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read solar voltage constant: %s", esp_err_to_name(err));
    }

    // Read solar current constant
    required_size = sizeof(*solar_current_const);
    err = nvs_get_blob(nvs_handle, "sol_curr_const", solar_current_const, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read solar current constant: %s", esp_err_to_name(err));
    }

    // Read battery voltage constant
    required_size = sizeof(*battery_voltage_const);
    err = nvs_get_blob(nvs_handle, "bat_volt_const", battery_voltage_const, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read battery voltage constant: %s", esp_err_to_name(err));
    }

    // Read temperature constant
    required_size = sizeof(*temperature_const);
    err = nvs_get_blob(nvs_handle, "temp_const", temperature_const, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature constant: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Calibration constants read successfully");
}


void Calibration_task(void *pvParameter) {
      
    Calibration_Flag = read_calibration_flag();
    ESP_LOGI(TAG, "Calibration flag status: %s",Calibration_Flag? "true" : "false");

    // Device needs to be recalibrated , When a keystroke happens re-calibration process happens manually
    //set_recalibration_flag() //  Function to set recalibration_flag to 1 for testing 
    
        if (!Calibration_Flag || recalibration_flag) {
            ESP_LOGI(TAG, "Performing recalibration...");
            
            // Perform calibration process
            perform_calibration();
            
            // Store calibration constants
            store_calibration_constants(solar_voltage_calibration_constant, 
                                        solar_current_calibration_constant, 
                                        battery_voltage_calibration_constant, 
                                        temp_sensor_calibration_constant);
            
            // Reset the recalibration flag
            recalibration_flag = false;
            Calibration_Flag = true;
            // Save Calibration Flag to NVS
            save_calibration_flag(Calibration_Flag);
            
        }
        if (Calibration_Flag) {
            // If Calibration_Flag is set, read the calibration constants from NVS
            ESP_LOGI(TAG, "Reading calibration constants from NVS...");

            read_calibration_constants(&solar_voltage_calibration_constant, 
                                   &solar_current_calibration_constant, 
                                   &battery_voltage_calibration_constant, 
                                   &temp_sensor_calibration_constant);

            ESP_LOGI(TAG, "Calibration data: Solar Voltage: %.2f, Solar Current: %.2f, Battery Voltage: %.2f, Temperature: %.2f",
                 solar_voltage_calibration_constant, 
                 solar_current_calibration_constant, 
                 battery_voltage_calibration_constant, 
                 temp_sensor_calibration_constant);
        }
    vTaskDelete(calibrationTaskHandle);   // Task will now delete itself after first execution
}
