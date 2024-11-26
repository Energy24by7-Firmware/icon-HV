#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "cJSON.h"


// Include the header from components
#include "esp32s2_pac1952.h"
#include "esp32s2_i2c_protocol.h"
#include "esp32s2_fatal_err.h"
#include "samples_read.h"
#include "Relay_Control.h"
#include "mqtt_client.h"
#include "Server_buffer.h"
#include "gpio_control.h"
#include "calibration.h"
#include "ble_server.h" 
#include "wifi_mqtt.h"
#include "switch_detector.h"
#include "ota.h"
#include "esp_task_wdt.h"


#define TAG "MAIN"



void app_main()
{

    //printf("//////////////// ICON HV FIRMWARE //////////// ");
    printf("//////////////// OTA UPDATED FIRMWARE //////////// ");
    printf("Firmware update is complete. The new firmware version  is now running. OTA update process succeeded.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("///////////// START TO WORK ///////////");
    
    printf("\n");
    
    gpio_control_init();  // Buzzer , Fan , Led

    samples_read_init(); //ADC PINS initialise

    relay_init();  //Initialise all relay

    UPS();//default state
    previous_state = STATE_UPS;

    
   
    ////   Initialize the NVS flash  /////
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    
    

    //////  PAC 1592 SECTION ///////

    init_i2c();  // Initialize the I2C bus
    
    ret = i2c_master_detect_slave(); // Call the function to detect the slave device
    
    // Check the result and log appropriate messages
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C slave detected successfully");
    } else {
        ESP_LOGE(TAG, "Failed to detect I2C slave, error code: %d", ret);
    }


    init_pac1952(); 
    
     // Set an overvoltage limit, to pac1952
    printf(" // PAC ALERT Threshold saving section //\n");
    float overVoltageLimit = 1.64375; // 263/160
    uint16_t errorCode;

    errorCode = PAC194x5x_SetOverVolt_limit(pPACdevice, overVoltageLimit);
    if (errorCode != PAC194X5X_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set overvoltage limit, error code: %d", errorCode);
        return;
    }

    ESP_LOGI(TAG, "Overvoltage limit set to %f V", overVoltageLimit);
    printf("\n");


    wifi_initialization();
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    all_initialization(); //SPIFFS , SNTP SERVER , Semaphore





    ///////////     Configuration Part      /////////////////
    /*if(ATE_Config_flag)  
    {
        load_config_flag();
        if(config_flag){
            ESP_LOGI(TAG, "Preload Configuration Values");
        }
    }*/



    //switch_detector_init(); //Initialize the switch detector
    //ble_server_init();  // Initialize BLE server
    //esp_wifi_stop();  //For testing the adc channel 2 takes reaing after wifi disconnect 


    
    ///////////    Create FreeRTOS tasks   /////////////
    
    //**  CONFIGURATION RELATED TASKS  */
    //xTaskCreate(switch_monitor_task, "switch_monitor_task", 2048, NULL, 6, &switch_monitor_taskTaskHandle);
    // Create the BLE task
    //xTaskCreate(checkAndEnableBLETask,"BLE Init Task", 2048,NULL,5, &bleTaskHandle);



    xTaskCreate(Calibration_task, "Calibration_Task", 4096, NULL, 7, &calibrationTaskHandle); 

   
     
    //Task for checking the wifi and mqqt connection
    xTaskCreate(monitor_connections_task, "monitor_connections_task", 4096, NULL, 7, &monitor_task_handle);

    
    xTaskCreate(time_task, "Time Task", 2048, NULL, 7, &timeTaskHandle);//Time printing task 
    
    //Main tasks for save sensor reading and relay control section
    xTaskCreate(collect_data_task, "collect_data_task", 4096, NULL, 6, &collectDataTaskHandle);
    xTaskCreate(device_control_task, "device_control_task", 4096, NULL, 6, &device_control_task_handle);
    
    //Control led corresponding to states
    xTaskCreate(led_blink_task, "LED Blink Task", 2048, NULL, 6, &led_blink_task_handle);

    //Tak for save the averages
    xTaskCreate(avg_4s_task, "Avg4sTask", 4096, NULL, 5, &x4SecTaskHandle);
    xTaskCreate(avg_1min_task, "Avg1minTask", 4096, NULL, 5, &x1MinTaskHandle);
    xTaskCreate(avg_15min_task, "Avg15minTask", 4096, NULL, 5, &x15MinTaskHandle);
    
    //Task related to PAC Alert
    xTaskCreate(monitor_overvoltage_task, "monitor_overvoltage_task", 2048, NULL, 5, &monitorTaskHandle);
    xTaskCreate(pac_alert_occurs, "pac_alert_occurs", 2048, NULL, 5, &alertTaskHandle);

    
    // Create the detect_state_change task and pass the pointer to state_change_flag
    xTaskCreate(detect_state_change, "Detect State Change", 4096, &state_change_flag, 5, &xStateChangeTaskHandle);
    
    //Task for mqqt connection check publish failed buffer data section and copy failed buffer to spiff
    xTaskCreate(monitor_time_and_mqtt_task, "monitor_time_and_mqtt_task", 8096, NULL, 5, &monitor_time_and_mqtt_taskHandle);
   
    //Task of OTA
    xTaskCreate(&ota_main_task, "ota_main_task", 8192, NULL, 4, &otaTaskHandle);

   
}
