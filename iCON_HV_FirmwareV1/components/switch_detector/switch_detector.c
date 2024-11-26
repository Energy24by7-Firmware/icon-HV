#include "switch_detector.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "switch_detector";
static bool switch_flag = false;  // Flag to indicate switch change

static bool ATE_Config_flag = false ;

TaskHandle_t switch_monitor_taskTaskHandle = NULL;



// ISR for GPIO 5
void IRAM_ATTR gpio_isr_handler(void *arg) {
    switch_flag = true;  // Set the flag when switch state changes
}

// Initialize the switch detector
void switch_detector_init(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;  // Trigger on both rising and falling edges
    io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_5); // Correct bit mask for GPIO 5
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Enable pull-up
    gpio_config(&io_conf);

    // Install the ISR service
    gpio_install_isr_service(0); // Use 0 or appropriate flag
    gpio_isr_handler_add(GPIO_NUM_5, gpio_isr_handler, NULL);
}


// Get the switch flag
bool get_switch_flag(void) {
    return switch_flag;
}

// Reset the switch flag
void reset_switch_flag(void) {
    switch_flag = false;
}

void switch_monitor_task(void *pvParameter) {
    while (1) {
        if (get_switch_flag()) {
            ESP_LOGI(TAG, "Switch state changed!");

            // If the flag is set it goes to the ATE for configuration procedure. 
            if(!ATE_Config_flag)
            {
                //if(APP1_flag)
            }

            reset_switch_flag();  // Reset the flag after handling
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay to prevent task from hogging CPU
    }
}
