#include <stdbool.h>

// gpio_control.c
#include "gpio_control.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"


// Flags to states
static bool fan1_state = false; 
static bool fan2_state = false; 
static bool buzzer_state = false; 

// Flags to keep track of LED states
static bool led_states[3] = {false, false, false}; // 0: Green, 1: Blue, 2: Red

// Flag to track the state of the magenta LED
static bool magenta_on = false; // Initially, the magenta LED is off



static uint64_t last_blink_time = 0; // To track the last blink time
gpio_num_t current_led = -1;  // To track the currently active LED
static uint32_t current_interval = 0; // To track the current interval


// Initialize GPIO pins
void gpio_control_init(void) {
    printf("Initialised GPIO pins\n");
    // Bit mask for all GPIO pins we want to configure
    uint64_t pin_mask = (1ULL << LED_GREEN_PIN) | 
                        (1ULL << LED_BLUE_PIN) | 
                        (1ULL << LED_RED_PIN) | 
                        (1ULL << BUZZER_PIN) | 
                        (1ULL << FAN1_PIN) | 
                        (1ULL << FAN2_PIN);

    // Configure all GPIO pins as outputs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;      // Disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;            // Set GPIO as output mode
    io_conf.pin_bit_mask = pin_mask;            // Select all defined GPIO pins
    io_conf.pull_down_en = 0;                   // Disable pull-down mode
    io_conf.pull_up_en = 0;                     // Disable pull-up mode
    gpio_config(&io_conf);                      // Apply the configuration

     // Ensure all outputs are initially off
    led_off(LED_GREEN_PIN);
    led_off(LED_BLUE_PIN);
    led_off(LED_RED_PIN);
    buzzer_off();
    fan1_off();
    fan2_off();
}


// Function to turn on the specified LED
void led_on(gpio_num_t led) {
    
    //printf("call led_on function for LED on pin %d\n", led);
    if (led == LED_GREEN_PIN && !led_states[0]) { // Check if green LED is off
        gpio_set_level(LED_GREEN_PIN, 1);
        led_states[0] = true; 
        printf("GREEN LED is ON\n");
    } else if (led == LED_BLUE_PIN && !led_states[1]) { // Check if blue LED is off
        gpio_set_level(LED_BLUE_PIN, 1);
        led_states[1] = true; 
        printf("BLUE LED is ON\n");
    } else if (led == LED_RED_PIN && !led_states[2]) { // Check if red LED is off
        gpio_set_level(LED_RED_PIN, 1);
        led_states[2] = true; 
        printf("RED LED is ON\n");
    }
}

// Function to turn off the specified LED
void led_off(gpio_num_t led) {
    if (led == LED_GREEN_PIN && led_states[0]) { // Check if green LED is on
        gpio_set_level(LED_GREEN_PIN, 0);
        led_states[0] = false; 
        printf("GREEN LED is OFF\n");
    } else if (led == LED_BLUE_PIN && led_states[1]) { // Check if blue LED is on
        gpio_set_level(LED_BLUE_PIN, 0);
        led_states[1] = false; 
        printf("BLUE LED is OFF\n");
    } else if (led == LED_RED_PIN && led_states[2]) { // Check if red LED is on
        gpio_set_level(LED_RED_PIN, 0);
        led_states[2] = false; 
        printf("RED LED is OFF\n");
    }
}



void blink_led(gpio_num_t led, uint32_t interval_ms) {
    
    // Check if the LED or the interval has changed
    if (led != current_led || interval_ms != current_interval) {
        
        printf("LED changed\n");
        // Turn off the previous LED if changing
        if (current_led != -1) {
            led_off(current_led);
        }
        
        // Set the new LED and interval
        current_led = led;
        current_interval = interval_ms;
        last_blink_time = xTaskGetTickCount() * portTICK_PERIOD_MS; 
    }
    
    uint64_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS; 
    
    
    if (current_time - last_blink_time >= current_interval) {
        
        
        switch (current_led) {
            case LED_GREEN_PIN:
                printf("Current LED: GREEN\n");
                break;
            case LED_BLUE_PIN:
                printf("Current LED: BLUE\n");
                break;
            case LED_RED_PIN:
                printf("Current LED: RED\n");
                break;
            default:
                printf("Unknown LED\n");
        }

        // Toggle LED based on its current state
        /*if (led_states[current_led]) {
            led_off(current_led); // Turn off if currently on
        } else {
            led_on(current_led);  // Turn on if currently off
        }*/
        led_on(current_led);
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_off(current_led);
        last_blink_time = xTaskGetTickCount() * portTICK_PERIOD_MS;//current_time; 
    }
}





// Function to turn on the LED to display magenta color
void led_magenta_on(void) {
    if (!magenta_on) {  
        led_on(LED_RED_PIN);   
        led_on(LED_BLUE_PIN);  
        magenta_on = true;     
        printf("Magenta LED is ON\n");
    }
}

// Function to turn off the LED magenta color
void led_magenta_off(void) {
    if (magenta_on) {  
        led_off(LED_RED_PIN);  
        led_off(LED_BLUE_PIN); 
        magenta_on = false;    
        printf("Magenta LED is OFF\n");
    }
}


// Function to toggle magenta LED on/off at a specified interval
void blink_magenta_led(uint32_t interval_ms) {
    static uint64_t last_blink_time = 0;
    uint64_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;  

    if (current_time - last_blink_time >= interval_ms) {
        if (magenta_on) {
            led_magenta_off();  
        } else {
            led_magenta_on();   
        }
        last_blink_time = current_time;  
    }
}




// Turn on the buzzer
void buzzer_on(void) {
    if (!buzzer_state) { 
        printf("BUZZER ON\n");
        gpio_set_level(BUZZER_PIN, 1);
        buzzer_state = true; 
    }
}

// Turn off the buzzer
void buzzer_off(void) {
    if (buzzer_state) { 
        printf("BUZZER OFF\n");
        gpio_set_level(BUZZER_PIN, 0);
        buzzer_state = false; 
    }
}

// Turn on Fan 1
void fan1_on(void) {
   
    if (!fan1_state) { 
        gpio_set_level(FAN1_PIN, 1);
        fan1_state = true; 
    }
}

// Turn off Fan 1
void fan1_off(void) {
    
    if (fan1_state) { 
        gpio_set_level(FAN1_PIN, 0);
        fan1_state = false; 
    }
}

// Turn on Fan 2
void fan2_on(void) {
    if (!fan2_state) { 
        gpio_set_level(FAN2_PIN, 1);
        fan2_state = true; 
    }
}

// Turn off Fan 2
void fan2_off(void) {
    if (fan2_state) { 
        gpio_set_level(FAN2_PIN, 0);
        fan2_state = false; 
    }
}
