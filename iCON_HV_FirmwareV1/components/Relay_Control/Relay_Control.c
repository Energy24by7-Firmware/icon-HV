
#include <stdio.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdbool.h>


#include "Relay_Control.h"
#define TAG "Relay_Control"


//components Lib:
#include "esp32s2_pac1952.h"
#include "esp32s2_i2c_protocol.h"
#include "esp32s2_fatal_err.h"
#include "gpio_control.h"


// Task handle for the LED blink task
TaskHandle_t led_blink_task_handle = NULL;



// Global static variables
static bool sunup_flag = false;
static bool sundown_flag = false;
static uint16_t Solar_available_flag = 0;
//static uint16_t state_change_flag = 0;
static bool mosfet_failure_flag = false;
static uint16_t main_failure_flag = 0;
static uint16_t FT_flag = 0;
static uint16_t maincharging_flag = 0;
static uint16_t solarcharging_flag = 0;
static uint16_t solarassisted_flag = 0;

//static uint16_t current_check_flag = 0;

bool battery_disconnect_flag = false ;
//static int64_t mosfet_failure_time = 0;


static TickType_t current_check_time;

static uint16_t Maintain_State = 1 ;
static TickType_t  State_change_time;



device_state_t previous_state = STATE_NONE; // Store the previous state
int state ;




static bool initial_stage = false;
static bool batteryHigherThanMaxThreshold = false ;
static bool batteryHigherThanMinThreshold = false ;
static bool FT_mode_goes = false ;




//Sunup sundown declaration  //
#define MAX_SUNUP_COUNT 4
#define MAX_SUNDOWN_COUNT 30
#define MAX_SOLAR_DISCONNECT_COUNT 20

#define NOW_NIGHT_TIME 0
#define NOW_DAY_TIME 1

#define EXPECTED_SUNUP_DURATION 8 * 3600 // Expected sunup duration in seconds (8 hours)


int is_day_flag = NOW_NIGHT_TIME;    // Day or night flag
int is_sol_sensed = 0;               // Solar panel connection status
             

int sunup_count = 0;                 // Counter for sunup confirmation
int sundown_count = 0;               // Counter for sundown confirmation
int solar_disconnect_count = 0;      // Counter for solar disconnect 

int solar_pannel_error = 0 ;         // Error flag for solar panel issues
static int cloud_sensed_flag = 0;    // Flag to track cloud interference

int no_of_days = 0;                  // Tracks total days
uint32_t Solar_Time = 0;                  // Tracks hours during day
uint32_t solar_time_seconds = 0;          // Tracks seconds during day
uint32_t previous_day_sunup_duration = 0; // Previous day sunup duration (in seconds)
uint32_t current_day_sunup_duration = 0;  // Current day sunup duration (in seconds)

int sunup_conf_enable = 0x00;        // Sunup confidence flag
static bool less_sunup_duration = false ; 
static bool solar_have_problem = false ;
static bool alert_triggered_at_night = false ;



//////////             INTERRUPT SECTION            //////////

// Global flag for overvoltage condition
volatile bool overVoltageFlag = false;
uint8_t pac_alert_received = 0 ;
static bool ft_active_alert = false;
// Semaphore handle for ISR synchronization
SemaphoreHandle_t alert_flag_mutex;

// Task Handles
TaskHandle_t monitorTaskHandle = NULL;
TaskHandle_t alertTaskHandle = NULL;

// ISR to handle the interrupt when the PAC ALERT pin detects an over-voltage condition
void IRAM_ATTR alert_isr_handler() 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Only trigger ISR if the overVoltageFlag is set
    if (overVoltageFlag) {
        xSemaphoreGiveFromISR(alert_flag_mutex, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}


void pac_alert_occurs(void *pvParameter) {
    for (;;) {
        // Wait for the semaphore to be given by the ISR
        if (xSemaphoreTake(alert_flag_mutex, portMAX_DELAY)) {
            
            // Perform the overvoltage check outside the ISR
            if (overVoltageFlag) {

                ESP_LOGI(TAG, "Alert Received");                
                //if(FT_flag == 0 && !sundown_flag) //Old prgm live_test_bin_v5
                if ((FT_flag == 0) &&(is_day_flag == NOW_DAY_TIME)) //New section edited after sunup and sundown 
                { 
                    printf("Overvoltage alert triggered , Hence goes to FT mode\n");              
                    Force_Trip();  // STATE : 4
                    
                }
                

                if(FT_flag==1)
                {
                    batteryHigherThanMaxThreshold = false ; 
                    batteryHigherThanMinThreshold = false ;
                    initial_stage = false; 
                    ft_active_alert = true ;
                    printf("Current working flags are RESET & set ft_active_alert\n");
                    printf("FT_Mode active due to pac alert !!!\n");
                }
                               
                overVoltageFlag = false;
                
                printf("\n");
                                
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  
    }
}


// Initialize GPIO for PAC1952 ALERT pin and configure interrupt
void pac_alert_pin_init(void) 
{
    printf("Initialize and configure GPIO for PAC ALERT interrupt\n");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Interrupt on falling edge (PAC ALERT pin active low)
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PAC_ALERT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up to avoid floating pin issues
        .pull_down_en = 0,
    };
    gpio_config(&io_conf);

    // Install the ISR service, no flags required or use specific flags like ESP_INTR_FLAG_IRAM
    gpio_install_isr_service(0);  

    // Attach the interrupt handler
    gpio_isr_handler_add(PAC_ALERT_PIN, alert_isr_handler, (void*) PAC_ALERT_PIN);

    ESP_LOGI(TAG, "PAC1952 ALERT pin configured and interrupt handler added");
}




void monitor_overvoltage_task(void *pvParameter) {
    for (;;) {
       
        uint8_t overVoltageStatus = getOverVoltageStatus();
        
        if (overVoltageStatus) {
            ESP_LOGI(TAG, "Overvoltage alert triggered on CH1 !!!!");
            //if(FT_flag == 0 && !sundown_flag) 
            if ((FT_flag == 0) &&(is_day_flag == NOW_DAY_TIME)) 
            {               
                overVoltageFlag = true;  //Flag set only once
                printf("Overvoltage flag set to activate the interrupt!\n");
                
            }
        } else {
            ESP_LOGI(TAG, "No overvoltage detected on CH1");
            overVoltageFlag = false; 
            
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}





//The pins are all configured as outputs.
//Pull-up and pull-down resistors are disabled as they're not needed for output.
//All relays and switches are initialized to LOW (off state).
void relay_init(void) 
{
    ESP_LOGI(TAG, "Relay_Initialise");

    // GPIO configuration structure
    gpio_config_t io_conf;

    // Disable interrupts, as they are not needed for these outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;

    // Set all GPIOs as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    // Disable pull-down and pull-up resistors since not needed for output
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    // Configure multiple GPIOs in one go using a combined pin mask
    io_conf.pin_bit_mask = (1ULL << SOLAR_RELAY_PIN) | 
                           (1ULL << SOLAR_SWITCH_PIN) | 
                           (1ULL << MAINS_RELAY_PIN) | 
                           (1ULL << MAINS_CONTACTOR_PIN) | 
                           (1ULL << MAINS_TRIAC_PIN);

    // Apply configuration
    gpio_config(&io_conf);

    // Set initial levels for all GPIO pins (all to LOW)
    gpio_set_level(SOLAR_RELAY_PIN, 0);
    gpio_set_level(SOLAR_SWITCH_PIN, 0);
    gpio_set_level(MAINS_RELAY_PIN, 1); // Turns OFF the relay by setting the transistor base HIGH 
    gpio_set_level(MAINS_CONTACTOR_PIN, 1); // Turns OFF the relay by setting the transistor base HIGH
    gpio_set_level(MAINS_TRIAC_PIN, 0);

    ///   PAC Alert section   ///
    // Create a binary semaphore for alert handling
    alert_flag_mutex = xSemaphoreCreateBinary();
    if (alert_flag_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }


    pac_alert_pin_init();

    ESP_LOGI(TAG, "GPIO pins for relay and switch have been initialized");
}




/////////   SUNUP & SUNDOWN SECTION  /////////




int calculate_total_solar_panels(float max_voltage_per_panel, float total_measured_voltage) {
    if (max_voltage_per_panel <= 0) {
        printf("Error: Invalid max voltage per panel.\n");
        return -1; 
    }

    int total_panels = (int)(total_measured_voltage / max_voltage_per_panel);
    return total_panels;
}




void sun_down_routine() 
{
    is_day_flag = NOW_NIGHT_TIME;
    is_sol_sensed = 0;

    // Calculate the current day's sunup duration
    current_day_sunup_duration = Solar_Time * 3600 + solar_time_seconds;

    if(no_of_days>2)
    {
       sunup_conf_enable = 1 ; 
    }else{
       sunup_conf_enable = 0 ; 
    }

    // Save the current day's sunup duration
    previous_day_sunup_duration = current_day_sunup_duration;

    // Reset solar time for the next day
    Solar_Time = 0;
    solar_time_seconds = 0;
    current_day_sunup_duration = 0 ;
    sundown_count = 0 ;
   
}


void sun_routine_algorithm(float voltage) 
{
    
    //////     ** Sunup Detection Logic **     //////

    if (voltage > SOL_HIGH_THR && is_day_flag == NOW_NIGHT_TIME) 
    {
        if (sunup_count < MAX_SUNUP_COUNT) {
            sunup_count++;
        } else {

            // Confirm sunup
            sunup_count = 0;           
            is_sol_sensed = 1;
            is_day_flag = NOW_DAY_TIME;
            cloud_sensed_flag = 0 ;
            solar_pannel_error = 0 ;  

            
            no_of_days++;
            printf("No: of days: %d\n", no_of_days);

            // Update sunup confidence flag after two days
            if (no_of_days > 1) {
                sunup_conf_enable = 1;
            }
            printf("SUNUP CONFIRMED \n");
        }
    } 
    else {
        sunup_count = 0;
    }



    // ** Time Tracking During Sunup **
    if (is_day_flag == NOW_DAY_TIME)
    {

        if(voltage > SOL_HIGH_THR) 
        {
            solar_time_seconds++;
            if (solar_time_seconds >= 3600) {
                solar_time_seconds = 0;
                Solar_Time++;
                printf("Solar Time: %lu\n", Solar_Time);
            }
        }
        else
        {
            //Calculate the current day's sunup duration
            current_day_sunup_duration = Solar_Time * 3600 + solar_time_seconds;
            printf("current_day_sunup_duration:  %lu\n", current_day_sunup_duration);


            // Check if sunup duration is significantly lower than expected > This loop works only once up to flag set
            if((current_day_sunup_duration < EXPECTED_SUNUP_DURATION) && (Solar_available_flag==1)) 
            {
                printf("Check if sunup duration is significantly lower than expected \n");
                solar_have_problem = true ;

                if(sunup_conf_enable==1) //This flag set only after 2 days of install.Ater that only we can compare the 2 days duration
                {
                   if(current_day_sunup_duration < previous_day_sunup_duration)
                   {
                        printf("Previous_day_sunup_duration:  %lu\n", previous_day_sunup_duration);
                        printf("Compare with previous day's sunup duration \n");
                        less_sunup_duration = true ;
                   }else{
                        less_sunup_duration = false;
                   }
                }
                if ((solar_have_problem && (voltage < SOL_LOW_H_THR && voltage > SOL_LOW_L_THR)) || less_sunup_duration) 
                {
                    if (sundown_count < MAX_SUNDOWN_COUNT ) 
                    {
                        printf("Gradual voltage decrease indicates cloud in day time \n");
                        sundown_count++;
                    }else 
                    {                   
                        // Gradual voltage decrease indicates cloud cover
                        cloud_sensed_flag = 1;
                        sundown_count = 0 ; 
                        less_sunup_duration = false;                       
                        printf("SUNDOWN CONFORMED DUE TO CLOUD.\n");                  
                    }
                              
                }
                else if (voltage <= SOL_LOW_L_THR && sundown_count < MAX_SUNDOWN_COUNT) 
                {
                    // Sudden voltage drop detection for solar panel error
                    if (solar_disconnect_count < MAX_SOLAR_DISCONNECT_COUNT) 
                    {
                        printf("Sudden voltage drop detection for solar panel error \n");
                        solar_disconnect_count++;
                    } 
                    else 
                    {
                    
                        solar_pannel_error = 1; // Solar panel disconnection
                        solar_disconnect_count = 0;
                        sundown_count = 0 ;
                        less_sunup_duration = false;
                        printf("Error_DAY: SOLAR PANEL DISCONNECTED.\n");
                                                          
                    }
                } 
                else 
                {
                    // Reset counts if solar voltage recovers
                    sundown_count = 0;
                    solar_disconnect_count = 0;
                    cloud_sensed_flag = 0;
                    solar_pannel_error = 0 ;
                    
                }
            }
            else   ////  * SUNDOWN SECTION   >>> current_day_sunup_duration > EXPECTED_SUNUP_DURATION(6hrs)
            {
            
                if ((voltage < SOL_LOW_H_THR && voltage > SOL_LOW_L_THR) || (voltage <= SOL_LOW_L_THR))
                {
                    if (sundown_count < MAX_SUNDOWN_COUNT ) 
                    {
                        printf("Gradual voltage decrease indicates sundown \n");
                        sundown_count++;  // Gradual voltage decrease indicates sundown
                    }
                    else 
                    {                                            
                        // Confirm sundown                        
                        sun_down_routine();
                        printf("SUNDOWN CONFIRMED \n");
                        printf("\n");
                                        
                    }
                              
                }
                
            }
        } // end of voltage < SOL_HIGH_THR

        if(solar_pannel_error==1)
        {
            printf("SOLAR PANNEL DISCONNECTED DUE TO ERROR !!!! \n");
        }else if(cloud_sensed_flag==1){
            printf("SUNDOWN DETECTED DUE TO CLOUD !!!\n");
        }

    }  // end of is_day_flag == NOW_DAY_TIME
}


void Solar_Relay_Tracking() 
{
    
    if (mosfet_failure_flag)
    {
        printf("Solar_Relay OFF due to MOSFET Failure\n");
        return;
    }else{
      mosfet_failure_flag = false ;  
    }

    if (is_sol_sensed==1 && solar_pannel_error ==0 && cloud_sensed_flag==0) 
    {
        
        if (Solar_available_flag == 0 && !mosfet_failure_flag) {
            gpio_set_level(SOLAR_RELAY_PIN, 1);
           
            Solar_available_flag = 1 ;
            ESP_LOGI(TAG, "SOLAR_RELAY_PIN ON \n ");
            printf("\n"); 
                     
           
        }

    } else {
        if (Solar_available_flag == 1 && !mosfet_failure_flag) {
            gpio_set_level(SOLAR_RELAY_PIN, 0);
            Solar_available_flag = 0 ;
            ESP_LOGI(TAG, "SOLAR_RELAY_PIN OFF \n");
            printf("\n");
            solar_have_problem = false ;
        }
        
    }
}



void check_mosfet_failure(float solar_current_now) {
static int confirmation_count = 0 ;
    ESP_LOGI("<MOSFET>","Check MOSFET FAIURE \n");
    if(solar_current_now <= 0)  // Solar current is negative value or zero
    {
       
        if(solar_have_problem)
        {
            mosfet_failure_flag = false;
            ESP_LOGI("<MOSFET>","Solar current dropped to zero >> Checking reason for voltage drop  at solar pannel side \n");//cloud comformed (30 counts) , solar panel error (20 counts)
        }
        else
        {            
            if(Solar_available_flag==1)
            {
                ESP_LOGI("<MOSFET>", "Solar Pannel and climate are normal , Hence confirmed Solar current dropped to zero due to Mosfet Failure \n");
                confirmation_count++ ;
                if (!mosfet_failure_flag && confirmation_count>=5) 
                {
                    ESP_LOGI("<MOSFET>", "Solar switch failure detected, turning off solar relay to protect battery \n");
                    gpio_set_level(SOLAR_RELAY_PIN, 0); // Turn off solar relay
                    Solar_available_flag = 0 ;
        
                    mosfet_failure_flag = true;
                       
                    batteryHigherThanMinThreshold = false ;//Exit from state 6 if the device is in state 6
                    initial_stage = false ; //Goes to initial stage(Exit from state 7 if the device is in state 7)
                    ESP_LOGI("<MOSFET>", "Goes to UPS Mode after after MOSFET failure detection!!!!!"); 
                    UPS();    
                    confirmation_count = 0 ;      
                }
                

            }
            else{

                ESP_LOGI("<MOSFET>", "Current has dropped to zero not due to MOSFET failure, but because solar voltage is unavailable.");
                confirmation_count = 0 ; 
            }
            
        }
              
    }else{
        mosfet_failure_flag = false;
    }
}




/////////////////////   Old section of code    ///////////////////


/*void Day_Tracking(float voltage) {
    if (voltage > VOLTAGE_THRESHOLD_SOLAR) {
        // Voltage is above threshold, set sunup flag and turn on relay
        sunup_flag = true;
        sundown_flag = false;

       
        if (mosfet_failure_flag)
        {
            printf("Solar_Relay OFF due to MOSFET Failure\n");
        	return;
        }


        if (Solar_available_flag == 0) {
            gpio_set_level(SOLAR_RELAY_PIN, 1);
           // Solar_available_flag = (gpio_get_level(SOLAR_RELAY_PIN) == 1) ? 1 : 0;
            Solar_available_flag = 1 ;

            ESP_LOGI(TAG, "/////////////////   **********  DAY TIME  ***********   ////////////////");
            ESP_LOGI(TAG, "SOLAR_RELAY_PIN ON ,SUNUP FLAG SET");

            printf("\n");
            
            // Reset the mosfet_failure_flag if the relay successfully turns on and no MOSFET failure is detected, 
            // or if a failed MOSFET has been replaced and the device is restarting.

            if (Solar_available_flag == 1) {
                mosfet_failure_flag = false;
            }
        }
    } else {
        // Voltage is below threshold, set sundown flag and turn off relay
        sunup_flag = false;
        sundown_flag = true;

        if (Solar_available_flag == 1 && !mosfet_failure_flag) {
            gpio_set_level(SOLAR_RELAY_PIN, 0);
            //Solar_available_flag = (gpio_get_level(SOLAR_RELAY_PIN) == 1) ? 1 : 0;
            Solar_available_flag = 0 ;
             ESP_LOGI(TAG, "/////////////////   **********  NIGHT TIME  ***********   ////////////////");
            ESP_LOGI(TAG, "SOLAR_RELAY_PIN OFF ,SUNDOWN FLAG SET");
            printf("\n");
        }
        
    }
}

void check_mosfet_failure(float solar_current_now) {
    static int confirmation_count = 0 ;
    printf("Check MOSFET FAIURE\n");
    if(solar_current_now <= 0)  // Solar current is negative value or zero
    {
        
        if(sunup_flag)//flag set when solar_voltage > solat threshold(here 20 v)
        {   
            confirmation_count++ ;
            if (!mosfet_failure_flag && confirmation_count>=5) 
            {
                ESP_LOGI("MOSFET>", "Solar switch failure detected, turning off solar relay to protect battery.");
                gpio_set_level(SOLAR_RELAY_PIN, 0); // Turn off solar relay
                Solar_available_flag = 0 ;
        
                mosfet_failure_flag = true;
                       
                batteryHigherThanMinThreshold = false ;//Exit from state 6 if the device is in state 6
                initial_stage = false ; //Goes to initial stage(Exit from state 7 if the device is in state 7)
                ESP_LOGI("MOSFET>", " going to activate STATE:5 after MOSFET Failure \n");
                confirmation_count = 0 ;
                UPS();

            }

        }else{
            ESP_LOGI("MOSFET>", "Current has dropped to zero not due to MOSFET failure, but because solar voltage unavailable.");
            mosfet_failure_flag = false;
            confirmation_count = 0 ;
        }
        
    }else{
        mosfet_failure_flag = false;
        confirmation_count = 0 ;
    }

}*/


////////////////////////////////////////////////////////



static int voltage_check_counter = 0;
static bool voltage_increasing = false;
static bool has_increased = false;
static float previous_voltage = 0;
static bool off_ups = false ;

void reset_voltage_monitoring() {
    voltage_check_counter = 0;
    previous_voltage = 0;
    has_increased = false;
    voltage_increasing = false;
}

void main_failure(float current_batvoltage) {
    if (voltage_check_counter == 0) {
        previous_voltage = current_batvoltage;
        ESP_LOGI(TAG, "Starting battery voltage monitoring for main failure detection...");
    } else {
        if (current_batvoltage > previous_voltage && !voltage_increasing) {
            has_increased = true;
            voltage_increasing = true;
        } else if (current_batvoltage < previous_voltage && voltage_increasing) {
            voltage_increasing = false;
        }
        previous_voltage = current_batvoltage;
    }

    voltage_check_counter++;

    // After 20 checks (20 seconds), determine if there's a main failure
    if (voltage_check_counter >= 20) {
        if (has_increased && voltage_increasing) {
            main_failure_flag = 0;
            ESP_LOGI(TAG, "No main failure detected.");
        } else {
            main_failure_flag = 1;
            ESP_LOGI(TAG, "Main failure detected.");
        }
        reset_voltage_monitoring();
    }
}

void check_night_mode(float battery_voltage) {
    //if (sundown_flag)  //Old code without sunup and sundown
    if (is_day_flag == NOW_NIGHT_TIME) {
        if (battery_voltage > 263) {
            ESP_LOGI("NIGHT >", "Exit from STATE:5 to maintain float condition.\n");
            Off_ups_at_night();
            off_ups = true;
            reset_voltage_monitoring();
        } else if (battery_voltage <= 256) {
            ESP_LOGI("NIGHT >", "Going to STATE:5 to maintain float condition.\n");
            UPS(); // STATE: 5
            off_ups = false;
        }
    }
}

void monitor_main_failure(float battery_voltage) {
    if (maincharging_flag == 1 && !off_ups) {
        main_failure(battery_voltage);
    }
}




device_state_t state_change_occurs(void)
{
    device_state_t current_state = STATE_NONE;  // Default state

    // Determine the current state based on the flags
    if (FT_flag == 1) {
        current_state = STATE_FORCE_TRIP;
    }
    else if (maincharging_flag == 1) {
        current_state = STATE_UPS;
    }
    else if (solarcharging_flag == 1 && solarassisted_flag == 0) {
        current_state = STATE_SOLAR_ASSISTED_FT;
    }
    else if (solarassisted_flag == 1) {
        current_state = STATE_SOLAR_ASSISTED_BATTERY_CHARGING;
    }


    return current_state;
}

void Force_Trip(void)  // STATE : 4 Load supported by battery and battery not charged
{
    solarcharging_flag = 0;
    solarassisted_flag = 0;
    maincharging_flag = 0;

    if (FT_flag == 0) {
        
        ESP_LOGI("STATE - 4", "Force_Trip Mode");

        // Turn off solar switch
        gpio_set_level(SOLAR_SWITCH_PIN, 0);

        // Turn on mains triac
        gpio_set_level(MAINS_TRIAC_PIN, 1);

        // Wait for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn off mains relay and mains contactor
        gpio_set_level(MAINS_RELAY_PIN, 1);     // Turns OFF the relay(SSR) by setting the transistor base HIGH
        gpio_set_level(MAINS_CONTACTOR_PIN, 1); // Turns OFF the relay(SSR) by setting the transistor base HIGH

        // Wait for another 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn off mains triac
        gpio_set_level(MAINS_TRIAC_PIN, 0);

        //FT_flag = (gpio_get_level(SOLAR_SWITCH_PIN) == 0 && gpio_get_level(MAINS_RELAY_PIN) == 0 && gpio_get_level(MAINS_CONTACTOR_PIN) == 0) ? 1 : 0;
        
        FT_flag = 1 ;
        ESP_LOGI("STATE - 4", "Force Trip flag set to %d", FT_flag);
        
    }
}

void UPS(void)  //STATE 5 : Load supported by utility grid  &  battery charged by utility grid
{
    FT_flag = 0;
    solarcharging_flag = 0;
    solarassisted_flag = 0;

    if (maincharging_flag == 0) {
        ESP_LOGI("STATE - 5", "UPS Mode");

        gpio_set_level(SOLAR_SWITCH_PIN, 0);

        // Turn on TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 1);

        // Wait for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn on the relay (Utility Grid)
        gpio_set_level(MAINS_RELAY_PIN, 0); // Turns ON the relay(SSR) by setting the transistor base LOW
        gpio_set_level(MAINS_CONTACTOR_PIN, 0); // Turns ON the relay(SSR) by setting the transistor base LOW

        // Wait for another 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn off TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 0);

        maincharging_flag  = 1 ;
        ESP_LOGI("STATE - 5", "UPS Mode flag set to %d", maincharging_flag);
    }
}

void Solar_assisted_FT(void)  //STATE 6  :  Load supported by Battery + solar energy & battery charged by solar energy
{
    FT_flag = 0;
    maincharging_flag = 0;
    solarassisted_flag = 0;

    if (solarcharging_flag == 0) {
        ESP_LOGI("STATE - 6", "Solar_assisted_FT Mode");

        if (Solar_available_flag == 1 && !mosfet_failure_flag) {
                
                gpio_set_level(SOLAR_SWITCH_PIN, 1);
                printf("MOSFET ON for activate STATE- 6\n");
                

            }

        // Turn on TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 1);

        // Wait for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn off the relay (Utility Grid)
        gpio_set_level(MAINS_RELAY_PIN, 1);// Turns OFF the relay by setting the transistor base HIGH
        gpio_set_level(MAINS_CONTACTOR_PIN, 1);// Turns OFF the relay by setting the transistor base HIGH

        // Wait for another 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn off TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 0);

        //solarcharging_flag = (gpio_get_level(SOLAR_SWITCH_PIN) == 1 && gpio_get_level(MAINS_RELAY_PIN) == 0 && gpio_get_level(MAINS_CONTACTOR_PIN) == 0) ? 1 : 0;
         
        solarcharging_flag = 1 ;

        current_check_time = xTaskGetTickCount()* portTICK_PERIOD_MS;

        // Print current state of each relay pin
        ESP_LOGI("STATE - 6", "Solar_assisted_FT Mode flag set to %d", solarcharging_flag);
       
    }
}


void Solar_assisted_battery_charging(void)  //STATE 7 : Load supported by Utility grid & battery charged by solar energy + utility grid
{
    FT_flag = 0;
    maincharging_flag = 0;
    solarcharging_flag = 0;

    if (solarassisted_flag == 0) {
        ESP_LOGI("STATE - 7", "Solar_assisted_battery_charging Mode");

        if (Solar_available_flag == 1 && !mosfet_failure_flag) {  
                gpio_set_level(SOLAR_SWITCH_PIN, 1);
                printf("MOSFET ON for activate STATE- 7\n");
            }

        // Turn on TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 1);

        // Wait for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn on the relay (Utility Grid)
        gpio_set_level(MAINS_RELAY_PIN, 0);// Turns ON the relay(SSR) by setting the transistor(NPN) base LOW
        gpio_set_level(MAINS_CONTACTOR_PIN, 0);// Turns ON the relay(SSR) by setting the transistor(NPN) base LOW

        // Wait for another 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn off TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 0);

        //solarassisted_flag = (gpio_get_level(SOLAR_SWITCH_PIN) == 1 && gpio_get_level(MAINS_RELAY_PIN) == 1 && gpio_get_level(MAINS_CONTACTOR_PIN) == 1) ? 1 : 0;

        solarassisted_flag = 1 ;
        current_check_time = xTaskGetTickCount()* portTICK_PERIOD_MS;
        ESP_LOGI("STATE - 7", "Solar_assisted_battery_charging Mode flag set to %d", solarassisted_flag);
        
    }
}

void Off_ups_at_night()
{
    FT_flag = 0;
    solarcharging_flag = 0;
    solarassisted_flag = 0;

    if (maincharging_flag == 1) {
        ESP_LOGI(" NIGHT > ", "GOING TO EXIT FROM STATE : 5");

        gpio_set_level(SOLAR_SWITCH_PIN, 0);

        // Turn on TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 1);

        // Wait for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn on the relay (Utility Grid)
        gpio_set_level(MAINS_RELAY_PIN, 1); // Turns OFF the relay(SSR) by setting the transistor base HIGH
        gpio_set_level(MAINS_CONTACTOR_PIN, 1); // Turns OFF the relay(SSR) by setting the transistor base HIGH

        // Wait for another 100ms
        vTaskDelay(pdMS_TO_TICKS(100));

        // Turn off TRIAC
        gpio_set_level(MAINS_TRIAC_PIN, 0);

        maincharging_flag  = 0 ;
        ESP_LOGI(" NIGHT > ", "EXIT FROM UPS Mode flag set to %d", maincharging_flag);
        ESP_LOGI(" NIGHT > "," SUCESSFULLY EXIT FROM STATE 5");

    }  
}




void check_current() /// For testing  purpose only .Measure current in pac1952
{
    
	gpio_set_level(SOLAR_SWITCH_PIN, 1);
    gpio_set_level(SOLAR_RELAY_PIN, 1);
    
}

bool check_state_change(device_state_t current_state, device_state_t *previous_state, uint32_t *State_change_time, int *Maintain_State)
{
    
    bool state_changed = (current_state != *previous_state);
    if (state_changed) {
        printf("STATE CHANGE OCCURRED: Previous state = %d, New state = %d\n", *previous_state, current_state);
        *Maintain_State = 1;
        *State_change_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    
    if (*Maintain_State == 1)
    {
        uint32_t time_diff_maintain_state = (xTaskGetTickCount() * portTICK_PERIOD_MS) - *State_change_time;
        if (time_diff_maintain_state > 5000)
        {
            *Maintain_State = 0;
            printf("Maintain the state for 5 seconds is completed\n");
        }
    }

    
    *previous_state = current_state;

    
    return state_changed;
}




// RTOS Task to manage LED blinking
void led_blink_task(void *pvParameter) {
    while (true) {
        
        if (maincharging_flag == 1) {
            blink_led(LED_BLUE_PIN, 1000); // Blink Blue every 1 second
            //printf("blink BLUE LED every 1 Sec\n");
        } else if (solarcharging_flag == 1) {
            blink_led(LED_GREEN_PIN, 5000); // Blink Green every 5 seconds
           // printf("blink GREEN LED every 5 Sec\n");
        } else if (solarassisted_flag == 1) {
            blink_led(LED_GREEN_PIN, 1000); // Blink Green every 1 second
            //printf("blink GREEN LED every 1 Sec\n");
        } else if (FT_flag == 1) {
            blink_led(LED_GREEN_PIN, 5000); // Blink Green every 5 seconds
            //printf("blink GREEN LED every 5 Sec\n");
        } else {
            
            if (current_led != -1) {
                led_off(current_led);
                current_led = -1; // Reset current LED
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}


// Function to delete the LED blink task
/*void delete_led_blink_task() {
    if (led_blink_task_handle != NULL) {
        vTaskDelete(led_blink_task_handle);
        led_blink_task_handle = NULL; // Reset the handle after deletion
    }
}*/




// Define a flag for temp
volatile bool Temp_High = false;
volatile bool Temp_VERY_High = false;


bool update_device_state(float battery_voltage, float solar_voltage, float solar_current, float mosfet_temp)
{


    device_state_t current_state;
    static uint32_t state_toggle_time = 0;
    static bool rapid_toggle_detected = false;
    


    /*///  Total no: of pannels section   ////

    int total_panels = calculate_total_solar_panels(max_voltage_per_panel, total_measured_voltage);

    if (total_panels >= 0) {
        printf("Total number of solar panels connected: %d\n", total_panels);
    }*/

    
    // Track the solar voltage for detecting day and night time
    sun_routine_algorithm(solar_voltage);
    Solar_Relay_Tracking();



    //Day_Tracking(solar_voltage);// Track the solar voltage for detecting day and night time(oldsection)

    if (mosfet_temp >= 75) {
        printf("Temp >= 75 \n");
        Temp_VERY_High = true;
        Temp_High = true;
        fan1_on();
        fan2_on();

    } else if (mosfet_temp >= 70) {
        printf("Temp >= 70 \n");
        Temp_High = true;
        Temp_VERY_High = false;
        fan1_on();
        fan2_on();

    } else if (mosfet_temp < 65) {
         printf("Temp < 65 \n");
        Temp_High = false;
        Temp_VERY_High = false;
        fan1_off();
        fan2_off();
    }

    

   

    // INITIAL STAGE 
    if(!initial_stage && !ft_active_alert)
    {   
        //Solar_available_flag==1 > Confirm solar_relay is ON.If a MOSFET failure occurs, the solar relay should remain off, even during the daytime.
        if(solar_voltage > battery_voltage && Solar_available_flag==1)
        {
            
            printf("initial_stage :solar_voltage > battery_voltage & initial_stage flag set\n");            
            if(Maintain_State==0 && !Temp_VERY_High)
            {
                ESP_LOGI(TAG, "initial_stage :Load supported by Battery + solar energy & battery charged by solar energy.");
                Solar_assisted_battery_charging();  // STATE :7
                initial_stage = true;
            }
            
        }
        else 
        {            
            printf("initial_stage :solar_voltage < battery_voltage & initial_stage flag Reset\n");  
            
            UPS();  // STATE :5
            if(maincharging_flag==1)
            {
                ESP_LOGI(TAG, "initial_stage :Load supported by utility grid & battery charged by utility grid");
                initial_stage = false;

            }
            
        }
    }
    


    // SECOND STAGE 
    if(initial_stage && !batteryHigherThanMaxThreshold)
    {
        printf("\n");
        if (battery_voltage > BATTERY_MAX_THRESHOLD)
        {
            batteryHigherThanMaxThreshold = true ;
            printf("2nd STAGE :battery_voltage > BATTERY_MAX_THRESHOLD \n");          
            
            if(FT_flag==0)
            {
               
                FT_mode_goes = true ;
                ESP_LOGI(TAG, "2nd stage : FT_mode_goes  = %s", FT_mode_goes ? "true" : "false");
                printf("Goes to FT mode in 2nd stage flag set\n");
                /*Force_Trip();// STATE : 4

                if(FT_flag==1)
                {
                    Maintain_State = 1 ;
                    State_change_time = xTaskGetTickCount()* portTICK_PERIOD_MS;
                    printf("In 2nd stage :5 second timer starts in STATE : 4\n");
                    FT_mode_goes = false ;
                }*/
              
            } 
           
            printf("2nd STAGE : batteryHigherThanMaxThreshold set hence goes to 3rd section\n");
                
            
                      
            
        }
        else
        {
            batteryHigherThanMaxThreshold = false ;
            printf("2nd STAGE :battery_voltage < BATTERY_MAX_THRESHOLD & batteryHigherThanMaxThreshold flag Not_Set\n");
            if (solarassisted_flag == 1) 
            { 
                uint32_t time_diff =(xTaskGetTickCount()* portTICK_PERIOD_MS) - current_check_time;
                ESP_LOGI("STATE:7", "Checking time difference: %lu", time_diff);
                
                if (time_diff > 2000) // Check if 2 seconds have passed
                {
                    check_mosfet_failure(solar_current);
                    if(!mosfet_failure_flag)
                    {
                        ESP_LOGI(TAG, "Going to Measure current");
                        if (solar_current < SOLAR_CURRENT_THRESHOLD)
                        {
                            ESP_LOGI(TAG, "Going to change mode after current monitoring");
                            ESP_LOGI(TAG, "2nd STAGE :solar_current < SOLAR_CURRENT_THRESHOLD");
                       
                            UPS();  // STATE :5
                            if(maincharging_flag==1)
                            {
                                ESP_LOGI(TAG, "2nd STAGE :Load supported by utility grid & battery charged by utility grid.");
                            
                                initial_stage = false ; //Goes to initial stage
                            }
                        
                        }
                        else
                        {
                                              
                            if(Maintain_State==0 && !Temp_VERY_High)
                            {
                                ESP_LOGI(TAG, "2nd STAGE :Load supported by Battery + solar energy & battery charged by solar energy.");
                                Solar_assisted_battery_charging();  // STATE :7
                            }
                       
                        }
                    }
                   
                   
                    current_check_time = xTaskGetTickCount()* portTICK_PERIOD_MS;  // Reset check time
                }   
            }
        }

    }



    //THIRD STAGE
    if(batteryHigherThanMaxThreshold && !batteryHigherThanMinThreshold)
    {
        printf("\n");
        ESP_LOGI(TAG, "3rd STAGE : batteryHigherThanMaxThreshold section of code");
        
        if(FT_mode_goes) 
        {  
                                
            printf("Goes to ACTIVATE FT mode in 3rd stage\n");
            Force_Trip();// STATE : 4
                
            if(FT_flag==1)
            {
                Maintain_State = 1 ;
                State_change_time = xTaskGetTickCount()* portTICK_PERIOD_MS;
                printf("In 3rd stage : 5 second timer starts in STATE : 4\n");
                FT_mode_goes = false ;
            }

            //// Retrun For collecting latest values : after goes to ft mode  
            current_state = state_change_occurs(); 
            ESP_LOGI(TAG, "Current state: %d", current_state);

            bool state_changed = (current_state != previous_state);
            if (state_changed) 
            {
                printf("STATE CHANGE OCCURRED: Previous state = %d, New state = %d\n", previous_state, current_state);

                // Check if the state toggles between STATE_FORCE_TRIP (4) and STATE_SOLAR_ASSISTED_FT (6) within 7 seconds
                if ((previous_state == 4 && current_state == 6) || (previous_state == 6 && current_state == 4)) 
                {
               
                    uint32_t time_diff_toggle = (xTaskGetTickCount() * portTICK_PERIOD_MS) - state_toggle_time;

                    if (time_diff_toggle < 8000) { 
                        rapid_toggle_detected = true;  // Toggle happened too fast
                        printf("Rapid toggle detected between STATE :4 and STATE :6 within 8 seconds. State change ignored.\n");
                    } 
                    else {
                        rapid_toggle_detected = false; // Toggle is acceptable
                        printf("No rapid toggling detected, state change is valid.\n");
                    }           
                    state_toggle_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                }else{
                    rapid_toggle_detected = false;   
                }
                
                previous_state = current_state;
            }
    
            if (Maintain_State == 1) 
            {
                uint32_t time_diff_maintain_state = (xTaskGetTickCount() * portTICK_PERIOD_MS) - State_change_time;
                if (time_diff_maintain_state > 5000) 
                {
                    Maintain_State = 0;
                    printf("Maintained the state for 5 seconds. Timer completed.\n");
                }
            }



            // If rapid toggling was detected, return false to ignore the state change
            if (rapid_toggle_detected) {
                printf("Toggling detected, hence returns False\n");
                return false;
            }

            // If no rapid toggling was detected, return true to indicate state change
            return state_changed;
              

        }
        if(FT_flag==1)
        {
            ESP_LOGI(TAG, "3rd STAGE :Load supported by battery & battery not charged(STATE:4)");
        }
            
    
        //Solar_available_flag==1 > Confirm solar_relay is ON.If a MOSFET failure occurs, the solar relay should remain off, even during the daytime.
        if (battery_voltage > BATTERY_LOW_THRESHOLD && solar_voltage > battery_voltage && Solar_available_flag==1 )
        {
            printf("battery_voltage > BATTERY_LOW_THRESHOLD && solar_voltage > battery_voltage\n");
            
            
                ESP_LOGI(TAG, "3rd STAGE : battery_voltage > BATTERY_LOW_THRESHOLD & batteryHigherThanMinThreshold flag set");
           
                if(Maintain_State==0 && !Temp_VERY_High)
                {
                    batteryHigherThanMinThreshold = true ;
                    batteryHigherThanMaxThreshold = false ;
                    ESP_LOGI(TAG, "3rd STAGE : Load supported by Battery + solar energy  & battery charged by solar energy.");
                    Solar_assisted_FT();  // STATE :6
                }
            
                               
        }
        else
        {
            
            ESP_LOGI(TAG, "3rd STAGE : battery_voltage < BATTERY_LOW_THRESHOLD & batteryHigherThanMinThreshold flag Not_Set");
            
            UPS();  // STATE : 5
            if(maincharging_flag==1)
            {
                
                ESP_LOGI(TAG, "3rd STAGE : Load supported by utility grid & battery charged by utility grid.");
                
                batteryHigherThanMinThreshold = false ;
                batteryHigherThanMaxThreshold = false ;
                initial_stage = false ; //Goes to initial stage 
            }
             
                       
        }
        

    }

    
    //4th STAGE
    if(batteryHigherThanMinThreshold)
    {
        printf("\n");
        ESP_LOGI(TAG, "4th STAGE : batteryHigherThanMinThreshold section of code");
        
        if (battery_voltage > BATTERY_MAX_THRESHOLD)
        {
            batteryHigherThanMaxThreshold = true ; // Goes to 3rd stage 
            batteryHigherThanMinThreshold = false;
            printf("4th STAGE :battery_voltage > BATTERY_MAX_THRESHOLD & batteryHigherThanMaxThreshold flag set\n");

            if(FT_flag==0)
            {
               FT_mode_goes = true ;
               ESP_LOGI(TAG, "4th stage : FT_mode_goes = %s", FT_mode_goes ? "true" : "false");
               printf("Goes to FT mode in 4th stage flag set\n");
               
            }
            
        }
        else
        {
            if (battery_voltage > BATTERY_LOW_THRESHOLD)
            {
                printf("4th STAGE :battery_voltage > BATTERY_LOW_THRESHOLD\n");
                if (solarcharging_flag  == 1) 
                { 
                    uint32_t time_diff =(xTaskGetTickCount()* portTICK_PERIOD_MS) - current_check_time;
                    ESP_LOGI("STATE:6", "Checking time difference: %lu", time_diff);
                
                    if (time_diff > 2000) // Check if 2 seconds have passed
                    {                        
                        check_mosfet_failure(solar_current);
                        if(!mosfet_failure_flag)
                        {
                            ESP_LOGI(TAG, "Going to Measure current");
                            if (solar_current < SOLAR_CURRENT_THRESHOLD)
                            {
                                ESP_LOGI(TAG, "Going to change mode after current monitoring");
                                ESP_LOGI(TAG, "4th STAGE :solar_current < SOLAR_CURRENT_THRESHOLD");
                            
                                UPS();  // STATE : 5
                                if(maincharging_flag==1)
                                {
                                    ESP_LOGI(TAG, "4th STAGE :Load supported by utility grid & battery charged by utility grid.");
                               
                                    initial_stage = false ; //Goes to initial stage 
                                    batteryHigherThanMinThreshold = false ;
                                }
                            
                          
                            }
                            else
                            {
                                                        
                                if(Maintain_State==0 && !Temp_VERY_High)
                                {
                                    ESP_LOGI(TAG, "4th STAGE :Load supported by Battery + solar energy  & battery charged by solar energy.");
                                    Solar_assisted_FT();  // STATE :6
                                }
                            
                            }
                        }
                        
                        
                        current_check_time = xTaskGetTickCount()* portTICK_PERIOD_MS;  // Reset check time
                        

                    }   
                }

            }
            else
            {
                printf("4th STAGE :battery_voltage < BATTERY_LOW_THRESHOLD\n");

                UPS();  // STATE : 5            
                if(maincharging_flag==1)
                {
                    ESP_LOGI(TAG, "4th STAGE :Load supported by utility grid & battery charged by utility grid.");
                    
                    initial_stage = false ; //Goes to initial stage
                    batteryHigherThanMinThreshold = false ;

                }
                
                 
            }

        }
    }




 
    //    *  Float condition at night time  *     //
    check_night_mode(battery_voltage);
    
    

    // Check main charging flag and handle main failure
    monitor_main_failure(battery_voltage);
    

    // Check for battery voltage conditions
    /*if (main_failure_flag && battery_voltage < BATTERY_LOW_THRESHOLD) 
    {
        printf("Battery Disconnected !!!!\n");
        battery_disconnect_flag = true;
    } 
    else 
    {
        battery_disconnect_flag = false;
    }*/




    //  Temperature HIGH section :  //

    if(Temp_VERY_High)
    {
        if(battery_voltage > BATTERY_MAX_THRESHOLD)
        {
            batteryHigherThanMaxThreshold = true ; // Goes to 3rd stage 
            batteryHigherThanMinThreshold = false;
           
            printf("HIGH SSR TEMP : Battery_voltage > BATTERY_MAX_THRESHOLD & batteryHigherThanMaxThreshold flag set\n");
            printf("going to OFF MOSFET due to HIGH TEMPERATURE >= 75\n");
            if(FT_flag==0)
            {
               FT_mode_goes = true ;
               ESP_LOGI(TAG, "HIGH SSR TEMP : FT_mode_goes = %s", FT_mode_goes ? "true" : "false");
               
               
            }  
            
        }
        else
        {
            initial_stage = false ;
            UPS();  // STATE : 5
            if(maincharging_flag==1)
            {
                
                ESP_LOGI(TAG, "HIGH SSR TEMP : Load supported by utility grid & battery charged by utility grid.");
                printf("MOSFET OFF due to HIGH TEMPERATURE >= 75\n");
                batteryHigherThanMinThreshold = false ;
                batteryHigherThanMaxThreshold = false ;
                initial_stage = false ; //Goes to initial stage 
            }
        }           

    }
    
    //Maintain at  FT mode After pac_alert 
    if(ft_active_alert && FT_flag==1)
    {
           
        printf("Maintained in FT Mode !!!!\n");
               
        if (battery_voltage < BATTERY_MAX_THRESHOLD)
        {
            printf("battery_voltage < BATTERY_MAX_THRESHOLD\n");
            ft_active_alert = false ;
            initial_stage = false ;
            printf("Going to check main flow flowchart!!!\n");
            
        }else{
            printf("battery_voltage >= BATTERY_MAX_THRESHOLD\n");
        }
    }


   
    ////      To avoid data publish during state change toggling     ////
           
    current_state = state_change_occurs(); 
    ESP_LOGI(TAG, "Current state: %d", current_state);

    bool state_changed = (current_state != previous_state);
    if (state_changed) 
    {
        printf("STATE CHANGE OCCURRED: Previous state = %d, New state = %d\n", previous_state, current_state);

        // Check if the state toggles between STATE_FORCE_TRIP (4) and STATE_SOLAR_ASSISTED_FT (6) within 7 seconds
        if ((previous_state == 4 && current_state == 6) || 
        (previous_state == 6 && current_state == 4)) 
        {
        
        
            uint32_t time_diff_toggle = (xTaskGetTickCount() * portTICK_PERIOD_MS) - state_toggle_time;

            if (time_diff_toggle < 8000) { 
                rapid_toggle_detected = true;  // Toggle happened too fast
                printf("Rapid toggle detected between STATE :4 and STATE :6 within 8 seconds. State change ignored.\n");
            } 
            else {
                rapid_toggle_detected = false; // Toggle is acceptable
                printf("No rapid toggling detected, state change is valid.\n");
            }
        
            
            state_toggle_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }else{
            rapid_toggle_detected = false;   
        }

        // Check if we entered STATE 5 and maintain it for 5 seconds
        if (current_state == 5) 
        {
            Maintain_State = 1;
            State_change_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            printf("5 Second timer starts in STATE: 5!! \n");
        }

    
        previous_state = current_state;
    }



    
    if (Maintain_State == 1) {
        uint32_t time_diff_maintain_state = (xTaskGetTickCount() * portTICK_PERIOD_MS) - State_change_time;
        if (time_diff_maintain_state > 5000) 
        {
            Maintain_State = 0;
            printf("Maintained the state for 5 seconds. Timer completed.\n");
        }
    }



    // If rapid toggling was detected, return false to ignore the state change
    if (rapid_toggle_detected) {
        printf("Toggling detected, hence returns False\n");
        return false;
    }



    // If no rapid toggling was detected, return true to indicate state change
    return state_changed;
   
    
}











