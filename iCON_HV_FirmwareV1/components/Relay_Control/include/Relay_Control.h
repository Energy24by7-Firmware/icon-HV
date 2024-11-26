#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"



// GPIO pin definitions

#define SOLAR_RELAY_PIN GPIO_NUM_21
#define SOLAR_SWITCH_PIN GPIO_NUM_22

#define MAINS_RELAY_PIN GPIO_NUM_23
#define MAINS_CONTACTOR_PIN GPIO_NUM_13
#define MAINS_TRIAC_PIN GPIO_NUM_15

#define PAC_ALERT_PIN GPIO_NUM_25

//extern bool state_changed = false ;

//// Declare it as extern
extern bool battery_disconnect_flag ; 

extern TaskHandle_t monitorTaskHandle ;
extern TaskHandle_t alertTaskHandle ;
extern TaskHandle_t led_blink_task_handle ;
extern SemaphoreHandle_t alert_flag_mutex;
  

// Define thresholds
#define BATTERY_MAX_THRESHOLD 260 //in practical case for HV Side : 260
#define BATTERY_LOW_THRESHOLD 240 //in practical case for HV Side : 240 
#define SOLAR_CURRENT_THRESHOLD 0.5 
#define VOLTAGE_THRESHOLD_SOLAR 20  //in practical case 15// minimun solar voltage to turn on solar relay in day time.


#define SOL_HIGH_THR 20.0           // Example threshold for sunup detection (voltage in volts)
#define SOL_LOW_H_THR 10   // Upper threshold for sundown detection
#define SOL_LOW_L_THR  2   // Lower threshold for sundown detection
//#define SOLAR_RELAY_OFF_DELAY_MS (15 * 60 * 1000000) // 15 minutes in microseconds
//#define SOLAR_RELAY_OFF_DELAY_MS (15 * 60 * 1000)  // 15 minutes in milliseconds

// Define the enumeration for device states

typedef enum {
    STATE_NONE = 0,
    STATE_FORCE_TRIP = 4,
    STATE_UPS = 5,
    STATE_SOLAR_ASSISTED_FT = 6,
    STATE_SOLAR_ASSISTED_BATTERY_CHARGING = 7
} device_state_t;

extern device_state_t previous_state;
extern int state;


// Function declarations

void IRAM_ATTR alert_isr_handler();
void pac_alert_occurs(void *pvParameter);
void pac_alert_pin_init(void);
void monitor_overvoltage_task(void *pvParameter);



void relay_init(void);
void Day_Tracking(float voltage);
void check_mosfet_failure(float solar_current_now );
device_state_t state_change_occurs(void);

void Force_Trip(void);
void UPS(void);
void Solar_assisted_FT(void);
void Solar_assisted_battery_charging(void);
void Off_ups_at_night();
void main_failure(float current_batvoltage);

void led_blink_task(void *pvParameter);
bool update_device_state(float battery_voltage, float solar_voltage, float solar_current, float mosfet_temp);

void manageBatteryState(float batteryVoltage , float solarVoltage,float solar_current);
void check_current(void);



#endif // RELAY_CONTROL_H
