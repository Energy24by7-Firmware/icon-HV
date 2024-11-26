//void func(void);
#ifndef SAMPLES_READ_H
#define SAMPLES_READ_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_err.h"




// GPIO Pins
#define REFERENCE_VOLTAGE_PIN     GPIO_NUM_39  // Reference voltage sensing
#define SOLAR_VOLTAGE_PIN         GPIO_NUM_36  // Solar voltage sensing from ADC
#define BATTERY_VOLTAGE_PIN       GPIO_NUM_37  // Battery voltage sensing from ADC
#define POWER_SUPPLY_VOLTAGE_PIN  GPIO_NUM_38  // Power supply voltage
#define SOLAR_HEAT_SINK_PIN       GPIO_NUM_34  // Solar heat sink temperature sensing
#define INTERNAL_TEMPERATURE_PIN  GPIO_NUM_35  // Internal temperature sensing
#define SSR_TEMPERATURE_PIN       GPIO_NUM_2   // SSR temperature sensing

extern uint8_t adc_all_buff[16];
extern float avrg_bat_volt ;
extern float avrg_sol_amp ;
extern float avrg_sol_volt ;

// Function prototypes
void samples_read_init(void);


float get_reference_voltage(void);
void save_reference_voltage_to_eeprom(float reference_voltage);
float load_reference_voltage_from_eeprom(void);


float get_solar_voltage(float reference_voltage);
float get_battery_voltage(float reference_voltage);
float get_power_supply_voltage(float reference_voltage);
float get_solar_heat_sink_temperature(float reference_voltage);
float get_internal_temperature(float reference_voltage);
float get_ssr_temperature(float reference_voltage);

// Function prototypes

void mark_as_i2c_in_use();
void mark_as_i2c_free();
uint8_t if_i2c_bus_is_busy();
int8_t read_adc_all(uint8_t *adc_all_buff);
float get_bat_vol();
float get_sol_amp();
float get_sol_volt();


#endif // SAMPLES_READ_H
