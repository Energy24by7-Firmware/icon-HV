#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>  // Include this for standard boolean type
#include "freertos/FreeRTOS.h"  // Add this for TaskHandle_t
#include "freertos/task.h"       // Add this for xTaskCreate and task management

extern float solar_voltage_calibration_constant;
extern float solar_current_calibration_constant;
extern float battery_voltage_calibration_constant;
extern float temp_sensor_calibration_constant;

extern bool recalibration_flag;  
extern bool Calibration_Flag;

// Task Handles declarations
extern TaskHandle_t calibrationTaskHandle;



// Declare the functions to be used in main.c
void save_calibration_flag(bool flag);
bool read_calibration_flag(void);
void set_recalibration_flag(void);
//void fetch_calibration_constants();  fetch calibration constant from the server
void perform_calibration(void);
void store_calibration_constants(float solar_voltage_const, float solar_current_const, float battery_voltage_const, float temperature_const);
void read_calibration_constants(float *solar_voltage_const, float *solar_current_const, float *battery_voltage_const, float *temperature_const);
void Calibration_task(void *pvParameter);
#endif // CALIBRATION_H
