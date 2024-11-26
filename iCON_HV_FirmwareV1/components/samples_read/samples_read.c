#include <stdio.h>
#include "esp_err.h"
#include <unistd.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/adc.h"


#include "samples_read.h"
#include "esp32s2_pac1952.h"
#include "esp32s2_i2c_protocol.h"
#include "esp32s2_fatal_err.h"
#include "calibration.h"
#include "gpio_control.h"




uint8_t i2cFlag=0;
#define pac_v_bus_const 0.000488 //  32 /65535

/*
65536: Represents 2^16, the number of possible values for a 16-bit ADC (0-65535).
Vbus_ch1 / 65536: Normalizes the raw ADC value to a range between 0 and 1. Essentially, this is converting the raw ADC value into a fraction of the full-scale range.
32.0: Assumes that the maximum measurable voltage is 32V. So if the normalized value is 1, it represents 32V.
*/

#define pac_v_sense_const 0.00000305176 // 0.2/65535 = 3.05176×10^6  ≈ 3.05176μV (+100mili volt to -100 milli volt in full scale defection.so 200mv in full sacle deflection)


#define PAC_BUSY_MAX_CNT 10
uint8_t byteSize = 16;  // Define byteSize and assign value
uint8_t adc_all_buff[16];
float avrg_bat_volt = 0.00;
float avrg_sol_amp = 0.00 ;
float avrg_sol_volt =0.00 ;



// Tag for logging
static const char *TAG1 = "ADC_READING";

static const char *TAG2 = "PAC1952_READING";

// ADC1 Channels corresponding to GPIOs

#define ADC1_CHANNEL_SOLAR_VOLTAGE        ADC1_CHANNEL_0  // GPIO36
#define ADC1_CHANNEL_BATTERY_VOLTAGE      ADC1_CHANNEL_1  // GPIO37 
#define ADC1_CHANNEL_POWER_SUPPLY_VOLTAGE ADC1_CHANNEL_2  // GPIO38
#define ADC1_CHANNEL_REFERENCE_VOLTAGE    ADC1_CHANNEL_3  // GPIO39
#define ADC1_CHANNEL_SOLAR_HEAT_SINK      ADC1_CHANNEL_6  // GPIO34
#define ADC1_CHANNEL_INTERNAL_TEMP        ADC1_CHANNEL_7  // GPIO35
//#define ADC1_CHANNEL_SSR_TEMP             ADC2_CHANNEL_2  // GPIO2 {doubt}



////////  ADC SECTION   //////////////

// ADC1 Channels
#define ADC1_CHANNEL_SOLAR_VOLTAGE        ADC1_CHANNEL_0  // GPIO36
#define ADC1_CHANNEL_BATTERY_VOLTAGE      ADC1_CHANNEL_1  // GPIO37
#define ADC1_CHANNEL_REFERENCE_VOLTAGE    ADC1_CHANNEL_3  // GPIO39
#define ADC1_CHANNEL_SOLAR_HEAT_SINK      ADC1_CHANNEL_6  // GPIO34
#define ADC1_CHANNEL_INTERNAL_TEMP        ADC1_CHANNEL_7  // GPIO35
#define ADC1_CHANNEL_POWER_SUPPLY_VOLTAGE ADC1_CHANNEL_2  // GPIO38

// ADC2 Channel
#define ADC2_CHANNEL_SSR_TEMP             ADC2_CHANNEL_2  // GPIO2

// Function to check ADC initialization and configuration status
esp_err_t adc_check_config(adc1_channel_t channel, const char *channel_name) {
    esp_err_t status = adc1_config_channel_atten(channel, ADC_ATTEN_DB_11); // Use ADC_ATTEN_DB_12
    if (status == ESP_OK) {
        ESP_LOGI(TAG1, "%s configured successfully on ADC1", channel_name);
    } else {
        ESP_LOGE(TAG1, "Failed to configure %s on ADC1", channel_name);
    }
    return status;
}

// Function to read ADC1 values
int read_adc1_value(adc1_channel_t channel) {
    return adc1_get_raw(channel);
}

// Function to read ADC2 values
int read_adc2_value(adc2_channel_t channel) {
    int raw_value;
    esp_err_t status = adc2_get_raw(channel, ADC_WIDTH_BIT_12, &raw_value);
    return (status == ESP_OK) ? raw_value : -1; // Return -1 on error
}

// Function to get voltage for ADC1 channels
float get_adc1_voltage(adc1_channel_t channel, const char *channel_name) {
    int raw_value = read_adc1_value(channel); 
    
    float voltage = 0.0; 
    if (raw_value >= 0) {
        
        //ESP_LOGI(TAG1, "Raw ADC Value for %s: %d", channel_name, raw_value);

        // Calculate voltage (Assuming 3.3V reference and 12-bit ADC)
        voltage = (raw_value * 3.3) / 4095; 
        //ESP_LOGI(TAG1, "%s Voltage at adc pin : %.2f V", channel_name, voltage);
    } else {
        ESP_LOGE(TAG1, "Failed to read %s", channel_name);
    }
    return voltage; 
}


// Function to get voltage for ADC2 channels
float get_adc2_voltage(adc2_channel_t channel, const char *channel_name) {
    int raw_value = read_adc2_value(channel);
    float voltage = 0.0; 
    if (raw_value >= 0) {

        //ESP_LOGI(TAG1, "Raw ADC Value for %s: %d", channel_name, raw_value);

        // Calculate voltage (Assuming 3.3V reference and 12-bit ADC)
        voltage = (raw_value * 3.3) / 4095; 
        //ESP_LOGI(TAG1, "%s Voltage: %.2f V", channel_name, voltage);
    } else {
        //ESP_LOGE(TAG1, "Failed to read %s", channel_name);
        ESP_LOGE(TAG1, "Failed to read Temp3(GPIO 2)");
    }
    return voltage; 
}

// Initialize ADC channels and NVS
void samples_read_init(void) {
    printf("ADC initialization");

    // Set ADC width for ADC1 and ADC2
    adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit width for ADC1

   
    //adc2_config_width(ADC_WIDTH_BIT_12);  // 12-bit width for ADC2
    // Note: adc2_config_width is not needed; you directly configure the ADC2 channel's attenuation
    // Configure ADC2 Channel (SSR Temp on GPIO2)
    esp_err_t status = adc2_config_channel_atten(ADC2_CHANNEL_SSR_TEMP, ADC_ATTEN_DB_11); // Use ADC_ATTEN_DB_12
    if (status == ESP_OK) {
        ESP_LOGI(TAG1, "SSR Temp channel configured successfully on GPIO2");
    } else {
        ESP_LOGE(TAG1, "Failed to configure SSR Temp channel on GPIO2");
    }


    // Configure ADC1 Channels
    adc_check_config(ADC1_CHANNEL_SOLAR_VOLTAGE, "Solar Voltage (GPIO36)");
    adc_check_config(ADC1_CHANNEL_BATTERY_VOLTAGE, "Battery Voltage (GPIO37)");
    adc_check_config(ADC1_CHANNEL_POWER_SUPPLY_VOLTAGE, "Power Supply Voltage (GPIO38)");
    adc_check_config(ADC1_CHANNEL_REFERENCE_VOLTAGE, "Reference Voltage (GPIO39)");
    adc_check_config(ADC1_CHANNEL_SOLAR_HEAT_SINK, "Solar Heat Sink (GPIO34)");
    adc_check_config(ADC1_CHANNEL_INTERNAL_TEMP, "Internal Temp (GPIO35)");
   
}


// Function to get the reference voltage from GPIO39
float get_reference_voltage(void) {
    
    float reference_voltage = get_adc1_voltage(ADC1_CHANNEL_REFERENCE_VOLTAGE, "Reference Voltage (GPIO39)");
    ESP_LOGI(TAG1, "Reference Voltage: %.2f V", reference_voltage);
    return reference_voltage;
}

// Save the reference voltage to EEPROM
void save_reference_voltage_to_eeprom(float reference_voltage) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_set_blob(nvs_handle, "ref_voltage", &reference_voltage, sizeof(reference_voltage));
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG1, "Reference voltage saved to EEPROM");
    } else {
        ESP_LOGE(TAG1, "Failed to open NVS");
    }
}

// Load the reference voltage from EEPROM
float load_reference_voltage_from_eeprom(void) {
    nvs_handle_t nvs_handle;
    float reference_voltage = 0;
    size_t required_size = sizeof(reference_voltage);

    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        ret = nvs_get_blob(nvs_handle, "ref_voltage", &reference_voltage, &required_size);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG1, "Reference voltage loaded from EEPROM");
        } else {
            ESP_LOGE(TAG1, "Failed to load reference voltage from EEPROM");
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG1, "Failed to open NVS");
    }
    ESP_LOGI(TAG1, "Reference Voltage: %.2f V", reference_voltage);
    return reference_voltage;
}



// Function to get the solar voltage
float get_solar_voltage(float reference_voltage) {
  
    float solar_voltage = get_adc1_voltage(ADC1_CHANNEL_SOLAR_VOLTAGE, "Solar Voltage (GPIO36)");
    float V_in = (solar_voltage * 164.63)+18; // solar_voltage_calibration_constant;
    //ESP_LOGI(TAG1, "Solar Voltage : %.2f V", V_in);
    return V_in ;
}

// Function to get the Battery voltage
float get_battery_voltage(float reference_voltage) {

    float battery_voltage = get_adc1_voltage(ADC1_CHANNEL_BATTERY_VOLTAGE, "Battery Voltage (GPIO37)");
    float V_in = (battery_voltage * 160)+18 ; //battery_voltage_calibration_constant ;
    //ESP_LOGI(TAG1, "Battery Voltage: %.2f V", V_in);
    return V_in;
}

// Function to get the power supply voltage
float get_power_supply_voltage(float reference_voltage) {

    float power_supply = get_adc1_voltage(ADC1_CHANNEL_POWER_SUPPLY_VOLTAGE, "Power Supply Voltage (GPIO38)");
    float V_in = power_supply *  4.898;
    //ESP_LOGI(TAG1, "Power Supply Voltage: %.2f V", V_in);
    return V_in;
}

// Function to get the solar heat sink temperature (using ADC)
float get_solar_heat_sink_temperature(float reference_voltage) {
    
    float solar_heat_sink_voltage = get_adc1_voltage(ADC1_CHANNEL_SOLAR_HEAT_SINK, "Solar Heat Sink (GPIO34)");
    int degree_1 = solar_heat_sink_voltage * 35.67;//temp_sensor_calibration_constant ;
    //ESP_LOGI(TAG1, "Temp:1 > Solar Heat Sink Temperature:%d °C", degree_1);

    if(degree_1 >=70)
    {
        fan1_on();
        fan2_on();
    }
    else if(degree_1 < 65)
    {
        fan1_off();
        fan2_off();
    }
    
    return degree_1;
}

// Function to get the internal temperature (using ADC)
float get_internal_temperature(float reference_voltage) {
    
    float internal_temp_voltage = get_adc1_voltage(ADC1_CHANNEL_INTERNAL_TEMP, "Internal Temp (GPIO35)");
    int degree_2 = internal_temp_voltage * 35.67;//temp_sensor_calibration_constant ;
    //ESP_LOGI(TAG1, "Temp:2 > Internal Temperature:%d °C", degree_2);

    if(degree_2 >=70)
    {
        fan1_on();
        fan2_on();
    }
    else if(degree_2 < 65)
    {
        fan1_off();
        fan2_off();
    }

    return degree_2;
}

// Function to get the SSR temperature (using ADC)
float get_ssr_temperature(float reference_voltage) {
    float SSR_temp_voltage = get_adc2_voltage(ADC2_CHANNEL_SSR_TEMP, "SSR Temp (GPIO2)");
    int degree_3 = SSR_temp_voltage * 35.67; //temp_sensor_calibration_constant ;
    //ESP_LOGI(TAG1, "Temp : 3 > SSR Temperature:%d °C", degree_3);
    return degree_3;
}





/////   i2c SECTION    //////

void mark_as_i2c_in_use()
{
    i2cFlag = 1;
}

void mark_as_i2c_free()
{
    i2cFlag = 0;
}

uint8_t if_i2c_bus_is_busy()
{
    return i2cFlag;
}


/*//ORG_CODE
// Function to read all PAC1952 sensor data average
int8_t read_adc_all(uint8_t *adc_all_buff) {
    uint16_t log_count = 0;
    uint8_t reset_count = 0;
    static uint8_t pac_busy_count = 0;
    uint16_t uint16_pac_reg = 0; // Changed int16_t to uint16_t
    
    while(if_i2c_bus_is_busy() ) {
        vTaskDelay(10/portTICK_PERIOD_MS);
        log_count++;
        if(log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG2,"[%s] I2C bus busy",__func__);
            log_count = 0;
            reset_count++;
        }
        if(reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI(TAG2,"Calling fatal error handler,since i2c bus is busy");
            fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
        }
    };

    mark_as_i2c_in_use();
    if( pac1952_busy() != 0) {
        ESP_LOGI(TAG2,"----------------------------------------------------");
        ESP_LOGI(TAG2,"Error reading PAC1953 sensor, since i2c bus is busy");
        ESP_LOGI(TAG2,"----------------------------------------------------");
        pac_busy_count++;
        mark_as_i2c_free();
        if(pac_busy_count >= PAC_BUSY_MAX_CNT) {
            ESP_LOGI(TAG2,"Calling fatal error handler,since PAC1952 is busy");
            fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
        }
        return 1;
    }*/

    ///// Edited section in my code 
    /*while (if_i2c_bus_is_busy()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        log_count++;
        if (log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG2, "[%s] I2C bus busy", __func__);
            log_count = 0;
            reset_count++;
        }
        if (reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI(TAG2, "Calling fatal error handler, since I2C bus is busy");
            fatal_err_handler(I2C_MODULE, I2C_BUS_IS_BUSY);
            return -1;
        }
    }

    mark_as_i2c_in_use();
    if (pac1952_busy() != 0) {
        pac_busy_count++;
        mark_as_i2c_free();
        if (pac_busy_count >= PAC_BUSY_MAX_CNT) {
            fatal_err_handler(I2C_MODULE, I2C_BUS_IS_BUSY);
            return -2;
        }
        return -2;
    }
    pac_busy_count = 0;*/
    //vTaskDelay(10/portTICK_PERIOD_MS); // added 1ms delay 

    /*uint16_t pac_stat = PAC194x5x_GetVIP_digital(pPACdevice, adc_all_buff, 16);

    if (pac_stat != 0) {
        mark_as_i2c_free();
        return -3;
    }
    mark_as_i2c_free();
    
    // Retrieve Vsense1 and Vsense2 (16-bit values)
int16_t Vsense1 = (int16_t)((adc_all_buff[0] << 8) | adc_all_buff[1]);
int16_t Vsense2 = (int16_t)((adc_all_buff[2] << 8) | adc_all_buff[3]);

// Retrieve VBus1 and VBus2 (16-bit values)
int16_t VBus1 = (int16_t)((adc_all_buff[4] << 8) | adc_all_buff[5]);
int16_t VBus2 = (int16_t)((adc_all_buff[6] << 8) | adc_all_buff[7]);

// If the byteSize is 8, stop here (optional based on your logic)
if (byteSize == 8) {
    // Values retrieved are only Vsense1, Vsense2, VBus1, and VBus2
    return 0;
}

// Retrieve VPower1 and VPower2 (32-bit signed values)
//int32_t VPower1 = (int32_t)((adc_all_buff[8] << 24) | (adc_all_buff[9] << 16) | (adc_all_buff[10] << 8) | adc_all_buff[11]);
//int32_t VPower2 = (int32_t)((adc_all_buff[12] << 24) | (adc_all_buff[13] << 16) | (adc_all_buff[14] << 8) | adc_all_buff[15]);

// Now you can use these values (Vsense1, Vsense2, VBus1, VBus2, VPower1, VPower2)

printf("Vsense1 of pac1952(current): %d\n", Vsense1);
printf("Vsense2 of pac1952: %d\n", Vsense2);
printf("VBus1 of pac1952(Battery): %d\n", VBus1);
printf("VBus2 of pac1952(Solar): %d\n", VBus2);

//printf("VPower1: %ld\n", VPower1);
//printf("VPower2: %ld\n", VPower2);


//Battery Voltage
avrg_bat_volt = (float)VBus1 *pac_v_bus_const *160; //battery_voltage_calibration_constant ;
//ESP_LOGI(TAG2,"Battery_voltage from PAC 1952 : %.2f V\n", avrg_bat_volt);  


//Solar voltage
//VBus = Vin* (R2/R1+R2) ; Vbus == vdrop , Vin = solar voltage
//Vin = VBus * (R1+R2 /R2) > Calibration constant = (R1+R2 /R2)

avrg_sol_volt = (float)VBus2 *pac_v_bus_const * 164.63;
//ESP_LOGI(TAG2,"Solar_voltage from PAC 1952 : %.2f V\n", avrg_sol_volt); // solar_voltage_calibration_constant;


//CURRENT
//Calibration constant : 0.000287  >>> Rsense = 300 micro ohm in hardware
//avrg_sol_amp = (Vsense1* pac_v_sense_const ) /0.000287; // 0.07 >value as per calibration (i = v/r ; r= unknown , v = get from vsense value , i= shown in powersupply when battery charging start / a motor is connected across battery)
avrg_sol_amp = (Vsense1* pac_v_sense_const ) /0.000287; // solar_current_calibration_constant
//ESP_LOGI(TAG2,"Solar current from PAC 1952 : %.2f A\n",avrg_sol_amp);



return 0;
}*/


// My logic section
static float prev_battery_voltage = 0.0f;
static float prev_solar_voltage = 0.0f;
static float prev_solar_current = 0.0f;

int8_t read_adc_all(uint8_t *adc_all_buff) {
    uint16_t log_count = 0;
    uint8_t reset_count = 0;
    static uint8_t pac_busy_count = 0;
    uint16_t uint16_pac_reg = 0; 
    
    while(if_i2c_bus_is_busy()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        log_count++;
        if (log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG2, "[%s] I2C bus busy", __func__);
            log_count = 0;
            reset_count++;
        }
        if (reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI(TAG2, "Calling fatal error handler, since I2C bus is busy");
            fatal_err_handler(I2C_MODULE, I2C_BUS_IS_BUSY);
        }
    }

    mark_as_i2c_in_use();
    if (pac1952_busy() != 0) {
        ESP_LOGI(TAG2, "----------------------------------------------------");
        ESP_LOGI(TAG2, "Error reading PAC1952 sensor, since I2C bus is busy");
        ESP_LOGI(TAG2, "----------------------------------------------------");
        pac_busy_count++;
        mark_as_i2c_free();
        if (pac_busy_count >= PAC_BUSY_MAX_CNT) {
            ESP_LOGI(TAG2, "Calling fatal error handler, since PAC1952 is busy");
            fatal_err_handler(I2C_MODULE, I2C_BUS_IS_BUSY);
        }
        return 1;
    }

    uint16_t pac_stat = PAC194x5x_GetVIP_digital(pPACdevice, adc_all_buff, 16);

    if (pac_stat != 0) {
        mark_as_i2c_free();
        return -3;
    }
    mark_as_i2c_free();

    // Retrieve Vsense1 and Vsense2 (16-bit values)
    int16_t Vsense1 = (int16_t)((adc_all_buff[0] << 8) | adc_all_buff[1]);
    int16_t Vsense2 = (int16_t)((adc_all_buff[2] << 8) | adc_all_buff[3]);

    // Retrieve VBus1 and VBus2 (16-bit values)
    int16_t VBus1 = (int16_t)((adc_all_buff[4] << 8) | adc_all_buff[5]);
    int16_t VBus2 = (int16_t)((adc_all_buff[6] << 8) | adc_all_buff[7]);

   // Calculate Battery Voltage
    float current_battery_voltage = (float)VBus1 * pac_v_bus_const * 160;
    if (current_battery_voltage <= 0.0f && prev_battery_voltage <= 0.0f) {
        avrg_bat_volt = 0.0f;
        prev_battery_voltage = avrg_bat_volt; 
    } else if (current_battery_voltage <= 0.0f && prev_battery_voltage > 0.0f) {
        avrg_bat_volt = prev_battery_voltage;
        prev_battery_voltage = current_battery_voltage; // Maintain zero if real voltage is zero
    } else {
        avrg_bat_volt = current_battery_voltage;
        prev_battery_voltage = avrg_bat_volt; 
    }

    // Calculate Solar Voltage
    float current_solar_voltage = (float)VBus2 * pac_v_bus_const * 164.63;
    if (current_solar_voltage <= 0.0f && prev_solar_voltage <= 0.0f) {
       avrg_sol_volt = 0.0f;
       prev_solar_voltage = avrg_sol_volt; 
    } else if (current_solar_voltage <= 0.0f && prev_solar_voltage > 0.0f) {
       avrg_sol_volt = prev_solar_voltage;
       prev_solar_voltage = current_solar_voltage; // Maintain zero if real voltage is zero
    } else {
       avrg_sol_volt = current_solar_voltage;
       prev_solar_voltage = avrg_sol_volt; 
    }

    // Calculate Solar Current   
    float current_solar_current = (Vsense1 * pac_v_sense_const) / 0.000287;

    // Handle zero and negative current values
    if ((current_solar_current <= 0.0f) && (prev_solar_current <= 0.0f)) {
        avrg_sol_amp = 0.0f;  
        prev_solar_current = avrg_sol_amp;
    } else if ((current_solar_current <= 0.0f) && (prev_solar_current > 0.0f)) {
        avrg_sol_amp = prev_solar_current;  
        prev_solar_current = current_solar_current; 
    } else {
        avrg_sol_amp = current_solar_current;  
        prev_solar_current = avrg_sol_amp;    
    }

    
    ESP_LOGI(TAG2, "Battery Voltage: %.2f V", avrg_bat_volt);
    ESP_LOGI(TAG2, "Solar Voltage: %.2f V", avrg_sol_volt);
    ESP_LOGI(TAG2, "Solar Current: %.2f A", avrg_sol_amp);

    return 0;
}


















// Function to get battery voltage from PAC1952
/*float get_bat_vol() {
    uint16_t log_count = 0;
    uint8_t reset_count = 0;
    while (if_i2c_bus_is_busy()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        log_count++;
        if (log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG2, "[%s] I2C bus busy", __func__);
            log_count = 0;
            reset_count++;
        }
        if (reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI(TAG2, "Calling fatal error handler, since I2C bus is busy");
            fatal_err_handler(I2C_MODULE, I2C_BUS_IS_BUSY);
        }
    }
    uint16_t batValue;
    float batVoltage;
    mark_as_i2c_in_use();
    PAC194X5X_getBatteryVoltage(pPACdevice, &batValue); // Assuming channel 1 for battery voltage
    mark_as_i2c_free();
    batVoltage = pac_v_bus_const * batValue;
    batVoltage = batVoltage * battery_voltage_calibration_constant;
    ESP_LOGI(TAG2, "Battery Voltage from pac: %.2f V", batVoltage);
    return batVoltage;
}



// Function to get solar current
float get_sol_amp() {
    uint16_t log_count = 0;
    uint8_t reset_count = 0;
    while (if_i2c_bus_is_busy()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        log_count++;
        if (log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG2, "[%s] I2C bus busy", __func__);
            log_count = 0;
            reset_count++;
        }
        if (reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI(TAG2, "Calling fatal error handler, since I2C bus is busy");
            fatal_err_handler(I2C_MODULE, I2C_BUS_IS_BUSY);
        }
    }
    uint16_t sol_ampValue;
    float solcurrent;
    mark_as_i2c_in_use();
    PAC194X5X_getSolarcurrent(pPACdevice, &sol_ampValue); // Assuming channel 2 for solar current
    mark_as_i2c_free();
    solcurrent = (float)sol_ampValue * solar_current_calibration_constant;
    ESP_LOGI(TAG2, "Solar current from pac: %.2f A", solcurrent);
    return solcurrent;
}



// Function to get solar voltage
float get_sol_volt() {
    uint16_t log_count = 0;
    uint8_t reset_count = 0;
    while (if_i2c_bus_is_busy()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        log_count++;
        if (log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG2, "[%s] I2C bus busy", __func__);
            log_count = 0;
            reset_count++;
        }
        if (reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI(TAG2, "Calling fatal error handler, since I2C bus is busy");
            fatal_err_handler(I2C_MODULE, I2C_BUS_IS_BUSY);
        }
    }
    uint16_t sol_voltValue;
    float solvoltage;
    mark_as_i2c_in_use();
    PAC194X5X_getSolarvoltage(pPACdevice, &sol_voltValue); // Corrected function for solar voltage
    mark_as_i2c_free();
    solvoltage = pac_v_sense_const * sol_voltValue;
    solvoltage = solvoltage * solar_voltage_calibration_constant;
    ESP_LOGI(TAG2, "Solar voltage from pac: %.2f V", solvoltage);
    return solvoltage;
}*/





