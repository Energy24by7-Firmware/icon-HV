  /**
 ****************************************************************************************
 *
 * @file ESP32_PAC195X.h
 *
 * @brief  This is a standard library from microchip for PAC194x/5x family customized to work with ESP32 and ESP32s2.
 *
 * @Author Akshay Das
 * 
 * Copyright (C) 2022-23 Nyquest Innovations Labs.
 * 
 * @Date July 2022
 ****************************************************************************************
 **/


#ifndef _ESP32_PAC195X_H_
#define _ESP32_PAC195X_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_types.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <string.h>

#define PAC194X5X_CLIENT_ADDRESS_1    0x28
#define PAC194X5X_BUS_ID_1            0x01

//#define PAC194X5X_CLIENT_ADDRESS_1    0x14
//#define PAC194X5X_BUS_ID_1            0x54
    
#define RSENSE_MICRO_CH1  300    //micro-ohms
#define RSENSE_MICRO_CH2  300    //micro-ohms
#define RSENSE_MICRO_CH3  300    //micro-ohms
#define RSENSE_MICRO_CH4  300    //micro-ohms

#define VRAIL_VBUS_RATIO_CH1    1.0
#define VRAIL_VBUS_RATIO_CH2    1.0
#define VRAIL_VBUS_RATIO_CH3    1.0
#define VRAIL_VBUS_RATIO_CH4    1.0  

#define PAC194X5X_MAX_CH_COUNT       4

/*
 * PAC194x/5x register address
 */
#define PAC194X5X_REFRESH_ADDR                0x00
#define PAC194X5X_CTRL_ADDR                   0x01
#define PAC194X5X_ACC_COUNT_ADDR              0x02
#define PAC194X5X_VACC1_ADDR                  0x03
#define PAC194X5X_VACC2_ADDR                  0x04
#define PAC194X5X_VACC3_ADDR                  0x05
#define PAC194X5X_VACC4_ADDR                  0x06
#define PAC194X5X_VBUS1_ADDR                  0x07
#define PAC194X5X_VBUS2_ADDR                  0x08
#define PAC194X5X_VBUS3_ADDR                  0x09
#define PAC194X5X_VBUS4_ADDR                  0x0A
#define PAC194X5X_VSENSE1_ADDR                0x0B
#define PAC194X5X_VSENSE2_ADDR                0x0C
#define PAC194X5X_VSENSE3_ADDR                0x0D
#define PAC194X5X_VSENSE4_ADDR                0x0E
#define PAC194X5X_VBUS1_AVG_ADDR              0x0F
#define PAC194X5X_VBUS2_AVG_ADDR              0x10
#define PAC194X5X_VBUS3_AVG_ADDR              0x11
#define PAC194X5X_VBUS4_AVG_ADDR              0x12
#define PAC194X5X_VSENSE1_AVG_ADDR            0x13
#define PAC194X5X_VSENSE2_AVG_ADDR            0x14
#define PAC194X5X_VSENSE3_AVG_ADDR            0x15
#define PAC194X5X_VSENSE4_AVG_ADDR            0x16
#define PAC194X5X_VPOWER1_ADDR                0x17
#define PAC194X5X_VPOWER2_ADDR                0x18
#define PAC194X5X_VPOWER3_ADDR                0x19
#define PAC194X5X_VPOWER4_ADDR                0x1A
#define PAC194X5X_SMBUS_SETTINGS_ADDR         0x1C
#define PAC194X5X_NEG_PWR_FSR_ADDR            0x1D
#define PAC194X5X_REFRESH_G_ADDR              0x1E
#define PAC194X5X_REFRESH_V_ADDR              0x1F
#define PAC194X5X_SLOW_ADDR                   0x20
#define PAC194X5X_CTRL_ACT_ADDR               0x21
#define PAC194X5X_NEG_PWR_FSR_ACT_ADDR        0x22
#define PAC194X5X_CTRL_LAT_ADDR               0x23
#define PAC194X5X_NEG_PWR_FSR_LAT_ADDR        0x24
#define PAC194X5X_ACCUM_CONFIG_ADDR           0x25
#define PAC194X5X_ALERT_STATUS_ADDR           0x26
#define PAC194X5X_SLOW_ALERT1_ADDR            0x27
#define PAC194X5X_GPIO_ALERT2_ADDR            0x28
#define PAC194X5X_ACC_FULLNESS_LIMITS_ADDR    0x29
#define PAC194X5X_OC_LIMIT1_ADDR              0x30
#define PAC194X5X_OC_LIMIT2_ADDR              0x31
#define PAC194X5X_OC_LIMIT3_ADDR              0x32
#define PAC194X5X_OC_LIMIT4_ADDR              0x33
#define PAC194X5X_UC_LIMIT1_ADDR              0x34
#define PAC194X5X_UC_LIMIT2_ADDR              0x35
#define PAC194X5X_UC_LIMIT3_ADDR              0x36
#define PAC194X5X_UC_LIMIT4_ADDR              0x37
#define PAC194X5X_OP_LIMIT1_ADDR              0x38
#define PAC194X5X_OP_LIMIT2_ADDR              0x39
#define PAC194X5X_OP_LIMIT3_ADDR              0x3A
#define PAC194X5X_OP_LIMIT4_ADDR              0x3B
#define PAC194X5X_OV_LIMIT1_ADDR              0x3C
#define PAC194X5X_OV_LIMIT2_ADDR              0x3D
#define PAC194X5X_OV_LIMIT3_ADDR              0x3E
#define PAC194X5X_OV_LIMIT4_ADDR              0x3F
#define PAC194X5X_UV_LIMIT1_ADDR              0x40
#define PAC194X5X_UV_LIMIT2_ADDR              0x41
#define PAC194X5X_UV_LIMIT3_ADDR              0x42
#define PAC194X5X_UV_LIMIT4_ADDR              0x43
#define PAC194X5X_OC_LIMIT_N_ADDR             0x44
#define PAC194X5X_UC_LIMIT_N_ADDR             0x45
#define PAC194X5X_OP_LIMIT_N_ADDR             0x46
#define PAC194X5X_OV_LIMIT_N_ADDR             0x47
#define PAC194X5X_UV_LIMIT_N_ADDR             0x48    
#define PAC194X5X_ALERT_ENABLE_ADDR           0x49
#define PAC194X5X_ACCUM_CONFIG_ACT_ADDR       0x4A
#define PAC194X5X_ACCUM_CONFIG_LAT_ADDR       0x4B

#define PAC194X5X_PRODUCT_ID_ADDR             0xFD
#define PAC194X5X_MANUFACTURER_ID_ADDR        0xFE
#define PAC194X5X_REVISION_ID_ADDR            0xFF


/*
 * DEVICE REGISTER SIZES (bytes)
 */
#define PAC194X5X_ACC_COUNT_SZ       4
#define PAC194X5X_VACCN_SZ           7
#define PAC194X5X_VPOWERN_SZ         4
#define PAC194X5X_OPLIMIT_SZ         3
#define PAC194X5X_OTHERLIMIT_SZ      2
#define PAC194X5X_IDREG_SZ           1
#define PAC194X5X_CTRL_SZ            2
#define PAC194X5X_VBUS_VSENSE_SZ     2
#define PAC194X5X_SMBUS_SZ           1
#define PAC194X5X_NEGPWRFSR_SZ       2
#define PAC194X5X_SLOW_SZ            1
#define PAC194X5X_ACCUMCONFIG_SZ     1
#define PAC194X5X_ALERT_SZ           3
#define PAC194X5X_OTHERLIMIT_N_SZ    1

/*
 * DEVICE REGISTER WIDTH (bits)
 */
#define PAC194X5X_VPOWERN_WIDTH      30
#define PAC194X5X_VBUSN_WIDTH        16
#define PAC194X5X_VSENSEN_WIDTH      16
#define PAC194X5X_VLIMIT_WIDTH       16
#define PAC194X5X_PLIMIT_WIDTH       24


/*
 * PAC194x/5x library error codes
 */
#define PAC194X5X_I2C_ERRCLASS                     0x1000
#define PAC194X5X_SUCCESS                          0x00
#define PAC194X5X_INVALID_I2C_BUSID                0x01
#define PAC194X5X_INVALID_INPUT_VALUE              0x02
#define PAC194X5X_INVALID_DEVICE                   0x03
#define PAC194X5X_INVALID_SAMPLE_MODE              0x04
#define PAC194X5X_SINGLE_SHOT_MODE_CONFIGURED      0x05
#define PAC194X5X_SINGLE_SHOT_MODE_NOT_CONFIGURED  0x06
#define PAC194X5X_VACC_POWER_MODE_NOT_CONFIGURED   0x07
#define PAC194X5X_VACC_VSENSE_MODE_NOT_CONFIGURED  0x08
#define PAC194X5X_INVALID_ACCUMULATION_MODE        0x09
#define PAC194X5X_NO_SHUNT                         0x10


/*
 * PAC194x/5x library constants
 */
#define BIT_0       0x01
#define BIT_1       0x02
#define BIT_2       0x04
#define BIT_3       0x08
#define BIT_4       0X10
#define BIT_5       0x20
#define BIT_6       0x40
#define BIT_7       0x80
#define BIT_8       0x100
#define BIT_9       0x200
#define BIT_10      0x400
#define BIT_11      0x800
#define BIT_12      0x1000
#define BIT_13      0x2000
#define BIT_14      0x4000
#define BIT_15      0x8000

#define SMBUS_BYTECNT   1   //the extra byte for SMBUS block read protocol

 /*
  *  PAC194x/5x bit field structures
  */

typedef struct _PAC194X5X_i2cParams {
    uint8_t i2cAddress;     // i2c client 7-bit address
    uint8_t i2cBusID;       // parameter used only by "i2c classic lib":
                            // 1 for "i2c1" or "i2c", 2 for "i2c2", 3 for "i2c3", 4 for "i2c4"                    
} PAC194X5X_i2cParams, *PPAC194X5X_i2cParams;

 typedef struct _PAC194X5X_deviceID {
    uint8_t product;        // PRODUCT_ID register value
    uint8_t manufacturer;   // MANUFACTURER_ID register value
    uint8_t revision;       // REVISION_ID register value
 } PAC194X5X_deviceID, *PPAC194X5X_deviceID;

typedef struct _PAC194X5X_CTRL_REGISTER{
    //PAC register MSB mapped into struct LSB
    uint8_t SLOW_ALERT1 : 2;
    uint8_t GPIO_ALERT2 : 2;
    uint8_t SAMPLE_MODE : 4;
    //PAC register LSB mapped into struct MSB
    uint8_t             : 4;
    uint8_t CH4_OFF     : 1;
    uint8_t CH3_OFF     : 1;
    uint8_t CH2_OFF     : 1;
    uint8_t CH1_OFF     : 1;
} PAC194X5X_CTRL_REGISTER, *PPAC194X5X_CTRL_REGISTER; 

typedef struct _PAC194X5X_SMBUS_SETTINGS_REGISTER {
    uint8_t I2C_HISPEED : 1;
    uint8_t NO_SKIP     : 1;
    uint8_t BYTE_COUNT  : 1;
    uint8_t TIMEOUT     : 1;
    uint8_t POR         : 1;
    uint8_t ANY_ALERT   : 1;
    uint8_t GPIO_DATA1  : 1;
    uint8_t GPIO_DATA2  : 1;
} PAC194X5X_SMBUS_SETTINGS_REGISTER, *PPAC194X5X_SMBUS_SETTINGS_REGISTER;

typedef struct _PAC194X5X_NEG_PWR_FSR_REGISTER {
// PAC register MSB mapped into struct LSB    
    unsigned char CFG_VS4 : 2;
    unsigned char CFG_VS3 : 2;
    unsigned char CFG_VS2 : 2;
    unsigned char CFG_VS1 : 2;
// PAC register LSB mapped into struct MSB    
    unsigned char CFG_VB4 : 2;
    unsigned char CFG_VB3 : 2;
    unsigned char CFG_VB2 : 2;
    unsigned char CFG_VB1 : 2;
} PAC194X5X_NEG_PWR_FSR_REGISTER, *PPAC194X5X_NEG_PWR_FSR_REGISTER;

typedef struct _PAC194X5X_SLOW_REGISTER {
    unsigned char               : 1;
    unsigned char RefreshVFall  : 1;
    unsigned char RefreshFall   : 1;
    unsigned char RefreshVRise  : 1;
    unsigned char RefreshRise   : 1;
    unsigned char SlowHighLow   : 1;
    unsigned char SlowLowHigh   : 1;
    unsigned char Slow          : 1;
} PAC194X5X_SLOW_REGISTER, *PPAC194X5X_SLOW_REGISTER;

typedef struct _PAC194X5X_ACCUM_CONFIG_REGISTER {
    unsigned char ACC4_CONFIG : 2;
    unsigned char ACC3_CONFIG : 2;
    unsigned char ACC2_CONFIG : 2;
    unsigned char ACC1_CONFIG : 2;
} PAC194X5X_ACCUM_CONFIG_REGISTER, *PPAC194X5X_ACCUM_CONFIG_REGISTER;

typedef struct _PAC194X5X_ALERT_STATUS_REGISTER {
// PAC register MSB mapped into struct LSB    
    unsigned char CH4_UC    : 1;
    unsigned char CH3_UC    : 1;
    unsigned char CH2_UC    : 1;
    unsigned char CH1_UC    : 1;
    unsigned char CH4_OC    : 1;
    unsigned char CH3_OC    : 1;
    unsigned char CH2_OC    : 1;
    unsigned char CH1_OC    : 1;
// PAC register middle byte mapped into struct middle byte
    unsigned char CH4_UV    : 1;
    unsigned char CH3_UV    : 1;
    unsigned char CH2_UV    : 1;
    unsigned char CH1_UV    : 1;
    unsigned char CH4_OV    : 1;
    unsigned char CH3_OV    : 1;
    unsigned char CH2_OV    : 1;
    unsigned char CH1_OV    : 1;
// PAC register LSB mapped into struct MSB    
    unsigned char           : 2;
    unsigned char ACC_COUNT : 1;
    unsigned char ACC_OVF   : 1;
    unsigned char CH4_OP    : 1;
    unsigned char CH3_OP    : 1;
    unsigned char CH2_OP    : 1;
    unsigned char CH1_OP    : 1;
} PAC194X5X_ALERT_STATUS_REGISTER, *PPAC194X5X_ALERT_STATUS_REGISTER;


typedef struct _PAC194X5X_ALERT_REGISTER {
    // PAC register MSB mapped into struct LSB    
    unsigned char CH4_UC    : 1;
    unsigned char CH3_UC    : 1;
    unsigned char CH2_UC    : 1;
    unsigned char CH1_UC    : 1;
    unsigned char CH4_OC    : 1;
    unsigned char CH3_OC    : 1;
    unsigned char CH2_OC    : 1;
    unsigned char CH1_OC    : 1;
    // PAC register middle byte mapped into struct middle byte
    unsigned char CH4_UV    : 1;
    unsigned char CH3_UV    : 1;
    unsigned char CH2_UV    : 1;
    unsigned char CH1_UV    : 1;
    unsigned char CH4_OV    : 1;
    unsigned char CH3_OV    : 1;
    unsigned char CH2_OV    : 1;
    unsigned char CH1_OV    : 1;
    // PAC register LSB mapped into struct MSB    
    unsigned char UN_IMP    : 1; // added line 
    unsigned char ALERT_CCx : 1;
    unsigned char ACC_COUNT : 1;
    unsigned char ACC_OVF   : 1;
    unsigned char CH4_OP    : 1;
    unsigned char CH3_OP    : 1;
    unsigned char CH2_OP    : 1;
    unsigned char CH1_OP    : 1;
} PAC194X5X_ALERT_REGISTER, *PPAC194X5X_ALERT_REGISTER;


typedef struct _PAC194X5X_ACCUM_LIMITS_REGISTER {
    // PAC register MSB mapped into struct LSB    
    unsigned char ACC_CH4   : 2;
    unsigned char ACC_CH3   : 2;
    unsigned char ACC_CH2   : 2;
    unsigned char ACC_CH1   : 2;
    // PAC register LSB mapped into struct MSB    
    unsigned char           : 6;
    unsigned char ACC_COUNT : 2;
} PAC194X5X_ACCUM_LIMITS_REGISTER, *PPAC194X5X_ACCUM_LIMITS_REGISTER;


typedef struct _PAC194X5X_LIMIT_NSAMPLES_REGISTER {
    unsigned char Nsamples_CH4 : 2;
    unsigned char Nsamples_CH3 : 2;
    unsigned char Nsamples_CH2 : 2;
    unsigned char Nsamples_CH1 : 2;
} PAC194X5X_LIMIT_NSAMPLES_REGISTER, *PPAC194X5X_LIMIT_NSAMPLES_REGISTER;

/* !!! WARNING !!!
 * The device context structure is initialized by PAC194x5x_Device_Initialize() function call
 */ 
typedef struct _PAC194X5X_DEVICE_CONTEXT{
    uint8_t i2cBusID;
    uint8_t i2cAddress;
    uint8_t HwChannels;
    uint8_t LowSide;
    uint16_t VbusFSR;
    uint16_t VsenseFSR;
    uint16_t VPowerFSR;
    
    // Voltage divider ratios (ratio = Vrail / Vbus) used to determine the rail voltage (Vrail)
    // from the measured Vbus with the PAC194x/5x-2 devices ("low-side" connected devices). 
    // The voltage divider ratios are ignored by the PAC194x/5x-1 "high-side" devices.  
     float VrailToVbusRatio[PAC194X5X_MAX_CH_COUNT];
    
    // Sense resistor values, expressed in micro-Ohm units.
     uint32_t rsense[PAC194X5X_MAX_CH_COUNT];
    
    // cached BYTE_COUNT flag from the SMBUS_SETTINGS register,
    // kept in sync by PAC194x5x_SetSMBusSettings_reg() and PAC194x5x_GetSMBusSettings_reg()
    bool ENABLE_BYTE_COUNT_FLAG;

    //cached channel scale values
    //"scale" and "sign" depends on negPwrFsr configuration
    //the cache must be marked "Invalid" by any "Refresh/V/G" or by "PAC194x5x_SetNegPwrFsr_reg()" 
    uint16_t VbusScale[PAC194X5X_MAX_CH_COUNT];         //VbusScale;
    uint16_t VsenseScale[PAC194X5X_MAX_CH_COUNT];       //VsenseScale;
    uint16_t VPowerScale[PAC194X5X_MAX_CH_COUNT];       //VPowerScale;
    uint8_t  IsSignedPower[PAC194X5X_MAX_CH_COUNT];     //IsSignedPower;
    uint8_t  IsSignedVbus[PAC194X5X_MAX_CH_COUNT];      //Vbus is signed
    uint8_t  IsSignedVsense[PAC194X5X_MAX_CH_COUNT];    //Vsense is signed
    bool ScaleValues_cached[PAC194X5X_MAX_CH_COUNT];    //the computed scale values are valid or not

    //cached device registers   
    PAC194X5X_deviceID deviceID;                        //updated by the PAC194x5x_GetDeviceID() function call
    PAC194X5X_CTRL_REGISTER ctrl_LAT;                   //updated by the PAC194x5x_GetCtrl_reg() function call
    PAC194X5X_NEG_PWR_FSR_REGISTER negPwrFsr_LAT;       //updated by the PAC194x5x_GetNegPwrFsr_reg() function call
    PAC194X5X_ACCUM_CONFIG_REGISTER accumConfig_LAT;    //updated by the PAC194x5x_GetAccumConfig_reg() function call
    
    //cached validity flags
    bool deviceID_cached;
    bool ctrl_LAT_cached;
    bool negPwrFsr_LAT_cached;
    bool accumConfig_LAT_cached;
} PAC194X5X_DEVICE_CONTEXT, *PPAC194X5X_DEVICE_CONTEXT;


/* 
 * PAC194x/5x library interface
 */

void init_pac1952(void);
esp_err_t pac1952_busy(void);
/**
    @Summary
        Initializes the PAC194x/5x instance.
    @Description
        This routine initializes the PAC194X5X_DEVICE_CONTEXT device context data and
        the PAC194x/5x device to the default configuration,
        or to the configuration selected by the user in the MPLAB MCC GUI.
        It also reads the device ID registers and caches their values in the device context data. 
        NOTE: 1ms delay must be allowed between the PAC194x5x_Device_Initialize() function call and
        the following function calls. 
    @Preconditions
        None
    @Param
        i2cParams - the i2c bus ID and the i2c client 7-bit address
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        VrailToVbusRatio - Array of voltage divider ratios (ratio = Vrail / Vbus),
                           one value for each channel.
                           This parameter is only used to determine the rail voltage (Vrail)
                           from the measured Vbus with the PAC194x/5x-2 devices
                           ("low-side" connected devices). 
                           The parameter is ignored by the PAC194x/5x-1 "high-side" devices.  
    @Param
        prsense - Array of sense resistor values, expressed in micro-Ohm units,
                  one value for each channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_Device_Initialize(i2cParams, PACdevice, VrailToVbusRatio, rsense);
        </code>
*/
uint16_t PAC194x5x_Device_Initialize(PAC194X5X_i2cParams i2cParams, PPAC194X5X_DEVICE_CONTEXT pdevice, float *VrailToVbusRatio, uint32_t *prsense);


/**
    @Summary
        Gets the identification values for the PAC194x/5x.
    @Description
        This method gets the content of the ID registers: 
        PRODUCT_ID, MANUFACTURER_ID and REVISION_ID.
        The function caches the ID values in the device context data.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pdeviceID - content of the ID registers
    @Return
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetDeviceID(PACdevice);
        <code>
*/
uint16_t PAC194x5x_GetDeviceID(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_deviceID pdeviceID);


/**
    @Summary
        Executes a device 'REFRESH' command.
    @Description
        This method executes the device 'REFRESH' command. 
        The accumulator registers (power products, sample count) and the 
        Vbus, Vsense measurements are latched into the device readable registers,  
        the accumulators are reset and the configuration changes are applied. 
        The latched data is stable and can be read after 1ms.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_Refresh(PACdevice);
        <code>
*/
uint16_t PAC194x5x_Refresh(PPAC194X5X_DEVICE_CONTEXT pdevice);


/**
    @Summary
        Executes a 'REFRESH_G' command.
    @Description
        This method executes the device 'REFRESH_G' command, using i2c 
        General Call command. In this case, for all PAC194x/5x devices connected at the
        same i2c bus, the accumulator registers (power products, sample count) and the 
        Vbus, Vsense measurements are latched into the devices readable registers,  
        the accumulators are reset and the configuration changes are applied. 
        The latched data is stable and can be read after 1ms.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_RefreshG(PACdevice);
        <code>
*/
uint16_t PAC194x5x_RefreshG(PPAC194X5X_DEVICE_CONTEXT pdevice);


/**
    @Summary
        Executes a device 'REFRESH_V' command.
    @Description
        This method executes the device 'REFRESH_V' command. 
        The accumulator registers (power products, sample count) and the 
        Vbus, Vsense measurements are latched into the device readable registers,  
        and the configuration changes are applied but the accumulators are not reset. 
        The latched data is stable and can be read after 1ms.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_RefreshV(PACdevice);
        <code>
*/
uint16_t PAC194x5x_RefreshV(PPAC194X5X_DEVICE_CONTEXT pdevice);


/**
    @Summary
        Gets the CTRL register value.
    @Description
        This method gets the currently set, the currently active or the latched active control register value,
        depending on the reg_select mode.
        If CTRL_LAT is selected, the function caches the register value in the device context data.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        reg_select - 1- CTRL, 2- CTRL_ACT, 3- CTRL_LAT
    @Param
        pCtrl_reg - The control register value:
                        Bits 15-12:    Sample Mode
                        Bits 11-10:    GPIO Alert 2
                        Bits 9-8:      Slow Alert 1
                        Bit 7:         CH1_OFF
                        Bit 6:         CH2_OFF
                        Bit 5:         CH3_OFF
                        Bit 4:         CH4_OFF
                        Bits 3-0:      Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetCtrl_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetCtrl_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_CTRL_REGISTER pCtrl_reg);


/**
    @Summary
        Sets the CTRL register value.
    @Description
        This method sets the current control register value. Followed by the REFRESH, 
        REFRESH_V or REFRESH_G command, the new value of register will be activated.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        Ctrl_reg - the control register value to be set (16 bit value):
                        Bits 15-12:    Sample Mode
                        Bits 11-10:    GPIO Alert 2
                        Bits 9-8:      Slow Alert 1
                        Bit 7:         CH1_OFF
                        Bit 6:         CH2_OFF
                        Bit 5:         CH3_OFF
                        Bit 4:         CH4_OFF
                        Bits 3-0:      Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetCtrl_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetCtrl_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_CTRL_REGISTER Ctrl_reg);


/**
    @Summary
        Gets the current accumulator count.
    @Description
        This method gets the count for each time a power result has been summed 
        in the accumulator. 
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pregister_val - The accumulator count register value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetAccumulatorCount(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetAccumulatorCount(PPAC194X5X_DEVICE_CONTEXT pdevice, uint32_t* pregister_val);


#ifdef __XC8__
/**
    @Summary
        Gets the current register accumulated Vpower, Vsense or Vbus.
    @Description
        This method gets the register value of the accumulator sum of Vpower, Vsense or Vbus samples,
        depending on Accum Config register settings.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - the user buffer that receives the accumulator register value.
                        The buffer size must be 7 bytes.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVACCn_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetVACCn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint8_t* pregister_val);


#else   // __XC16__ or __XC32__
/**
    @Summary
        Gets the current register accumulated Vpower, Vsense or Vbus.
    @Description
        This method gets the register value of the accumulator sum of Vpower, Vsense or Vbus samples,
        depending on Accum Config register settings.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - the user variable that receives the accumulator register value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVACCn_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetVACCn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint64_t* pregister_val);
#endif


/**
    @Summary
        Gets the current real accumulated Vpower, Vsense or Vbus.
    @Description
        This method gets the calculated real value of the accumulated sum of Vpower, Vsense or Vbus samples,
        depending on Accum Config register settings.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the Vpower, Vsense or Vbus real accumulated value of the selected channel.
    @Param
        pmode - Vpower, Vsense or Vbus mode configured in Accum Config register:
                - Power accumulator - mode 0 - milli-Watt
                - Coulomb count     - mode 1 - milli-Amp*sec
                - Vbus accumulator  - mode 2 - milli-Volt
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PPAC194x5x_GetVACCn_real(PACdevice, 1, &value, &mode);
        <code>
*/
uint16_t PAC194x5x_GetVACCn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue, uint8_t* pmode);


/**
    @Summary
        Gets the accumulated energy value.
    @Description
        This method gets the calculated energy value that corresponds 
        with the accumulated power.
        The value unit is milli-Watt-hour.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The energy value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
        If Single Shot or Single Shot 8x is configured, the function returns PAC194X5X_SINGLE_SHOT_MODE_CONFIGURED error.
        If the accumulator is not configured in VPOWER mode, the function returns PAC194X5X_VACC_POWER_MODE_NOT_CONFIGURED error.
        
    @Example
        <code>
            errorCode = PAC194x5x_GetEnergy(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetEnergy(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the accumulated energy value, considering the user measured time interval of power accumulation.
    @Description
        This method gets the calculated energy value that corresponds to the accumulated power
        in the measured time interval lapsed between the accumulator reset and the last refresh command.
        The value unit is milli-Watt-hour.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The energy value for selected channel.
    @Param 
        time - the accumulation time to provide the energy measurement
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
        If the accumulator is not configured in VPOWER mode, the function returns PAC194X5X_VACC_POWER_MODE_NOT_CONFIGURED error.
    @Example
        <code>
            errorCode = PAC194x5x_GetTimedEnergy(PACdevice, 1, &value, time);
        <code>
*/
uint16_t PAC194x5x_GetTimedEnergy(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue, uint32_t time);


/**
    @Summary
        Gets the current Coulomb count value.
    @Description
        This method gets the calculated Coulomb count value that corresponds 
        with the accumulated Vsense.
        The value unit is milli-Amp*sec.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The Coulomb count value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
        If Single Shot or Single Shot 8x is configured, the function returns PAC194X5X_SINGLE_SHOT_MODE_CONFIGURED error.
        If the accumulator is not configured in VSENSE mode, the function returns PAC194X5X_VACC_VSENSE_MODE_NOT_CONFIGURED error.
    @Example
        <code>
            errorCode = PAC194x5x_GetCoulomb(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetCoulomb(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the current Coulomb count value, considering the user measured time interval of power accumulation.
    @Description
        This method gets the calculated Coulomb count value that corresponds to the accumulated Vsense
        in the measured time interval lapsed between the accumulator reset and the last refresh command.
        The value unit is milli-Amp*sec.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The Coulomb count value for selected channel.
    @Param 
        time - the accumulation time to provide the Coulomb count
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
        If the accumulator is not configured in VSENSE mode, the function returns PAC194X5X_VACC_VSENSE_MODE_NOT_CONFIGURED error.
    @Example
        <code>
            errorCode = PAC194x5x_GetTimedCoulomb(PACdevice, 1, &value, time);
        <code>
*/
uint16_t PAC194x5x_GetTimedCoulomb(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue, uint32_t time);


/**
    @Summary
        Gets the current Vbus register value.
    @Description
        This method gets the most recent register value of a bus voltage sample.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - the bus voltage register value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVBUSn_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetVBUSn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Gets the current real Vbus value.
    @Description
        This method gets the most recent calculated real value of a bus voltage sample.
        The value unit is milli-Volt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the bus voltage calculated value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVBUSn_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetVBUSn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the current Vsense register value.
    @Description
        This method gets the most recent register value of the sense voltage samples.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - the register value of current sense voltage for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVSENSEn_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetVSENSEn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Gets the current real Vsense value.
    @Description
        This method gets the most recent calculated real value of the sense voltage samples.
        The value unit is milli-Volt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the calculated value of current sense voltage for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVSENSEn_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetVSENSEn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the current Isense value.
    @Description
        This method gets the most recent calculated value of the sense current samples.
        The value unit is milli-Amp.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the calculated value of sense current for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetISENSEn_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetISENSEn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the current Vbus average register value.
    @Description
        This method gets the most recent register value of a rolling average of 
        the 8 most recent bus voltage measurements
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - the bus voltage average register value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVBUSn_AVG_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetVBUSn_AVG_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Gets the current real Vbus average value.
    @Description
        This method gets the most recent calculated real value of a rolling average of 
        the 8 most recent bus voltage measurements.
        The unit value is milli-Volt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the bus voltage calculated average value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVBUSn_AVG_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetVBUSn_AVG_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the current Vsense average register value.
    @Description
        This method gets the most recent register value of a rolling average of 
        the 8 most recent sense voltage measurements.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - the register value of current sense voltage average for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVSENSEn_AVG_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetVSENSEn_AVG_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Gets the current real Vsense average value.
    @Description
        This method gets the most recent calculated real value of a rolling average of 
        the 8 most recent sense voltage measurements.
        The value unit is milli-Volt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the calculated value of current sense voltage average for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVSENSEn_AVG_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetVSENSEn_AVG_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Returns the current average Isense value.
    @Description
        This method returns the most recent calculated amperage value considering the rolling average of 
        the 8 most recent sense voltage measurements.
        The value unit is milli-Amp.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the calculated value of sense amperage average for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetISENSEn_AVG_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetISENSEn_AVG_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Returns the current Vpower register value.
    @Description
        This method returns the register value of the proportional power for each channel.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - the register value of power for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVPOWERn_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetVPOWERn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint32_t* pregister_val);


/**
    @Summary
        Returns the current real Vpower value.
    @Description
        This method returns the most recent calculated value of the proportional power for each channel.
        The value unit is milli-Watt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - the calculated value of power for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVPOWERn_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetVPOWERn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint32_t* VpowerReg);


/**
    @Summary
        Gets the SMBus Settings register value.
    @Description
        This method gets the currently set SMBus Settings register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pSMBus_reg - The SMBus Settings register value:
                        Bit 7:         GPIO_DATA2
                        Bit 6:         GPIO_DATA1
                        Bit 5:         ANY_ALERT
                        Bit 4:         POR
                        Bit 3:         TIMEOUT
                        Bit 2:         BYTE_COUNT
                        Bit 1:         NO SKIP
                        Bit 0:         I2C_HISPEED
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetSMBusSettings_reg(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetSMBusSettings_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_SMBUS_SETTINGS_REGISTER pSMBus_reg);


/**
    @Summary
        Sets the SMBus Settings register value.
    @Description
        This method sets the current SMBus Settings register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        SMBus_reg - The SMBus Settings register value to be set:
                        Bit 7:         GPIO_DATA2
                        Bit 6:         GPIO_DATA1
                        Bit 5:         ANY_ALERT
                        Bit 4:         POR
                        Bit 3:         TIMEOUT
                        Bit 2:         BYTE_COUNT
                        Bit 1:         NO SKIP
                        Bit 0:         I2C_HISPEED
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetSMBusSettings_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetSMBusSettings_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_SMBUS_SETTINGS_REGISTER SMBus_reg);


/**
    @Summary
        Gets the NEG_PWR_FSR register value.
    @Description
        This method gets the currently set, the currently active or the latched active NEG_PWR_FSR register value,
        depending on the reg_select mode.
        If NEG_PWR_FSR_LAT is selected, the function caches the register value in the device context data.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        reg_select - 1- NEG_PWR_FSR, 2- NEG_PWR_FSR_ACT, 3- NEG_PWR_FSR_LAT
    @Param
        pNegPwrFsr_reg - The NEG_PWR_FSR register value:
                        Bits 15-14:       CFG_VS1
                        Bits 13-12:       CFG_VS2
                        Bits 11-10:       CFG_VS3
                        Bits 9-8:         CFG_VS4
                        Bits 7-6:         CFG_VB1
                        Bits 5-4:         CFG_VB2
                        Bits 3-2:         CFG_VB3
                        Bits 1-0:         CFG_VB4
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetNegPwrFsr_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetNegPwrFsr_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_NEG_PWR_FSR_REGISTER pNegPwrFsr_reg);


/**
    @Summary
        Sets the NEG_PWR_FSR register value.
    @Description
        This method sets the current NEG_PWR_FSR register value. Followed by the REFRESH, 
        REFRESH_V or REFRESH_G command, the new value of register will be activated.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        NegPwrFsr_reg - the NEG_PWR_FSR register value to be set (16 bit value):
                        Bits 15-14:       CFG_VS1
                        Bits 13-12:       CFG_VS2
                        Bits 11-10:       CFG_VS3
                        Bits 9-8:         CFG_VS4
                        Bits 7-6:         CFG_VB1
                        Bits 5-4:         CFG_VB2
                        Bits 3-2:         CFG_VB3
                        Bits 1-0:         CFG_VB4
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetNegPwrFsr_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetNegPwrFsr_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_NEG_PWR_FSR_REGISTER NegPwrFsr_reg);


/**
    @Summary
        Gets the SLOW register value.
    @Description
        This method gets the currently set SLOW register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pSlow_reg - The SLOW register value:
                        Bit 7:         Slow
                        Bit 6:         SlowLowHigh
                        Bit 5:         SlowHighLow
                        Bit 4:         RefreshRise
                        Bit 3:         RefreshVRise
                        Bit 2:         RefreshFall
                        Bit 1:         RefreshVFall
                        Bit 0:         Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetSlow_reg(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetSlow_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_SLOW_REGISTER pSlow_reg);


/**
    @Summary
        Sets the SLOW Settings register value.
    @Description
        This method sets the current SLOW register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        Slow_reg - The SLOW register value to be set:
                        Bit 7:         Slow
                        Bit 6:         SlowLowHigh
                        Bit 5:         SlowHighLow
                        Bit 4:         RefreshRise
                        Bit 3:         RefreshVRise
                        Bit 2:         RefreshFall
                        Bit 1:         RefreshVFall
                        Bit 0:         Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetSlow_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetSlow_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_SLOW_REGISTER Slow_reg);


/**
    @Summary
        Gets the Accum Config register value.
    @Description
        This method gets the currently set, the currently active or the latched active Accum Config register value,
        depending on the reg_select mode.
        If Accum_Config_LAT is selected, the function caches the register value in the device context data.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        reg_select - 1- Accum Config, 2- Accum Config ACT, 3- Accum Config LAT
    @Param
        pAccumConfig_reg - The Accum Config register value:
                        Bits 7-6:         ACC1_CONFIG
                        Bits 5-4:         ACC2_CONFIG
                        Bits 3-2:         ACC3_CONFIG
                        Bits 1-0:         ACC4_CONFIG
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetAccumConfig_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetAccumConfig_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_ACCUM_CONFIG_REGISTER pAccumConfig_reg);


/**
    @Summary
        Sets the Accum Config register value.
    @Description
        This method sets the current Accum Config register value. Followed by the REFRESH, 
        REFRESH_V or REFRESH_G command, the new value of register will be activated.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        AccumConfig_reg - the Accum Config register value to be set (8 bit value):
                        Bits 7-6:         ACC1_CONFIG
                        Bits 5-4:         ACC2_CONFIG
                        Bits 3-2:         ACC3_CONFIG
                        Bits 1-0:         ACC4_CONFIG
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetAccumConfig_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetAccumConfig_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ACCUM_CONFIG_REGISTER AccumConfig_reg);


/**
    @Summary
        Gets the ALERT STATUS register value.
    @Description
        This method gets the currently set ALERT STATUS register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pAlertStatus_reg - The ALERT STATUS register value:
                        Bit 23:         CH1_OC
                        Bit 22:         CH2_OC
                        Bit 21:         CH3_OC
                        Bit 20:         CH4_OC
                        Bit 19:         CH1_UC
                        Bit 18:         CH2_UC
                        Bit 17:         CH3_UC
                        Bit 16:         CH4_UC
                        Bit 15:         CH1_OV
                        Bit 14:         CH2_OV
                        Bit 13:         CH3_OV
                        Bit 12:         CH4_OV
                        Bit 11:         CH1_UV
                        Bit 10:         CH2_UV
                        Bit 9:          CH3_UV
                        Bit 8:          CH4_UV   
                        Bit 7:          CH1_OP
                        Bit 6:          CH2_OP
                        Bit 5:          CH3_OP
                        Bit 4:          CH4_OP
                        Bit 3:          ACC_OVF
                        Bit 2:          ACC_COUNT
                        Bits 1-0:       Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetAlertStatus_reg(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetAlertStatus_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_STATUS_REGISTER pAlertStatus_reg);


/**
    @Summary
        Gets the ALERT ENABLE register value.
    @Description
        This method gets the currently set ALERT ENABLE register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pAlertEnable_reg - The ALERT ENABLE register value:
                        Bit 23:         CH1_OC
                        Bit 22:         CH2_OC
                        Bit 21:         CH3_OC
                        Bit 20:         CH4_OC
                        Bit 19:         CH1_UC
                        Bit 18:         CH2_UC
                        Bit 17:         CH3_UC
                        Bit 16:         CH4_UC
                        Bit 15:         CH1_OV
                        Bit 14:         CH2_OV
                        Bit 13:         CH3_OV
                        Bit 12:         CH4_OV
                        Bit 11:         CH1_UV
                        Bit 10:         CH2_UV
                        Bit 9:          CH3_UV
                        Bit 8:          CH4_UV   
                        Bit 7:          CH1_OP
                        Bit 6:          CH2_OP
                        Bit 5:          CH3_OP
                        Bit 4:          CH4_OP
                        Bit 3:          ACC_OVF
                        Bit 2:          ACC_COUNT
                        Bit 1:          ALERT_CC
                        Bit 0:          Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetAlertEnable_reg(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetAlertEnable_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_REGISTER pAlertEnable_reg);


/**
    @Summary
        Sets the ALERT ENABLE register value.
    @Description
        This method sets the current ALERT ENABLE register value. Followed by the REFRESH, 
        REFRESH_V or REFRESH_G command, the new value of register will be activated.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        AlertEnable_reg - the ALERT ENABLE register value to be set (24 bit value):
                        Bit 23:         CH1_OC
                        Bit 22:         CH2_OC
                        Bit 21:         CH3_OC
                        Bit 20:         CH4_OC
                        Bit 19:         CH1_UC
                        Bit 18:         CH2_UC
                        Bit 17:         CH3_UC
                        Bit 16:         CH4_UC
                        Bit 15:         CH1_OV
                        Bit 14:         CH2_OV
                        Bit 13:         CH3_OV
                        Bit 12:         CH4_OV
                        Bit 11:         CH1_UV
                        Bit 10:         CH2_UV
                        Bit 9:          CH3_UV
                        Bit 8:          CH4_UV   
                        Bit 7:          CH1_OP
                        Bit 6:          CH2_OP
                        Bit 5:          CH3_OP
                        Bit 4:          CH4_OP
                        Bit 3:          ACC_OVF
                        Bit 2:          ACC_COUNT
                        Bit 1:          ALERT_CC
                        Bit 0:          Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetAlertEnable_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetAlertEnable_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ALERT_REGISTER AlertEnable_reg);


/**
    @Summary
        Gets the SLOW_ALERT1 register value.
    @Description
        This method gets the currently set SLOW_ALERT1 register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pSlowAlert1_reg - The SLOW_ALERT1 register value:
                        Bit 23:         CH1_OC
                        Bit 22:         CH2_OC
                        Bit 21:         CH3_OC
                        Bit 20:         CH4_OC
                        Bit 19:         CH1_UC
                        Bit 18:         CH2_UC
                        Bit 17:         CH3_UC
                        Bit 16:         CH4_UC
                        Bit 15:         CH1_OV
                        Bit 14:         CH2_OV
                        Bit 13:         CH3_OV
                        Bit 12:         CH4_OV
                        Bit 11:         CH1_UV
                        Bit 10:         CH2_UV
                        Bit 9:          CH3_UV
                        Bit 8:          CH4_UV   
                        Bit 7:          CH1_OP
                        Bit 6:          CH2_OP
                        Bit 5:          CH3_OP
                        Bit 4:          CH4_OP
                        Bit 3:          ACC_OVF
                        Bit 2:          ACC_COUNT
                        Bit 1:          ALERT_CC1
                        Bit 0:          Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetSlowAlert1_reg(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetSlowAlert1_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_REGISTER pSlowAlert1_reg);


/**
    @Summary
        Sets the SLOW_ALERT1 register value.
    @Description
        This method sets the current SLOW_ALERT1 register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        SlowAlert1_reg - the SLOW_ALERT1 register value to be set (24 bit value):
                        Bit 23:         CH1_OC
                        Bit 22:         CH2_OC
                        Bit 21:         CH3_OC
                        Bit 20:         CH4_OC
                        Bit 19:         CH1_UC
                        Bit 18:         CH2_UC
                        Bit 17:         CH3_UC
                        Bit 16:         CH4_UC
                        Bit 15:         CH1_OV
                        Bit 14:         CH2_OV
                        Bit 13:         CH3_OV
                        Bit 12:         CH4_OV
                        Bit 11:         CH1_UV
                        Bit 10:         CH2_UV
                        Bit 9:          CH3_UV
                        Bit 8:          CH4_UV   
                        Bit 7:          CH1_OP
                        Bit 6:          CH2_OP
                        Bit 5:          CH3_OP
                        Bit 4:          CH4_OP
                        Bit 3:          ACC_OVF
                        Bit 2:          ACC_COUNT
                        Bit 1:          ALERT_CC1
                        Bit 0:          Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetSlowAlert1_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetSlowAlert1_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ALERT_REGISTER SlowAlert1_reg);


/**
    @Summary
        Gets the GPIO_ALERT2 register value.
    @Description
        This method gets the currently set GPIO_ALERT2 register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pGpioAlert2_reg - The GPIO_ALERT2 register value:
                        Bit 23:         CH1_OC
                        Bit 22:         CH2_OC
                        Bit 21:         CH3_OC
                        Bit 20:         CH4_OC
                        Bit 19:         CH1_UC
                        Bit 18:         CH2_UC
                        Bit 17:         CH3_UC
                        Bit 16:         CH4_UC
                        Bit 15:         CH1_OV
                        Bit 14:         CH2_OV
                        Bit 13:         CH3_OV
                        Bit 12:         CH4_OV
                        Bit 11:         CH1_UV
                        Bit 10:         CH2_UV
                        Bit 9:          CH3_UV
                        Bit 8:          CH4_UV   
                        Bit 7:          CH1_OP
                        Bit 6:          CH2_OP
                        Bit 5:          CH3_OP
                        Bit 4:          CH4_OP
                        Bit 3:          ACC_OVF
                        Bit 2:          ACC_COUNT
                        Bit 1:          ALERT_CC2
                        Bit 0:          Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetGpioAlert2_reg(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetGpioAlert2_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_REGISTER pGpioAlert2_reg);


/**
    @Summary
        Sets the GPIO_ALERT2 register value.
    @Description
        This method sets the current GPIO_ALERT2 register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        GpioAlert2_reg - the GPIO_ALERT2 register value to be set (24 bit value):
                        Bit 23:         CH1_OC
                        Bit 22:         CH2_OC
                        Bit 21:         CH3_OC
                        Bit 20:         CH4_OC
                        Bit 19:         CH1_UC
                        Bit 18:         CH2_UC
                        Bit 17:         CH3_UC
                        Bit 16:         CH4_UC
                        Bit 15:         CH1_OV
                        Bit 14:         CH2_OV
                        Bit 13:         CH3_OV
                        Bit 12:         CH4_OV
                        Bit 11:         CH1_UV
                        Bit 10:         CH2_UV
                        Bit 9:          CH3_UV
                        Bit 8:          CH4_UV   
                        Bit 7:          CH1_OP
                        Bit 6:          CH2_OP
                        Bit 5:          CH3_OP
                        Bit 4:          CH4_OP
                        Bit 3:          ACC_OVF
                        Bit 2:          ACC_COUNT
                        Bit 1:          ALERT_CC2
                        Bit 0:          Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetGpioAlert2_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetGpioAlert2_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ALERT_REGISTER GpioAlert2_reg);


/**
    @Summary
        Gets the ACC Fullness limits register value.
    @Description
        This method gets the currently set ACC Fullness limits register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        pAccFullnessLimits_reg - The ACC Fullness limits register value:
                        Bits 15-14:       ACC_CH1
                        Bits 13-12:       ACC_CH2
                        Bits 11-10:       ACC_CH3
                        Bits 9-8:         ACC_CH4
                        Bits 7-6:         ACC_COUNT
                        Bits 5-0:         Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetAccFullness_reg(PACdevice, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetAccFullness_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ACCUM_LIMITS_REGISTER pAccFullnessLimits_reg);


/**
    @Summary
        Sets the ACC Fullness limits register value.
    @Description
        This method sets the current ACC Fullness limits register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        AccFullnessLimits_reg - the ACC Fullness limits register value to be set (16 bit value):
                        Bits 15-14:       ACC_CH1
                        Bits 13-12:       ACC_CH2
                        Bits 11-10:       ACC_CH3
                        Bits 9-8:         ACC_CH4
                        Bits 7-6:         ACC_COUNT
                        Bits 5-0:         Unimplemented
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetAccFullness_reg(PACdevice, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetAccFullness_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ACCUM_LIMITS_REGISTER AccFullnessLimits_reg);


/**
    @Summary
        Gets the OC limit register value.
    @Description
        This method gets the currently set Over Current limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - The OC limit register value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetOClimit_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetOClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Sets the OC limit register value.
    @Description
        This method sets the current Over Current limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        register_val - the OC limit register value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetOClimit_reg(PACdevice, 1, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetOClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val);


/**
    @Summary
        Gets the real OC limit value.
    @Description
        This method gets the currently set Over Current limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The OC limit real value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetOClimit_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetOClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Sets the real OC limit value.
    @Description
        This method sets the current Over Current limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        value - the OC limit real value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetOClimit_real(PACdevice, 1, value);
        <code>
*/
uint16_t PAC194x5x_SetOClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value);


/**
    @Summary
        Gets the UC limit register value.
    @Description
        This method gets the currently set Under Current limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - The UC limit register value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetUClimit_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetUClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Sets the UC limit register value.
    @Description
        This method sets the current Under Current limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        register_val - the UC limit register value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetUClimit_reg(PACdevice, 1, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetUClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val);


/**
    @Summary
        Gets the real UC limit value.
    @Description
        This method gets the currently set Under Current limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The UC limit real value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetUClimit_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetUClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Sets the real UC limit value.
    @Description
        This method sets the current Under Current limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        value - the UC limit real value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetUClimit_real(PACdevice, 1, value);
        <code>
*/
uint16_t PAC194x5x_SetUClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value);


/**
    @Summary
        Gets the OP limit register value.
    @Description
        This method gets the currently set Over Power limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - The OP limit register value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetOPlimit_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetOPlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint32_t* pregister_val);


/**
    @Summary
        Sets the OP limit register value.
    @Description
        This method sets the current Over Power limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        register_val - the OP limit register value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetOPlimit_reg(PACdevice, 1, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetOPlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint32_t register_val);


/**
    @Summary
        Gets the real OP limit value.
    @Description
        This method gets the currently set Over Power limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The OP limit real value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetOPlimit_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetOPlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Sets the real OP limit value.
    @Description
        This method sets the current Over Power limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        value - the OP limit real value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetOPlimit_real(PACdevice, 1, value);
        <code>
*/
uint16_t PAC194x5x_SetOPlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value);


/**
    @Summary
        Gets the OV limit register value.
    @Description
        This method gets the currently set Over Voltage limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - The OV limit register value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetOVlimit_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetOVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Sets the OV limit register value.
    @Description
        This method sets the current Over Voltage limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        register_val - the OV limit register value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetOVlimit_reg(PACdevice, 1, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetOVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val);


/**
    @Summary
        Gets the real OV limit value.
    @Description
        This method gets the currently set Over Voltage limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The OV limit real value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetOVlimit_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetOVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Sets the real OV limit value.
    @Description
        This method sets the current Over Voltage limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        value - the OV limit real value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetOVlimit_real(PACdevice, 1, value);
        <code>
*/
uint16_t PAC194x5x_SetOVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value);


/**
    @Summary
        Gets the UV limit register value.
    @Description
        This method gets the currently set Under Voltage limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pregister_val - The UV limit register value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetUVlimit_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetUVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val);


/**
    @Summary
        Sets the UV limit register value.
    @Description
        This method sets the current Under Voltage limit register value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        register_val - the UV limit register value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetUVlimit_reg(PACdevice, 1, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetUVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val);


/**
    @Summary
        Gets the real UV limit value.
    @Description
        This method gets the currently set Under Voltage limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The UV limit real value.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetUVlimit_real(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetUVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Sets the real UV limit value.
    @Description
        This method sets the current Under Voltage limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        value - the UV limit real value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetUVlimit_real(PACdevice, 1, value);
        <code>
*/
uint16_t PAC194x5x_SetUVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value);


/**
    @Summary
        Gets the OC, UC, OP, OV or UV limit Nsamples register value.
    @Description
        This method gets the OC, UC, OP, OV or UV limit Nsamples register value, depending on the reg_select mode.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        reg_select - 1- OC limit Nsamples, 2- UC limit Nsamples, 3- OP limit Nsamples, 4- OV limit Nsamples, 5- UV limit Nsamples
    @Param
        pLimitNsamples_reg - The OC, UC, OP, OV or UV limit Nsamples register value:
                        Bits 7-6:         Nsamples_CH1
                        Bits 5-4:         Nsamples_CH2
                        Bits 3-2:         Nsamples_CH3
                        Bits 1-0:         Nsamples_CH4
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetLimitNsamples_reg(PACdevice, 1, &reg_value);
        <code>
*/
uint16_t PAC194x5x_GetLimitNsamples(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_LIMIT_NSAMPLES_REGISTER pLimitNsamples_reg);


/**
    @Summary
        Sets the OC, UC, OP, OV or UV limit Nsamples register value.
    @Description
        This method sets the current OC, UC, OP, OV or UV limit Nsamples register value depending on the reg_select mode.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        reg_select - 1- OC limit Nsamples, 2- UC limit Nsamples, 3- OP limit Nsamples, 4- OV limit Nsamples, 5- UV limit Nsamples
    @Param
        LimitNsamples_reg - the OC, UC, OP, OV or UV limit Nsamples register value to be set (8 bit value):
                        Bits 7-6:         Nsamples_CH1
                        Bits 5-4:         Nsamples_CH2
                        Bits 3-2:         Nsamples_CH3
                        Bits 1-0:         Nsamples_CH4
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetLimitNsamples_reg(PACdevice, 1, reg_value);
        <code>
*/
uint16_t PAC194x5x_SetLimitNsamples(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PAC194X5X_LIMIT_NSAMPLES_REGISTER LimitNsamples_reg);


/**
    @Summary
        Gets the current VBUS LSB unit value.
    @Description
        This method gets the calculated VBUS LSB unit value, also valid for
        the VACCn accumulator configured in Vbus accumulation mode.
        The value unit is milli-Volt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The VBUS LSB unit value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVBUS_LSBunit(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_VBUS_LSBunit(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the current VSENSE LSB unit value.
    @Description
        This method gets the calculated VSENSE LSB unit value, also valid for
        the VACCn accumulator configured in VSENSE accumulation mode.
        The value unit is milli-Volt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The VSENSE LSB unit value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVSENSE_LSBunit(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_VSENSE_LSBunit(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Gets the current VPOWER LSB unit value.
    @Description
        This method gets the calculated VPOWER LSB unit value, also valid for
        the VACCn accumulator configured in power accumulation mode.
        The value unit is milli-Watt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Param
        pvalue - The VPOWER LSB unit value for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVPOWER_LSBunit(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_VPOWER_LSBunit(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue);


/**
    @Summary
        Updates the Channel Polarity information in the device context.
    @Description
        This method updates the Channel Polarity Device Context.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Param
        channelNo - the channel index. Accepted values are 1, 2, 3 or 4.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_UpdateContext_ChannelPolarity(PACdevice, 1);
        <code>
*/
uint16_t PAC194x5x_UpdateContext_ChannelPolarity(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo);


/**
    @Summary
        Updates the device CTRL_LAT register value cached in the device context.
    @Description
        This method updates the device CTRL_LAT register value cached in the device context.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_UpdateContext_Ctrl(PACdevice);
        <code>
*/
uint16_t PAC194x5x_UpdateContext_Ctrl(PPAC194X5X_DEVICE_CONTEXT pdevice);


/**
    @Summary
        Updates the device ACCUM_CONFIG_LAT register value cached in the device context.
    @Description
        This method updates the device ACCUM_CONFIG_LAT register value cached in the device context.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_UpdateContext_AccumConfig(PACdevice);
        <code>
*/
uint16_t PAC194x5x_UpdateContext_AccumConfig(PPAC194X5X_DEVICE_CONTEXT pdevice);

/**
    @Summary
        Returns the current analog Vbus, Isense and Vpower values.
    @Description
        This method returns the most recent calculated value of the proportional voltage, current and power for each channel.
        The value unit is mV,mA and milli-Watt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        pvalue - the calculated values of voltage current and power for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVIP_Analog(PACdevice, 1, &value);
        <code>
*/
uint16_t PAC194x5x_GetVIP_Analog(PPAC194X5X_DEVICE_CONTEXT pdevice,float *pValue);

/**
    @Summary
        Returns the current digital Vbus, Isense and Vpower values.
    @Description
        This method returns the most recent calculated value of the proportional voltage, current and power for each channel.
        The value unit is mV,mA and milli-Watt.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data
    @Param
        pvalue - the calculated values of voltage current and power for selected channel.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_GetVIP_digital(PACdevice, 1, &value);
        <code>
*/

uint16_t PAC194x5x_GetVIP_digital(PPAC194X5X_DEVICE_CONTEXT pdevice,uint8_t *pValue, uint8_t byteSize);

/**
    @Summary
        Sets OV limit value.
    @Description
        This method sets the current Over Voltage limit calculated real value.
    @Preconditions
        None
    @Param
        pdevice - PAC194x/5x device context data.

    @Param
        value - the OV limit real value to be set.
    @Returns
        The error code. For execution success, the method returns PAC194X5X_SUCCESS value.
    @Example
        <code>
            errorCode = PAC194x5x_SetOverVolt_limit(PACdevice,value);
        <code>
*/
uint16_t PAC194x5x_SetOverVolt_limit(PPAC194X5X_DEVICE_CONTEXT pdevice,float value);

uint16_t PAC194X5X_getBatteryVoltage(PPAC194X5X_DEVICE_CONTEXT pdevice,uint16_t *pValue);
uint16_t PAC194X5X_getSolarcurrent(PPAC194X5X_DEVICE_CONTEXT pdevice,uint16_t *pValue);
uint16_t PAC194X5X_getSolarvoltage(PPAC194X5X_DEVICE_CONTEXT pdevice,uint16_t *pValue);

extern PAC194X5X_DEVICE_CONTEXT PACdevice;
extern PPAC194X5X_DEVICE_CONTEXT pPACdevice;

extern uint8_t getOverVoltageStatus();
extern float getBatteryVoltage();

#ifdef __cplusplus

#endif /* __cplusplus */

#endif  /* PAC194X5X_H */
