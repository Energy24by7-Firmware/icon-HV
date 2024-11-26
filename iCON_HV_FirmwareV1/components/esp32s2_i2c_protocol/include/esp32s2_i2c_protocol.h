#ifndef _esp32s2_i2c_protocol_
#define _esp32s2_i2c_protocol_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_types.h"
#include "esp_log.h"

#define SDA_GPIO   18
#define SCL_GPIO   19
#define I2C_MASTER I2C_NUM_0

typedef enum{
    eeprom_addr  = 0xA8,
    ina_bat_addr = 0x82,
    ina_sol_addr = 0x8A,
}i2c_slave_addr_t;

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

extern i2c_port_t i2c_num; 
extern i2c_slave_addr_t i2c_slave_addr;

void init_i2c(void);
esp_err_t i2c_master_detect_slave(void);
bool lock_i2c_bus(uint32_t wait_time_ms);
void unlock_i2c_bus();

#endif
