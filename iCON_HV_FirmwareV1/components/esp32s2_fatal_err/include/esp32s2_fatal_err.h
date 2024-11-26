#ifndef _FATAL_ERR_H
#define _FATAL_ERR_H

#define SYS_RESET_COUNT 2

typedef enum module_type {
    I2C_MODULE = 0,
    OTA_UPDATE = 1,
    SERVER_RESET = 2,
    WATCHDOG = 3,
    TIME_SYNC = 4,
    FACTORY = 5,
    TCP_SERVER = 6,
} module_type_t;

typedef enum error_type {
    I2C_BUS_IS_BUSY = 0,
    PAC_READ_ERR = 1,
    EEPROM_READ_ERR = 2,
    EEPROM_WRITE_ERR = 3,
    OTA_UPDATE_DONE = 4,
    RST_SERVER_RESET = 5,
    WD_EXPIRED = 6,
    TIME_SYNC_FAILS = 7,
    FACTORY_RESET = 8,
    START_TCP_SERVER_FAILED = 9,
    BIND_FAILS = 10,
    LISTEN_FAILS = 11,
    SELECT_FAILS = 12,
    ACCEPT_FAILS = 13,



} error_type_t;

void fatal_err_handler(module_type_t mod,error_type_t error);
int get_soft_reset_reason(char* reason);
#endif // _FATAL_ERR_H
