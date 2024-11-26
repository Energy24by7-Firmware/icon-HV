#ifndef COMPONENT_H
#define COMPONENT_H
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_err.h"
#include "nimble/nimble_port.h"

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
extern TaskHandle_t bleTaskHandle;
extern char wifi_ssid[32];
extern char wifi_password[64];

extern bool APP1_flag;     // Use 'extern' to declare
extern bool config_flag;   // Use 'extern' to declare
extern bool ATE_Config_flag;

// Function declarations

void load_config_flag(void);


void ble_app_advertise(void);
void ble_server_init(void);
void process_config_command(const char *key, const char *value);
void send_response_to_app(const char *message);
extern const struct ble_gatt_svc_def gatt_svcs[];
int ble_gap_event(struct ble_gap_event *event, void *arg);

void checkAndEnableBLETask(void *pvParameters);
#endif // COMPONENT_H
