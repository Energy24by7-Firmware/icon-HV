// ota_component.h
#ifndef OTA_COMPONENT_H
#define OTA_COMPONENT_H
#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "native_ota_example";

extern TaskHandle_t otaTaskHandle ;  // Declare a task handle


/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
extern const uint8_t chain_pem_start[] asm("_binary_chain_pem_start");
extern const uint8_t chain_pem_end[] asm("_binary_chain_pem_end");



#define OTA_URL_SIZE 256
void ota_example_task(void *pvParameter);  // Declaration of the OTA task function
void start_ota_update();
//void ota_main(void);
void ota_main_task(void *pvParameter);

#endif // OTA_COMPONENT_H
