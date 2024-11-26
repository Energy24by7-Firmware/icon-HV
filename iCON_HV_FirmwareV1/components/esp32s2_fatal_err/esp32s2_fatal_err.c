#include "esp32s2_fatal_err.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#define TAG "FATAL_ERR"
#define FILE_RESTART_RSN "/spiffs/FILE_RESTART_RSN"

static inline void update_soft_reset_reason(module_type_t mod)
{
    FILE* fp = fopen(FILE_RESTART_RSN, "w+");
    if(fp != NULL) {
        if( fwrite(&mod, 1, sizeof(module_type_t), fp) == sizeof(module_type_t)) {
            ESP_LOGI(TAG, "Writing value [%d] to [%s] SUCCESS",mod,FILE_RESTART_RSN);
        } else {
            ESP_LOGI(TAG, "Writing value [%d] to [%s] FAILS",mod,FILE_RESTART_RSN);
        }
        fclose(fp);
    } else {
        ESP_LOGI(TAG,"File [%s] open fails",FILE_RESTART_RSN);
    }
}

int get_soft_reset_reason(char* reason)
{
    int ret = 0;
    module_type_t mod;
    FILE* fp = fopen(FILE_RESTART_RSN, "r");
    if(fp != NULL) {
        fseek(fp, 0, SEEK_SET);
        fread(&mod, 1, sizeof(module_type_t), fp);
        fclose(fp);
        switch(mod) {
            case I2C_MODULE:
                strcpy(reason,"I2C MODULE");
                break;

            case OTA_UPDATE:
                strcpy(reason, "OTA_UPDATE");
                break;

            case SERVER_RESET:
                strcpy(reason,"SERVER_RESET");
                break;

            case WATCHDOG:
                strcpy(reason, "WATCHDOG");
                break;

            case TIME_SYNC:
                strcpy(reason, "TIME_SYNC_FAILED");
                break;

            case FACTORY:
                strcpy(reason, "FACTORY_RESET");
                break;

            case TCP_SERVER:
                strcpy(reason, "TCP_SERVER");
                break;

            default:
                strcpy(reason, "UNKNOWN");
                break;
        }
        ret = 0;
    } else {
        ret = -1;
    }
    return ret;
}

void fatal_err_handler(module_type_t mod,error_type_t error)
{
    ESP_LOGI(TAG,"FATAL ERROR HANDLER: module: %d, reason: %d",mod,error);
    update_soft_reset_reason(mod);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    esp_restart();
}
