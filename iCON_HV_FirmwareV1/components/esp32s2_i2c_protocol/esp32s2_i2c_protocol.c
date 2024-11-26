#include "esp32s2_i2c_protocol.h"
#include "esp_log.h"
 
i2c_port_t i2c_num; 
i2c_slave_addr_t i2c_slave_addr;
SemaphoreHandle_t i2c_bus_lock_semaphore = NULL;

static const char *TAG = "i2c_protocol";

/*This function initializes the I2C bus as an I2C master and sets up the bus configuration and driver. Additionally, it creates a semaphore to manage access to the I2C bus between different tasks.*/
void init_i2c(void)
{   
    //Configure I2C parameters:
    i2c_num = I2C_MASTER;
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000};

    
    // Apply the configuration to the I2C master
    esp_err_t err = i2c_param_config(I2C_MASTER, &i2c_config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C parameters configured successfully");
    } else {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        return;
    }

    // Installs the I2C driver in master mode. The last three arguments specify buffer sizes for slave mode, which aren't needed for master mode, so they are set to zero.
    err = i2c_driver_install(I2C_MASTER, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C driver installed successfully");
    } else {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
        return;
    }


    // Create a semaphore for i2c locking purpose..It will be used to ensure exclusive access to the I2C bus.
    i2c_bus_lock_semaphore = xSemaphoreCreateMutex();
    assert(i2c_bus_lock_semaphore != NULL); //if any error prints it..otherwise always true 
    
}



/*This function attempts to detect a slave device on the I2C bus by sending a write command to the slave's address. It performs a transaction to see if the slave acknowledges the request.*/
/*This function attempts to communicate with a slave device on the I2C bus and returns the result (whether the slave responded or not). It's used to detect if a specific slave is connected and functioning properly.*/
esp_err_t i2c_master_detect_slave(void)
{
    //Creates a command link handle, which is used to store I2C commands in sequence.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command link");
        return ESP_FAIL;
    }

    //Generates an I2C start condition on the bus, which signals the start of communication.
    i2c_master_start(cmd);

    //Sends the slave's address combined with the write bit (low). ACK_CHECK_EN means it will check for an acknowledgment from the slave.
    i2c_master_write_byte(cmd, (i2c_slave_addr) | WRITE_BIT, ACK_CHECK_EN);

    //Sends the stop condition to end the I2C transaction.
    i2c_master_stop(cmd);


    //Sends the entire I2C transaction to the bus. The function waits for up to 1000ms to get a response from the slave. If a response is received, the function returns ESP_OK; otherwise, it returns an error.
    //esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 500 / portTICK_PERIOD_MS);

    //Deletes the command link handle to free resources.
    i2c_cmd_link_delete(cmd);
    return ret;
}


/*This function is used to acquire the semaphore (i.e., lock the I2C bus) so that only one task can access the bus at a time.*/
/*This function is used to ensure that no other task can use the I2C bus while the current task is performing a transaction. It enforces mutual exclusion and avoids bus contention.*/
bool lock_i2c_bus(uint32_t wait_time_ms)
{
    //Ensures that the semaphore has been created.
    if(i2c_bus_lock_semaphore != NULL) {
        /*Attempts to take the semaphore. If successful within the specified wait_time_ms, it returns pdTRUE; otherwise, it fails and returns pdFALSE.*/
        if( xSemaphoreTake( i2c_bus_lock_semaphore, (wait_time_ms/portTICK_PERIOD_MS) ) == pdTRUE ) {
            // Able to obatin the semaphore, can access the i2c bus now
            return true;
        } else {
            // Could not obtain the semaphore, cannot access the i2c bus now
            return false;
        }
    
    } else {
        ESP_LOGI("I2C", "Locking semaphore is not initialised");
    }
    return false;
}


/*This function releases the semaphore (i.e., unlocks the I2C bus) so that other tasks can use it.*/
void unlock_i2c_bus()
{
     if(i2c_bus_lock_semaphore != NULL) {
         xSemaphoreGive(i2c_bus_lock_semaphore);
     }
}
