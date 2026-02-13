#include "bio_impedance.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "BIO";


esp_err_t bio_impedance_init(void)
{
    ESP_LOGI(TAG, "Bio device init");

    i2c_master_port conf = {
        // TODO : the below values are from the lcd so replace with appropriate values for bio-impedance
        .mode = I2C_MODE_MASTER, // Set as I2C master mode
        .sda_io_num = I2C_MASTER_SDA_IO, // GPIO number for I2C SDA
        .scl_io_num = I2C_MASTER_SCL_IO, // GPIO number for I2C SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Enable pull-up on SDA line
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Enable pull-up on SCL line
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Set I2C clock frequency
    }

}


void bio_impedance_read(uint16_t *value)
{
    
}

// Add all the AD5933.c code needed to do the measurements