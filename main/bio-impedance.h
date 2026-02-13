#ifndef BIO_IMPEDANCE_H
#define BIO_IMPEDANCE_H

#include "esp_err.h"
#include <stdint.h>

#define BIO_I2C_ADDR   ?? # TODO: Get the real address

esp_err_t bio_impedance_init(void);
esp_err_t bio_impedance_read(uint16_t *value);

#endif
