#include "esp_system.h"
#include "bio_impedance.h"

// #define I2C_MASTER_SCL_IO GPIO_NUM_20
// #define I2C_MASTER_SDA_IO GPIO_NUM_21
#define SLAVE_ADDRESS1 0x27
#define SLAVE_ADDRESS2 0x26

void check_addr(uint8_t addr, char* cmsg);
void BIOIMP_reset(uint8_t addr);
void BIOIMP_task(void* argptr);
