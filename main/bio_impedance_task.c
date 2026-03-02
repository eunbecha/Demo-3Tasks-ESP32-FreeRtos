#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "led_strip.h"
#include "esp_system.h"
#include "bio_impedance.h"
#include "bio_impedance_task.h"
#include "unistd.h"

extern SemaphoreHandle_t i2cMutex;
extern void handle_error(char* msg);

#ifdef CONFIG_DRIVE_BIOIMP_YES

#define BIOIMP_TAG "BIOIMP_Tasks:"

uint8_t check_addr(uint8_t addr, char* cmsg)
{
    if (addr != SLAVE_ADDRESS1 && addr != SLAVE_ADDRESS2) {
        char local_msg[100];
        snprintf(local_msg, sizeof(local_msg),
                 "illegal address (i2c bus): 0x%x [%s]",
                 addr, cmsg);
        handle_error(local_msg);
    }
    return addr;
}

void BIOIMP_reset(uint8_t addr)
{
    check_addr(addr, "BIOIMP_reset");

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

        AD5933_set_reg_value(
            AD5933_REG_CONTROL_HB,
            AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_STANDBY)
        );

        xSemaphoreGive(i2cMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

void BIOIMP_task(void* argptr)
{
    ESP_LOGI(BIOIMP_TAG, "BIOIMP_task starting");

    uint8_t addr = *((uint8_t*)argptr);
    check_addr(addr, "BIOIMP_task startup");

    int16_t real_arr[NUM_INCR];
    int16_t imag_arr[NUM_INCR];

    while (1) {

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

            AD5933_start_freq_sweep(real_arr, imag_arr);

            xSemaphoreGive(i2cMutex);
        }

        for (int i = 0; i < NUM_INCR; i++) {

            double impedance =
                AD5933_calculate_impedance(
                    gain_factor,
                    real_arr[i],
                    imag_arr[i]);

            ESP_LOGI(BIOIMP_TAG,
                     "Freq %d -> Z = %f",
                     i,
                     impedance);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// #else

// // Stubs if feature disabled
// void BIOIMP_reset(uint8_t addr) {}
// void BIOIMP_task(void* argptr) {}

// #endif