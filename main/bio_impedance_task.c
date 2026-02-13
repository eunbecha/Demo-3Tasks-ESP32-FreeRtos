#include "bio_impedance_task.h"
#include "bio_impedance.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

extern SemaphoreHandle_t i2cMutex;

#define BIO_TASK_PERIOD_MS   250

static const char *BIO_tasks_TAG = "BIO_Tasks:";

void BIO_reset(void)
{
    ESP_LOGI(BIO_tasks_TAG, "BIO_reset() starting");

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

        if (bio_impedance_init() != ESP_OK) {
            ESP_LOGE(BIO_tasks_TAG, "Bio device init failed");
        } else {
            ESP_LOGI(BIO_tasks_TAG, "Bio device initialized successfully");
        }

        xSemaphoreGive(i2cMutex);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void bio_impedance_task(void *arg)
{
    ESP_LOGI(BIO_tasks_TAG, "bio_impedance_task is starting up NOW");

    uint16_t bio_value = 0;

    BIO_reset();

    ESP_LOGI(BIO_tasks_TAG, "bio_impedance_task entering LOOP");

    while (1) {

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

            if (bio_impedance_read(&bio_value) == ESP_OK) {
                ESP_LOGI(BIO_tasks_TAG, "Bio value = %d", bio_value);
            } else {
                ESP_LOGE(BIO_tasks_TAG, "Bio read error");
            }

            xSemaphoreGive(i2cMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(BIO_TASK_PERIOD_MS));
    }
}
