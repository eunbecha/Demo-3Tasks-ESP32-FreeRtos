/* University of Washington TPT-Finder Project
 * Blake Hannaford,  July 25
 *
 * Derived from:
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

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
#include "i2c_lcd.h"
#include "LCD_task.h"
#include "unistd.h"
#include "timer_Photo.h"
#include "state_machine.h"

#include "bio_impedance_task.h"


//
//  Blake's demo of multiple FreeRTOS tasks
//            (Jul 25)
//
//     Adapted to drive the TPT-Finder photonics LED/Photodiode board
//            (Nov/Dec 25)
//


/*------------------------------------------------------------*/
/* Macros */
#define PROMPT_STR CONFIG_IDF_TARGET
#define TASK_PRIO_4         4
#define TASK_PRIO_3         3
#define TASK_PRIO_2         2
#define TASK_PRIO_1         1
#define COMP_LOOP_PERIOD    5000
#define SEM_CREATE_ERR_STR      "semaphore creation failed"
#define QUEUE_CREATE_ERR_STR    "queue creation failed"

//
//  Choose tasks which will be run
//
#define TASK_ON              1
#define TASK_OFF             0

#define LED_TASK            TASK_ON
#define LCD_TASK            TASK_OFF
#define STATE_MACHINE       TASK_OFF     // PHOTONIC TASK now timer driven by this
#define PHOTONIC_TASK       TASK_OFF
#define HELLO_WORLD_TASK    TASK_ON
#define PHOTONICS_TEST      TASK_OFF
#define CPU_LOAD_TASK       TASK_OFF

#define BIO_IMPEDANCE_TASK  TASK_ON


// LED Task related functions (in this file)
static void configure_led(void);
static void setLedFromState(void);
static void setLedFromArg(uint8_t);


/////////// BH
// Local function prototypes:

void handle_error(char* );  // log an error to console and freeze


//FREE-RTOS tasks defined here:
// blinker state
static uint8_t s_led_state = 0;
static void LED_task(void*);
static void hello_task(void *arg);

#define TAG  "TPT-main.c"

//   BH defines
#define DEFAULT_STACK  4096
#define BLINK_PERIOD    300 //ms
#define BLINK_GPIO      8
#define LED_BIT_ON      (uint8_t) 1
#define LED_BIT_OFF     (uint8_t) 0
#define IDLE_GPIO       18    // PC bd Test Point TP35


// Configure LED task
#define LED_TASK_TIMED    1  // 1 = periodic as above; 0 = load avg pwm



void handle_error(char* msg){
#define TAGe  "TASK ERROR: "
    while(1){  // freeze the system (sort of)
        ESP_LOGI(TAGe, "%s", msg);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}


extern char lcd_LOG_message[];

// i2c mutex:  this is used to make sure only one task can transact on i2c at a time.
SemaphoreHandle_t i2cMutex = NULL;


/*
 *   LED blink task
 */

static void LED_task(void*)
{
    while (1) {
        if(LED_TASK_TIMED){
            //
            // Normal LED Task:
            //
            // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
            setLedFromState();
            /* Toggle the LED state */
            s_led_state = !s_led_state;
            vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);
            }
        else {
            //
            //   LED task to show Free CPU Time
            //
            setLedFromArg(LED_BIT_ON); // turn   LED  (indicate busy)
            gpio_set_level(IDLE_GPIO, 0);  // Test point 33 on V02board
            vTaskDelay(1); // wait for next tick
            }
        // ESP_LOGI(TAG, "Starting LED cycle (%d)", (int)LED_TASK_TIMED);
        }
}

// set to lowest priority
static void cpu_load_task(void*)
{
    while(1){
        setLedFromArg(LED_BIT_OFF); // turn   LED (indicate idle)
        gpio_set_level(IDLE_GPIO, 1);  // Test point  V02board
        vTaskDelay(1);
        }
}

/*********************************************************************
 *   Initialize LED task differently depending on hardware
 *
 *   Following code block (from "blink" example) is overkill for
 *     Waveshare ESP32c6-zero for which
 *        we should configure:   Blink LED type:  "LED strip"
 *        and
 *                               LED strip backend peripheral "RMT"
 *
 *   TODO:  move this to a separate .c file for LED hardware setup.
 */

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void setLedFromArg(uint8_t on_off)
{
    /* If the addressable LED is enabled */
    if (on_off) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void setLedFromState(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configure pins to blink LED_STRIP LED");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
// RMT is the required config for WaveShare ESP32-C6
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    ESP_LOGI(TAG, "Configure LED_STRIP back-end RMT");

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}



// if this were configured to use GPIO (NOT what waveshare ESP32C6-Zero uses)
//
//   NOT for TPT-Finder HW
//
#elif CONFIG_BLINK_LED_GPIO

static void setLedFromState(void)
{
    ESP_LOGI(TAG,"GPIO: Setting LED from state");
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void setLedFromArg(uint8_t on_off)
    {
    gpio_set_level(BLINK_GPIO, on_off);
    }

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configure GPIO pins (GPIO LED MODE ONLY)!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

}

#else
#error "unsupported LED type"
#endif

/*
 *
 *    End of "blink code block"
 ***************************************************************/


static void hello_task(void *arg)
{

    int i=0;
    while(1) {
        vTaskDelay(2000/portTICK_PERIOD_MS);
        printf("\n\n\n");
        i++;
        printf("Hello world! (task rep: %d) \n", i);
        printf("\n\n\n");
    }
}

void app_main(void)
{


    /*
     * Validate task configuration constraints
     */
    if (CPU_LOAD_TASK == TASK_ON && LED_TASK == TASK_OFF){
        ESP_LOGI(TAG,"Error:  LED_TASK must be ON for CPU_LOAD_TASK.");
        handle_error("Stopping.");
        }
    if (STATE_MACHINE == TASK_ON && PHOTONIC_TASK == TASK_ON){
        ESP_LOGI(TAG,"Error:  PHOTONIC_TASK must be OFF for STATE_MACHINE.");
        handle_error("Stopping.");
    }



    /***************************************************************
     *
     * Hardware and Software setups and INITIALIZATIONS
     *
     */
    //   Set up i2c for all tasks
    i2cMutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "mutex created");
    i2c_master_init();  // now separate from lcd_init()
    ESP_LOGI(TAG, "i2c master is inited");
    
    if (LCD_TASK == TASK_ON) {
        // initialize LCD hardware
        LCD_reset(SLAVE_ADDRESS1_LCD);
        // LCD_reset(SLAVE_ADDRESS2_LCD);
        ESP_LOGI(TAG, "LCD device init completed ");
        }

    // config hardware GPIO pins for on-board LED (board-specific)
    if (LED_TASK == TASK_ON) {
        configure_led();   // defined above for two configs
        ESP_LOGI(TAG, "on-board LED hardware has been configured.");
        }

    if (STATE_MACHINE==TASK_ON){
        state_machine_init();
        ESP_LOGI(TAG, "State Machine has been set up.");
        }

    if (PHOTONIC_TASK == TASK_ON || PHOTONICS_TEST==TASK_ON) {
        // setup for photonics board interface.
        init_photonics();
        ESP_LOGI(TAG, "photonics pinouts have been set");
        }

    if (CPU_LOAD_TASK == TASK_ON){
        configure_led();   // defined above for two configs
        ESP_LOGI(TAG, "on-board LED hardware has been configured.");
        // Test Point TP33
        gpio_reset_pin(IDLE_GPIO);
        /* Set the GPIO as a push/pull output */
        gpio_set_direction(IDLE_GPIO, GPIO_MODE_OUTPUT);
    }
    if (BIO_IMPEDANCE_TASK == TASK_ON) {

        xTaskCreatePinnedToCore(
            BIOIMP_task,
            "Bio Impedance Task",
            DEFAULT_STACK,
            NULL,
            TASK_PRIO_2,
            NULL,
            tskNO_AFFINITY
        );

        ESP_LOGI(TAG, "Bio impedance task created");
    }



    /***********************************************************************
     *
     * Start up the Free-RTOS Tasks
     */
    void* argptr = NULL;  // use for task arguments

    ESP_LOGI(TAG, "\n\n      Starting task(s)...\n\n");


    if (STATE_MACHINE==TASK_ON){
         xTaskCreatePinnedToCore(state_machine_task, "State Machine Task", DEFAULT_STACK, NULL, TASK_PRIO_2, NULL, tskNO_AFFINITY);
        ESP_LOGI(TAG, "State Machine Task Created");
        }

    if (HELLO_WORLD_TASK==TASK_ON) {
    /*
     *   HELLO WORLD on serial console
     */
        xTaskCreatePinnedToCore(hello_task, "Hello World Task", DEFAULT_STACK, NULL, TASK_PRIO_2, NULL, tskNO_AFFINITY);
        ESP_LOGI(TAG, "Hello world (serial) task created");
        }

    if (LED_TASK==TASK_ON) {
        /*
        * Flash the onboard LED
        */
        //  set to highest priority for use in Idle time
        xTaskCreatePinnedToCore(LED_task, "LED Task", DEFAULT_STACK, NULL, TASK_PRIO_4, NULL, tskNO_AFFINITY);
        ESP_LOGI(TAG, "LED task created");

        //
        //  launch the cpu_load_task for measurement (via LED) of idle time
        //
        if (!LED_TASK_TIMED){
            ESP_LOGI(TAG, "LED configured for idle time... (!LED_TASK_TIMED)");
            //
            // low prio task to indicate idle CPU (light off)
            xTaskCreatePinnedToCore(cpu_load_task, "CPU load Task", DEFAULT_STACK, NULL, TASK_PRIO_2, NULL, tskNO_AFFINITY);

            ESP_LOGI(TAG, "cpu idle task created");
            }

        }

    if (PHOTONIC_TASK==TASK_ON) {
        //
        // Generate 100Hz cycle and coordinate OFF time  ON time and
        //        ADC readings
        //
        argptr = NULL;
        xTaskCreatePinnedToCore(photonic_task, "Photonics Task", DEFAULT_STACK, argptr, TASK_PRIO_3, NULL, tskNO_AFFINITY);
        ESP_LOGI(TAG, "Photonics task created");
        }

    if (LCD_TASK==TASK_ON) {
        //
        // Display messages on the LCD
        //
        uint8_t lcd_address1 = SLAVE_ADDRESS1_LCD;
        argptr = &lcd_address1;
        // uint8_t lcd_address2 = SLAVE_ADDRESS2_LCD;
        xTaskCreatePinnedToCore(LCD_task1, "LCD Task", DEFAULT_STACK, argptr, TASK_PRIO_2, NULL, tskNO_AFFINITY);
        // xTaskCreatePinnedToCore(LCD_task2, "LCD 16x2 Task", DEFAULT_STACK, (void*)lcd_address2, TASK_PRIO_2, NULL, tskNO_AFFINITY);
        ESP_LOGI(TAG, "LCD task created");
        }

    if(PHOTONICS_TEST==TASK_ON){
        while(1){
                int testval = photonic_test();
                ESP_LOGI(TAG, "A/D Test value: %d", testval);
                vTaskDelay(pdMS_TO_TICKS(1000));
                }
        }
}
