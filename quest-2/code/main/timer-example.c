// John Kircher, Alex Prior, Allen Zou
// 10/8/2020

/*
  - Barebones example using one timer as an interval alarm
  - Code prints every 1 second
  - Adapted and simplified from ESP-IDF timer example
  - See examples for more timer functionality

  Emily Lam, September 2019

*/

//Code chunks that we edited starts around line 367

#include <stdio.h>
#include <math.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_attr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_SEC (1)                       // Sample test interval for the first timer
#define TEST_WITH_RELOAD 1                           // Testing will be done with auto reload
#define GPIOFLAG 14

// ADC initialization
static esp_adc_cal_characteristics_t *adc_chars;

// Thermistor initialization
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  //Multisampling
int counter = 1;
int voltageRunSum = 0;
int voltageAVG = 0;
char tempStr[5];

//Thermistor ADC pin
static const adc_channel_t thermistorChannel = ADC_CHANNEL_6; //GPIO34 if ADC1
// IR ADC pin
static const adc_channel_t irChannel = ADC_CHANNEL_4; //GPIO32 if ADC1
// Ultrasonic ADC pin
static const adc_channel_t ultrasonicChannel = ADC_CHANNEL_5; //GPIO33 if ADC1

static const adc_atten_t atten = ADC_ATTEN_DB_11; // Changed attenuation to extend range of measurement up to approx. 2600mV (Appears to accurately read input voltage up to at least 3300mV though)
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        // printf("eFuse Two Point: Supported\n");
    }
    else
    {
        // printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        // printf("eFuse Vref: Supported\n");
    }
    else
    {
        // printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        // printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        // printf("Characterized using eFuse Vref\n");
    }
    else
    {
        // printf("Characterized using Default Vref\n");
    }
}

//This task is responsible for reading the thermistor data
static void thermoHandler()
{
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(thermistorChannel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)thermistorChannel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)thermistorChannel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)thermistorChannel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        voltageRunSum += voltage;
        //every two seconds do this
        if (counter == 2)
        {
            counter = 0;
            //calculate variable resistance and temperature
            float resistance = (10000 * (3.3 - voltage / 1000.0)) / (voltage / 1000.0);
            float temperature = 1.0 / ((1.0 / 298) + (1.0 / 3435.0) * log(resistance / 10000.0)); // Simplified B-parameter Steinhart-Hart equation
            temperature -= 273.15;

            printf("Temperature,%f\n", temperature);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        counter++;
    }
}

// This task is responsible for reading the IR sensor data
static void IRhandler()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(irChannel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)irChannel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)irChannel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)irChannel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float range = 146060 * (pow(voltage, -1.126)) - 50;
        printf("irDistance,%.2f\n", range);
        //do this every two seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

//This task is responsible for reading the ultrasonic data
static void ultrasonicHandler()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ultrasonicChannel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)ultrasonicChannel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)ultrasonicChannel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)ultrasonicChannel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        uint32_t vcm = 3.222;             //conversion to get volts per centimeter. This is found by 3.3V / 1024
        float range = voltage / vcm - 30; //calculation to get range in centimeters.

        printf("ultraDistance,%.2f\n", range);
        vTaskDelay(pdMS_TO_TICKS(1000)); //2 second delay
    }
}

void app_main(void)
{
    xTaskCreate(thermoHandler, "thermoHandler_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(ultrasonicHandler, "ultrasonicHandler_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(IRhandler, "IRhandler_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,
                                        256, 0, 0, NULL, 0));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
}
