// John Kircher, Alex Prior, Allen Zou
// 11/24/2020

/* servo motor control example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// ADC Includes
#include "driver/adc.h"
#include "esp_adc_cal.h"

// I2C Includes
#include <math.h>
#include "driver/i2c.h"
// #include "./ADXL343.h"

#include <string.h>
#include <sys/param.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
// #include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "freertos/event_groups.h"

///
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

///
// #include "addr_from_stdin.h"

// #if defined(CONFIG_EXAMPLE_IPV4)
// #define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
// #elif defined(CONFIG_EXAMPLE_IPV6)
// #define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
// #else
#define HOST_IP_ADDR "192.168.1.164" // NEED UPDATE
// #endif

#define PORT 9001

static const char *TAG = "example";
static const char *payload = "";
bool startCrawler = false;

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1)
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1)
        {
            payload = "Hello";
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else
            {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "On!", 3) == 0)
                {
                    printf("!!!!! RECEIVED ON:!!!!!!!!!!!!!!!!!! \n");
                    //ESP_LOGI(TAG, "Received expected message, reconnecting");
                    startCrawler = !startCrawler;
                    break;
                }
                else
                {
                    printf("!!!!! RECEIVED NO:!!!!!!!!!!!!!!!!!! \n");
                }
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

// ADC Defines
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  //Multisampling

// I2C Defines
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_SCL_IO2 27       // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO2 33       // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_NUM2 I2C_NUM_1   // second i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   // i2c master clock freq
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF                       // i2c nack value

// ADXL343
#define SLAVE_ADDR 0x62 // Lidar slave address

// Global Ultrasonic ADC variable
int range;
int counter;

// Global wheelSpeed Variable
float wheelSpeed;

// Global I2C Lidar Variable
int lidarDistance;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC1_CHANNEL_3;     //GPIO39 if ADC1, GPIO14 if ADC2
static const adc_channel_t speedChannel = ADC_CHANNEL_6; //GPIO39 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define DRIVE_MIN_PULSEWIDTH 900     //Minimum pulse width in microsecond
#define DRIVE_MAX_PULSEWIDTH 1900    //Maximum pulse width in microsecond
#define DRIVE_MAX_DEGREE 180         //Maximum angle in degree upto which servo can rotate
#define STEERING_MIN_PULSEWIDTH 700  //Minimum pulse width in microsecond
#define STEERING_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define STEERING_MAX_DEGREE 180      //Maximum angle in degree upto which servo can rotate

// 14-Segment Display
#define SLAVE_ADDR2 0x70             // alphanumeric address
#define OSC 0x21                     // oscillator cmd
#define HT16K33_BLINK_DISPLAYON 0x01 // Display on cmd
#define HT16K33_BLINK_OFF 0          // Blink off cmd
#define HT16K33_BLINK_CMD 0x80       // Blink cmd
#define HT16K33_CMD_BRIGHTNESS 0xE0  // Brightness cmd

//number to binary conversion from ADA fruit backpack for display
static const uint16_t alphafonttable[] = {

    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0000100000000000, // ,
};

// LIDAR ---------------------------------------
// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init()
{
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define Lidar I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                        // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);     // Configure
    if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }

    // Install Lidar I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n");
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    // Debug
    printf("\n>> Alphanumeric i2c Config\n");

    // // // Port configuration
    int i2c_master_port2 = I2C_EXAMPLE_MASTER_NUM2;

    /// Define I2C configurations
    i2c_config_t conf2;
    conf2.mode = I2C_MODE_MASTER;                        // Master mode
    conf2.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO2;       // Default SDA pin
    conf2.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf2.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO2;       // Default SCL pin
    conf2.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf2.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
    err = i2c_param_config(i2c_master_port2, &conf2);    // Configure
    if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port2, conf2.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n\n");
    }

    // // Dat in MSB mode
    // i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
    /////////////////////////////////////////////////////////////////////////////////////////

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner()
{
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."
           "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++)
    {
        // printf("0x%X%s", i, "\n");
        if (testConnection(i, scanTimeout) == ESP_OK)
        {
            printf("- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
    {
        printf("- No I2C devices found!"
               "\n");
    }
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
// int getDeviceID(uint8_t *data)
// {
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data)
{
    // int ret;
    // printf("--Writing %d to reg %d!--\n", data, reg);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start command
    i2c_master_start(cmd);
    //slave address followed by write bit
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    //register pointer sent
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //data sent
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    //stop command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    // if (ret == ESP_OK)
    // {
    //   printf("I2C SUCCESSFUL \n");
    // }
    i2c_cmd_link_delete(cmd);
}

// Read register
uint8_t readRegister(uint8_t reg)
{
    uint8_t value;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start command
    i2c_master_start(cmd);
    //slave followed by write bit
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    //register pointer sent
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //stop command
    // i2c_master_stop(cmd);

    //repeated start command
    i2c_master_start(cmd);
    //slave followed by read bit
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    //place data from register into bus
    i2c_master_read_byte(cmd, &value, ACK_CHECK_DIS);
    //stop command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return value;
}

static void lidarRead()
{
    uint8_t initReg = 0x00;
    uint8_t initData = 0x04;
    uint8_t data = 0x06;
    vTaskDelay(22);
    while (1)
    {
        // Initialize lidar device

        writeRegister(initReg, initData);
        data = readRegister(0x01);
        while ((data & 1) != 0x00)
        {
            data = readRegister(0x01);
            // printf("content of reg 1 LSB is %x \n", (data & 1));
            vTaskDelay(10);
        }
        // printf("content of reg 1 LSB outsie of the while loop is %x \n", (data & 1));
        // if (data == 0)
        // {
        // Read high and low distance bits off device
        uint8_t distHigh = readRegister(0x11);
        uint8_t distLow = readRegister(0x10);

        // Print high and low distance data
        printf("distHigh is: %x \n", distHigh);
        printf("distLow is: %x \n", distLow);

        //Add the two distances into a 16 bit int
        lidarDistance = (distHigh << 8) + distLow;

        printf("--------------------Distance is: %d-------------------- \n", lidarDistance);
        // }
        vTaskDelay(10);
    }
}

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator()
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR2 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM2, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set blink rate to off
int no_blink()
{
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (SLAVE_ADDR2 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM2, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val)
{
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (SLAVE_ADDR2 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM2, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

static void test_alpha_display()
{
    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if (ret == ESP_OK)
    {
        printf("- oscillator: ok \n");
    }
    // Set display blink off
    ret = no_blink();
    if (ret == ESP_OK)
    {
        printf("- blink: off \n");
    }
    ret = set_brightness_max(0xF);
    if (ret == ESP_OK)
    {
        printf("- brightness: max \n");
    }
    // // Write to characters to buffer
    uint16_t displaybuffer[8];
    int alphaSpeed;

    // Continually writes the same command
    while (1)
    {
        alphaSpeed = wheelSpeed * 10;
        // printf("alphaSpeed is: %d \n", alphaSpeed);

        displaybuffer[0] = alphafonttable[0];          // 1.
        displaybuffer[1] = alphafonttable[0];          // 2.
        displaybuffer[2] = alphafonttable[10];         // 3.
        displaybuffer[3] = alphafonttable[alphaSpeed]; // 4.
        // vTaskDelay(100);

        // Send commands characters to display over I2C
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, (SLAVE_ADDR2 << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i = 0; i < 8; i++)
        {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM2, cmd4, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd4);

        // for (int i = 0; i < 8; i++)
        // {
        //     printf("%04x\n", displaybuffer[i]);
        // }

        // if (ret == ESP_OK)
        // {
        //     printf("- wrote:  \n\n");
        // }
    }
}

// PID -----------------------------------------

// Flag for dt

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 26); //Set GPIO 26 as PWM0A, to which drive wheels are connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 25); //Set GPIO 25 as PWM0A, to which steering servo is connected
}

// /**
//  * @brief Use this function to calcute pulse width for per degree rotation
//  *
//  * @param  degree_of_rotation the angle in degree to which servo has to rotate
//  *
//  * @return
//  *     - calculated pulse width
//  */
// static uint32_t drive_per_degree_init(uint32_t degree_of_rotation)
// {
//     uint32_t cal_pulsewidth = 0;
//     cal_pulsewidth = (DRIVE_MIN_PULSEWIDTH + (((DRIVE_MAX_PULSEWIDTH - DRIVE_MIN_PULSEWIDTH) * (degree_of_rotation)) / (DRIVE_MAX_DEGREE)));
//     return cal_pulsewidth;
// }

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t steering_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (STEERING_MIN_PULSEWIDTH + (((STEERING_MAX_PULSEWIDTH - STEERING_MIN_PULSEWIDTH) * (degree_of_rotation)) / (STEERING_MAX_DEGREE)));
    return cal_pulsewidth;
}

void pwm_init()
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
}

void calibrateESC()
{
    vTaskDelay(3000 / portTICK_PERIOD_MS);                                // Give yourself time to turn on crawler
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds - backwards
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // LOW signal in microseconds - forwards
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
/**
 * @brief Configure MCPWM module
 */

int dt_complete = 0;

int dt = 50;
int setpoint = 100;

float previous_error = 0.00; // Set up PID loop
float integral = 0.00;
float derivative;
float output;
int error;

int Kp;
int Kd;
int Ki;

void PID()
{
    integral = 0.00;
    previous_error = 0.00;
    while (1)
    {
        if (startCrawler)
        {
            error = setpoint - range;
            printf("\nError is: %d\n", error);
            if (error > 74)
            {
                // vTaskDelay(10);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
                printf("Range: %d \n", range);
                printf("1400 \n");
            }
            else if (error > 20)
            {
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1285);
                printf("Range: %d \n", range);
                printf("1300 \n");
            }
            else if (error <= 20)
            {
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1279);
                printf("Range: %d \n", range);
                printf("1280 \n");
            }

            integral = integral + error * dt;
            derivative = (error - previous_error) / dt;
            output = Kp * error + Ki * integral + Kd * derivative;
            previous_error = error;
            vTaskDelay(dt);
        }
        else
        {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
        }
    }
}

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

void steering_control(void *arg)
{

    uint32_t slightlyRight, center, slightlyLeft;
    // right,
    // slightlyMoreRight,
    // left,
    // slightlyMoreLeft,

    vTaskDelay(60);

    while (1)
    {

        // right = steering_per_degree_init(0);
        // slightyMoreRight = steer_per_degree_init(35);
        slightlyRight = steering_per_degree_init(70);

        center = steering_per_degree_init(105);

        // left = steering_per_degree_init(180);
        // slightlyMoreLeft = steer_per_degree_init(150);
        slightlyLeft = steering_per_degree_init(140);
        if (lidarDistance <= 35)
        {
            printf("GOING LEFT and LIDAR IS %d ===============\n", lidarDistance);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, slightlyRight);
            // vTaskDelay(5);
            // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, center);
            vTaskDelay(10);
        }
        else if (lidarDistance > 36)
        {
            printf("GOING RIGHT and LIDAR is %d ============== \n", lidarDistance);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, slightlyLeft);
            // vTaskDelay(5);
            // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, center);
            vTaskDelay(10);
        }
        // else
        // {
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, center);
        // }
    }
}

void speedCalc(void *arg)
{
    while (1)
    {
        counter = 0;
        vTaskDelay(100);
        printf("Counter: %d \n", counter);
        wheelSpeed = (counter * (2 * 3.14159 * 7 / 12)) / 100;
        printf("Wheel Speed: %.1f m/s \n", wheelSpeed);
        counter = 0;
    }
}

void opticalData(void *arg)
{
    counter = 0;

    bool pulsed = false;
    //Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)speedChannel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)speedChannel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;

        if (adc_reading < 4095 && pulsed == false)
        {
            counter++;
            printf(" ");
            // printf("Counter incremented to: %d \n", counter);
            pulsed = true;
        }
        else if (adc_reading == 4095)
        {
            pulsed = false;
        }
    }
}

void ultrasonicData(void *arg)
{
    //Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        uint32_t vcm = 3.222;  //conversion to get volts per centimeter. This is found by 3.3V / 1024
        range = voltage / vcm; //calculation to get range in centimeters.
        // printf("Raw: %d\tCentimeters: %dcm\n", adc_reading, range);
        vTaskDelay(20); //random number rn CHANGE!!
    }
}

void app_main(void)
{
    pwm_init();
    calibrateESC();
    i2c_master_init();
    i2c_scanner();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, 5, NULL);

    printf("Testing servo motor.......\n");
    // xTaskCreate(drive_control, "drive_control", 4096, NULL, 4, NULL);
    xTaskCreate(steering_control, "steering_control", 4096, NULL, 5, NULL);

    // PID Task
    xTaskCreate(PID, "PID", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    xTaskCreate(test_alpha_display, "test_alpha_display", 4096, NULL, 5, NULL);
    xTaskCreate(ultrasonicData, "ultrasonicData", 4096, NULL, 5, NULL);
    xTaskCreate(speedCalc, "speedCalc", 4096, NULL, 5, NULL);
    xTaskCreate(opticalData, "opticalData", 4096, NULL, 5, NULL);

    // Create task to poll Lidar
    xTaskCreate(lidarRead, "lidarRead", 4096, NULL, 5, NULL);
}