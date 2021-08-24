// Alex Prior, John Kircher, Allen Zou
// 10/29/2020

/* Infrared IR/UART beacons for crawler capture the flag!
   November 2019 -- Emily Lam

   RMT Pulse          -- pin 26 -- A0
   UART Transmitter   -- pin 25 -- A1
   UART Receiver      -- pin 34 -- A2

   Hardware interrupt -- pin 4 -- A5
   ID Indicator       -- pin 13 -- Onboard LED

   Red LED            -- pin 33
   Green LED          -- pin 32
   Blue LED           -- Pin 14

   Features:
   - Sends UART payload -- | START | myColor | myID | Checksum? |
   - Outputs 38kHz using RMT for IR transmission
   - Onboard LED blinks device ID (myID)
   - Button press to change device ID
   - RGB LED shows traffic light state (red, green, yellow)
   - Timer controls traffic light state (r - 10s, g - 10s, y - 2s)
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <sys/param.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_types.h"
#include "freertos/queue.h"
#include <ctype.h>
#include <stdlib.h>
#include "esp_vfs_dev.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/event_groups.h"

#define MAX 100

// RMT definitions
#define RMT_TX_CHANNEL 1                                 // RMT channel for transmitter
#define RMT_TX_GPIO_NUM 25                               // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV 100                                  // RMT counter clock divider
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US 9500                       // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1 4
#define BUTTON2 33
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_1
int sendFlag = 0;

// LED Output pins definitions
#define BLUEPIN 14
#define GREENPIN 32
#define REDPIN 15
#define ONBOARD 13
#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_2_SEC (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD 1 // Testing will be done with auto reload

//Timer Variables
#define ELECTION_TIMEOUT 3
#define LEADER_TIMEOUT 6
#define HEARTBEAT 1
#define UDP_TIMER 1

//Voter Variables
char voterVote = 'B';
char voterVote_char[MAX] = "B";

char voterID = '0';
char voterID_char[MAX] = "0";

int IRSentFlag = 0;
int RecvFlag = 0;
int UDPFlag = 0;

int timeout = ELECTION_TIMEOUT;
int udpTimer = UDP_TIMER;

static const char *TAG = "example";

static const char *payload = "";

typedef enum
{
    ELECTION_STATE,
    LEADER_STATE,
    FOLLOWER_STATE
} state_e;

char status[MAX] = "No_Leader";
char myID[MAX] = "1";
char myID_CHAR = '1';
char deviceAge[MAX] = "New";
char data[MAX];
char leaderHeartbeat[MAX] = "Dead";
state_e deviceState = ELECTION_STATE;
char transmitting[MAX] = "Yes";
char leaderIP[MAX] = "";

char IPtable[3][20] = {
    "192.168.1.171",
    "192.168.1.165",
    "192.168.1.189"};

#define NUM_FOBS 3

// Default ID/color
#define ID 1
#define COLOR 'G'

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char colorID = (char)ID;
char myColor = COLOR;
int len_out = 4;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// A simple structure to pass "events" to main task
typedef struct
{
    int flag; // flag for enabling stuff in timer task
} timer_event_t;

// System tags
static const char *TAG_SYSTEM = "system"; // For debug logs

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para)
{

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Yellow is shorter
    if (myColor == 'G')
    {
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
    }
    else
    {
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    }

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum
char genCheckSum(char *p, int len)
{
    char temp = 0;
    for (int i = 0; i < len; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("%X\n",temp);

    return temp;
}
bool checkCheckSum(uint8_t *p, int len)
{
    char temp = (char)0;
    bool isValid;
    for (int i = 0; i < len - 1; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("Check: %02X ", temp);
    if (temp == p[len - 1])
    {
        isValid = true;
    }
    else
    {
        isValid = false;
    }
    return isValid;
}

// Init Functions //////////////////////////////////////////////////////////////
// RMT tx init
static void rmt_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init()
{
    // Basic configs
    uart_config_t uart_config = {
        .baud_rate = 1200, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);

    // Set UART pins using UART0 default pins
    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Reverse receive logic line
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    // Install UART driver
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init()
{
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
}

// Configure timer
static void alarm_init()
{
    // Select and initialize basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
                       (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Button interrupt init
static void button_init()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task

    // other button init
    gpio_pad_select_gpio(BUTTON2);
    gpio_set_direction(BUTTON2, GPIO_MODE_INPUT);
    // Button 2 task-- send data through IR
}

////////////////////////////////////////////////////////////////////////////////

// Tasks ///////////////////////////////////////////////////////////////////////
// Button task -- rotate through colorIDs
void button_task()
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            xSemaphoreTake(mux, portMAX_DELAY);
            if (colorID == 1)
            {
                colorID = 0;
            }
            else
            {
                colorID++;
            }
            xSemaphoreGive(mux);
            printf("Button pressed.\n");
            printf("colorID: %i", colorID);
            printf("\n");
            // printf(myID);
            // printf("\n");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void button_2_task()
{
    int button_flag;
    while (1)
    {
        button_flag = gpio_get_level(BUTTON2);
        if (!button_flag)
        {
            printf("button 2 pressed\n");
            sendFlag = 1;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        sendFlag = 0;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Receives task -- looks for Start byte then stores received values
void recv_task()
{
    // Buffer for input data
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        if (sendFlag && deviceState != LEADER_STATE)
        {
            char *data_out = (char *)malloc(len_out);
            xSemaphoreTake(mux, portMAX_DELAY);
            // printf("myID is %c \n", myID[0]);
            data_out[0] = start;
            data_out[1] = myColor;
            data_out[2] = myID_CHAR;
            data_out[3] = genCheckSum(data_out, len_out - 1);

            // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

            for (int i = 0; i < 5; i++)
            {
                uart_write_bytes(UART_NUM_1, data_out, len_out);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            sendFlag = 0;
            printf("data sucessfully sent\n");
            xSemaphoreGive(mux);
        }
        else
        {
            int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
            if (len_in > 0)
            {
                if (data_in[0] == start)
                {
                    if (checkCheckSum(data_in, len_out))
                    {

                        ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
                        //NOTE: CHANGE MYCOLOR AND COLORID TO IRRECVCOLOR AND ID
                        voterVote = (char)data_in[1];
                        voterID = (char)data_in[2];
                        if (voterVote == 'R')
                        {
                            gpio_set_level(REDPIN, 1);
                            gpio_set_level(BLUEPIN, 0);
                        }
                        else if (voterVote == 'B')
                        {
                            gpio_set_level(REDPIN, 0);
                            gpio_set_level(BLUEPIN, 1);
                        }
                        RecvFlag = 1;
                        printf("ir data successfully recieved!\n");
                        printf("and the voter is %c and they are voting for %c \n", voterID, voterVote);
                        //TRIP GLOBAL IRRECV FLAG

                        // ********** recieved a vote here ******** //
                        // - communicate the vote to the poll leader through UDP
                        //
                    }
                }
            }
            else
            {
                // printf("Nothing received.\n");
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

// LED task to light LED based on traffic state
void led_task()
{
    while (1)
    {
        switch ((int)colorID)
        {
        case 1: // Red
            // gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 1);
            gpio_set_level(BLUEPIN, 0);
            myColor = 'R';
            // printf("Current state: %c\n",status);
            break;
        case 0: // Blue
            // gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 1);
            myColor = 'B';
            // printf("Current state: %c\n",status);
            break;
        }

        if (deviceState == LEADER_STATE)
        {
            gpio_set_level(GREENPIN, 1);
        }
        else
        {
            gpio_set_level(GREENPIN, 0);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// LED task to blink onboard LED based on ID
void id_task()
{
    while (1)
    {
        for (int i = 0; i < (int)colorID; i++)
        {
            gpio_set_level(ONBOARD, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(ONBOARD, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// The main task of this example program
static void timer_evt_task(void *arg)
{
    while (1)
    {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1)
        {
            // printf("Action!\n");
            timeout--;
            udpTimer--;
            if (timeout <= 0 && deviceState == ELECTION_STATE)
            {
                printf("GOING TO LEADER STATE\n");
                deviceState = LEADER_STATE; // Change to leader state (Last remaining device in election state)
            }

            if (timeout <= 0 && deviceState == FOLLOWER_STATE)
            {
                deviceState = ELECTION_STATE; // Change to election state
                timeout = ELECTION_TIMEOUT;   // Change timeout variable to election timeout constant
            }

            if (deviceState == LEADER_STATE)
            {
                if (udpTimer < 0)
                {
                    udpTimer = HEARTBEAT;
                }
                printf("I AM THE LEADER!!! with timeout %d\n", timeout);
                strcpy(leaderHeartbeat, "Alive"); // Change leaderHeartbeat parameter in payload to "Alive" upon being elected leader
                strcpy(status, "Leader");         // Change status to "Leader"
            }
        }
    }
}

//UDP server port
#define PORT 9002

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1)
    {
        if (addr_family == AF_INET)
        {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        }
        else if (addr_family == AF_INET6)
        {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6)
        {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1)
        {
            char recv_status[MAX];
            char recv_ID[MAX];
            char recv_deviceAge[MAX];
            char recv_leaderHeartbeat[MAX];
            char tempBuffer[MAX];
            // int counter=0;

            int irMsgFlag = 0;

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
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
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET)
                {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                }
                else if (source_addr.sin6_family == PF_INET6)
                {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...

                strcpy(tempBuffer, rx_buffer);
                // Returns first token
                char *token = strtok(rx_buffer, ",");

                // Keep printing tokens while one of the
                // delimiters present in str[].
                for (int i = 0; i < 4; i++)
                {
                    // printf("%s\n", token);
                    if (i == 0)
                    {
                        // printf("token at i=0: %s", token);
                        // printf("\n");
                        strcpy(recv_status, token);
                    }
                    else if (i == 1)
                    {
                        if (strcmp(token, "R") == 0 || strcmp(token, "B") == 0)
                        {
                            irMsgFlag = 1;
                            break;
                        }
                        strcpy(recv_ID, token);
                        // printf("token at i=1: %s", token);
                        // printf("\n");
                    }
                    else if (i == 2)
                    {
                        strcpy(recv_deviceAge, token);
                        // printf("token at i=2: %s", token);
                        // printf("\n");
                    }
                    else if (i == 3)
                    {
                        strcpy(recv_leaderHeartbeat, token);
                        // printf("token at i=3: %s", token);
                        // printf("\n");
                    }

                    token = strtok(NULL, ",");
                }

                printf("TEMPBUFFER is %s \n", tempBuffer);
                char *token2 = strtok(tempBuffer, ",");
                printf("token2 successfully created \n");

                if (irMsgFlag == 1)
                {
                    while (token2 != NULL)
                    {
                        printf(" %s\n", token2);
                        // if (counter == 0){
                        //     voterID_char = token2;
                        //     counter++;
                        // }
                        // else if (counter == 1){
                        //     voterVote_char = token2;
                        //     counter=0;
                        // }
                        if (strcmp(token2, "B") == 0)
                        {
                            voterVote = 66;
                        }
                        else if (strcmp(token2, "R") == 0)
                        {
                            voterVote = 82;
                        }
                        else if (strcmp(token2, "0") == 0)
                        {
                            voterID = 48;
                        }
                        else if (strcmp(token2, "1") == 0)
                        {
                            voterID = 49;
                        }
                        else if (strcmp(token2, "2") == 0)
                        {
                            voterID = 50;
                        }
                        else if (strcmp(token2, "3") == 0)
                        {
                            voterID = 51;
                        }
                        else if (strcmp(token2, "4") == 0)
                        {
                            voterID = 52;
                        }
                        else if (strcmp(token2, "5") == 0)
                        {
                            voterID = 53;
                        }
                        else if (strcmp(token2, "6") == 0)
                        {
                            voterID = 54;
                        }
                        else if (strcmp(token2, "7") == 0)
                        {
                            voterID = 55;
                        }
                        else if (strcmp(token2, "8") == 0)
                        {
                            voterID = 56;
                        }
                        else if (strcmp(token2, "9") == 0)
                        {
                            voterID = 56;
                        }

                        token2 = strtok(NULL, ",");
                    }
                    // for (int i = 0; i < 2; i++) {

                    //     printf("Loop #%d", i);
                    //     printf("\n");

                    //     printf("Token2 is %s", token2);
                    //     printf("\n");

                    //     if (i == 0) {
                    //         strcpy(voterID, token2);
                    //     } else if (i == 1) {
                    //         strcpy(voterVote, token2);
                    //     }
                    //     token2 = strtok(NULL, ",");
                    // }
                    printf("================ voterID is %d \n", voterID);
                    printf("================ voterVote is %d \n", voterVote);
                    RecvFlag = 1;
                    irMsgFlag = 0;
                }

                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                printf("recv_status is %s, recv_ID is %s, recv_deviceAge is %s, recv_leaderHeartbeat is %s \n", recv_status, recv_ID, recv_deviceAge, recv_leaderHeartbeat);
                ESP_LOGI(TAG, "%s", rx_buffer);

                // Check device state and handle incoming data accordingly
                int myID_num = atoi(myID);
                int recv_ID_num = atoi(recv_ID);
                if (deviceState == ELECTION_STATE)
                {
                    printf("ELECTION STATE and timeout is %d\n", timeout);
                    strcpy(status, "No_Leader"); // Status is "No_Leader"
                    strcpy(transmitting, "Yes"); // Continue transmitting
                    if (myID_num < recv_ID_num)
                    {
                        deviceState = ELECTION_STATE; // Stay in election state
                        strcpy(status, "No_Leader");  // Status is "No_Leader"
                        strcpy(transmitting, "Yes");  // Continue transmitting
                        timeout = ELECTION_TIMEOUT;   // Reset election timeout
                    }
                    else if (myID_num > recv_ID_num)
                    {
                        deviceState = FOLLOWER_STATE; // Change to follower state
                        timeout = LEADER_TIMEOUT;     // Change timeout variable to leader timeout constant
                    }
                }
                else if (deviceState == FOLLOWER_STATE)
                {
                    printf("FOLLOWER STATE with timeout %d\n", timeout);
                    udpTimer = UDP_TIMER * 2;
                    strcpy(transmitting, "No"); // Stop transmitting
                    if (strcmp(recv_deviceAge, "New") == 0)
                    {
                        deviceState = ELECTION_STATE; // Change to election state
                    }
                    else if (strcmp(recv_leaderHeartbeat, "Alive") == 0)
                    {
                        timeout = LEADER_TIMEOUT; // Reset leader timeout upon receiving leader heartbeat
                        strcpy(status, "Leader"); // Update status to leader upon receiving leader heartbeat
                        printf("leaderIP is originally %s \n", leaderIP);
                        strcpy(leaderIP, IPtable[recv_ID_num]);
                        printf("leaderIP is now changed to %s with the recv_ID_num of %d \n", leaderIP, recv_ID_num);
                    }
                    else if (timeout <= 0)
                    {
                        deviceState = ELECTION_STATE; // Change to election state
                        timeout = ELECTION_TIMEOUT;   // Change timeout variable to election timeout constant
                    }
                }
                else if (deviceState == LEADER_STATE)
                {
                    if (strcmp(recv_deviceAge, "New") == 0)
                    {
                        deviceState = ELECTION_STATE; // Change state to election state
                        timeout = ELECTION_TIMEOUT;   // Change timeout variable to election timeout constant
                    }
                }

                memset(recv_deviceAge, 0, sizeof(data));
                memset(recv_ID, 0, sizeof(data));
                memset(recv_status, 0, sizeof(data));
                memset(recv_leaderHeartbeat, 0, sizeof(data));

                //ERROR CHECKING
                int err = sendto(sock, payload, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
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

//UDP client
char HOST_IP_ADDR[MAX] = "192.168.1.171";
#define NODE_IP_ADDR "192.168.1.190"
#define PORT2 9002

static void
udp_client_task(void *pvParameters)
{
    // char rx_buffer[128];
    // char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1)
    {
        // printf("-------------------------------- \n");
        // for (int i = 0; i < NUM_FOBS; i++)
        // {
        //     printf("i is %d \n", i);
        // }

        // #elif defined(CONFIG_EXAMPLE_IPV6)
        //             struct sockaddr_in6 dest_addr = {0};
        //             inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        //             dest_addr.sin6_family = AF_INET6;
        //             dest_addr.sin6_port = htons(PORT2);
        //             dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        //             addr_family = AF_INET6;
        //             ip_protocol = IPPROTO_IPV6;
        // #elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        //             struct sockaddr_in6 dest_addr = {0};
        //             ESP_ERROR_CHECK(get_addr_from_stdin(PORT2, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
        // #endif
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT2);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT2);

        while (1)
        {
            if (udpTimer <= 0)
            {
                for (int i = 0; i < NUM_FOBS; i++)
                {
                    // printf("HELLOOOOOOOOOOOOOOOOOOOOOOOOO \n");
                    if (i == atoi(myID))
                    {
                        // printf("SAME IP ADDRESS =========================  \n");
                        continue;
                    }
                    // printf("COPYING THE IP TO HOST_IP and it is %s ========================= \n", IPtable[i]);
                    strcpy(HOST_IP_ADDR, IPtable[i]);
                    printf("COPYING THE IP TO HOST_IP and it is %s ========================= \n", HOST_IP_ADDR);
                    // #if defined(CONFIG_EXAMPLE_IPV4)
                    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);

                    memset(data, 0, sizeof(data));
                    strcat(data, status);
                    strcat(data, ",");
                    strcat(data, myID);
                    strcat(data, ",");
                    strcat(data, deviceAge);
                    strcat(data, ",");
                    strcat(data, leaderHeartbeat);

                    payload = data;
                    printf("payload is: %s and timeout is %d \n", payload, timeout);
                    // printf("\n");
                    // printf("7\n");
                    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    // printf("8\n");

                    // printf(deviceAge);
                    // printf("\n");
                    if (err < 0)
                    {
                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        break;
                    }
                    printf("sending to ip addess %s \n", HOST_IP_ADDR);
                    ESP_LOGI(TAG, "Message sent");

                    // struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
                    // socklen_t socklen = sizeof(source_addr);

                    udpTimer = UDP_TIMER;
                }
                strcpy(deviceAge, "Old");
            }
            if (RecvFlag == 1 && deviceState == LEADER_STATE)
            {
                dest_addr.sin_addr.s_addr = inet_addr(NODE_IP_ADDR);

                printf("SENDING TO NODE SERVER \n");

                printf("voterVote inside udp_client is %d ", voterVote);
                printf("\n");

                printf("voterID inside udp_client is %d ", voterID);
                printf("\n");

                if (voterVote == 66)
                {
                    strcpy(voterVote_char, "B");
                }
                else if (voterVote == 82)
                {
                    strcpy(voterVote_char, "R");
                }

                switch (voterID)
                {
                case 48:
                    strcpy(voterID_char, "0");
                    break;
                case 49:
                    strcpy(voterID_char, "1");
                    break;
                case 50:
                    strcpy(voterID_char, "2");
                    break;
                case 51:
                    strcpy(voterID_char, "3");
                    break;
                case 52:
                    strcpy(voterID_char, "4");
                    break;
                case 53:
                    strcpy(voterID_char, "5");
                    break;
                case 54:
                    strcpy(voterID_char, "6");
                    break;
                case 55:
                    strcpy(voterID_char, "7");
                    break;
                case 56:
                    strcpy(voterID_char, "8");
                    break;
                case 57:
                    strcpy(voterID_char, "9");
                    break;
                }

                printf("voterID_char is %s \n", voterID_char);
                printf("voterVote_char is %s \n", voterVote_char);

                memset(data, 0, sizeof(data));
                strcat(data, voterID_char);
                strcat(data, ",");
                strcat(data, voterVote_char);

                payload = data;
                // printf("payload is: %s", payload);
                // printf("\n");
                // printf("7\n");
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                // printf("8\n");
                strcpy(deviceAge, "Old");
                // printf(deviceAge);
                // printf("\n");
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                printf("sending to ip addess %s \n", HOST_IP_ADDR);
                ESP_LOGI(TAG, "Message sent");

                // struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
                // socklen_t socklen = sizeof(source_addr);
                RecvFlag = 0;
            }
            else if (RecvFlag == 1 && deviceState != LEADER_STATE)
            {
                dest_addr.sin_addr.s_addr = inet_addr(leaderIP);

                printf("SENDING TO POLL LEADER \n");

                if (voterVote == 66)
                {
                    strcpy(voterVote_char, "B");
                }
                else if (voterVote == 82)
                {
                    strcpy(voterVote_char, "R");
                }

                switch (voterID)
                {
                case 48:
                    strcpy(voterID_char, "0");
                    break;
                case 49:
                    strcpy(voterID_char, "1");
                    break;
                case 50:
                    strcpy(voterID_char, "2");
                    break;
                case 51:
                    strcpy(voterID_char, "3");
                    break;
                case 52:
                    strcpy(voterID_char, "4");
                    break;
                case 53:
                    strcpy(voterID_char, "5");
                    break;
                case 54:
                    strcpy(voterID_char, "6");
                    break;
                case 55:
                    strcpy(voterID_char, "7");
                    break;
                case 56:
                    strcpy(voterID_char, "8");
                    break;
                case 57:
                    strcpy(voterID_char, "9");
                    break;
                }

                memset(data, 0, sizeof(data));
                strcat(data, voterID_char);
                strcat(data, ",");
                strcat(data, voterVote_char);

                payload = data;
                // printf("payload is: %s", payload);
                // printf("\n");
                // printf("7\n");
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                // printf("8\n");
                strcpy(deviceAge, "Old");
                // printf(deviceAge);
                // printf("\n");
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                printf("sending to ip address %s \n", leaderIP);
                ESP_LOGI(TAG, "Message sent");

                // struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
                // socklen_t socklen = sizeof(source_addr);
                RecvFlag = 0;
            }
            else if (sendFlag == 1 && deviceState == LEADER_STATE)
            {
                dest_addr.sin_addr.s_addr = inet_addr(NODE_IP_ADDR);

                printf("SENDING TO NODE SERVER \n");

                if (myColor == 66)
                {
                    strcpy(voterVote_char, "B");
                }
                else if (myColor == 82)
                {
                    strcpy(voterVote_char, "R");
                }

                switch (myID_CHAR)
                {
                case 48:
                    strcpy(voterID_char, "0");
                    break;
                case 49:
                    strcpy(voterID_char, "1");
                    break;
                case 50:
                    strcpy(voterID_char, "2");
                    break;
                case 51:
                    strcpy(voterID_char, "3");
                    break;
                case 52:
                    strcpy(voterID_char, "4");
                    break;
                case 53:
                    strcpy(voterID_char, "5");
                    break;
                case 54:
                    strcpy(voterID_char, "6");
                    break;
                case 55:
                    strcpy(voterID_char, "7");
                    break;
                case 56:
                    strcpy(voterID_char, "8");
                    break;
                case 57:
                    strcpy(voterID_char, "9");
                    break;
                }

                memset(data, 0, sizeof(data));
                strcat(data, voterID_char);
                strcat(data, ",");
                strcat(data, voterVote_char);

                payload = data;
                // printf("payload is: %s", payload);
                // printf("\n");
                // printf("7\n");
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                // printf("8\n");
                strcpy(deviceAge, "Old");
                // printf(deviceAge);
                // printf("\n");
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                printf("sending to ip addess %s \n", HOST_IP_ADDR);
                ESP_LOGI(TAG, "Message sent");

                // struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
                // socklen_t socklen = sizeof(source_addr);
                sendFlag = 0;
            }
            // int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            // if (len < 0) {
            //     ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            //     break;
            // }
            // Data received
            // else {
            //     rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            //     ESP_LOGI(TAG, "Client received %d bytes from %s:", len, host_ip);
            //     printf(rx_buffer);
            //     printf("\n");
            //     ESP_LOGI(TAG, "%s", rx_buffer);
            //     if (strncmp(rx_buffer, "Ok!", 3) == 0) {
            //         printf("EXECUTING---------------------");
            //         ESP_LOGI(TAG, "Received expected message, reconnecting");
            //         break;
            //     }
            // }

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        vTaskDelete(NULL);
    }
}

void app_main()
{

    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();

    // Create a FIFO queue for timer-based events
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Create task to handle timer-based events
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, configMAX_PRIORITIES, NULL);

    // Initialize all the things
    rmt_tx_init();
    uart_init();
    led_init();
    alarm_init();
    button_init();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // Create tasks for receive, send, set gpio, and button
    xTaskCreate(recv_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(send_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(id_task, "set_id_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_task, "button_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_2_task, "button_2_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(udp_server_task, "udp_server", 4096, (void *)AF_INET, configMAX_PRIORITIES, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}
