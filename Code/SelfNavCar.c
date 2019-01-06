/* brushed dc motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/


//ADDed in IR. Have to check GPIO for IR.
// Commented out BLINK_GPIO may need to put back in
//Set duty cycle as a variable so we can increment/ decrement based on how far from the wall. 


#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <string.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"

//static const char* NEC_TAG = "NEC";

#define GPIO_PWM0A_OUT 26   //Set GPIO 15 as PWM0A
#define GPIO_IN1 25  //Set GPIO 16 as PWM0B
#define GPIO_IN2 18

#define GPIO_PWM0B_OUT 14   //Set GPIO 15 as PWM0A
#define GPIO_IN3 15  //Set GPIO 16 as PWM0B
#define GPIO_IN4 32

#define GPIO_LED 13

//ULTRASONIC
#define RMT_TX_CARRIER_EN    0 
#define RMT_TX_CHANNEL    1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  27     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  12    /*!< GPIO number for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         20                          /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define RMT_TX_DATA_NUM  100    /*!< NEC tx test data number */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

//IR Rangefinder
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4        //Multisampling
//#define BLINK_GPIO_0 26

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t channel1 = ADC_CHANNEL_5;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

#define console_TXD  (UART_PIN_NO_CHANGE)
#define console_RXD  (UART_PIN_NO_CHANGE)
#define ir_TXD      18
#define ir_RXD      39
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define lidar_TXD      17
#define lidar_RXD      16



#define BUF_SIZE (1024)
const int uart_num = UART_NUM_2; 
//const int uart_num = UART_NUM_2;

static inline void nec_fill_item_level(rmt_item32_t* item, int high_us, int low_us)
{
    item->level0 = 1;
    item->duration0 = (high_us) / 10 * RMT_TICK_10_US;
    item->level1 = 0;
    item->duration1 = (low_us) / 10 * RMT_TICK_10_US;
}
/*
 * @brief RMT transmitter initialization
 */
static void nec_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
    rmt_tx.tx_config.idle_level = 0;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}


/*
 * @brief RMT receiver initialization
 */
static void nec_rx_init()
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    

}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    //mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    gpio_set_level(GPIO_IN1, 1);
    gpio_set_level(GPIO_IN2, 0);
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    //mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    //mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
    gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 1);
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}
static void check_efuse()
{  
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, ir_TXD, ir_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_INVERSE_RXD);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void lidar_uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 115200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(uart_num, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(uart_num, lidar_TXD, lidar_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  //uart_set_line_inverse(uart_num,UART_INVERSE_RXD);

  // Install UART driver
  uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void ir_init(){
    check_efuse();

if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

}

static void ir2_init(){
    check_efuse();

if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel1, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel1, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

}


int front_dist; //Ultrasonic Sensor
int side_dist; //IR Sensor 
int state; 
char rxID;
char expectedID;


bool recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  for(int i = 0; i < 10; i++){
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in >0) {
      for (int i=0; i < 24; i++) {
        if (data_in[i] == 0x0A) {

          rxID = data_in[i+1];

          printf("Received comm from ID 0x%02X", rxID);

          // Signal to election
          return true;
        }
      }
    }
    else{
      printf("Nothing received.\n");

    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
  return false; 
  free(data_in);
}

static void ultrasonic(){
/**
 * @brief RMT transmitter demo, this task will periodically send NEC data. (100 * 32 bits each time.)
 *
 */
    front_dist = 0;
    nec_tx_init();
    int channel_rx = RMT_RX_CHANNEL;
    nec_rx_init();
    //esp_log_level_set(NEC_TAG, ESP_LOG_INFO);
    //int front_dist = 0;
    RingbufHandle_t rb = NULL;
    int channel_tx = RMT_TX_CHANNEL;
    int nec_tx_num = RMT_TX_DATA_NUM;
    size_t size = (sizeof(rmt_item32_t) * NEC_DATA_ITEM_NUM * nec_tx_num);
    int time = 10;
    int temp = 0;
    int count = 0;
    rmt_item32_t* item_tx = (rmt_item32_t*) malloc(size);
    int item_num = NEC_DATA_ITEM_NUM * nec_tx_num;
       nec_fill_item_level(item_tx, time, time);
       //uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
       char* distp = (char *) malloc(6);


    while(1){
    rmt_write_items(channel_tx, item_tx, item_num, false);

    vTaskDelay(100/ portTICK_RATE_MS);
    
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel_rx, &rb);
    rmt_rx_start(channel_rx, 1);
    while(rb){
        count = 0;
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item)
        {
            vTaskDelay(10/ portTICK_RATE_MS);
            if(item->level0 == 1){
                if((item->duration0 / 58) > 2){
                front_dist = item->duration0 / 58;
                }

                //sprintf(distp,"%2.2f\r\n", dist);
                //uart_write_bytes(uart_num_c, distp, 6);
               // printf("Front Dist: %d\n", front_dist);
            }
            else if(item->level1 == 1){
                if((item->duration1 / 58) > 2){
                front_dist = item->duration1 / 58;
                }
                //sprintf(distp,"%2.2f\r\n", dist);
                //uart_write_bytes(uart_num_c, distp, 6);
                //printf("Front Dist: %d\n", front_dist);
            }
            vRingbufferReturnItem(rb, (void*) item);
        }
            //uart_write_bytes(uart_num_c, distp, 6);
        //free(item);
    }
    }   

    rmt_rx_stop(channel_rx);
    rmt_memory_rw_rst(channel_rx);
    rmt_driver_uninstall(channel_tx);
    rmt_driver_uninstall(channel_rx);
    free(item_tx);

}


static void ir(){

    //Continuously sample ADC1
    //char* distp = (char *) malloc(6);
    while(1){
        //gpio set level
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t mV = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float voltage = mV/1000.0;
        float power = powf(voltage,-1.18);
        side_dist = (60.3*power); 
        //meters = meters/100.0;
        //sprintf(distp,"%02.2f\r\n", meters);
        //uart_write_bytes(uart_num_c, distp, 6);
        vTaskDelay(50/portTICK_RATE_MS);
    }

}

static void front_ir(){

    //Continuously sample ADC1
    //char* distp = (char *) malloc(6);
    while(1){
        //gpio set level
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel1);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel1, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t mV = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float voltage = mV/1000.0;
        float power = powf(voltage,-1.18);
        front_dist = (60.3*power); 
        //meters = meters/100.0;
        //sprintf(distp,"%02.2f\r\n", meters);
        //uart_write_bytes(uart_num_c, distp, 6);
        vTaskDelay(50/portTICK_RATE_MS);
    }

}

static void lidar()
{
    
   // vTaskDelay(1000 / portTICK_RATE_MS);
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    //int len = uart_read_bytes(uart_num, data, BUF_SIZE, 100/ portTICK_RATE_MS);
    uint8_t lidar_data[9];
    char* distp = (char *) malloc(6);
        //Read data from UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, 20/ portTICK_RATE_MS);
        if (len > 0) {
            if((*data == 0x59) & (data[1] == 0x59)){
                for (int i = 0; i < 9; i++) {
                    lidar_data[i] = data[i];
                }
            } 
            //else
               // data = data +1;         
        }
        else 
            printf("no data \n");

        int low_dist = lidar_data[2];
        int high_dist = lidar_data[3];
        high_dist = high_dist << 8;
        front_dist = low_dist + high_dist;
        //vTaskDelay(250/ portTICK_RATE_MS);

        //front_dist = dist;
        //printf("Distance = %f\n", dist);
        //free(data);
        /*
        if(dist< ){
            sprintf(distp,"%02.2f\r\n", dist);
        }
        else{
            sprintf(distp,"%d\r\n", dist);
        }*/
      //  sprintf(distp,"%02.2f\r\n", dist);
       // uart_write_bytes(uart_num_c, distp, 6);
        uart_flush(uart_num);
}

static void lidar_task(){
    while(1){
        vTaskDelay(100/portTICK_RATE_MS);
        lidar();
    }

}


static void state_control(){
    state = 1;
    while(1) {
        switch(state){
          case 1 ://STOP
           // printf("State = Stop\n");
            if(front_dist >= 30){
                state = 2;
            }
            else if(front_dist < 30 && (recv_task())){
            // printf("Current state: %c\n",status);
                state = 3; 
            }
            break;
          case 2 ://DRIVE 
           // printf("State = Drive\n");
            //Turn case
            if(front_dist < 30){
                state = 1;
            }
            // printf("Current state: %c\n",status);
            break;
          case 3 ://TURN
            // printf("State: Turn\n");
             vTaskDelay(1000 / portTICK_RATE_MS);
            break;
        }

        

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }


}



/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void driving_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();
    gpio_pad_select_gpio(GPIO_IN1);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_IN1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(GPIO_IN2);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_IN2, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(GPIO_IN3);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_IN3, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(GPIO_IN4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_IN4, GPIO_MODE_OUTPUT);
     gpio_pad_select_gpio(GPIO_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);


    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    float duty_A = 74.0;
    float duty_B = 85.0;
    state = 1;
    side_dist = 20;
    front_dist = 0;
    while (1) {
        /*
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);*/
        //lidar();

        printf("Front_dist %d\n",front_dist);
        printf("Side_dist %d\n", side_dist);


        if(front_dist < 30){
            //gpio_set_level(GPIO_LED,1);
        }
        else{
           // gpio_set_level(GPIO_LED,0);
        }
        //printf("duty_A = %f\n",duty_A);
        //printf("duty_B = %f\n",duty_B);
        
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_A);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_B);
         mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            gpio_set_level(GPIO_IN1, 1);
            gpio_set_level(GPIO_IN2, 0);
            gpio_set_level(GPIO_IN3, 1);
            gpio_set_level(GPIO_IN4, 0);

        switch(state){
          case 1 :
          {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.1);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.1);
            // printf("Current state: %c\n",status);
            break;
          }
          case 2:
          {
            
            if(side_dist < 29){
                //Too close to wall, speed up right wheel 
                if(duty_A < 76){

                duty_A = duty_A + 2;
                /*vTaskDelay(500 / portTICK_RATE_MS);
                duty_A = duty_A -10;
                vTaskDelay(500 / portTICK_RATE_MS);
                duty_A = duty_A + 5;
                printf("%f\n",duty_A);*/

                gpio_set_level(GPIO_LED,0);
                }
            }
            else if(side_dist > 31){
                if(duty_A > 72){
                //Too far from wall, slow down right wheel
                duty_A = duty_A - 2; 
                /*vTaskDelay(500 / portTICK_RATE_MS);
                duty_A = duty_A + 10;
                vTaskDelay(500 / portTICK_RATE_MS);
                duty_A = duty_A - 5;
                printf("%f\n",duty_A);
                gpio_set_level(GPIO_LED,0);*/
                }
            }
            else if(side_dist == 25){
                gpio_set_level(GPIO_LED,1);
            }

                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_A);
            //mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_B);
            // printf("Current state: %c\n",status);
                //vTaskDelay(500/portTICK_RATE_MS);
            break;

          }
          case 3 :
          {
            /*Turn command*/ 
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 75.0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.1);
            vTaskDelay(600 / portTICK_RATE_MS);
            state = 2;
            // printf("Current state: %c\n",status);
            break;
          }   
        }
       // gpio_set_level(GPIO_LED,0);
        /*Stop at obstacle
        if(front_dist < 10){
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.1);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.1);
           // printf("FRont Dist %d\n", front_dist);
        }*//*
        else{
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_A);
                    (comment)mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_B);*/
            //printf("FRont Dist %d\n", front_dist);
           // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            
           // vTaskDelay(5000 / portTICK_RATE_MS);
            /*Turn command 
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 75.0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.1);*/
            //gpio_set_level(GPIO_IN1, 0);
           // gpio_set_level(GPIO_IN2, 1);
            //vTaskDelay(700 / portTICK_RATE_MS);
            /*mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 45.0);
            vTaskDelay(3000 / portTICK_RATE_MS);*/


            /*Turning State 
            if(state = Turn){
                 mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 75.0);
                 mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.1);
                 vTaskDelay(700 / portTICK_RATE_MS);
                 state = driving;
            }
            */

        
        vTaskDelay(100 / portTICK_RATE_MS);
    
    }
}


void app_main()
{
    //lidar_uart_init();
    uart_init();
    ir_init();
    ir2_init();
    //xTaskCreate(recv_task, "recv_task", 4096, NULL, 5, NULL);
    //xTaskCreate(ultrasonic, "ultrasonic", 4096, NULL, 5, NULL);
    //xTaskCreate(lidar_task,"Lidar_task", 4096, NULL, 5, NULL);
    xTaskCreate(driving_control, "driving_control", 4096, NULL, 5, NULL);
    xTaskCreate(state_control, "state_control", 4096, NULL, 5, NULL);
    
    xTaskCreate(ir, "ir", 4096, NULL, 5, NULL);
    xTaskCreate(front_ir, "front_ir", 4096, NULL, 5, NULL);
    
    

}
