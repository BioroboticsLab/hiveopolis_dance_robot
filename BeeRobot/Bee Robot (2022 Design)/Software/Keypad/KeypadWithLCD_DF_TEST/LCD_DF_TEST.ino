#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "math.h"

#include "esp_log.h"
#include <Wire.h>
#include "DFRobot_LCD.h"

DFRobot_LCD lcd(16,2);  //16 characters and 2 lines of show
int encoder_val = 0;
int CurrMode = 1;

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#define TIMER_DIVIDER   80              // should result in 1 MHz ^= 1µs timer interval

#define ECHO_TEST_TXD   17               // 1 and 3 are the respective pins for UART0
#define ECHO_TEST_RXD   16
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      2       // set for UART1
#define ECHO_UART_BAUD_RATE     57600
#define ECHO_TASK_STACK_SIZE    2048

#define ISR_MIN_PERIOD          500     // µs
#define ISR_MAX_PERIOD         1500     // µs
#define ISR_PERIO_INCREMENT     100     // µs

#define BUF_SIZE 1024

// keypad flags
#define KEY_ZCCW    0x01
#define KEY_ZCW     0x02
#define KEY_DOWN    0x04
#define KEY_RIGHT   0x08
#define KEY_UP      0x10
#define KEY_LEFT    0x20
#define KEY_D_ONOFF   0x40
#define KEY_MODE   0x80

//keypad flag second byte
#define KEY_SPDUP   0x10
#define KEY_SPDDWN  0x20
#define KEYPAD_PRESENT 0x40

// button_GPIO
#define GPIO_KEY_ZCCW    15
#define GPIO_KEY_ZCW     19
#define GPIO_KEY_DOWN    27
#define GPIO_KEY_RIGHT   18
#define GPIO_KEY_UP      23
#define GPIO_KEY_LEFT    2
#define GPIO_KEY_DANCE   4
#define GPIO_KEY_ONOFF   5
#define GPIO_KEY_SPDUP   35
#define GPIO_KEY_SPDDWN  34



enum commands {ZCCW, ZCW, DOWN, RIGHT, UP, LEFT, D_ONOFF, MODE, SPDUP, SPDDWN};
int keys_translate[4][10] = {{0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x10, 0x20}, {15, 19, 27, 18, 23, 2, 4, 5, 35, 34}, {0, 0, 0, 0, 0, 0, 0, 0, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int check_parameter[2] = {0x00, 0x00};
int disp = 0;
static const char* TAG = "MyModule";

uint8_t *data = (uint8_t *) malloc(128);
uint8_t *data2  = (uint8_t *) malloc(128);
void init_GPIO(gpio_num_t motor_gpio, gpio_int_type_t interrupt_type, gpio_mode_t mode)
{
  gpio_config_t io_conf;
  io_conf.intr_type       = interrupt_type;
  io_conf.pin_bit_mask    = ((uint64_t)(((uint64_t)1) << motor_gpio));
  io_conf.mode            = mode;
  io_conf.pull_down_en    = (gpio_pulldown_t)0;
  io_conf.pull_up_en      = (gpio_pullup_t)0;
  gpio_config(&io_conf);
}



void init_gpio_and_isr()
{
  //////// PIN CONFIGURATION

  // homing switches
  init_GPIO((gpio_num_t)GPIO_KEY_ZCCW, GPIO_INTR_DISABLE , GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_ZCW, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_DOWN, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_RIGHT, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_UP, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_LEFT, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_DANCE, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_ONOFF, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_SPDUP, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
  init_GPIO((gpio_num_t)GPIO_KEY_SPDDWN, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);

}
static void displayTask(void *arg)
{
  // initialize
  lcd.init();
  // Print a message to the LCD.
//  lcd.setCursor(0, 0);
//  lcd.print("Motor:ON Mode:1   ");
//  lcd.setCursor(0, 1);
//  lcd.print("Ang:360 Spd:255   ");
//  vTaskDelay( 50 / portTICK_PERIOD_MS );
  
  while (1)
  {

    
    char mystr[30]; 
    sprintf(mystr, "Mtr:%02u Mode:%02d  ",(data[0]&keys_translate[0][MODE]?1:0),CurrMode );  
    lcd.setCursor(0, 0);
    lcd.print(mystr);
    int f = (encoder_val*36)/200;
    sprintf(mystr, "Ang:%03d Spd:%03u    ",f,data[1]);  
    lcd.setCursor(0, 1);
    lcd.print(mystr);
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    
  }
  
}

void setup() {
    

    init_gpio_and_isr();
  /* Configure parameters of an UART driver,
     communication pins and install the driver */
//  uart_config_t uart_config = {
//    .baud_rate  = ECHO_UART_BAUD_RATE,
//    .data_bits  = UART_DATA_8_BITS,
//    .parity     = UART_PARITY_DISABLE,
//    .stop_bits  = UART_STOP_BITS_1,
//    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
//    //.source_clk = UART_SCLK_APB,
//  };
//  int intr_alloc_flags = 0;
//
//#if CONFIG_UART_ISR_IN_IRAM
//  intr_alloc_flags = ESP_INTR_FLAG_IRAM;
//#endif
//
//  ESP_ERROR_CHECK(uart_driver_install((uart_port_t)ECHO_UART_PORT_NUM, BUF_SIZE , 0, 0, NULL, intr_alloc_flags));
//  ESP_ERROR_CHECK(uart_param_config((uart_port_t)ECHO_UART_PORT_NUM, &uart_config));
//  ESP_ERROR_CHECK(uart_set_pin((uart_port_t)ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

  // Configure a temporary buffer for the incoming data

  data[0] = 0;
  data[1] = 100;
  xTaskCreate(displayTask, "displayTask", 2048, NULL, 9, NULL);
  //Serial.begin(9600);
  Serial2.begin(57600, SERIAL_8N1, ECHO_TEST_RXD, ECHO_TEST_TXD);
}

void loop() {

  int change = 0;
  int len = 2;
  if (data[0]&KEY_MODE)
  {
    data[0] = 0x00;
    data[0] |= KEY_MODE;
  }
  else
    data[0] = 0x00;
  int cnt = 0;
  for (cnt = 0 ; cnt < 10; cnt++) {
    keys_translate[3][cnt] += gpio_get_level((gpio_num_t)keys_translate[1][cnt]);
  }
  vTaskDelay( 10 / portTICK_PERIOD_MS );
  for (cnt = 0 ; cnt < 10; cnt++) {
    keys_translate[3][cnt] += gpio_get_level((gpio_num_t)keys_translate[1][cnt]);
  }
  if ( keys_translate[3][7] >= 2 && (data[0]&KEY_MODE) && !gpio_get_level((gpio_num_t)keys_translate[1][7]))
  {
    change++;
    data[keys_translate[2][7]] &= ~keys_translate[0][7];
    keys_translate[3][7] = 0;

  }
  else if ( keys_translate[3][7] >= 2 && !(data[0]&KEY_MODE) && !gpio_get_level((gpio_num_t)keys_translate[1][7]))
  {
    change++;
    data[keys_translate[2][7]] |= keys_translate[0][7];
    keys_translate[3][7] = 0;
  }

  for (cnt = 0 ; cnt < 8; cnt++) {
    if (cnt != 7 && keys_translate[3][cnt] == 2) {
      change++;
      data[keys_translate[2][cnt]] |= keys_translate[0][cnt];
      keys_translate[3][cnt] = 0;
    }
    else if (cnt != 7)
      keys_translate[3][cnt] = 0;

  }
  //ESP_LOGI(TAG, "DATA SENT %d  %d ",keys_translate[3][8],data[1]);
  if (keys_translate[3][8] >= 2)
  {

    change++;
    if (data[keys_translate[2][8]] > 0)
      data[keys_translate[2][8]]--;
    keys_translate[3][8] = 0;

  }
  else if (keys_translate[3][9] >= 2)
  {

    change++;
    if (data[keys_translate[2][8]] < 255)
      data[keys_translate[2][8]]++;
    keys_translate[3][9] = 0;

  }
  
//  
//  uart_write_bytes((uart_port_t)ECHO_UART_PORT_NUM, (const char *) data, len);
//  if(uart_read_bytes((uart_port_t)ECHO_UART_PORT_NUM, data2, 12, 150 / portTICK_RATE_MS))
//  {  
//      encoder_val = (data2[1]<<8) + data2[0];
//      Serial.println(encoder_val);
//  }
//  if (data[0] != 64)
//    ESP_LOGI(TAG, "DATA SENT %d  %d ", data[0], data[1]);
  Serial2.write(data,2);
  if (Serial2.available()) {
    Serial2.readBytes(data2,3);
    encoder_val = (data2[1]<<8) + data2[0];
    CurrMode = data2[2];
    }
 
  //vTaskDelay( 10 / portTICK_PERIOD_MS );
  
  check_parameter[0] = 0x00;
  check_parameter[1] = 0x00;
}
