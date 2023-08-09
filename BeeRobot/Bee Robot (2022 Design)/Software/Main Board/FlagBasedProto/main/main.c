#include <stdio.h>
#include <stdlib.h>
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
#include "dance_model_static_trajectory.h"
#include "motor.h"
#include "gpio.h"
#include "math.h"
#include "commands.h"
#include "dancegen.h"
#include "types.h"
#include "motor.h"
#include "esp_log.h"
#include "esp_int_wdt.h"
#include "soc/rtc_wdt.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#define TIMER_DIVIDER   80             // should result in 1 MHz ^= 1µs timer interval

/////////////////////////////////////////////////////////////////////////////////////
//Global Parameters for the GUI UART0
#define GUI_TXD   1               // 1 and 3 are the respective pins for UART0
#define GUI_RXD   3
#define GUI_RTS (UART_PIN_NO_CHANGE)
#define GUI_CTS (UART_PIN_NO_CHANGE)

#define GUI_UART_PORT_NUM      0       // set for UART0
#define GUI_UART_BAUD_RATE     115200
#define GUI_TASK_STACK_SIZE    2048

/////////////////////////////////////////////////////////////////////////////////////
//Global Parameters for the GUI UART2
#define ENC_TXD   26               //  26 and 25 are the respective pins for UART2
#define ENC_RXD   25
#define ENC_RTS (UART_PIN_NO_CHANGE)
#define ENC_CTS (UART_PIN_NO_CHANGE)

#define ENC_UART_PORT_NUM      2       // set for UART2
#define ENC_UART_BAUD_RATE     115200
#define ENC_TASK_STACK_SIZE    2048


//////////////////////////////////////////////////////////////////////////////////////
//Global Parameters for the Keypad UART1
#define KEYPAD_TXD   17               // 17 and 16 are the respective pins for UART1
#define KEYPAD_RXD   16
#define KEYPAD_RTS (UART_PIN_NO_CHANGE)
#define KEYPAD_CTS (UART_PIN_NO_CHANGE)

#define KEYPAD_UART_PORT_NUM      1   // set for UART1
#define KEYPAD_UART_BAUD_RATE     57600
#define KEYPAD_TASK_STACK_SIZE    2048

#define BUF_SIZE 1024

////////////////////////Keypad UART recv data
u8 keypad = 0;
u8 keypadSpeed = 0;

s16 motorSpeedFactor = 10;
s16 motorX_Key_Offset = 0;
s16 motorY_Key_Offset = 0;
s16 motorZ_Key_Offset = 0;

int sysState =0;
u8 state = 0;
u8 limitSwitches[2][4] = {{GPIO_HOME_SWITCH_X_MIN, GPIO_HOME_SWITCH_X_MAX, GPIO_HOME_SWITCH_Y_MIN, GPIO_HOME_SWITCH_Y_MAX},{0,0,0,0}}; 

float diff;
float fTemp;
float w_old;
float w_cur;
float vx;
float vy;
float vx_old = 0;
float vy_old = 0;
V2Df p_old;
V2Df p_cur;
u16 i;

u16 wTemp =0;
u16 wPeriod = 3;  // in ms
u16 wTotalPeriod = 77; // in ms
u8 firstWaggleStart;
u8 testWings;
u8 wings_timer=0;

enum Mode {RotateNewWaggle, RotateContinueWaggle, FixedWaggle, RotateMotion};
u8 CurrMode = 0;
u8 ModeSwitch = 1;
u8 ResetEnc = 0;
volatile int stopCounter =0;

#define RAMP_ACCEL_FACTOR 0.03
#define MOTION_ACCEL_FACTOR 0.003
float rampFactor;
float motionFactorX;
float motionFactorY;

//static xQueueHandle s_timer_queue;


DANCE_PARAMS params;// = (DANCE_PARAMS *)malloc(sizeof(DANCE_PARAMS));;

#define RotAng           0       // angle with 100 multiplier 
#define xdash(x,y)             x*cos(RotAng/100) + y*sin(RotAng/100)    // rotation x 
#define ydash(x,y)             y*cos(RotAng/100) - x*sin(RotAng/100)    // rotation y
volatile short  encoder_val            = 0;
volatile short  encoder_GPIO_A         = 0;
volatile short  encoder_GPIO_B         = 0;
volatile short  temp_GPIO_A         = 0;
volatile short  temp_GPIO_B         = 0;


volatile int    global_tick_count   = 0;
volatile uint   cnt[3]              = {0,0,0};
volatile const uint pins[6]         = {GPIO_MOTOR_PHI_STEP, GPIO_MOTOR_PHI_DIR, GPIO_MOTOR_X_STEP, GPIO_MOTOR_X_DIR, GPIO_MOTOR_Y_STEP, GPIO_MOTOR_Y_DIR};
volatile int    cnt_phi             = 0;
volatile int    cnt_x               = 0;
volatile int    cnt_y               = 0;
volatile short  motor_flags         = 0; //bit mask, which motor timer to start / stop 

#define EncAng           (float) encoder_val*PI2/2000   
#define EncAngDeg           (float) encoder_val*360/2000   

#define TopSpeed 1000
#define ReduSpeed 20

#define beelength 0.22
volatile int current_angle = 0;
volatile int Rotation_speed_Phi = 1540;
volatile int Rotation_speed_X = 0;
volatile int Rotation_speed_Y = 0;
volatile int mflag = 0;
float Rotz = 0;
volatile short  Rot_dir_z         = 0;

int finaltime=0,inittime=0;

#define Kp  0.01  
#define output  0.00     

//global variable to be updated upon keyboard events
volatile int    period  = 0;
volatile int    addspeedX = 0;
volatile int    addspeedY = 0;

char out_buf[100];

static const char* TAG = "MyModule";

int a[] = {0,0,0,0};
void motor_isr_toggle(int motor_id, int dir);
void motor_on_off(int motor_id, bool on);
void test_motor_phi();

volatile int dirToggle = 0;
int GP = 26;

volatile int dirToggle2 = 0;
int GP2 = 32;


typedef struct {
    int motorx;
    int motory;
    int motorz;
} motor_timers_t;

volatile int oneMilliSecondFlag = 0 ;

bool IRAM_ATTR global_tick_isr() // timer group 0, ISR
{   
    oneMilliSecondFlag = 1; 
    
    return 0; 
}

void motorXYZDrive(int x,int y,int z)
{   
        uint timer_val;
        timer_val = (z == 0) ? 1000000 : rotary_speed_to_timer_value( abs(z) ) ; // set timer value for a long duration (1 s) if speed is 0
        set_motor_direction_pin(GPIO_MOTOR_PHI_DIR, z > 0 ? 0:1)    
        timer_set_alarm_value(0, 1, timer_val);
            
        timer_val = (x == 0) ? 1000000 : linear_speed_to_timer_value( abs(x ) ) ; 
        set_motor_direction_pin(GPIO_MOTOR_X_DIR, x  > 0 ? 1:0)    
        timer_set_alarm_value(1, 0, timer_val );

            
        timer_val = (y == 0) ? 1000000 : linear_speed_to_timer_value( abs(y ) ) ; 
        set_motor_direction_pin(GPIO_MOTOR_Y_DIR, y > 0 ? 1:0)    
        timer_set_alarm_value(1, 1, timer_val );
}

void motor_isr_toggle(int motor_id, int dir)
{
    // select which timer group and timer idx
    int group = (motor_id == MOTOR_PHI)? 0 : 1;
    int timer = (motor_id == MOTOR_X)? 0 : 1;
    
    // if motor is on, switch it off
    set_motor_direction_pin(Motor_Direction[motor_id],dir);
    if (motor_flags & (1<<motor_id) )
        timer_pause(group, timer);
    else
    {
        timer_set_alarm_value(group, timer, linear_speed_to_timer_value(2500));
        timer_start(group, timer);

    }// update motor flag
    motor_flags ^= (1<<motor_id);
}

bool IRAM_ATTR pin_toggle(void * param)
{ 
    uint motor_idx = (uint) param;
    gpio_set_level(pins[2*motor_idx], cnt[motor_idx] % 2);
    cnt[motor_idx]++;
    return false;
}


static void homing_switch_handler(void * param )
{   
    int change =0;
    int i=0;
    while(1)
    {
        
        for(i=0;i<4;i++)
        {
            limitSwitches[1][i] = !gpio_get_level(limitSwitches[0][i]);
        }
        //ESP_LOGI(TAG, "%d",encoder_val);
        vTaskDelay( 50 / portTICK_PERIOD_MS );
        for(i=0;i<4;i++)
        {
            limitSwitches[1][i] &= !gpio_get_level(limitSwitches[0][i]);
        }
        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
    
}

static void init_timer(timer_group_t timer_group, timer_idx_t timer_idx, uint64_t alarm_val, timer_isr_t isr_callback, void * param, bool start_timer)
{
    timer_config_t config;
    config.alarm_en         = 1; // timer alarm enable
    config.auto_reload      = 1; // auto-reload enable
    config.counter_dir      = TIMER_COUNT_UP; 
    config.divider          = TIMER_DIVIDER;
    config.intr_type        = TIMER_INTR_LEVEL; // timer config: interrupt mode = timer triggers isr
    config.counter_en       = TIMER_PAUSE; // do not start timer right away

    // configure timer
    timer_init(timer_group, timer_idx, &config);
    // stop timer counter (should not be necessary because we initialized it to be paused)
    timer_pause(timer_group, timer_idx);
    // load counter value
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    // set alarm value (in µs)
    timer_set_alarm_value(timer_group, timer_idx, alarm_val);
    // enable timer interrupt
    timer_enable_intr(timer_group, timer_idx);
    // set interrupt service routine
    // timer_isr_register(timer_group, timer_idx, timer_G0T0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    //if (isr_function) 
    //    timer_isr_register(timer_group, timer_idx, isr_function, NULL, ESP_INTR_FLAG_IRAM, NULL);
    //else        
    timer_isr_callback_add(timer_group, timer_idx, isr_callback, param, 0);
    
    if (start_timer)
        timer_start(timer_group, timer_idx);
}

void motor_toggle(int motor_id, int dir)
{
    // select which timer group and timer idx
    int group = (motor_id == MOTOR_PHI)? 0 : 1;
    int timer = (motor_id == MOTOR_X)? 0 : 1;
    
    // if motor is on, switch it off
    set_motor_direction_pin(Motor_Direction[motor_id],dir);
    if (motor_flags & (1<<motor_id) )
        timer_pause(group, timer);
    else
    {
        timer_set_alarm_value(group, timer, linear_speed_to_timer_value(628));
        timer_start(group, timer);

    }// update motor flag
    motor_flags ^= (1<<motor_id);
}

void init_GPIO(gpio_num_t motor_gpio, gpio_int_type_t interrupt_type, gpio_mode_t mode)
{
    gpio_config_t io_conf;
    io_conf.intr_type       = interrupt_type;
    io_conf.pin_bit_mask    = ((uint64_t)(((uint64_t)1)<<motor_gpio));
    io_conf.mode            = mode;
    io_conf.pull_down_en    = 0;
    io_conf.pull_up_en      = 0;
    gpio_config(&io_conf);
}

void Init_GPIO_ISR()
{
    //////// PIN CONFIGURATION
    
    
    // // wing switches
    init_GPIO(GPIO_WING_IN1, GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_WING_IN2, GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);

    // // homing switches
    init_GPIO(GPIO_HOME_SWITCH_X_MIN, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    init_GPIO(GPIO_HOME_SWITCH_X_MAX, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    init_GPIO(GPIO_HOME_SWITCH_Y_MIN, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    init_GPIO(GPIO_HOME_SWITCH_Y_MAX, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    
    // motor pins   
    init_GPIO(GPIO_MOTOR_PHI_DIR,   GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_PHI_STEP,  GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_X_DIR,     GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_X_STEP,    GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_Y_DIR,     GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_Y_STEP,    GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);

    init_GPIO(GPIO_MOTOR_ENABLE,    GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    // init_GPIO(GPIO_MOTOR_X_ENABLE,      GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    // init_GPIO(GPIO_MOTOR_Y_ENABLE,      GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);

    // init_GPIO(GP, GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);    
    // init_GPIO(GP2, GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);    

    // encoder
    //init_GPIO(GPIO_ENCODER_A, GPIO_INTR_POSEDGE, GPIO_MODE_INPUT);
    //init_GPIO(GPIO_ENCODER_A, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);

    //init_GPIO(GPIO_ENCODER_B, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    

    /////////// INSTALL ISRs
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    // ISR for rotary encoder
    //gpio_isr_handler_add(GPIO_ENCODER_A, encoder_isr_handler, 0);
    
}

void Init_AllTimers()
{
    init_timer(TIMER_GROUP_0, TIMER_1, 1000000, pin_toggle, MOTOR_PHI,  true);
    init_timer(TIMER_GROUP_1, TIMER_0, 1000000, pin_toggle, MOTOR_X,  true);
    init_timer(TIMER_GROUP_1, TIMER_1, 1000000, pin_toggle, MOTOR_Y,  true);
    init_timer(TIMER_GROUP_0, TIMER_0, 1000, global_tick_isr, NULL, true);
    
}


void Rotateatapoint(float w_old,float speed, float x, float y,float z)
{
    motor_timers_t mtt;
    w_cur = w_old + (speed / 1000000);
    //float exc_length = 23.0;
    p_old.x = params.exc_length*(cosf(w_old)); 
    p_old.y = params.exc_length*(sinf(w_old));

    p_cur.x = params.exc_length*(cosf(w_cur)); 
    p_cur.y = params.exc_length*(sinf(w_cur));

    mtt.motorx = -100000*(p_old.x - p_cur.x) + x;
    mtt.motory = -100000*(p_old.y - p_cur.y) + y;
    mtt.motorz = speed;
    if(mtt.motorx != 0)
        timer_start(TIMER_GROUP_1, TIMER_0);
    if(mtt.motory != 0)
        timer_start(TIMER_GROUP_1, TIMER_1);
    if(mtt.motorz != 0)
        timer_start(TIMER_GROUP_0, TIMER_1); 
    motorXYZDrive( mtt.motorx, mtt.motory, mtt.motorz);
}

static void read_from_Keypad_UART_task(void *arg)
{
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = KEYPAD_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(KEYPAD_UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(KEYPAD_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(KEYPAD_UART_PORT_NUM, KEYPAD_TXD, KEYPAD_RXD, KEYPAD_RTS, KEYPAD_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len = 0;
    int temp_val = encoder_val;
    int temp_val2 = CurrMode;
    data[0]= 0;
    data[1]= 0;
    data[2]= 0;
    uart_write_bytes((uart_port_t)KEYPAD_UART_PORT_NUM, data, 3);
    while (1) {
        if (temp_val != encoder_val || temp_val2 != CurrMode)
        {
            temp_val = encoder_val;
            temp_val2 = CurrMode;
            
            data[0]= temp_val & 0b11111111;
            data[1]= temp_val >>8;
            data[2]= CurrMode;
            uart_write_bytes((uart_port_t)KEYPAD_UART_PORT_NUM, data, 3);
            //SP_LOGI(TAG, "K %d E %d",(data[1]<<8) + data[0],encoder_val);

        }
        // // Read data from the UART
        data[0] = 0;
        data[1] = 0;
        len = uart_read_bytes(KEYPAD_UART_PORT_NUM, data, 2, 10 / portTICK_RATE_MS);
        
        // if data was received
        if ((len))
        {   
            
            //looks for any change in the keypad buffer and check if keypad is enabled for control
            if ((sysState & SYS_STATE_KEYBOARD_CTRL_ENABLED)&& data[0] != keypad )  
            {
                keypad= data[0];
                keypadSpeed = data[1];
            }   
        }
        
        len = 0;    
        
    }
}

static void read_from_Encoder_UART_task(void *arg)
{
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = ENC_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(ENC_UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ENC_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ENC_UART_PORT_NUM, ENC_TXD, ENC_RXD, ENC_RTS, ENC_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len =0;
    while (1) {
        // Read data from the UART
        
        if(ResetEnc == 1)
        {
            data[0] = 3;
            data[1] = 1;
            uart_write_bytes(ENC_UART_PORT_NUM, (const char *) data, 1);
            ESP_LOGI(TAG, "RESET ");
            ResetEnc = 0;
        }
        else 
        {
            data[0] = 1;
            data[1] = 1;
            uart_write_bytes(ENC_UART_PORT_NUM, (const char *) data, 1);
            data[0] = 0;
            data[1] = 0;

            len = uart_read_bytes(ENC_UART_PORT_NUM, data, 12, 15 / portTICK_RATE_MS);
            //ESP_LOGI(TAG, "DATA RECV %d  %d %d",data[0],data[1],len);
            // if data was received
            if ((len))
            {   
                //looks for any change in the keypad buffer and check if keypad is enabled for control
                
                data[len]=0;
                encoder_val = data[0]?-1*((data[2]<<8) + data[1]):((data[2]<<8) + data[1]);
                if (encoder_val<0)
                    encoder_val += 2000;
                //ESP_LOGI(TAG, "Enc %d",encoder_val);
                
                
            }
            
            len = 0;    
        }
    }
}

static void read_from_GUI_UART_task(void *arg)
{
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = GUI_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(GUI_UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(GUI_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GUI_UART_PORT_NUM, GUI_TXD, GUI_RXD, GUI_RTS, GUI_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len =0;
    while (1) {
        // Read data from the UART
        len = uart_read_bytes(GUI_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // if data was received
        //ESP_LOGI(TAG, "GUI RECV %d ",encoder_val);
        if (len)
        {
            switch (data[0])
            {
            case '1':
                ESP_LOGI(TAG, "UART1 ");
                break;
            case '3':
                motor_isr_toggle(MOTOR_X,1);
                break;

            case '4':
                motor_isr_toggle(MOTOR_X,0);
                break;    
            default: 
                break;
            
            }
            
        }
    }
}

void app_main()
{	  
    printf("Hiveopolis RoboBee - Minimal Prototype\n");
    ESP_LOGI(TAG, "try ");
    Init_GPIO_ISR();
    sysState |= SYS_STATE_KEYBOARD_CTRL_ENABLED;
    ESP_LOGI(TAG, "try2 ");
    printf("Hiveopolis RoboBee - Checking\n");
    
    printf("Hiveopolis RoboBee - All Motor Timers and Global Loop Timer Initialised\n");
    ESP_LOGI(TAG, "sysState %d",sysState);
    xTaskCreate(read_from_Keypad_UART_task, "read_from_Keypad_UART_task", KEYPAD_TASK_STACK_SIZE, NULL, 9, NULL);
    //xTaskCreate(read_from_GUI_UART_task, "read_from_GUI_UART_task", GUI_TASK_STACK_SIZE, NULL, 9, NULL);
    xTaskCreate(homing_switch_handler, "homing_switch_handler", GUI_TASK_STACK_SIZE, NULL, 9, NULL);
    xTaskCreate(read_from_Encoder_UART_task, "read_from_Encoder_UART_task", ENC_TASK_STACK_SIZE, NULL, 9, NULL);
    ESP_LOGI(TAG, "sysState %d",sysState);
    rampFactor = 1;
    w_old = 0;
    firstWaggleStart = 1;
    V3Df temp;
    motor_timers_t mtt;
    mtt.motorx = 0;
    mtt.motory = 0;
    mtt.motorz = 0;
    
    Init_AllTimers();
    stopCounter =0;
    u8 DanceInitFlag = 0;
    global_tick_count =0;

    gpio_set_level(GPIO_WING_IN1, 0);
    gpio_set_level(GPIO_WING_IN2, 1);
    while (1)
    {   
        mtt.motorx = 0;
        mtt.motory = 0;
        mtt.motorz = 0;

        temp.x = 0.0;
        temp.y = 0.0;
        temp.z = 0.0;
        
        if(sysState & SYS_STATE_KEYBOARD_CTRL_ENABLED)
        {
            //ESP_LOGI(TAG, "key press: %d  %d   %d  %d", limitSwitches[2][0],limitSwitches[2][1],limitSwitches[2][2],limitSwitches[2][3]);
            if((keypad & KEY_LEFT) && !(keypad & KEY_RIGHT))
            {    
                //ESP_LOGI(TAG, "KEY_LEFT ");
                if(limitSwitches[1][0])
                    motionFactorX = 0;
                else 
                {
                    if ( motionFactorX < 1 )
                    {
                        motionFactorX += MOTION_ACCEL_FACTOR;
                    }
                    else     
                        motionFactorX = 1;
                }
                motorX_Key_Offset = -keypadSpeed*motionFactorX;

            }
            else if(!(keypad & KEY_LEFT) && (keypad & KEY_RIGHT))
            {    
                //ESP_LOGI(TAG, "KEY_RIGHT");
                if(limitSwitches[1][1])
                    motionFactorX = 0;
                else 
                {
                    if ( motionFactorX < 1 )
                    {
                        motionFactorX += MOTION_ACCEL_FACTOR;
                        
                    }
                    else     
                        motionFactorX = 1;
                }
                motorX_Key_Offset = keypadSpeed*motionFactorX;
            }
            else
            {
                motorX_Key_Offset = 0;
                motionFactorX = 0;
            }
            if((keypad & KEY_UP) && !(keypad & KEY_DOWN))
            {    
                //ESP_LOGI(TAG, "KEY_UP");
                if(limitSwitches[1][3])
                    motionFactorY = 0;
                else 
                {
                    if ( motionFactorY < 1 )
                    {
                        motionFactorY += MOTION_ACCEL_FACTOR;
                        
                    }
                    else     
                        motionFactorY = 1;
                }
                motorY_Key_Offset = keypadSpeed*motionFactorY;

            }
            else if(!(keypad & KEY_UP) && (keypad & KEY_DOWN))
            {    
                //ESP_LOGI(TAG, "KEY_DOWN");
                if(limitSwitches[1][2])
                    motionFactorY = 0;
                else 
                {
                    if ( motionFactorY < 1 )
                    {
                        motionFactorY += MOTION_ACCEL_FACTOR;
                        //ESP_LOGI(TAG, "motionFactorY - %f",motionFactorY );
                    }
                    else     
                        motionFactorY = 1;
                    
                }
                motorY_Key_Offset = -keypadSpeed*motionFactorY;
            }  
            else 
            {   
                motionFactorY = 0;
                motorY_Key_Offset = 0;    
            }

            // Z motor CW/CCW (only one works)
            if ((keypad & KEY_ZCW) && !(keypad & KEY_ZCCW))
                motorZ_Key_Offset = keypadSpeed;
            else if (!(keypad & KEY_ZCW) && (keypad& KEY_ZCCW))
                motorZ_Key_Offset = -keypadSpeed;
            else
                motorZ_Key_Offset = 0;

            state |= STATE_NEW_KEY_EVENT;    
        }
        else
        {
            keypad = 0xFF;
            state &= ~STATE_NEW_KEY_EVENT;
        }   
        
        //Check if robot has reached its side limits i.e. Limit switches
        if(!(limitSwitches[1][0]|limitSwitches[1][1]|limitSwitches[1][2]|limitSwitches[1][3]))
            sysState &= ~SYS_STATE_DOWN_ROBOT;
        else
            sysState |= SYS_STATE_DOWN_ROBOT;  

        if(oneMilliSecondFlag)
        {   
            if(wings_timer<0 || wings_timer>7699)
                wings_timer = 0;
            else    
                wings_timer++;    
            
            oneMilliSecondFlag = 0;
            
            if(!(sysState&SYS_STATE_DOWN_ROBOT) && (DanceInitFlag & DSTATE_ANGLE_CORR)&& (keypad & KEY_D_ONOFF) )
            {
                w_old = EncAng;
                if(DanceInitFlag & DSTATE_DIRECTION_CORR )
                {
                    Rotz = params.angle + params.divergence/2.0;
                    if (Rotz < 0) 
                        Rotz += PI2;
                    Rot_dir_z = -1.0;
                    diff = Rotz - w_old;
                    if ( ((diff > 0) && (diff < PI)) || ((diff < 0) && (abs(diff) > PI)) )
                    {
                            Rot_dir_z = 1.0;
                    }
                    ESP_LOGI(TAG, "DIRECTION CORR %f",Rotz );
                    DanceInitFlag &= ~DSTATE_DIRECTION_CORR;
                }  
                else if(DanceInitFlag & DSTATE_DIR_RETURN_CORR)
                {
                    Rotz = params.angle - params.divergence/2.0;
                    if (Rotz < 0) 
                        Rotz += PI2;
                    Rot_dir_z = -1.0;
                    diff = Rotz - w_old;
                    if ( ((diff > 0) && (diff < PI)) || ((diff < 0) && (abs(diff) > PI)) )
                    {
                            Rot_dir_z = 1.0;
                    }
                    ESP_LOGI(TAG, "RETURN RUN CORR %f",Rotz );
                    DanceInitFlag &= ~DSTATE_DIR_RETURN_CORR;
                }

                diff = Rotz - w_old;
                
                if ( fabs(diff) < 0.006 ) 
                {
                    DanceInitFlag &= ~DSTATE_ANGLE_CORR;
                    state |= STATE_DANCE_ACTIVE; 
                }
                if (fabs(diff) < 0.15)
                {
                    if (Rot_dir_z < 0)
                    Rot_dir_z = -MIN_SET_ORIENTATION_SPEED;
                    else
                    Rot_dir_z = MIN_SET_ORIENTATION_SPEED;
                }
                else
                {
                    if (Rot_dir_z < 0)
                    Rot_dir_z = -MAX_SET_ORIENTATION_SPEED;
                    else
                    Rot_dir_z = MAX_SET_ORIENTATION_SPEED;
                }
               
                Rotateatapoint(w_old,Rot_dir_z*(PI2),0,0,0);

            }
            else if ( !(sysState&SYS_STATE_DOWN_ROBOT)  &&( (state & STATE_DANCE_ACTIVE) || (state & STATE_WAGGLE_ACTIVE) ) )
            {   
                //ESP_LOGI(TAG, "lol %d",sysState&SYS_STATE_DOWN_ROBOT); 
                stopCounter=0;
                global_tick_count = global_tick_count % NUM_POINTS_TRAJECTORY;

                //temp = danceGetNextVelocities(); 
                //ESP_LOGI(TAG, "Robot On dance if");  
                // gpio_set_level(GP2, dirToggle2);
                // dirToggle2 = !dirToggle2;
                if(global_tick_count == 0) 
                {
                    DanceInitFlag |= DSTATE_DIRECTION_CORR;
                    DanceInitFlag |= DSTATE_ANGLE_CORR;

                }
                else if(global_tick_count == NUM_POINTS_TRAJECTORY/2) 
                {
                    DanceInitFlag |= DSTATE_DIR_RETURN_CORR;
                    DanceInitFlag |= DSTATE_ANGLE_CORR;

                } 
                wTemp = wings_timer%77;
                if(wTemp>18 || dance_speed_x[global_tick_count] != dance_speed_x[0] )
                {   
                    gpio_set_level(GPIO_WING_IN1, 0);
                    gpio_set_level(GPIO_WING_IN2, 1);
                    //ESP_LOGI(TAG, "l" );
                } 
                else if(wTemp==0 || wTemp==4 ||wTemp==8)
                {   
                    gpio_set_level(GPIO_WING_IN1, 0);
                    gpio_set_level(GPIO_WING_IN2, 0);
                }
                else if(wTemp==2 || wTemp==6 ||wTemp==10)
                {   
                    gpio_set_level(GPIO_WING_IN1, 1);
                    gpio_set_level(GPIO_WING_IN2, 1);
                }  
                
                mtt.motorx= (int) (cosf(params.angle)*dance_speed_x[global_tick_count]- sinf(params.angle)*dance_speed_y[global_tick_count]) + motorX_Key_Offset*motorSpeedFactor;
                mtt.motory =(int) (cosf(params.angle)*dance_speed_y[global_tick_count]+ sinf(params.angle)*dance_speed_x[global_tick_count])  + motorY_Key_Offset*motorSpeedFactor;
                mtt.motorz =  dance_speed_a[global_tick_count] + motorZ_Key_Offset;   
                global_tick_count++;
                
                if (sysState & SYS_STATE_KEYBOARD_CTRL_ENABLED)
                {
                    if ( !(keypad & KEY_D_ONOFF) )
                    {
                        
                            state &= ~STATE_DANCE_ACTIVE; 
                            // //Set system status
                            // sysState &= ~SYS_STATE_DANCING;
                            rampFactor = 0.005;
                            motorXYZDrive(0,0,0);           // Motor Stop
                            //timer_pause(TIMER_GROUP_0, TIMER_0);         // Dancetimer stop   
                            timer_set_alarm_value(0, 0, 1000 );
                        
                    }
                    else
                    {
                        if (rampFactor < 1 )
                            rampFactor += RAMP_ACCEL_FACTOR;
                        else
                        {
                            rampFactor = 1;       
                        }
                    }
                }
                else
                {
                    rampFactor = 1;
                    motorX_Key_Offset = 0;
                    motorY_Key_Offset = 0;
                    motorZ_Key_Offset = 0;
                }
                //ESP_LOGI(TAG, "ramp %f",rampFactor);    
                mtt.motorx *= rampFactor;
                mtt.motory *= rampFactor;
                mtt.motorz *= rampFactor;  
                if (rampFactor > 0.005)
                    timer_set_alarm_value(0, 0, 1000 * rampFactor ); 
                else    
                    timer_set_alarm_value(0, 0, 1000 );   
                if(mtt.motorx != 0)
                    timer_start(TIMER_GROUP_1, TIMER_0);
                if(mtt.motory != 0)
                        timer_start(TIMER_GROUP_1, TIMER_1);
                if(mtt.motorz != 0)
                        timer_start(TIMER_GROUP_0, TIMER_1);
                motorXYZDrive( mtt.motorx , mtt.motory, mtt.motorz);
            }
            else if (  !(sysState&SYS_STATE_DOWN_ROBOT) && ((keypad & KEY_ZCW) || (keypad & KEY_ZCCW) ) && (state & STATE_KEY_MOTORS_ON) )
            {    
                stopCounter = 0;
                state |= STATE_KEY_MOTORS_ON;
                //gpio_set_level(GP2, dirToggle2);
                dirToggle2 = !dirToggle2;   
                if (!(sysState & SYS_STATE_PARAMS_SET))
                {
                    loadDefaultDanceParameters(&params);  
                    //ESP_LOGI(TAG, "Param load loop 1");             
                }
                if (rampFactor < 1)
                        rampFactor += RAMP_ACCEL_FACTOR;
                else
                {
                    rampFactor = 1;       
                }
                // current Z encoder angle in radians
                w_old = EncAng;
                Rotateatapoint(w_old,motorZ_Key_Offset*(PI2),0,0,0);
                
                
                //***************************************//
                //**************************************//
                //This section is for the rotation and waggle angle update function comment out if not needed
                if ((sysState & SYS_STATE_PARAMS_SET)& (CurrMode == RotateNewWaggle || CurrMode == RotateContinueWaggle) )
                {
                    params.angle = EncAng;
                    
                }    
                firstWaggleStart = 1;  
            
                
                //ESP_LOGI(TAG, "val set  %d %d %d %f %f",mtt.motorx, mtt.motory, mtt.motorz, EncAng,w_cur);
            }
            else if ( (state & STATE_NEW_KEY_EVENT)&&  !(state & STATE_DANCE_ACTIVE)) 
            {  
                state &= ~STATE_NEW_KEY_EVENT;
                if ( (keypad & KEY_D_ONOFF) &&  !(state & STATE_DANCE_ACTIVE) && (state & STATE_KEY_MOTORS_ON) )
                {
                //Start/Resume Waggle
                    if (firstWaggleStart)
                    {   
                        
                        // If no parameters have been set by the GUI yet, then load and use default parameters
                        if (!(sysState & SYS_STATE_PARAMS_SET))
                        {
                            loadDefaultDanceParameters(&params);  
                            //ESP_LOGI(TAG, "Param load loop 2 %d",global_tick_count );       
                            sysState|= SYS_STATE_PARAMS_SET;
                            
                        }
                        //danceInit(); 
                        global_tick_count = 0;
                        DanceInitFlag |= DSTATE_DIRECTION_CORR;
                        DanceInitFlag |= DSTATE_ANGLE_CORR;
                        ESP_LOGI(TAG, "DanceInit  %d",global_tick_count);
                        firstWaggleStart = 0;                                         
                    }
                    if(CurrMode == RotateNewWaggle)
                        global_tick_count = 0;
                    //sysState |= SYS_STATE_SROISTREAM_ENABLED;
                    sysState |= SYS_STATE_DANCING;
                    // DanceTimer starten
                    if(!(DanceInitFlag & DSTATE_ANGLE_CORR)&&!(DanceInitFlag & DSTATE_DIRECTION_CORR))
                        state |= STATE_DANCE_ACTIVE; 
                    //ESP_LOGI(TAG, "lol2 %d",sysState&SYS_STATE_DOWN_ROBOT); 
                    
                    timer_start(TIMER_GROUP_0, TIMER_0);  
                    
                }
                else if ( (keypad & KEY_MODE) )
                {
                    if (((state & STATE_KEY_MOTORS_ON) && (keypad==128) && !stopCounter))
                    {
                        // Motoren ausschalten
                        DanceInitFlag &= ~DSTATE_ANGLE_CORR;
                        stopCounter = 1;
                        state &= ~STATE_KEY_MOTORS_ON;
                        
                        //timer_pause(TIMER_GROUP_0, TIMER_0);           // Dancetimer stop
                        motorXYZDrive(0,0,0);   // Motoren stop
                        timer_pause(TIMER_GROUP_0, TIMER_1);
                        timer_pause(TIMER_GROUP_1, TIMER_0);
                        timer_pause(TIMER_GROUP_1, TIMER_1);
                        set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);
                        set_motor_direction_pin(GPIO_MOTOR_PHI_STEP,  0);
                        set_motor_direction_pin(GPIO_MOTOR_X_STEP,    0);
                        set_motor_direction_pin(GPIO_MOTOR_Y_STEP,    0); 
                        //ESP_LOGI(TAG, "Robot HALT %d ",encoder_val);
                    }
                    else if(keypad!=128)
                    {
                        // Motoren start
                        stopCounter = 0;
                        state |= STATE_KEY_MOTORS_ON;
                        set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);
                        
                        w_old = EncAng;
                        if(motorX_Key_Offset < 0)
                        {
                            if(motorY_Key_Offset == 0)
                                Rotz = PI;
                            else if (motorY_Key_Offset < 0)
                                Rotz = PI*5/4;   
                            else
                                Rotz = PI*3/4;    
                        }
                        else if(motorX_Key_Offset > 0)
                        {
                            if(motorY_Key_Offset == 0)
                                Rotz = 0;
                            else if (motorY_Key_Offset < 0)
                                Rotz = PI*7/4;   
                            else
                                Rotz = PI*1/4;    
                        }
                        else
                        {
                            if(motorY_Key_Offset > 0)
                                Rotz = PI/2;
                            else if (motorY_Key_Offset < 0)
                                Rotz = PI*3/2; 
                            else    
                                Rotz = 0;        
                        }   
                       
                        if (Rotz < 0) 
                            Rotz += PI2;
                        Rot_dir_z = -1.0;
                        diff = Rotz - w_old;
                        if ( ((diff > 0) && (diff < PI)) || ((diff < 0) && (abs(diff) > PI)) )
                        {
                                Rot_dir_z = 1.0;
                        }
                        diff = Rotz - w_old;
                        

                        timer_start(TIMER_GROUP_0, TIMER_1);
                        timer_start(TIMER_GROUP_1, TIMER_0);
                        timer_start(TIMER_GROUP_1, TIMER_1);
                        

                        if(!(keypad & KEY_D_ONOFF))
                        {
                            if(!(limitSwitches[1][0]||limitSwitches[1][1]||limitSwitches[1][2]||limitSwitches[1][3]) && CurrMode== RotateMotion)
                            {
                                if ( fabs(diff) < 0.01 ) 
                                {
                                    motorXYZDrive( motorX_Key_Offset*motorSpeedFactor, motorY_Key_Offset*motorSpeedFactor, 0);
                                    //ESP_LOGI(TAG, "f1 %f",diff );
                                }
                                else if (fabs(diff) < 0.15)
                                {
                                    if (Rot_dir_z < 0)
                                    Rot_dir_z = -MIN_SET_ORIENTATION_SPEED;
                                    else
                                    Rot_dir_z = MIN_SET_ORIENTATION_SPEED;
                                    //ESP_LOGI(TAG, "f2 %f",diff );
                                    Rotateatapoint(w_old,Rot_dir_z*(PI2),motorX_Key_Offset*motorSpeedFactor,motorY_Key_Offset*motorSpeedFactor,0);
                                }
                                else if (fabs(diff) > 0.15)
                                {
                                    if (Rot_dir_z < 0)
                                    Rot_dir_z = -MAX_SET_ORIENTATION_SPEED;
                                    else
                                    Rot_dir_z = MAX_SET_ORIENTATION_SPEED;
                                    //ESP_LOGI(TAG, "f3 %f",diff );
                                    Rotateatapoint(w_old,Rot_dir_z*(PI2),motorX_Key_Offset*motorSpeedFactor,motorY_Key_Offset*motorSpeedFactor,0);
                                }
                            }
                            else 
                            {
                                //ESP_LOGI(TAG, "f4 %f",diff );
                                motorXYZDrive( motorX_Key_Offset*motorSpeedFactor, motorY_Key_Offset*motorSpeedFactor, 0);
                            }  
                        }       
                    }
                } 
                else if(!(keypad & KEY_MODE) && (keypad & KEY_LEFT) && (CurrMode== FixedWaggle )&& ModeSwitch)
                { 
                    ESP_LOGI(TAG, "Robot Param  angle reset %f ",EncAng);
                    params.angle = EncAng;
                }
                else if(!(keypad & KEY_MODE) && (keypad & KEY_ZCW))
                { 
                    
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);  
                    motor_isr_toggle(MOTOR_PHI,1); 
                    //ESP_LOGI(TAG, "Rzcw ");
                }    
                else if(!(keypad & KEY_MODE) && (keypad & KEY_ZCCW))
                { 
                    
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);  
                    motor_isr_toggle(MOTOR_PHI,0);    
                } 
                else if(!(keypad & KEY_MODE) && (keypad & KEY_UP) && (keypad & KEY_DOWN)&& ModeSwitch)
                { 
                    ResetEnc = 1;
                    ESP_LOGI(TAG, "Reset Enc ");    
                    
                }
                else if(!(keypad & KEY_MODE) && (keypad & KEY_RIGHT)&& ModeSwitch)
                { 
                    CurrMode++;
                    CurrMode = (CurrMode<4)?CurrMode:0;
                    ESP_LOGI(TAG, "Currmode %d ",CurrMode);
                    ModeSwitch = 0;
                } 
                else if(!(keypad & KEY_MODE))
                {
                    firstWaggleStart = 1; 
                    stopCounter =1; 
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 1);  
                }
                if(!(keypad & KEY_MODE) && !(keypad & KEY_RIGHT))
                { 
                    ModeSwitch = 1;
                } 

            }
            else if((sysState&SYS_STATE_DOWN_ROBOT) )
            {   
                stopCounter =1;
                timer_pause(TIMER_GROUP_0, TIMER_1);
                timer_pause(TIMER_GROUP_1, TIMER_0);
                timer_pause(TIMER_GROUP_1, TIMER_1); 
                set_motor_direction_pin(GPIO_MOTOR_PHI_STEP,  0);
                set_motor_direction_pin(GPIO_MOTOR_X_STEP,    0);
                set_motor_direction_pin(GPIO_MOTOR_Y_STEP,   0);  
                ESP_LOGI(TAG, "Robot LIMIT SW %d ",encoder_val);
                DanceInitFlag &= ~DSTATE_ANGLE_CORR;
                //state &= ~STATE_KEY_MOTORS_ON;
                state &= ~STATE_DANCE_ACTIVE; 
                //sysState &= ~SYS_STATE_DANCING;       
            }
        
        }
    }

}
