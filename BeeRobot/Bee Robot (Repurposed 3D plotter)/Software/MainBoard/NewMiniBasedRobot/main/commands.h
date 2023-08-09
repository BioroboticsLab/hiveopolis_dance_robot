#ifndef __COMMANDS_H__
#define __COMMANDS_H__


typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;  /* Read Only */
typedef signed short const sc16;  /* Read Only */
typedef signed char  const sc8;   /* Read Only */

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;  /* Read Only */
typedef volatile signed short const vsc16;  /* Read Only */
typedef volatile signed char  const vsc8;   /* Read Only */

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;  /* Read Only */
typedef unsigned short const uc16;  /* Read Only */
typedef unsigned char  const uc8;   /* Read Only */

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;  /* Read Only */
typedef volatile unsigned short const vuc16;  /* Read Only */
typedef volatile unsigned char  const vuc8;   /* Read Only */



// System status flags
#define SYS_STATE_PARAMS_SET            0x00000001  // Parameters have been uploaded (ATTENTION: Will be set if last parameter
                                                    // [Return Duration] was set, otherwise constant queries would be necessary)
#define SYS_STATE_ZEROORIENTATION_SET   0x00000002 // Zero orientation was set
#define SYS_STATE_MOTORX_ENABLED        0x00000004 // X motor is enabled (if not then DOWN_ROBOT flag is set)
#define SYS_STATE_MOTORY_ENABLED        0x00000008 // Y motor is enabled
#define SYS_STATE_MOTORZ_ENABLED        0x00000010 // Z motor is enabled
#define SYS_STATE_SROISTREAM_ENABLED    0x00000020 // SROI stream is enabled
#define SYS_STATE_DANCING               0x00000040 // Robot is in the dancing phase
#define SYS_STATE_DOWN_ROBOT            0x00000080 // Robot has been shut down (MOTORX_ENABLED not set)
#define SYS_STATE_RECEIVING_CAMDATA     0x00000100 // Camera data is being received
#define SYS_STATE_WINGS_ENABLED         0x00000200 // Wing is enabled
#define SYS_STATE_TROPH_ENABLED         0x00000400 // trophallaxis is enabled
#define SYS_STATE_HEAT_ENABLED          0x00000800 // Heat is enabled
#define SYS_STATE_WINGS_TEST_ENABLED    0x00001000 // Wing test is enabled
#define SYS_STATE_TROPH_TEST_ENABLED    0x00002000 // Trophalaxis test is enabled
#define SYS_STATE_HEAT_TEST_ENABLED     0x00004000 // Heat test is enabled
#define SYS_STATE_GUI_CTRL_ENABLED      0x00008000 // GUI control is enabled
#define SYS_STATE_KEYBOARD_CTRL_ENABLED 0x00010000 // Keyboard control is enabled
//#define SYS_STATE_JOYSTICK_CTRL_ENABLED 0x00020000 // Joystick control is enabled

#define SYS_STATE_ROBOT_ONLINE          0x00800000  // Robot is Online

// Keyboard Flags

#define KEY_ZCCW    0x01
#define KEY_ZCW     0x02
#define KEY_DOWN    0x04
#define KEY_RIGHT   0x08
#define KEY_UP      0x10
#define KEY_LEFT    0x20
#define KEY_D_ONOFF   0x40
#define KEY_MODE   0x80 


//Dance Init Flags
#define DSTATE_DIRECTION_CORR 0x01
#define DSTATE_ANGLE_CORR  0x02
#define DSTATE_DIR_RETURN_CORR 0x04

// StatusFlags
#define STATESTREAM_ENABLE    0x01         
#define STATE_DANCE_ACTIVE    0x02        // Timer has been started and continuously prints the entire dance
#define STATE_WAGGLE_ACTIVE   0x04        // Timer has been started and only outputs one waggle run, after which it is stopped again
#define SROI_STREAM_ENABLED   0x08        // SROI Stream is started
#define STATE_NEW_KEY_EVENT   0x10
#define STATE_KEY_MOTORS_ON   0x20
#define STATE_JOY_MOTORS_ON   0x40
#define STATE_EFILED_ON       0x80


#define STATE_TICK_INTERVAL_GUI       120000
#define STATE_TICK_INTERVAL_JOYSTICK  2500
#define STATE_TICK_INTERVAL_KEYBOARD  120000


#define ROBO_CMD_SEND_ACK                   0xA0
#define ROBO_CMD_SEND_NACK_CRC              0xA1     
#define ROBO_CMD_SEND_NACK_CMD              0xA2
#define ROBO_CMD_SEND_STATE                 0xA3
#define ROBO_CMD_SEND_DEBUG                 0xA4
#define ROBO_CMD_SEND_NACK_CAM              0xA5

//#define ROBO_CMD_JOYSTICK_MOVE              0xB0
#define ROBO_CMD_SET_DANCEPARAMS            0xB1
#define ROBO_CMD_SET_ZEROORIENTATION        0xB2
#define ROBO_CMD_MOVE_ZEROORIENTATION       0xB3
#define ROBO_CMD_START_MAN_WAGGLE           0xB4
#define ROBO_CMD_START_DANCE                0xB5
#define ROBO_CMD_STOP_DANCE                 0xB6
#define ROBO_CMD_MOVE_DOWN                  0xB7
#define ROBO_CMD_SET_MOTOR_STATE            0xB8
#define ROBO_CMD_SET_ORIENTATION            0xB9
#define ROBO_CMD_STREAMING_STATE            0xBA
#define ROBO_CMD_PAUSE_DANCE                0xBB
#define ROBO_CMD_WINGS_CONFIG               0xBC
#define ROBO_CMD_TROPH_CONFIG               0xBD
#define ROBO_CMD_HEAT_CONFIG                0xBE
#define ROBO_CMD_SET_CTRL_MODE              0xBF



// #define CAM_ID_A                            0x00
// #define CAM_ID_B                            0x01		

// #define CAM_MODE_IDLE                       0x00
// #define CAM_MODE_FULLSCREEN                 0x01
// #define CAM_MODE_ROI                        0x02
// #define CAM_MODE_DETECTION                  0x03

// #define CAM_CMD_ACK                         0xAA
// #define CAM_CMD_NACK                        0xBB
// #define CAM_CRC_NACK                        0xCC

// #define CAM_SET_REGISTER                    0x01
// #define CAM_READ_REGISTER                   0x02
// #define CAM_SET_FULLSCREEN_POS              0x03
// #define CAM_GET_FULLSCREEN_POS              0x04
// #define CAM_SET_ROI_POS                     0x05
// #define CAM_GET_ROI_POS                     0x06
// #define CAM_SET_DETECTION_PARAMS            0x07
// #define CAM_GET_DETECTION_PARAMS            0x08
// #define CAM_SET_RAY_PIXEL_ID                0x09
// #define CAM_GET_RAY_PIXEL_ID                0x0A
// #define CAM_SET_THRESHOLD                   0x0B
// #define CAM_GET_THRESHOLD                   0x0C
// #define CAM_SET_CAM_MODE                    0x0D
// #define CAM_GET_CAM_MODE                    0x0E
// #define CAM_LEAVE_CONF_MODE                 0x0F

// #define ROBO_CMD_CAM_SET_REG                0xD0
// #define ROBO_CMD_CAM_READ_REG               0xD1
// #define ROBO_CMD_CAM_SET_MODE               0xD2
// #define ROBO_CMD_CAM_GET_MODE               0xD3
// #define ROBO_CMD_CAM_SET_FULLSCREEN_POS     0xD4
// #define ROBO_CMD_CAM_GET_FULLSCREEN_POS     0xD5
// #define ROBO_CMD_CAM_SET_ROI_POS            0xD6
// #define ROBO_CMD_CAM_GET_ROI_POS            0xD7
// #define ROBO_CMD_CAM_SET_DETECTION_PARAMS   0xD8
// #define ROBO_CMD_CAM_GET_DETECTION_PARAMS   0xD9
// #define ROBO_CMD_CAM_SET_RAY_PIXEL_ID       0xDA
// #define ROBO_CMD_CAM_GET_RAY_PIXEL_ID       0xDB
// #define ROBO_CMD_CAM_SET_THRESHOLD          0xDC
// #define ROBO_CMD_CAM_GET_THRESHOLD          0xDD


#endif
