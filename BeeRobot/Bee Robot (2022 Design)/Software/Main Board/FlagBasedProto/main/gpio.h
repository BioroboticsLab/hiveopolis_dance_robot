#ifndef GPIO_H
#define GPIO_H

/*
    PIN DEFINITIONS
*/

#define GPIO_MOTOR_ENABLE   4
// #define GPIO_MOTOR_X_ENABLE     16
// #define GPIO_MOTOR_Y_ENABLE     4

#define GPIO_MOTOR_PHI_STEP      19
#define GPIO_MOTOR_PHI_DIR       21

#define GPIO_MOTOR_X_STEP        22   
#define GPIO_MOTOR_X_DIR         23

#define GPIO_MOTOR_Y_STEP        5
#define GPIO_MOTOR_Y_DIR         18

#define GPIO_HOME_SWITCH_X_MIN   34
#define GPIO_HOME_SWITCH_X_MAX   15 

#define GPIO_HOME_SWITCH_Y_MIN   39 
#define GPIO_HOME_SWITCH_Y_MAX   13     

  #define GPIO_WING_IN1          32
  #define GPIO_WING_IN2        33

const int Motor_Direction[] = {GPIO_MOTOR_PHI_DIR, GPIO_MOTOR_X_DIR, GPIO_MOTOR_Y_DIR};
#endif