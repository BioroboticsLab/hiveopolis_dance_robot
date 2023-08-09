

#define MOTOR_PHI                               0
#define MOTOR_X                                 1
#define MOTOR_Y                                 2



#define MOTOR_DIRECTION_CW                      0
#define MOTOR_DIRECTION_CCW                     1

#define FULL_STEPS_PER_REVOLUTION               200
#define MICROSTEPS_PER_FULL_STEP                8
#define MICROSTEPS_PER_FULL_STEP_PHI            8
#define STEPS_PER_REVOLUTION                    (FULL_STEPS_PER_REVOLUTION*MICROSTEPS_PER_FULL_STEP)
#define STEPS_PER_REVOLUTION_PHI                   (FULL_STEPS_PER_REVOLUTION*MICROSTEPS_PER_FULL_STEP_PHI)
#define AXIS_MOVEMENT_PER_REVOLUTION_MM         4.88
#define CONVERSION_FACTOR_SPEED_TO_STEPS_XY     (STEPS_PER_REVOLUTION/AXIS_MOVEMENT_PER_REVOLUTION_MM)

#define PI2                                     6.283185307179586
#define CONVERSION_FACTOR_SPEED_TO_STEPS_PHI    STEPS_PER_REVOLUTION_PHI/PI2




#ifndef linear_speed_to_timer_value
//#define linear_speed_to_timer_value(speed)         1000000 / (speed*CONVERSION_FACTOR_SPEED_TO_STEPS_XY/ (FIXED_POINT_SCALER_YX / 1000) )
#define linear_speed_to_timer_value(speed)         500000 / (speed*CONVERSION_FACTOR_SPEED_TO_STEPS_XY / 100)
#endif

#ifndef rotary_speed_to_timer_value
//#define rotary_speed_to_timer_value(speed)         1000000 / (speed*( CONVERSION_FACTOR_SPEED_TO_STEPS_PHI / (FIXED_POINT_SCALER_PHI / 1000)))
#define rotary_speed_to_timer_value(speed)         500000 / (speed* CONVERSION_FACTOR_SPEED_TO_STEPS_PHI / 1000 ) // saves a division // divide by 2 because we toggle on/off and only upwards flank produces step
#endif

#ifndef set_motor_direction_pin
#define set_motor_direction_pin(motor_dir_pin, direction)   gpio_set_level(motor_dir_pin, direction);
#endif
