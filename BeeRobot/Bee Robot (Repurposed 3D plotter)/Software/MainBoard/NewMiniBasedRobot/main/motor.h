

#define MOTOR_PHI                               0
#define MOTOR_X                                 1
#define MOTOR_Z                                 2
#define MOTOR_Y                                 3



#define MOTOR_DIRECTION_CW                      0
#define MOTOR_DIRECTION_CCW                     1

#define FULL_STEPS_PER_REVOLUTION               200
#define MICROSTEPS_PER_FULL_STEP_XY               8
#define MICROSTEPS_PER_FULL_STEP_Z               8
#define MICROSTEPS_PER_FULL_STEP_PHI            8
#define STEPS_PER_REVOLUTION_XY                    (FULL_STEPS_PER_REVOLUTION*MICROSTEPS_PER_FULL_STEP_XY)
#define STEPS_PER_REVOLUTION_Z                    (FULL_STEPS_PER_REVOLUTION*MICROSTEPS_PER_FULL_STEP_Z)
#define STEPS_PER_REVOLUTION_PHI                (FULL_STEPS_PER_REVOLUTION*MICROSTEPS_PER_FULL_STEP_PHI)
#define AXIS_MOVEMENT_PER_REVOLUTION_MM_Z        16
#define AXIS_MOVEMENT_PER_REVOLUTION_MM_XY       16
#define CONVERSION_FACTOR_SPEED_TO_STEPS_Z     (STEPS_PER_REVOLUTION_Z/AXIS_MOVEMENT_PER_REVOLUTION_MM_Z)
#define CONVERSION_FACTOR_SPEED_TO_STEPS_XY     (STEPS_PER_REVOLUTION_XY/AXIS_MOVEMENT_PER_REVOLUTION_MM_XY)
#define PI2                                     6.283185307179586
#define CONVERSION_FACTOR_SPEED_TO_STEPS_PHI    STEPS_PER_REVOLUTION_PHI/PI2




#ifndef linear_speed_to_timer_value_Z
//#define linear_speed_to_timer_value(speed)         1000000 / (speed*CONVERSION_FACTOR_SPEED_TO_STEPS_Z/ (FIXED_POINT_SCALER_Z / 1000) )
#define linear_speed_to_timer_value_Z(speed)         500000 / (speed*CONVERSION_FACTOR_SPEED_TO_STEPS_Z / 100)
#endif

#ifndef linear_speed_to_timer_value_XY
//#define linear_speed_to_timer_value(speed)         1000000 / (speed*CONVERSION_FACTOR_SPEED_TO_STEPS_XY/ (FIXED_POINT_SCALER_XY / 1000) )
#define linear_speed_to_timer_value_XY(speed)         500000 / (speed*CONVERSION_FACTOR_SPEED_TO_STEPS_XY / 100)
#endif

#ifndef rotary_speed_to_timer_value
//#define rotary_speed_to_timer_value(speed)         1000000 / (speed*( CONVERSION_FACTOR_SPEED_TO_STEPS_PHI / (FIXED_POINT_SCALER_PHI / 1000)))
#define rotary_speed_to_timer_value(speed)         500000 / (speed* CONVERSION_FACTOR_SPEED_TO_STEPS_PHI / 1000 ) // saves a division // divide by 2 because we toggle on/off and only upwards flank produces step
#endif

#ifndef set_motor_direction_pin
#define set_motor_direction_pin(motor_dir_pin, direction)   gpio_set_level(motor_dir_pin, direction);
#endif
