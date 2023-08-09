#ifndef __DANCEGEN_H__
#define __DANCEGEN_H__

#include "types.h"
#include "commands.h"


#define MIN_SET_ORIENTATION_SPEED 50
#define MAX_SET_ORIENTATION_SPEED 100

#define PI2 6.2831853
#define PI  3.1415926

#define BR_DANCESIM_SUBPATH_WAGGLE_LEFT 0
#define BR_DANCESIM_SUBPATH_WAGGLE_RIGHT 1
#define BR_DANCESIM_SUBPATH_RETURN_LEFT 2
#define BR_DANCESIM_SUBPATH_RETURN_RIGHT 3
#define MOTOR_MAX_SPEED_IN_MM_PER_SEC 200

// Statusbits
#define DANCE_STATE_SETORIENTATION  0x01        // Ausrichtung der Orientierung
#define DANCE_STATE_RETURNRUN       0x02        // Der Return-Run wird ausgeführt
#define DANCE_STATE_RETURNRUN_1     0x04        // Erste schnelle Drehung des Return-Runs (nach Waggle)
#define DANCE_STATE_RETURNRUN_2     0x08        // langsamere Drehung des Return-Runs (langer Rücklauf)
#define DANCE_STATE_RETURNRUN_3     0x10        // Zweite schnelle Drehung des Return-Runs (vor Waggle)

typedef struct DANCE_PARAMS_
{
        float   angle;                                 // [rad]; mittlerer Winkel 
        float   divergence;                            // [rad]; waggle_run orientierung: angle+-angle_divergence/2

        float   waggle_speed;                          // [mm/s]; vortrieb des waggle runs
        float   waggle_duration;                       // [ms]
        float   waggle_freq;                           // [Hz]
        float   waggle_amp_a;                          // [rad]
        float   return_duration;                        // [ms]   
        
        float   exc_length;                             // [mm]; excenter length (usually 6.5 mm - 10 mm)
        
        //for convenience we store:
        float   return_smoothing_duration;
        u16     resampling_steps;
        float   turn_func_integral;
        float   fwd_swd_coeff;
        float   return_weight;
        float   return_turn_vel_weight_offset;
        float   return_turn_vel_weight_integral;

        
        
} DANCE_PARAMS;

extern u16 danceInitStepsZ;
extern u16 danceStepOffset;
extern u8 danceState;



void danceInit();
void danceDriveOrientation(float g);
V3Df danceGetNextVelocities();
void danceSetRunningTime(int time);
void loadDefaultDanceParameters(DANCE_PARAMS *);
#endif
