#include "dancegen.h"
#include "motor.h"
#include "types.h"
#include "math.h"
#include "stdlib.h"
//#include "cpu.h"
//#include "cpu.h"
#include "commands.h"



#define FWD_SWD_COEFF 0.8

u16 danceInitStepsZ = 0;
u8 danceState;
u16 danceStepOffset;
//DANCE_PARAMS params;
V3Df Pcur;

int T; //global time
int w; //waggle duration
int wr; // waggle plus return duration
int wrw; // ...
int wrwr;

u16 t_wings;    // lokale Flügelzeit
u8 start_wings; // Flügel-Start Flag 

float t_prcnt;
float ramp;
int sign;
float v_fwd;
float v_swd;
float c;
float s;
float c1;
float c2;
float resampling_turn_coeff;
float resampling_fwd_swd_coeff;
float resampling_coeff;
float turn_scale;
float number_of_waggle_periods;
float number_of_sample_points_for_one_period;       
float rampFactor;

float diff;
float w_old;
float w_cur;
float vx;
float vy;
float vz;
float angle;

int r;  // return duration

V2Df p_old;
V2Df p_cur;

inline float return_orientation_difference(int t);
inline float return_turn_velocity(float t,DANCE_PARAMS *);

void danceSetRunningTime(int time)
{
    T = time;
}   

// Ohne Exzenter!!
// void danceDriveOrientation(float g)
// {
//   s16 diff;
//   u16 temp;
//   GPIO_InitTypeDef GPIO_InitStructure;

//   // Orientierung der Biene (Z-Motor) ausrichten wenn Z-Motor an ist   
//   if (sysState & SYS_STATE_MOTORZ_ENABLED)
//   {
//     // Orientierungswinkeln im Bogenmaß in Encoderschritte umrechnen
//     //danceInitStepsZ = (g*180/PI) * ENC_RES_PER_DEG  + danceStepOffset;    
//     danceInitStepsZ = (g*180/PI) * ENC_RES_PER_DEG;
//     danceInitStepsZ %= 4096;   

//     // Aktuelle Positionsdifferenz bestimmen    
//     temp = Enc_GetCurrentRotaryStep();
//     diff = temp - danceInitStepsZ;
//     if (diff != 0)
//     {
//         // kürzeste Strecke suchen und dementsprechend die Drehrichtung einstellen
//         if (diff < 0)
//         {
//             if ( abs(diff) < 2048 )
//                 motorSetDir(MOTOR_Z, MOTOR_DIR_PLUS);
//             else
//                 motorSetDir(MOTOR_Z, MOTOR_DIR_MINUS);               
//         }
//         else
//             if ( abs(diff) < 2048 )
//                 motorSetDir(MOTOR_Z, MOTOR_DIR_MINUS); 
//             else
//                 motorSetDir(MOTOR_Z, MOTOR_DIR_PLUS);               
                
//         // manuelle Motortaktierung
//         // Timer für Z-Motor deaktivieren
//         MOTOR_Z_STOP;
//         // Pinfunktion ändern
//         GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
//         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//         GPIO_Init(GPIOA, &GPIO_InitStructure);  

//         // Z solange drehen bis Initposition erreicht ist (Tolleranz von 2 Schritten)
//         while(abs(diff) > 2)
//         {
//           GPIOA->ODR &= ~GPIO_Pin_0;
//           cpu_Delay_us(400);
//           GPIOA->ODR |= GPIO_Pin_0;
//           cpu_Delay_us(400);
//           diff = Enc_GetCurrentRotaryStep() - danceInitStepsZ;
//         }
        
//         // Pinfunktion wiederherstellen
//         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//         GPIO_Init(GPIOA, &GPIO_InitStructure);  
//     }
//   }
// }


void loadDefaultDanceParameters( DANCE_PARAMS *params)
{
    params->angle = 0;                                                           
    params->divergence = 32 * PI / 180;                      //(default im paper: ~32°)                                             
    params->waggle_speed = 15.04;;                        
    params->waggle_duration = 440;                     
    params->waggle_freq = 12.67;                        
    params->waggle_amp_a = 6.9 * PI / 180;                                         
    params->return_duration = 2130;    
    params->exc_length = 23.0;
    params->return_smoothing_duration = 30.0;    
    //wenn man die polynomiellen fits aus matlab (robio_plots.m) nimmt, dann muss man 
    //die anzahl der resampling steps daher nehmen!
    params->resampling_steps = 700;   
    // the integral of the turning function times return duration is 
    // the amount of degrees we turned after the return phase
    // this has to be scaled to actually turn 360 - divergence/2
// params.turn_func_integral = 0.1415;
    params->turn_func_integral = 0.14179;
    params->fwd_swd_coeff = 0.8;
    params->return_weight = 100;
}

// void danceInit()
// {
//     rampFactor = 0.1;   
    
//     int i;
//     float f;
    
//     float turn_func_integral;
    

//     resampling_coeff = params.resampling_steps / params.return_duration;
//     resampling_fwd_swd_coeff = resampling_coeff * params.fwd_swd_coeff;

//     turn_func_integral = 0.0;
            
//     params.return_turn_vel_weight_offset = exp( (float) params.return_weight / 10.0) / 100.0;
//     params.return_turn_vel_weight_integral = (1.0/3.0 - 0.5 + 0.25 + params.return_turn_vel_weight_offset);
    
//     for (i = 0; i < params.return_duration; i++)
//     {  
//       turn_func_integral += return_turn_velocity((float)i / params.return_duration);;
//     }
			
//     turn_scale = (360.0 - params.divergence * 180.0/PI) / (turn_func_integral);	
    
//     r = params.return_duration;    
    
//     w = params.waggle_duration;
//     wr = w+r;
//     wrw = w+r+w;
//     wrwr = w+r+w+r;
    
//     Pcur.x = 0;
//     Pcur.y = 0;
//     Pcur.z = params.angle - params.divergence/2.0;
//     if (Pcur.z < 0) 
//         Pcur.z += PI2;
          
    
//     number_of_waggle_periods = (((float) params.waggle_duration/1000.0)*params.waggle_freq);
//     number_of_sample_points_for_one_period = (float) 1000.0 / params.waggle_freq;
    
//     // Erste Waggle Orientierung einstellen
//     // kürzeste Richtung bestimmen
//     // aktueller Z-Encoderwinkel im Bogenmaß

//     w_old = ( (360/4096.0) * Enc_GetCurrentRotaryStep() ) * (PI/180.0);  
    
//     angle = Pcur.z;
//     if (angle < 0.0)
//       angle += (2*PI);
//     if (angle > (2*PI) )
//       angle -= (2*PI);
    
//     vz = -1.0;
//     diff = angle - w_old;
//     if ( ((diff > 0) && (abs(diff) < PI)) || ((diff < 0) && (abs(diff) > PI)) )
//       vz = 1.0;

//     while(1)
//     {
//       // aktueller Z-Encoderwinkel im Bogenmaß
//       w_old = ( (360/4096.0) * Enc_GetCurrentRotaryStep() ) * (PI/180.0);
      
//       diff = angle - w_old;
//       // bei 0.5 Grad (0.009 Rad) Toleranz zwischen angestrebten und aktuellem Winkel -> Abbruch
//       if ( fabs(diff) < 0.009 ) 
//         break;

//       // wenn Winkel fast erreicht (3 Grad (0,05 RAD) unterschied) dann langsamer drehen
//       if (fabs(diff) < 0.05)
//       {
//         if (vz < 0)
//           vz = -MIN_SET_ORIENTATION_SPEED;
//         else
//           vz = MIN_SET_ORIENTATION_SPEED;
//       }
//       else
//       {
//         if (vz < 0)
//           vz = -MAX_SET_ORIENTATION_SPEED;
//         else
//           vz = MAX_SET_ORIENTATION_SPEED;
//       }
//       // Richtung einstellen
            
//       w_cur = w_old + (vz*(PI/180) / 1000);

//       p_old.x = params.exc_length*(cosf(w_old)); 
//       p_old.y = params.exc_length*(sinf(w_old));

//       p_cur.x = params.exc_length*(cosf(w_cur)); 
//       p_cur.y = params.exc_length*(sinf(w_cur));
   
//       vx = -1000*(p_old.x - p_cur.x);
//       vy = -1000*(p_old.y - p_cur.y);

//       motorXYZDrive( (float) vx, (float) vy, (float) vz);
//       cpu_Delay_ms(1);
//     }
//     motorXYZDrive( (float) 0, (float) 0, (float) 0);

//     T = 0; 
//     t_wings = 0;
// }


//forward velocity in return run
// t in [0, 1] wobei 1 ^= return_duration
// polynomieller fit in matlab gemacht 
// siehe (robio_paper_plots.m und new_dance_model.m)
// return-wert in mm / msec (?)
inline float return_fwd_velocity(float t)
{
        return resampling_fwd_swd_coeff*(0.728*t*t*t*t - 1.466*t*t*t + 0.9225*t*t - 0.1805*t + 0.03409);
}

//sidewards (positive is outwards), sonst wie oben
inline float return_swd_velocity(float t)
{
        return resampling_fwd_swd_coeff*(0.2683*t*t*t*t - 0.2951*t*t*t - 0.04566*t*t + 0.1027*t - 0.0337);
}

//turning velocity (positive is counter-clockwise)
//turn velocity wird gewichtet mit quadratischer Funktion, und gleich normalisiert mit dem integral dieser funktion
inline float return_turn_velocity(float t, DANCE_PARAMS *params)
{
        return (( (t-0.5)*(t-0.5) + params->return_turn_vel_weight_offset ) / params->return_turn_vel_weight_integral ) * resampling_coeff*(-4.581*t*t*t + 7.517*t*t - 3.572*t + 0.8562);
}



inline char isTimeIn(int t, int left, int right)
{
    return ((t >= left) && (t < right));
}

// V3Df danceGetNextVelocities()
// {
//     V3Df R;
//     int pathID, t; 
//     float c1, c2, tempcos, tempsin, tempspeed;
    
    
//     if (state & STATE_WAGGLE_ACTIVE)
//     {
//         //if (T == 0)
//         //    danceInit();
            
//         if (T == w)
//         {
//             TIM_Cmd(TIM7, DISABLE);
//             state &= ~STATE_EFILED_ON;
//             TIM_Cmd(TIM6, DISABLE);         // Dancetimer stoppen   
//             motorXYZDrive(0,0,0);           // Motoren anhalten 
//             T = wr;
//             state &= ~STATE_WAGGLE_ACTIVE;
//             R.x = 0;
//             R.y = 0;
//             R.z = 0;
//             return R;
//         }
//         else if (T == wrw)
//         {
//             TIM_Cmd(TIM7, DISABLE);
//             state &= ~STATE_EFILED_ON;
//             TIM_Cmd(TIM6, DISABLE);         // Dancetimer stoppen   
//             motorXYZDrive(0,0,0);           // Motoren anhalten 
//             T = 0;
//             state &= ~STATE_WAGGLE_ACTIVE;
//             R.x = 0;
//             R.y = 0;
//             R.z = 0;

//             return R;
//         }        
//     }    
//     else
//     {
//         if (T == wrwr)
//         {
//           T = 0;
//         }
//     }
    
//     t = T; //local time in full waggle period
    
//     danceState = 0;
    
//     if (isTimeIn(t, 0, w))
//         pathID = BR_DANCESIM_SUBPATH_WAGGLE_LEFT;//waggle left
//     else if (isTimeIn(t, w, wr))
//     {
//         t -= w; //local time in path
//         pathID = BR_DANCESIM_SUBPATH_RETURN_LEFT;//return left
//     }
//     else if (isTimeIn(t, wr, wrw))
//     {
//         t -= wr; //local time in path
//         pathID = BR_DANCESIM_SUBPATH_WAGGLE_RIGHT;//waggle right
//     }
//     else if (isTimeIn(t, wrw, wrwr))
//     {
//         t -= wrw; //local time in path
//         pathID = BR_DANCESIM_SUBPATH_RETURN_RIGHT;//return right
//     }
//     //----

//     if (pathID < BR_DANCESIM_SUBPATH_RETURN_LEFT) // WAGGLE RUN
//     {

//         float t_prcnt = ((t+.5)/params.waggle_duration);
//         float ramp = 1;
//         int sign = 1;
        
//         if ( (t == 0) && (pathID == BR_DANCESIM_SUBPATH_WAGGLE_RIGHT) )
//         {
//             TIM_Cmd(TIM6, DISABLE);         // Dancetimer stoppen   
//             motorXYZDrive(0,0,0);           // Motoren anhalten 
            
//             // Waggleorientierung einstellen           
//             vz = -1.0;
//             // kürzeste Richtung bestimmen
//             // aktueller Z-Encoderwinkel im Bogenmaß
//             w_old = ( (360/4096.0) * Enc_GetCurrentRotaryStep() ) * (PI/180.0);  
//             diff = (params.angle + params.divergence/2.0) - w_old;
//             if ( ((diff > 0) && (abs(diff) < PI)) || ((diff < 0) && (abs(diff) > PI)) )
//               vz = 1.0;

//             while(1)
//             {
//               // aktueller Z-Encoderwinkel im Bogenmaß
//               w_old = ( (360/4096.0) * Enc_GetCurrentRotaryStep() ) * (PI/180.0);
              
//               diff = (params.angle + params.divergence/2.0) - w_old;
//               // bei 0.5 Grad (0.009 Rad) Toleranz zwischen angestrebten und aktuellem Winkel -> Abbruch
//               if ( fabs(diff) < 0.009 ) 
//                 break;

//               // wenn Winkel fast erreicht (3 Grad (0,05 RAD) unterschied) dann langsamer drehen
//               if (fabs(diff) < 0.05)
//               {
//                 if (vz < 0)
//                   vz = -MIN_SET_ORIENTATION_SPEED;
//                 else
//                   vz = MIN_SET_ORIENTATION_SPEED;
//               }
//               else
//               {
//                 if (vz < 0)
//                   vz = -MAX_SET_ORIENTATION_SPEED;
//                 else
//                   vz = MAX_SET_ORIENTATION_SPEED;
//               }
//              // Richtung einstellen
                    
//               w_cur = w_old + (vz*(PI/180) / 1000);
        
//               p_old.x = params.exc_length*(cosf(w_old)); 
//               p_old.y = params.exc_length*(sinf(w_old));
        
//               p_cur.x = params.exc_length*(cosf(w_cur)); 
//               p_cur.y = params.exc_length*(sinf(w_cur));
           
//               vx = -1000*(p_old.x - p_cur.x);
//               vy = -1000*(p_old.y - p_cur.y);
      
//               motorXYZDrive( (float) vx, (float) vy, (float) vz);
//               cpu_Delay_ms(1);
//             }
//             motorXYZDrive( (float) 0, (float) 0, (float) 0);

//             Pcur.z = params.angle + params.divergence/2.0;
//             TIM_Cmd(TIM6, ENABLE);         // Dancetimer starten   
//         }
//         else if ( (t == 0) && (pathID == BR_DANCESIM_SUBPATH_WAGGLE_LEFT) )
//         {
//             TIM_Cmd(TIM6, DISABLE);         // Dancetimer stoppen   
//             motorXYZDrive(0,0,0);           // Motoren anhalten 

//             // Waggleorientierung einstellen           
//             // kürzeste Richtung bestimmen
//             // aktueller Z-Encoderwinkel im Bogenmaß
//             w_old = ( (360/4096.0) * Enc_GetCurrentRotaryStep() ) * (PI/180.0);  
            
//             angle = (params.angle - params.divergence/2.0);
//             if (angle < 0.0)
//               angle += (2*PI);
//             if (angle > (2*PI) )
//               angle -= (2*PI);
            
//             vz = -1.0;
//             diff = angle - w_old;
//             if ( ((diff > 0) && (abs(diff) < PI)) || ((diff < 0) && (abs(diff) > PI)) )
//               vz = 1.0;

//             while(1)
//             {
//               // aktueller Z-Encoderwinkel im Bogenmaß
//               w_old = ( (360/4096.0) * Enc_GetCurrentRotaryStep() ) * (PI/180.0);
              
//               diff = angle - w_old;
//               // bei 0.5 Grad (0.009 Rad) Toleranz zwischen angestrebten und aktuellem Winkel -> Abbruch
//               if ( fabs(diff) < 0.009 ) 
//                 break;

//               // wenn Winkel fast erreicht (3 Grad (0,05 RAD) unterschied) dann langsamer drehen
//               if (fabs(diff) < 0.05)
//               {
//                 if (vz < 0)
//                   vz = -MIN_SET_ORIENTATION_SPEED;
//                 else
//                   vz = MIN_SET_ORIENTATION_SPEED;
//               }
//               else
//               {
//                 if (vz < 0)
//                   vz = -MAX_SET_ORIENTATION_SPEED;
//                 else
//                   vz = MAX_SET_ORIENTATION_SPEED;
//               }
//               // Richtung einstellen
                    
//               w_cur = w_old + (vz*(PI/180) / 1000);
        
//               p_old.x = params.exc_length*(cosf(w_old)); 
//               p_old.y = params.exc_length*(sinf(w_old));
        
//               p_cur.x = params.exc_length*(cosf(w_cur)); 
//               p_cur.y = params.exc_length*(sinf(w_cur));
           
//               vx = -1000*(p_old.x - p_cur.x);
//               vy = -1000*(p_old.y - p_cur.y);
      
//               motorXYZDrive( (float) vx, (float) vy, (float) vz);
//               cpu_Delay_ms(1);
//             }
//             motorXYZDrive( (float) 0, (float) 0, (float) 0);
//             Pcur.z = params.angle - params.divergence/2.0;
//             TIM_Cmd(TIM6, ENABLE);         // Dancetimer starten   
//         }

//         //damp!		
//         if (t < number_of_sample_points_for_one_period)
//             ramp = (float) t/number_of_sample_points_for_one_period;
//         else 
//         if (t > (number_of_sample_points_for_one_period * (number_of_waggle_periods - 1)))
//             ramp = (float) (params.waggle_duration - t)/number_of_sample_points_for_one_period;	

        
//         if ( (sysState & SYS_STATE_WINGS_ENABLED) && ( (state & STATE_EFILED_ON) == 0) )
//         {
//               // Flügel-Aufname starten
//               TIM7->CR1 &= 0x03FE;  // Timer7 deaktivieren
//               dac_ResetDMA();       // DAC DMA reseten
//               TIM7->CR1 |= 0x0001;  // Timer7 aktivieren
//               state |= STATE_EFILED_ON;
//         }

//         start_wings = (t >= wingsParam.pulseOffset);
//         // Wenn Flügel-Signal aktiviert und Offset erreicht
//         if ( (sysState & SYS_STATE_WINGS_ENABLED) && (start_wings) )
//         {
//             if ( (t_wings % wingsParam.pulseFreq) == 0 )
//             {
//               // Flügel parametrisiert starten
//               CPU_ENABLE_WING_MOTOR;
//               sig_startWings(wingsParam.carrierFreq);
//               t_wings = 0;
//             }
//             else if ( (t_wings % wingsParam.pulseDuration) == 0 )
//             {
//                // Flügel parametrisiert stoppen
//               CPU_ENABLE_WING_MOTOR;
//               sig_stopWings(); 
//             }

//             t_wings++;
//         }
        
//         /*
//             Erläuterung:
//             #
//             # cos(t_prcnt * 2 * CV_PI) eine periode auf einen waggle run
//             # cos(waggle_freq * t2pi) z.b. 13 pro waggle run
//             # (params->waggle_duration / 1000) waggle dauer in sec
//         */

//         R.z = params.waggle_amp_a * ramp * cosf( number_of_waggle_periods * t_prcnt * 2 * PI) / (1000 / (params.waggle_freq * 4) / (PI*0.5)) ;

//         if ( pathID == BR_DANCESIM_SUBPATH_WAGGLE_LEFT )
//             sign = -1;

//         R.z = sign * R.z;  
 
//         R.x = params.waggle_speed * cosf(params.angle + sign*params.divergence/2);
//         R.y = params.waggle_speed * sinf(params.angle + sign*params.divergence/2);
//         Pcur.z += R.z;
//         // R.z ist in Rad/ms
//         // umrechnen in Grad/s
//         // -> R.z * (180/PI) * 1000
//         R.z = (R.z * (180/PI)) * 1000;       
//     }
//     else // RETURN RUN
//     {
//         // Flügel-Aufname stoppen
//         TIM7->CR1 &= 0x03FE;  // Timer7 deaktivieren
//         state &= ~STATE_EFILED_ON;
        
//         sig_stopWings(); 
//         danceState |= DANCE_STATE_RETURNRUN;                
    
//         t_prcnt	= (((float)t+.5)/params.return_duration);
//         ramp = 1.0;
//         sign = ( pathID == BR_DANCESIM_SUBPATH_RETURN_LEFT) ? -1 : 1;       

//         if (t < params.return_smoothing_duration)
//           ramp = t / params.return_smoothing_duration;

//         R.z = turn_scale * ramp * sign * PI * return_turn_velocity(t_prcnt) / 180.0;
//         //R.z = turn_scale * sign * PI * return_turn_velocity(t_prcnt) / 180.0;
        
//         v_fwd = turn_scale * ramp * return_fwd_velocity(t_prcnt);
//         v_swd = turn_scale * ramp * sign * return_swd_velocity(t_prcnt);
//         c = cosf(Pcur.z);
//         s = sinf(Pcur.z);
//         c1 = (cosf(R.z + Pcur.z) - cosf(Pcur.z))*params.exc_length;
//         c2 = (sinf(R.z + Pcur.z) - sinf(Pcur.z))*params.exc_length;
  
//         //rotate
//         R.x = c*v_fwd - s*v_swd + c1;
//         R.y = s*v_fwd + c*v_swd + c2;
        
//         Pcur.z += R.z;

//         // von mm/ms in mm/s
//         R.x *= 1000;
//         R.y *= 1000;
//         // R.z ist in Rad/ms
//         // umrechnen in Grad/s
//         // -> R.z * (180/PI) * 1000
//         R.z = (R.z * (180/PI)) * 1000;
//     }        
//     T++;          
//     return R;
//}


