#ifndef __TYPES_H__
#define __TYPES_H__
#include "commands.h"

typedef struct V3Df_
{
  float   x;
  float   z;
  float   phi;
} V3Df;

typedef struct V2Df_
{
  float   x;
  float   z;
} V2Df;

typedef struct SROI_
{
  u8 x;
  u8 z;
  u8 w;
  u8 h;
  u16 pixelSize;
  u16 pixelIdx[400];  // SROI max. 20x20
}SROI;

typedef struct ROI_
{
  u8 x;
  u8 z;
  u8 w;
  u8 h;
}ROI;

typedef struct WingsParam_t {
    u16 carrierFreq;
    u8 pulseDuration;
    u16 pulseFreq;
    u8 pulseOffset;   
} WingsParam;

typedef struct TrophParam_t {
    u8 duration;
    u8 frequency;     
} TrophParam;

#define true 1
#define false 0

#endif
