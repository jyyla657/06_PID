#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include "main.h"

typedef struct
{
  int16_t rocker_I_;
  int16_t rocker_I1;
  int16_t rocker_r_;
  int16_t rocker_r1;

  int16_t SWA;
  int16_t SWB;
  int16_t SWC;
  int16_t SWD;

  int16_t VRA;
  int16_t VRB;

  uint8_t error_protect;
  uint8_t byte_lose;

} RC_ctrl_t;

extern RC_ctrl_t rc_data;

void RemoteControl_Init(void);

#endif /* _REMOTE_CONTROL_H */