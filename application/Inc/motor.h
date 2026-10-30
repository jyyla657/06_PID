#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"

typedef struct
{
  CAN_HandleTypeDef *can_handle;
  CAN_TxHeaderTypeDef txconf;
  uint32_t tx_id;
  uint32_t tx_mailbox;
  uint8_t tx_buff[8];
  uint8_t rx_buff[8];
  uint32_t rx_id;
  uint8_t rx_len;
} CANInstance;

typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t real_current;
  uint8_t temperature;

  int16_t real_current_handle;
  float speed_aps;
  uint16_t last_ecd;
  float angle_single_round;
  float total_angle;
  int32_t total_round;
} Motor_Measure_s;

extern CANInstance motor1;
extern Motor_Measure_s motor1_measure;

void MotorDataRead(void);
void Motor_Init(void);
void Motor_SendCurrent(int16_t current);

#endif /* _MOTOR_H */