#include "motor.h"
#include "can.h"
#include "memory.h"
#include "stdlib.h"

CANInstance motor1;
Motor_Measure_s motor1_measure;

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint8_t fifox)
{
  static CAN_RxHeaderTypeDef rxconf;
  uint8_t can_rx_buff[8];
  while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox) > 0)
  {
    HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);
    if (_hcan == motor1.can_handle && rxconf.StdId == motor1.rx_id)
    {
      motor1.rx_len = rxconf.DLC;
      memcpy(motor1.rx_buff, can_rx_buff, rxconf.DLC);
      motor1_measure.last_ecd = motor1_measure.ecd;
      MotorDataRead();
      motor1_measure.angle_single_round = (float)motor1_measure.ecd / 8192.0f * 360.0f;
      motor1_measure.speed_aps = (1.0f - 0.85f) *motor1_measure.speed_aps + 0.85f * (360.0f / 60.0f) * (float)motor1_measure.speed_rpm;
      motor1_measure.real_current_handle = (1.0f - 0.9f) * motor1_measure.real_current_handle + 0.9f * (float)motor1_measure.real_current;
      if (motor1_measure.ecd - motor1_measure.last_ecd > 4096)
        motor1_measure.total_round--;
      else if (motor1_measure.ecd - motor1_measure.last_ecd < -4096)
        motor1_measure.total_round++;
      motor1_measure.total_angle = motor1_measure.total_round * 360.0f + motor1_measure.angle_single_round;
      return;
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}

void MotorDataRead(void)
{
  motor1_measure.ecd = (motor1.rx_buff[0] << 8) | motor1.rx_buff[1];
  motor1_measure.speed_rpm = (motor1.rx_buff[2] << 8) | motor1.rx_buff[3];
  motor1_measure.real_current = (motor1.rx_buff[4] << 8) | motor1.rx_buff[5];
  motor1_measure.temperature = motor1.rx_buff[6];
}

void Motor_Init(void)
{
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  motor1.can_handle = &hcan1;
  motor1.txconf.StdId = 0x200;
  motor1.txconf.IDE = CAN_ID_STD;
  motor1.txconf.RTR = CAN_RTR_DATA;
  motor1.txconf.DLC = 8;
  motor1.tx_id = 0x200;
  motor1.rx_id = 0x201;

  CAN_FilterTypeDef can_filter_conf;
  static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;
  can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter_conf.FilterFIFOAssignment = (motor1.tx_id & 1) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
  can_filter_conf.SlaveStartFilterBank = 14;
  can_filter_conf.FilterIdLow = motor1.rx_id << 5;
  can_filter_conf.FilterBank = (motor1.can_handle == &hcan1) ? (can1_filter_idx++) : (can2_filter_idx++);
  can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;
  HAL_CAN_ConfigFilter(motor1.can_handle, &can_filter_conf);
}

void Motor_SendCurrent(int16_t current)
{
  if (HAL_CAN_GetTxMailboxesFreeLevel(motor1.can_handle) > 0)
  {
    motor1.tx_buff[0] = (uint8_t)(current >> 8);
    motor1.tx_buff[1] = (uint8_t)(current & 0x00ff);
    motor1.tx_buff[2] = 0;
    motor1.tx_buff[3] = 0;
    motor1.tx_buff[4] = 0;
    motor1.tx_buff[5] = 0;
    motor1.tx_buff[6] = 0;
    motor1.tx_buff[7] = 0;
    HAL_CAN_AddTxMessage(motor1.can_handle, &motor1.txconf, motor1.tx_buff, &motor1.tx_mailbox);
  }
}