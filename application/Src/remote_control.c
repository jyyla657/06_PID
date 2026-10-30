#include "remote_control.h"
#include "usart.h"

RC_ctrl_t rc_data;
uint8_t buf[256];
int16_t CH[16];
uint8_t start_byte;
uint8_t flag_byte;
uint8_t stop_byte;

void RemoteControl_Init(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart->Instance == USART3)
  {
    start_byte = buf[0];

    CH[0] = ((((int16_t)buf[1] >> 0) | ((int16_t)buf[2] << 8)) & 0x07FF) - 1024;
    CH[1] = ((((int16_t)buf[2] >> 3) | ((int16_t)buf[3] << 5)) & 0x07FF) - 1024;
    CH[2] = ((((int16_t)buf[3] >> 6) | ((int16_t)buf[4] << 2) | ((int16_t)buf[5] << 10)) & 0x07FF) - 1024;
    CH[3] = ((((int16_t)buf[5] >> 1) | ((int16_t)buf[6] << 7)) & 0x07FF) - 1024;
    CH[4] = ((((int16_t)buf[6] >> 4) | ((int16_t)buf[7] << 4)) & 0x07FF) - 1024;
    CH[5] = ((((int16_t)buf[7] >> 7) | ((int16_t)buf[8] << 1) | ((int16_t)buf[9] << 9)) & 0x07FF) - 1024;
    CH[6] = ((((int16_t)buf[9] >> 2) | ((int16_t)buf[10] << 6)) & 0x07FF) - 1024;
    CH[7] = ((((int16_t)buf[10] >> 5) | ((int16_t)buf[11] << 3)) & 0x07FF) - 1024;
    CH[8] = ((((int16_t)buf[12] >> 0) | ((int16_t)buf[13] << 8)) & 0x07FF) - 1024;
    CH[9] = ((((int16_t)buf[13] >> 3) | ((int16_t)buf[14] << 5)) & 0x07FF) - 1024;

    flag_byte = buf[23];
    stop_byte = buf[24];
    
    rc_data.rocker_r_ = CH[0];
    rc_data.rocker_r1 = CH[1];
    rc_data.rocker_I_ = CH[2];
    rc_data.rocker_I1 = CH[3];
    rc_data.SWA       = CH[6];
    rc_data.SWB       = CH[7];
    rc_data.SWC       = CH[8];
    rc_data.SWD       = CH[9];
    rc_data.VRA       = CH[4];
    rc_data.VRB       = CH[5];
    rc_data.error_protect = (flag_byte >> 3) & 0x01;
    rc_data.byte_lose     = (flag_byte >> 2) & 0x01;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}