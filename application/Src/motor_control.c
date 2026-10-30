#include "remote_control.h"
#include "motor.h"
#include "pid.h"
#include "main.h"
#include "tim.h"
#include "motor_control.h"

PIDInstance speed_pid;
PIDInstance angle_pid;

float angle_ref = 0.0f;
float angle_measure = 0.0f;
float angle_output = 0.0f;
float speed_ref = 0.0f;
float speed_measure = 0.0f;
float speed_output = 0.0f;
float output = 0.0f;

uint32_t time_cnt = 0;

void MotorControl_Init(void)
{
  PID_Init_Config_s speed_pid_config = {
      .Kp = 1.4f, //1.4
      .Ki = 10.0f, //10.0
      .Kd = 0.0f, //0.0
      .MaxOut = 8000.0f,
      .IntegralLimit = 3000.0f
  };
  PID_Init_Config_s angle_pid_config = {
      .Kp = 90.0f, //90.0
      .Ki = 0.0f, //0.0
      .Kd = 2.0f, //2.0
      .MaxOut = 3000.0f,
      .IntegralLimit = 0.0f
  };

  PIDInit(&speed_pid, &speed_pid_config, 0.001f);
  PIDInit(&angle_pid, &angle_pid_config, 0.001f);

  HAL_TIM_Base_Start_IT(&htim6);
}

void MotorControlTask(void)
{
  if (rc_data.SWA >= 0)
  {
    if (rc_data.SWB >= 0)
    {
        if (rc_data.SWC >= 0)
        {
            speed_ref = (float)rc_data.rocker_r1 / 784.0f * 3000.0f;
        }
        else if (rc_data.SWC < 0)
        {
            time_cnt++;
            if (time_cnt >= 5000)
            {
                time_cnt = 0;
                if (speed_ref >= 0.0f) speed_ref = -3000.0f;
                else if (speed_ref < 0.0f) speed_ref = 3000.0f;
            }
        }
        speed_measure = motor1_measure.speed_aps;
        speed_output = PIDCalculate(&speed_pid, speed_measure, speed_ref);
    }
    else if (rc_data.SWB < 0)
    {
        if (rc_data.SWC >= 0)
        {
            angle_ref += (float)rc_data.rocker_r1 / 784.0f * 360.0f * 0.001f;
        }
        else if (rc_data.SWC < 0)
        {
            time_cnt++;
            if (time_cnt >= 5000)
            {
                time_cnt = 0;
                if (angle_ref >= 0.0f) angle_ref = -180.0f;
                else if (angle_ref < 0.0f) angle_ref = 180.0f;
            }
        }
        angle_measure = motor1_measure.total_angle;
        angle_output = PIDCalculate(&angle_pid, angle_measure, angle_ref);

        speed_ref = angle_output;
        speed_measure = motor1_measure.speed_aps;
        speed_output = PIDCalculate(&speed_pid, speed_measure, speed_ref);
    }
    output = speed_output;
    Motor_SendCurrent((int16_t)output);
  }
  else
  {
    Motor_SendCurrent(0);
    PIDClear(&speed_pid);
    PIDClear(&angle_pid);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    MotorControlTask();
  }
}