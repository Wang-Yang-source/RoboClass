#ifndef chassis_H
#define chassis_H

#include "motor_pid.h"
#include <Arduino.h>
#define Wheel_spacing     0.11f   // 22cm 半宽
#define Wheel_axlespacing 0.09f   // 18cm 半长

struct Chassis_target
{
  float vx;
  float vy;
  float vz;
  float chassis_left_down_speed;
  float chassis_left_up_speed;
  float chassis_right_up_speed;
  float chassis_right_down_speed;
};

extern PID_config pid_chassis_vz_config;
extern PID_config pid_chassis_vy_config;
extern PIDInstance PID_chassis_vz;
extern PIDInstance PID_chassis_vy;
extern Chassis_target chassis_target;
extern char cmd_return_tmp[64];        // 电机命令返回临时存储
extern void chassis_Init(void);        // 底盘初始化
extern void Drive_Motor(float vx, float vy, float vz);  // 驱动电机
extern void chassis_transmit(float chassis_left_down_speed, float chassis_left_up_speed, float chassis_right_up_speed, float chassis_right_down_speed);  // 底盘速度传输
extern void Chassis_xunji_control(void);  // 底盘循迹控制

#endif

