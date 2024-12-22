#include <Arduino.h>
#include <serv_arm.h>
#include <ultra.h>
#include "motor_pid.h"
#include "chassis.h"

/**********************************************************/

/**************************超声波**************************/
/*  -----------               | y轴           ----------- */
/*  |motor1(6)|               |               |motor2(7)| */
/*  -----------               |               ----------- */
/*                            |                           */
/*                            |                           */
/* -----------------------------------------------------> */
/*                            |                     x轴   */
/*                            |                           */
/*  -----------               |               ----------- */
/*  |motor3(8)|               |               |motor4(9)| */
/*  -----------               |               ----------- */
/*                            |                           */
/************************ User Code *************************/

/**********************************************************/
PIDInstance pid;

void setup()
{
  servo_create(); //测试初始化舵机，抓取到物块
  delay(200);
  // Arm.clawClose();
  // delay(1000);
  // Arm.clawOpen();
  // 初始化 PID 实例
  PID_config config = {1.0, 0.1, 0.01, 1000, 500}; // 示例配置
  PID_init(&pid, &config);
  chassis_Init();

}

void cleanup()
{
  servo_clean();
}


void loop() 
{

  float vx = 100.0; // x 方向速度
  float vy = 50.0;  // y 方向速度
  float vz = 10.0;  // 角速度

  // 驱动电机
  Drive_Motor(vx, vy, vz);

  // 发送电机速度指令
  chassis_transmit(chassis_target.chassis_left_down_speed, chassis_target.chassis_left_up_speed, chassis_target.chassis_right_up_speed, chassis_target.chassis_right_down_speed);

  // 延时一段时间
  delay(1000);
  // 你的循环代码
}