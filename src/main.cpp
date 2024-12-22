#include <Arduino.h>
#include <serv_arm.h>
#include <ultra.h>
#include "motor_pid.h"
#include "chassis.h"
/******************* variable && constants  *******************************/

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
  PID_config config = {
	//PID 三参数
  .Kp = 15.0f,
	.Ki = 4.0f,
	.Kd = 0.1f,

	.max_out = 450,  //最大输出
	.max_iout = 100  //最大积分输出
  }; // 示例配置
  PID_init(&pid, &config);
  chassis_Init();
}

void cleanup()
{
  servo_clean();
}


void loop() 
{
  double vx = 200.0, vy = 0.0, vz = 0.0;
  vz = translate()*100;
  vy = processAngle();
  Drive_Motor(vx ,vy ,vz);
  // 驱动电机

  // 发送电机速度指令
  chassis_transmit(chassis_target.clds, chassis_target.clus, chassis_target.crus, chassis_target.crds);

  // 你的循环代码
}