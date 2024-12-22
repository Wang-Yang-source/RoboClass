#include "chassis.h"

char cmd_return_tmp[64];               // 电机
Chassis_target chassis_target;
PIDInstance PID_chassis_vz;
PIDInstance PID_chassis_vy;

PID_config pid_chassis_vy_config =  {
	//PID 三参数
  .Kp = 15.0f,
	.Ki = 4.0f,
	.Kd = 0.1f,

	.max_out = 450,  //最大输出
	.max_iout = 100  //最大积分输出
};

PID_config pid_chassis_vz_config =  {
	//PID 三参数
  .Kp = 150.0f,
	.Ki = 10.0f,
	.Kd = 0.1f,

	.max_out = 1500,  //最大输出
	.max_iout = 400  //最大积分输出
};


void chassis_Init(void)
{
  // 初始化串口通信
  Serial.begin(115200);
  PID_init(&PID_chassis_vz,&pid_chassis_vz_config);
  PID_init(&PID_chassis_vy,&pid_chassis_vy_config);
}

void Drive_Motor(float vx,float vy,float vz)
{
	chassis_target.chassis_left_down_speed = (vx+vy-vz*(Wheel_spacing+Wheel_axlespacing));
	chassis_target.chassis_left_up_speed = (vx-vy-vz*(Wheel_spacing+Wheel_axlespacing));
	chassis_target.chassis_right_up_speed = (vx+vy+vz*(Wheel_spacing+Wheel_axlespacing));
	chassis_target.chassis_right_down_speed = (vx-vy+vz*(Wheel_spacing+Wheel_axlespacing));
};

void chassis_transmit(float chassis_left_down_speed,float chassis_left_up_speed,float chassis_right_up_speed,float chassis_right_down_speed)
{
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, (int)(1500 + (round)(chassis_left_up_speed)), 0);  // 电机1前进
  Serial.println(cmd_return_tmp); 

  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, (int)(1500 - (round)(chassis_right_up_speed)), 0);  // 电机2前进 
  Serial.println(cmd_return_tmp);

  int flag = 1;
   if(chassis_left_down_speed >= 0)
  {
    flag = 1;
  }
    if(chassis_left_down_speed < 0)
  {
    flag = -1;
  }
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, (int)(1500 + (round)(  flag*(abs(chassis_left_down_speed)+50) )  ), 0);  // 电机3前进 
  Serial.println(cmd_return_tmp);

  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, (int)(1500 - (round)(chassis_right_down_speed)), 0);  // 电机4前进
  Serial.println(cmd_return_tmp);
}

void Chassis_xunji_control(void)
{
  // 声明角度和距离变量
  float angle = 0.0; // 初始化为默认值或根据需要计算
  float distance = 0.0; // 初始化为默认值或根据需要计算

  // 使用PID控制器计算目标角速度和目标速度
  chassis_target.vz = PID_calc(&PID_chassis_vz, angle, 90);
  chassis_target.vy = PID_calc(&PID_chassis_vy, distance, 0);
  
  // 驱动电机，设置速度
  // Drive_Motor(500, chassis_target.vy, chassis_target.vz);
  Drive_Motor(500, chassis_target.vy, 0);
  
  // 发送电机速度指令
  chassis_transmit(chassis_target.chassis_left_down_speed, chassis_target.chassis_left_up_speed, chassis_target.chassis_right_up_speed, chassis_target.chassis_right_down_speed);
}