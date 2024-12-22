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

// 驱动电机函数
void Drive_Motor(float vx, float vy, float vz)
{
  // 计算每个电机的速度
  chassis_target.clds = (vx + vy - vz * (Wheel_spacing + Wheel_axlespacing));
  chassis_target.clus = (vx - vy - vz * (Wheel_spacing + Wheel_axlespacing));
  chassis_target.crus = (vx + vy + vz * (Wheel_spacing + Wheel_axlespacing));
  chassis_target.crds = (vx - vy + vz * (Wheel_spacing + Wheel_axlespacing));
}

void chassis_transmit(float clds,float clus,float crus,float crds)
{
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, (int)(1500 + round(clus)), 0);  // 电机6前进
  Serial.println(cmd_return_tmp); 

  int flag = 1;
   if(clds < 0)
  {
    flag = 1;
  }
    if(clds > 0)
  {
    flag = -1;
  }

  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, (int)(1500 + (round)(flag*(abs(clds)-3))), 0);  // 电机7前进 
  Serial.println(cmd_return_tmp);

  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, (int)(1500 + round(clds)), 0);  // 电机8前进 
  Serial.println(cmd_return_tmp);

  int flag2 = 1;
   if(crds < 0)
  {
    flag2 = -1;
  }
    if(crds > 0)
  {
    flag2 = 1;
  }
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, (int)(1500 - (round)(flag2*(abs(crds)-0))), 0);  // 电机9前进
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
  Drive_Motor(300, chassis_target.vy, 0);
  
  // 发送电机速度指令
  chassis_transmit(chassis_target.clds, chassis_target.clus, chassis_target.crus, chassis_target.crds);
}