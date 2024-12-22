#include "motor_pid.h"

void  LimitMax(int32_t *input,int32_t max)   
    {                          
        if (*input > max)       
        {                      
            *input = max;       
        }                      
        else if (*input < - max) 
        {                      
            *input = - max;     
        }                      
    };

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
	
	void PID_init(PIDInstance *pid, PID_config *config)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->max_out = config->max_out;
    pid->max_iout = config->max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
int32_t PID_calc(PIDInstance *pid, int32_t ref, int32_t set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    // 满足最初的公式pid_out = Kp * E + Ki * (E累和) + Kd * (E1 - E0)
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];      
    LimitMax(&(pid->Iout), pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(&pid->out,pid->max_out);

    return (int16_t)pid->out;
}


/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(PIDInstance *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/**
  * @brief          6020角度pid过零处理
  * @param          tar为目标值，cur为现在的返还值
  * @retval         none
  */
int32_t Handle_Angle8191_PID_Over_Zero(int32_t *tar, uint16_t *cur)
{
	if(*tar - *cur > 4096)    //4096 ：半圈机械角度
	{
		*cur += 8192;        //8191,8192无所谓了，四舍五入
	}
	else if(*tar - *cur < -4096)
	{
		*cur = *cur - 8192;
	}
	return *cur;
}
