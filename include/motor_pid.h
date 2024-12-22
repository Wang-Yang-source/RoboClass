#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "math.h"
#include "stdlib.h"

typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    int32_t max_out;  //最大输出
    int32_t max_iout; //最大积分输出

    int32_t set;      //目标值
    int32_t fdb;      //返还值

    int32_t out;
    int32_t Pout;
    int32_t Iout;
    int32_t Dout;
    int32_t Dbuf[3];  //微分项 0最新 1上一次 2上上次
    int32_t error[3]; //误差项 0最新 1上一次 2上上次

} PIDInstance;

typedef struct
{
	//PID 三参数
	float Kp;
	float Ki;
	float Kd;

	int32_t max_out;  //最大输出
	int32_t max_iout; //最大积分输出
	
} PID_config;


/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
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
extern void PID_init(PIDInstance *pid, PID_config *config);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
int32_t PID_calc(PIDInstance *pid, int32_t ref, int32_t set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(PIDInstance *pid); 
extern float translate();
extern float processAngle();
/**
  * @brief          6020角度pid过零处理
  * @param          tar为目标值，cur为现在的返还值
  * @retval         none
  */
extern int32_t Handle_Angle8191_PID_Over_Zero(int32_t *tar, uint16_t *cur);



#endif
