/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/
/**
  ******************************************************************************
  * @file			pid.c
  * @version	V1.0.0
  * @date			2016年11月11日17:21:36
  * @brief   	对于PID， 反馈/测量习惯性叫get/measure/real/fdb,期望输入一般叫set/target/ref
  *******************************************************************************/
  
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "mytype.h"
#include <math.h>
//#include "cmsis_os.h"

#define ABS(x) ((x>0) ? (x) : (-x)) //定义宏，用于计算绝对值

void abs_limit(float *a, float ABS_MAX) //将指针a指向的值限制在 -ABS_MAX 到 ABS_MAX 的范围内
{
    if(*a >  ABS_MAX) *a =  ABS_MAX;
    if(*a < -ABS_MAX) *a = -ABS_MAX;
}

/*-----------------------------参数初始化---------------------------------*/
static void pid_param_init(
														pid_t 	*pid,
														uint32_t mode,
														uint32_t maxout,
														uint32_t intergral_limit,
														float 	 kp,
														float		 ki,
														float		 kd)
{
    pid -> IntegralLimit = intergral_limit;
    pid -> MaxOutput 		 = maxout;
    pid -> pid_mode 		 = mode;
    pid -> p 						 = kp;
    pid -> i 						 = ki;
    pid -> d 						 = kd;
}

/*--------------------------中途更改参数设定(调试)----------------------------------*/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid -> p = kp;
    pid -> i = ki;
    pid -> d = kd;
}

/**
	*@bref. calculate delta PID and position PID
  *@param[in] set：target
  *@param[in] real measure
	*/
float pid_calc(pid_t *pid, float get, float set)
{
    pid -> get[NOW] = get;
    pid -> set[NOW] = set;
    pid -> err[NOW] = set - get;	//set - measure
    if (pid -> max_err 	!= 0 && ABS(pid -> err[NOW]) > pid -> max_err)
		return 0;
	  if (pid -> deadband != 0 && ABS(pid -> err[NOW]) < pid -> deadband)
		return 0;
    
    if(pid -> pid_mode == POSITION_PID) //位置式p
    {
        pid -> pout  = pid -> p *  pid -> err[NOW];
        pid -> iout += pid -> i *  pid -> err[NOW];
        abs_limit(&(pid -> iout), pid -> IntegralLimit);			
        pid -> dout  = pid -> d * (pid -> err[NOW] - pid -> err[LAST]);

        pid -> pos_out = pid -> pout + pid -> iout + pid -> dout;
        abs_limit(&(pid -> pos_out), pid -> MaxOutput);
			  if(pid -> pos_out >= pid -> MaxOutput || pid -> pos_out <= - (pid -> MaxOutput))
        {
					pid -> iout -= pid -> i * pid -> err[NOW]; // 停止积分累积
        }
        pid -> last_pos_out = pid -> pos_out;	//update last time
    }
    else if(pid -> pid_mode == DELTA_PID) //增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid -> err[LLAST] = pid -> err[LAST];
    pid -> err[LAST] 	= pid -> err[NOW];
    pid -> get[LLAST] = pid -> get[LAST];
    pid -> get[LAST] 	= pid -> get[NOW];
    pid -> set[LLAST] = pid -> set[LAST];
    pid -> set[LAST] 	= pid -> set[NOW];
    return pid -> pid_mode == POSITION_PID ? pid -> pos_out : pid -> delta_out;
}

/**
    *@bref. special calculate position PID @attention @use @gyro data!!
    *@param[in] set： target
    *@param[in] real	measure
		*/
float pid_sp_calc(pid_t *pid, float get, float set, float gyro)
{
    pid -> get[NOW] = get;
    pid -> set[NOW] = set;
    pid -> err[NOW] = set - get;	//set - measure
    
    if(pid->pid_mode == POSITION_PID) //位置式PID
    {
        pid->pout = pid->p * pid->err[NOW];
				if(fabs(pid -> i) >= 0.001f) pid -> iout += pid -> i * pid -> err[NOW];
				else pid -> iout = 0;
        pid -> dout = - pid -> d * gyro / 100.0f;	
        abs_limit(&(pid -> iout), pid -> IntegralLimit);
        pid -> pos_out = pid -> pout + pid -> iout + pid -> dout;
        abs_limit(&(pid -> pos_out), pid -> MaxOutput);
        pid -> last_pos_out = pid -> pos_out;	//update last time
    }
    else if(pid -> pid_mode == DELTA_PID) //增量式PID
    {
//        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//        pid->iout = pid->i * pid->err[NOW];
//        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
//        
//        abs_limit(&(pid->iout), pid->IntegralLimit);
//        pid->delta_u = pid->pout + pid->iout + pid->dout;
//        pid->delta_out = pid->last_delta_out + pid->delta_u;
//        abs_limit(&(pid->delta_out), pid->MaxOutput);
//        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid -> err[LLAST] = pid -> err[LAST];
    pid -> err[LAST]  = pid -> err[NOW];
    pid -> get[LLAST] = pid -> get[LAST];
    pid -> get[LAST]  = pid -> get[NOW];
    pid -> set[LLAST] = pid -> set[LAST];
    pid -> set[LAST]  = pid -> set[NOW];
    return pid -> pid_mode == POSITION_PID ? pid -> pos_out : pid -> delta_out;
		
}

/*--------------------------pid总体初始化---------------------------------*/

void PID_struct_init(	pid_t 	*pid,
											uint32_t mode,
											uint32_t maxout,
											uint32_t intergral_limit,
											float 	 kp,
											float 	 ki,
											float 	 kd)
{
    /*init function pointer*/
    pid -> f_param_init = pid_param_init;
    pid -> f_pid_reset 	= pid_reset;
//		pid->f_cal_pid = pid_calc;	
//		pid->f_cal_sp_pid = pid_sp_calc;	//addition
		
    /*init pid param */
    pid -> f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}

pid_t pid_pos;
pid_t pid_spd[8] = {0}; //原本是4，现在改成8试试2020.12.14

void pid_test_init()
{
	//为了解决上位机调参的时候第一次赋值的时候清零其他参数， 应该提前把参数表填充一下！
}



/*-------------------------------- 温海写 --------------------------------*/

float WH_yaw_speed_pid(pid_t* pid, float get, float set,float deadband) //目前未使用
{
	float yaw_pid_p = 10.0f, yaw_pid_i = 0.0f, yaw_pid_d = 0.0f, yaw_IntegralLimit = 1900, yaw_MaxOutput=1900;
  pid -> get[NOW] = get;
  pid -> set[NOW] = set;
  pid -> err[NOW] = set - get;	//set - measure
  if (pid -> max_err != 0 && ABS(pid -> err[NOW]) >  pid -> max_err)
	return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
	return 0;
	
  pid->WH_speed_pout = yaw_pid_p * pid->err[NOW];
  pid->WH_speed_iout += yaw_pid_i * pid->err[NOW];
  pid->WH_speed_dout = yaw_pid_d * (pid->err[NOW] - pid->err[LAST] );
  abs_limit(&(pid->WH_speed_iout), yaw_IntegralLimit);
  pid->WH_yaw_speed_out = pid->WH_speed_pout + pid->WH_speed_iout + pid->WH_speed_dout;
  abs_limit(&(pid->WH_yaw_speed_out), yaw_MaxOutput);
  pid->last_pos_out = pid->WH_yaw_speed_out; //update last time 

  pid->err[LLAST] = pid->err[LAST];
  pid->err[LAST] = pid->err[NOW];
  pid->get[LLAST] = pid->get[LAST];
  pid->get[LAST] = pid->get[NOW];
  pid->set[LLAST] = pid->set[LAST];
  pid->set[LAST] = pid->set[NOW];
  return pid->WH_yaw_speed_out;

}

double WH_yaw_angle_pid1(pid_t* pid, float get, float set, float deadband) //跟随模式下，底盘的pid计算
{
		float yaw_pid_p = 65.0f, yaw_pid_i = 0.0f, yaw_pid_d = 0.0f, yaw_IntegralLimit = 2000, yaw_MaxOutput=2000;

    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if(pid -> max_err  != 0 && ABS(pid -> err[NOW]) > pid -> max_err ) return 0;
		if(pid -> deadband != 0 && ABS(pid -> err[NOW]) < pid -> deadband) return 0;
		
    pid -> WH_angle1_pout  = yaw_pid_p *  pid -> err[NOW];                     //计算比例积分微分输出
    pid -> WH_angle1_iout += yaw_pid_i *  pid -> err[NOW];
    pid -> WH_angle1_dout  = yaw_pid_d * (pid -> err[NOW] - pid -> err[LAST]);
		
    abs_limit(&(pid -> WH_angle1_iout), yaw_IntegralLimit);
    pid -> WH_yaw_angle_out1 = pid -> WH_angle1_pout + pid -> WH_angle1_iout + pid -> WH_angle1_dout;
    abs_limit(&(pid -> WH_yaw_angle_out1), yaw_MaxOutput);
    pid -> last_pos_out = pid -> WH_yaw_angle_out1;	//update last time 

		pid -> err[LLAST] = pid -> err[LAST];
    pid -> err[LAST]  = pid -> err[NOW];
    pid -> get[LLAST] = pid -> get[LAST];
    pid -> get[LAST]  = pid -> get[NOW];
    pid -> set[LLAST] = pid -> set[LAST];
    pid -> set[LAST]  = pid -> set[NOW];
    return pid -> WH_yaw_angle_out1;
	
}

float WH_yaw_angle_pid2(pid_t* pid, float get, float set,float deadband) //云台跟随底盘模式下，云台的pid
{
		float yaw_pid_p = 1200.0f, yaw_pid_i = 0.0f, yaw_pid_d = 50.0f, yaw_IntegralLimit = 8000, yaw_MaxOutput = 8000; //p=1050
    pid -> get[NOW] = get;
    pid -> set[NOW] = set;
    pid -> err[NOW] = set - get;	//set - measure
    if(pid -> max_err  != 0 && ABS(pid -> err[NOW]) > pid -> max_err ) return 0;
		if(pid -> deadband != 0 && ABS(pid -> err[NOW]) < pid -> deadband) return 0;
    
        pid->WH_angle2_pout = yaw_pid_p * pid->err[NOW];
        pid->WH_angle2_iout += yaw_pid_i * pid->err[NOW];
        pid->WH_angle2_dout = yaw_pid_d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->WH_angle2_iout), yaw_IntegralLimit);
        pid->WH_yaw_angle_out2 = pid->WH_angle2_pout + pid->WH_angle2_iout + pid->WH_angle2_dout;
        abs_limit(&(pid->WH_yaw_angle_out2), yaw_MaxOutput);
        pid->last_pos_out = pid->WH_yaw_angle_out2;	//update last time 

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->WH_yaw_angle_out2;

}

double WH_dodge_attacks_pid1(pid_t* pid, float get, float set,float deadband) //小陀螺模式下云台的pid计算
{
	float yaw_pid_p=940.0f,yaw_pid_i=0.9f,yaw_pid_d=2300.0f,yaw_IntegralLimit=13000,yaw_MaxOutput=13000;

    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    

        pid->WH_dodge_pout = yaw_pid_p * pid->err[NOW];
        pid->WH_dodge_iout += yaw_pid_i * pid->err[NOW];
        pid->WH_dodge_dout = yaw_pid_d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->WH_dodge_iout), yaw_IntegralLimit);
        pid->WH_dodge_out = pid->WH_dodge_pout + pid->WH_dodge_iout + pid->WH_dodge_dout;
        abs_limit(&(pid->WH_dodge_out), yaw_MaxOutput);
        pid->last_pos_out = pid->WH_dodge_out;	//update last time 

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->WH_dodge_out;

}

float WH_chassis_current_pid(pid_t* pid, float get, float set,float deadband) //目前未使用
{
	
	float yaw_pid_p=0.0f,yaw_pid_i=0.0f,yaw_pid_d=0.0f,yaw_IntegralLimit=0,yaw_MaxOutput=0;
  pid->get[NOW] = get;
  pid->set[NOW] = set;
  pid->err[NOW] = set - get;	//set - measure
  if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
	return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
	return 0;
  pid->WH_chassis_current_pout = yaw_pid_p * pid->err[NOW];
        pid->WH_chassis_current_iout += yaw_pid_i * pid->err[NOW];
        pid->WH_chassis_current_dout = yaw_pid_d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->WH_chassis_current_iout), yaw_IntegralLimit);
        pid->WH_chassis_current_out = pid->WH_chassis_current_pout + pid->WH_chassis_current_iout + pid->WH_chassis_current_dout;
        abs_limit(&(pid->WH_chassis_current_out), yaw_MaxOutput);
        pid->last_pos_out = pid->WH_chassis_current_out;	//update last time 

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->WH_chassis_current_out;
}

float  WH_dodge_move_pid(pid_t* pid, float get, float set,float deadband)
{
	float yaw_pid_p=1.5f,yaw_pid_i=0.1f,yaw_pid_d=0.0f,yaw_IntegralLimit=15000,yaw_MaxOutput=15000;
	
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	  if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
	  return 0;	
    

        pid -> WH_dodge_move_pout= yaw_pid_p * pid->err[NOW];
        pid -> WH_dodge_move_iout += yaw_pid_i * pid->err[NOW];
        pid ->	WH_dodge_move_dout = yaw_pid_d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->WH_dodge_move_iout), yaw_IntegralLimit);
        pid -> WH_dodge_move_out = pid->WH_dodge_move_pout + pid->WH_dodge_move_iout + pid->WH_dodge_move_dout;
        abs_limit(&(pid->WH_dodge_move_out), yaw_MaxOutput);
        pid -> last_pos_out = pid->WH_dodge_move_out;	//update last time 

    pid -> err[LLAST] = pid -> err[LAST];
    pid -> err[LAST] = pid -> err[NOW];
    pid -> get[LLAST] = pid -> get[LAST];
    pid -> get[LAST] = pid -> get[NOW];
    pid -> set[LLAST] = pid -> set[LAST];
    pid -> set[LAST] = pid -> set[NOW];
    return pid -> WH_dodge_move_out;
}
