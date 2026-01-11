/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "bsp_imu.h"

/* USER CODE BEGIN Includes */     
#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "tim.h"
#include "bsp_uart.h"
#include "main.h"
#include "arm_math.h"
#include "gpio.h" 		//注意
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
extern rc_info_t rc;
char buf[300];

/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void moto_chassis_task(void const* argument);
void all_jointcontrol(void const* argument);
void J2andJ3_task(void const* argument);
void J5andJ6_task(void const* argument);
void dbus_task(void const* argument);
void pump_task(void const* argument);
void set_robot_current_task(void const* argument);
void mpu_task(void const* argument);
void mpu_task_chassis_calc_task(void const* argument);
void mpu_task_head_calc_task(void const* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void J2andJ3_control(void);
void WH_moto_chassis(void);
void J1andJ4_control(void);
void J5andJ6_control(void);
void pump_oc(void);
void follow_head(void);
void dodge_attacks(void);
void WH_flip_poke(void);
void WH_dodge_move(void);
void MX_GPIO_Init(void); //注意
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

int    chassis_a           = 0;
int    cap_block_argument  = 10;

float change_yaw_p;
float change_pitch_p;
float J1_angle, J4_angle;
int   change_qe;

float dodge_speed;
int dodge_judge = 0;
double dodge_angle;
double taget_yaw;
float WH_mpu_yaw;
int move_speed_judge = 0;
int max_move_speed;

//小陀螺状态下用到的参数
float sin_yaw = 0.0f, cos_yaw = 0.0f;
float vx_set = 0.0f, vy_set = 0.0f;
float included_angle;
float vx,vy;

int PC_rc_ch1, PC_rc_ch2, PC_rc_ch3, PC_rc_ch4;
int rc_ch1_revise, rc_ch2_revise, rc_ch3_revise, rc_ch4_revise;

RobotCMD cmd;
extern QueueHandle_t robot_cmd_queue;
//int single_shoot=0;

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
	
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
	
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
	
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
	
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask1, moto_chassis_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask1), NULL);
	
  osThreadDef(defaultTask2, all_jointcontrol, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask2), NULL);
	
	osThreadDef(defaultTask3, J5andJ6_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask3), NULL);
	
  osThreadDef(defaultTask4, dbus_task, osPriorityAboveNormal , 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask4), NULL);
	
  osThreadDef(defaultTask5, J2andJ3_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask5), NULL);	
	
	osThreadDef(defaultTask6, pump_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask6), NULL);	
	
	osThreadDef(defaultTask7, set_robot_current_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask7), NULL);
	
	osThreadDef(defaultTask8, mpu_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask8), NULL);
	
	osThreadDef(defaultTask9, mpu_task_chassis_calc_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask9), NULL);
	
	osThreadDef(defaultTask10, mpu_task_head_calc_task, osPriorityNormal, 0, 128);//名称，线程，优先级，实例，堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask10), NULL);
	
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

double set_v, set_spd[8];

/* StartDefaultTask function */
void moto_chassis_task(void const* argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  while(1)
  {
		WH_moto_chassis();
		osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

float joyx = 0;
float joyy = 0;
void all_jointcontrol(void const* argument)
{
	while(1)
	{
		while(uxQueueMessagesWaiting(robot_cmd_queue) == 0)
		{
			vTaskDelay(1);
		}
		xQueueReceive(robot_cmd_queue, &cmd, 0);
		joyx = cmd.joystick_x;
		joyy = cmd.joystick_y;
		J1andJ4_control();
		J5andJ6_control();
		J2andJ3_control();
	}
}

void J5andJ6_task(void const* argument)
{
	while(1)
	{
		// J5andJ6_control();
		osDelay(5);
	}
}

void dbus_task(void const* argument)
{
	while(1)
	{
		dbus_uart_init();
		osDelay(5);
	}
}

void J2andJ3_task(void const* argument)
{
	while(1)
	{
		// J2andJ3_control();
		osDelay(5);
	}
}

void pump_task(void const* argument)
{
	while(1)
	{
		pump_oc();
		osDelay(5);
	}
}

//ch1 右摇杆左右 - ch2 右摇杆上下 - ch3 左摇杆左右 - ch4 左摇杆上下 - 摇杆最大值660 - sw从上到下为1,3,2
void WH_moto_chassis() //底盘运动
{
  rc_ch3_revise = rc.ch3 + 0; //左摇杆的值
	rc_ch4_revise = rc.ch4 + 0; //对遥控器的校准
	
	if(rc.key_shift == 1) //快速旋转(加速)
	{
		change_qe = 2500;
	}
	else change_qe = 1000;
	
/*--------------------------------控制底盘前后和左右平移--------------------------------*/
	
	if(rc.sw1 == 2)
	{
		if(dodge_judge != 1) //判断是否没开小陀螺
		{
			PC_rc_ch4 = rc_ch4_revise + ((rc.key_w * 10) * chassis_a) - ((rc.key_s * 10) * chassis_a);
			PC_rc_ch3 = rc_ch3_revise - ((rc.key_a * 10) * chassis_a) + ((rc.key_d * 10) * chassis_a);
		}
		if(dodge_judge == 1) //判断是否开了小陀螺
		{
			vx = (rc_ch4_revise -((rc.key_w * 10) * chassis_a)+((rc.key_s * 10) * chassis_a)) * 10;
			vy = (rc_ch3_revise -((rc.key_a * 10) * chassis_a)+((rc.key_d * 10) * chassis_a)) * 10;
		}
		//综合（倒数第一个是小陀螺和小陀螺状态下的运动，倒数第二是底盘跟随云台）
		set_spd[0] = PC_rc_ch3 * ( 10) + PC_rc_ch4 * ( 10) + (- rc.key_q * change_qe + rc.key_e * change_qe) + pid_spd[4].WH_yaw_angle_out1 + pid_spd[0].WH_dodge_move_out;
		set_spd[1] = PC_rc_ch3 * ( 10) + PC_rc_ch4 * (-10) + (- rc.key_q * change_qe + rc.key_e * change_qe) + pid_spd[4].WH_yaw_angle_out1 + pid_spd[1].WH_dodge_move_out;
		set_spd[2] = PC_rc_ch3 * (-10) + PC_rc_ch4 * ( 10) + (- rc.key_q * change_qe + rc.key_e * change_qe) + pid_spd[4].WH_yaw_angle_out1 + pid_spd[2].WH_dodge_move_out;
		set_spd[3] = PC_rc_ch3 * (-10) + PC_rc_ch4 * (-10) + (- rc.key_q * change_qe + rc.key_e * change_qe) + pid_spd[4].WH_yaw_angle_out1 + pid_spd[3].WH_dodge_move_out;
		
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
		}
		HAL_Delay(10);
	}
	
/*--------------------------------控制底盘原地左右转--------------------------------*/
	
	if(rc.sw1 == 3) //遥控器操控旋转
	{
		//旋转
		if(rc.ch3 < ( 50) || rc.ch3 > (-50))
		{
			rc.ch3 = 0;
		}
		if(rc.ch3 >= ( 20))
		{
			rc.ch3 = ( 20);
		}
		if(rc.ch3 <= (-20))
		{
			rc.ch3 = (-20);
		}
		set_spd[0] = rc_ch3_revise * (10);
		set_spd[1] = rc_ch3_revise * (10);
		set_spd[2] = rc_ch3_revise * (10);
		set_spd[3] = rc_ch3_revise * (10);
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
		}
		HAL_Delay(10);
	}
/*--------------------------------锁底盘--------------------------------*/
	if(rc.sw1 == 1) //锁底盘
	{
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, 0);
		}
		HAL_Delay(10);
	}

}



void J1andJ4_control()
{
	J1_angle = cmd.motor[0]; 
	J4_angle = cmd.motor[3];
	J4_angle = J4_angle > 210 ? 210 : J4_angle;
	J4_angle = J4_angle < 30 ? 30 : J4_angle;
	pid_calc(&pid_spd[4], (moto_chassis[4].round_cnt * 8191.0f + moto_chassis[4].last_angle) / 36.0f, J1_angle / 360.0f * 8191.0f);
	pid_calc(&pid_spd[5], moto_chassis[5].last_angle, J4_angle / 360.0f * 8191.0f);//(J4_angle / 360.0f * 8191.0f));
	
//	while(rc.sw2 == 1)
//	{
//			if(rc.ch1 > 100 && rc.ch1 < -100)
//			{
//				J1_angle += rc.ch1 / 330;
//			}
//			if(rc.ch2 > 100 && rc.ch2 < -100)
//			{
//				J4_angle += rc.ch1 / 330;
//			}
//			
//			if(J1_angle >= 180) //J1 is ok
//			{
//				J1_angle = 180;
//		  }
//			if(J1_angle <= 0)
//			{
//				J1_angle = 0;
//			}
//			if(J4_angle >= 210)
//			{
//				J4_angle = 210;
//			}
//			if(J4_angle <= 30)
//			{
//				J4_angle = 30;
//			}
//			pid_calc(&pid_spd[4], (moto_chassis[4].round_cnt * 8191.0f + moto_chassis[4].last_angle) / 36.0f, J4_angle / 360.0f * 8191.0f);
//			pid_calc(&pid_spd[5], moto_chassis[5].last_angle, (J4_angle / 360 * 8191));
//			HAL_Delay(10);
//	}
	
//	while(rc.sw2 != 1)
//	{
//		J4_angle = 90;
//		pid_calc(&pid_spd[4], (moto_chassis[4].round_cnt * 8191.0f + moto_chassis[4].last_angle) / 36.0f, J4_angle / 360.0f * 8191.0f);
//		pid_calc(&pid_spd[5], moto_chassis[5].last_angle, (J4_angle / 360 * 8191));
//		HAL_Delay(10);
//	}
}

float J5_angle = 0.0f;
float J6_angle = 0.0f;
void J5andJ6_control()
{	
	if(cmd.joystick_x == 4095.0f && cmd.joystick_y == 4095.0f)//防止编码器读数有问题 xy都是4095时丢弃
	{
		
	}
	if(cmd.joystick_x == 4095.0f && cmd.joystick_y !=4095.0f && cmd.joystick_y !=0.0f)//x向右推到最大向右转
	{
			J5_angle -= 2;
			J6_angle -= 2;		
	}

	if(cmd.joystick_x == 0.0f && cmd.joystick_y !=4095.0f && cmd.joystick_y !=0.0f)//x向左推到最大向左转
	{
			J5_angle += 2;
			J6_angle += 2;		
	}		
	if(cmd.joystick_y == 0.0f && cmd.joystick_x !=4095.0f && cmd.joystick_x !=0.0f)
	{
			J5_angle -= 2;
			J6_angle += 2;		
	}
	if(cmd.joystick_y == 4095.0f && cmd.joystick_x !=4095.0f && cmd.joystick_x !=0.0f)
	{
			J5_angle += 2;
			J6_angle -= 2;
	}
//		if(J5_angle >= 180) J5_angle = 180;
//		if(J5_angle <=   0) J5_angle =   0;
//		if(J6_angle >= 180) J6_angle = 180;
//		if(J6_angle <=   0) J6_angle =   0;
	//}
	pid_calc(&pid_spd[6], (moto_chassis[6].round_cnt * 8191.0f + moto_chassis[6].last_angle) / 36.0f, J5_angle / 360.0f * 8191.0f);
	pid_calc(&pid_spd[7], (moto_chassis[7].round_cnt * 8191.0f + moto_chassis[7].last_angle) / 36.0f, J6_angle / 360.0f * 8191.0f);
//	while(rc.sw2 == 2) 
//	{
//			if(rc.ch1 > 50 || rc.ch1 < -50)
//			{      
//				J5_angle += rc.ch1 / 330;
//				J6_angle += rc.ch1 / 330;
//			}
//			if(rc.ch2 > 50 || rc.ch2 < -50)
//			{
//				J5_angle +=  -(rc.ch2 / 330);
//				J6_angle += 	 rc.ch2 / 330;
//			}
//			if(J5_angle >= 180) J5_angle = 180;
//			if(J5_angle <=   0) J5_angle =   0;
//			if(J6_angle >= 180) J6_angle = 180;
//			if(J6_angle <=   0) J6_angle =   0;
//			pid_calc(&pid_spd[6], (moto_chassis[6].round_cnt * 8191.0f + moto_chassis[6].last_angle) / 36.0f, J5_angle / 360.0f * 8191.0f);
//			pid_calc(&pid_spd[7], (moto_chassis[7].round_cnt * 8191.0f + moto_chassis[7].last_angle) / 36.0f, J6_angle / 360.0f * 8191.0f);
//			HAL_Delay(10);
//	}

//	while(rc.sw2 == 1)
//	{
//		J5_angle = 90.0f;
//		J6_angle = 90.0f;
//		J5_zero_offset = moto_chassis[6].round_cnt * 8191.0 + moto_chassis[6].last_angle;
//		pid_calc(&pid_spd[6], (moto_chassis[6].round_cnt * 8191.0 + moto_chassis[6].last_angle) / 36.0, (moto_chassis[6].round_cnt * 8191.0 + moto_chassis[6].last_angle) / 36.0);
//		pid_calc(&pid_spd[7], (moto_chassis[7].round_cnt * 8191.0 + moto_chassis[7].last_angle) / 36.0, J6_angle / 360.0f * 8191.0f);
//		HAL_Delay(10);
//	}

//	while(rc.sw2 != 2)
//	{
//		J5_angle = 90.0f;
//		J6_angle = 90.0f;
//		pid_calc(&pid_spd[6], (moto_chassis[6].round_cnt * 8191.0 + moto_chassis[6].last_angle) / 36.0, J5_angle / 360.0f * 8191.0f);
//		pid_calc(&pid_spd[7], (moto_chassis[7].round_cnt * 8191.0 + moto_chassis[7].last_angle) / 36.0, J6_angle / 360.0f * 8191.0f);
//		HAL_Delay(10);
//	}
}


float	J3_angle = 0.0, J3_lastangle = 0.0;
float  J2_angle = 0.0, J2_lastangle = 0.0; 
void J2andJ3_control() //DM_Control  达妙电机的控制角度单位为rad J2_MAXANGLE的角度为我测试过的2.1个rad对应的角度
{
	J2_lastangle = J2_angle;
	J3_lastangle = J3_angle;
	J2_angle = cmd.motor[1] / 180.0f * 3.14f;	 			//达妙的角度单位为rad
	J3_angle = cmd.motor[2] / 180.0f * -3.14f;
	J2_angle = J2_angle > 2.1f ? 2.1f : J2_angle;
	J2_angle = J2_angle < 0.0f ? 0.0f : J2_angle;
	J3_angle = J3_angle > 2.1f ? 2.1f : J3_angle;
	J3_angle = J3_angle < 0.0f ? 0.0f : J3_angle;
	ctrl_motor2(&hcan1, 0x109, J2_angle, (J2_angle - J2_lastangle)/0.033f);
	ctrl_motor2(&hcan1, 0x10A, J3_angle, (J3_angle - J3_lastangle)/0.033f);
	
	
//	while(rc.sw2 == 3)
//	{
//		if(rc.ch1 < -100)
//		{
//			ctrl_motor2(&hcan1, 0x109, cmd.motor[1], -1.0f);
//			HAL_Delay(10);
//		}
//		if(rc.ch1 >  100)
//		{
//			ctrl_motor2(&hcan1, 0x109, cmd.motor[1], 1.0f);
//			HAL_Delay(10);
//		}
//		if(rc.ch1 >= -100 || rc.ch1 <= 100)
//		{
//			ctrl_motor2(&hcan1, 0x109, cmd.motor[1],  0.0f);
//			HAL_Delay(10);
//		}
//		if(rc.ch2 < -100)
//		{
//			ctrl_motor2(&hcan1, 0x10A, cmd.motor[2],-1.0f);
//			HAL_Delay(10);
//		}
//		if(rc.ch2 >  100)
//		{
//			ctrl_motor2(&hcan1, 0x10A, cmd.motor[2], 1.0f);
//			HAL_Delay(10);
//		}
//		if(rc.ch2 >= -100 || rc.ch2 <= 100)
//		{
//			ctrl_motor2(&hcan1, 0x10A, cmd.motor[2], 0.0f);
//			HAL_Delay(10);
//		}
//	}
//	while(rc.sw2 != 3)
//	{
//		ctrl_motor2(&hcan1, 0x109, cmd.motor[1],0.0f);
//		ctrl_motor2(&hcan1, 0x10A, cmd.motor[2],0.0f);
//		HAL_Delay(10);
//	}
}


bool chassis_pump_on = false;
bool arm_pump_on = false;
void pump_oc()
{
	chassis_pump_on = (cmd.chassis_pump != 0);
	arm_pump_on = (cmd.arm_pump != 0);
	if (arm_pump_on)
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
	if (chassis_pump_on)
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_RESET);
	vTaskDelay(100);
}


float test;
void set_robot_current_task(void const* argument) //发送CAN消息
{
	while(1)
	{
		test = pid_spd[6].pos_out;
		set_moto_current(&hcan1, pid_spd[0].pos_out,
														 pid_spd[1].pos_out,
														 pid_spd[2].pos_out,
														 pid_spd[3].pos_out,
														 0x200);
		set_moto_current(&hcan1, pid_spd[4].pos_out, //- pid_spd[4].WH_yaw_angle_out2 - pid_spd[4].WH_dodge_out
														 pid_spd[5].pos_out,
														 pid_spd[6].pos_out, //pid_spd[6].pos_out
													   pid_spd[7].pos_out, //pid_spd[7].pos_out
		                         0x1FF);
		
		set_moto_current(&hcan1, 0xFFFF, //达妙使能用
														 0xFFFF,
														 0xFFFF,
														 0xFFFC,
														 0x209);
		set_moto_current(&hcan1, 0xFFFF,
														 0xFFFF,
														 0xFFFF,
														 0xFFFC,
														 0x20A);
														 
		osDelay(5);
	}
}



void mpu_task(void const* argument)
{
	while(1)
	{
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		if(imu.yaw > 0)
		{
			WH_mpu_yaw = imu.yaw;
		}
		if(imu.yaw < 0)
		{
			WH_mpu_yaw = 360 + imu.yaw;
		}
		HAL_Delay(10);
		HAL_UART_Transmit(&huart6, (uint8_t *)buf, (COUNTOF(buf) - 1), 55);
		osDelay(5);
	}
}

void mpu_task_chassis_calc_task(void const* argument)
{
	while(1)
	{
		follow_head();
		dodge_attacks();
		WH_dodge_move();
		osDelay(10);
	}
}

void follow_head() //云台初始角为138
{
	while(rc.mouse_press_r == 1)
	{
		WH_yaw_angle_pid1(&pid_spd[4], moto_chassis[4].user_angle, 72, 10); //跟随模式下，底盘的速度计算
		//J1_angle = 72;
		HAL_Delay(10);
	}
	pid_spd[4].WH_yaw_angle_out1 = 0; //当按键松开后，让各速度马上回0
	pid_spd[4].WH_yaw_angle_out2 = 0;
}

void dodge_attacks() //小陀螺开启的判断
{
	if(rc.key_g == 1 && dodge_judge == 0)
	{
		HAL_Delay(100); //消抖
		if(rc.key_g == 1 && dodge_judge == 0)
		{
			dodge_judge = 1;
			dodge_speed = 3000;
		}
	}
	if(rc.key_b == 1 && dodge_judge == 1)
	{
		HAL_Delay(100); //消抖
		if(rc.key_b == 1 && dodge_judge == 1)
		{
			dodge_judge = 0;
			dodge_speed = 0;
			pid_spd[4].WH_dodge_out = 0;
		}
	}
}

void WH_dodge_move()
{
	included_angle = moto_chassis[4].user_angle - 138; //算云台和底盘的夹角
	sin_yaw = arm_sin_f32(included_angle * PI / 180); //算sin，转成弧度制了
	cos_yaw = arm_cos_f32(included_angle * PI / 180);
	if(dodge_judge == 1)
	{
		vy_set = +cos_yaw * vy + sin_yaw * vx; //将速度分解
		vx_set = -sin_yaw * vy + cos_yaw * vx;
//	wz=302.52;（目前没用）
//	distance_from_the_motor_to_the_center=477.66/2/1000;
		set_spd[0]= (-vx_set) + (	vy_set) + dodge_speed * 1.0f;
		set_spd[1]= (	vx_set) + (	vy_set) + dodge_speed * 1.0f;
		set_spd[2]= (-vx_set) + (-vy_set) + dodge_speed * 1.0f;
		set_spd[3]= (	vx_set) + (-vy_set) + dodge_speed * 1.0f;	
		for(int i=0; i<4; i++)
		{
			WH_dodge_move_pid(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i], 0);
		}
		HAL_Delay(10);
	}
	if(dodge_judge == 0)
	{
		pid_spd[0].WH_dodge_move_out = 0;
		pid_spd[1].WH_dodge_move_out = 0;
		pid_spd[2].WH_dodge_move_out = 0;
		pid_spd[3].WH_dodge_move_out = 0;
		HAL_Delay(10);
	}
}

void mpu_task_head_calc_task(void const* argument)
{
	double taget_yaw_a;
	double dodge_yaw_a;
	while(1)
	{
		if(dodge_judge != 1) //判断符没达成前一直更新数据
		{
			dodge_angle = WH_mpu_yaw;
		}
		
		if(rc.mouse_press_r != 1)
		{
			taget_yaw = WH_mpu_yaw;
		}
		
		if(rc.mouse_press_r == 1) //跟随
		{
			if(taget_yaw > 359.999) //解决角度跃变
			{
				taget_yaw = 1;
			}
			if(taget_yaw < 0.001)
			{
				taget_yaw = 359;
			}
			if((rc.mouse_x < (-10) && rc.key_shift == 1) || rc.ch1 > 550) //大幅
			{
				taget_yaw_a = (rc.mouse_x * 0.02) + rc.ch1 / 100;
				taget_yaw = taget_yaw + taget_yaw_a;
			}
			if(rc.mouse_x < (-10) || (rc.ch1 > 20 && rc.ch1 <= 550)) //小幅
			{
				taget_yaw_a = (rc.mouse_x * 0.007) + rc.ch1 / 200;
				taget_yaw = taget_yaw + taget_yaw_a;
			}
			if((rc.mouse_x > 10 && rc.key_shift == 1) || rc.ch1 < (-550)) //大幅
			{
				taget_yaw_a = (rc.mouse_x * 0.02) + rc.ch1 / 100;
				taget_yaw = taget_yaw + taget_yaw_a;
			}
			if(rc.mouse_x > 10 || (rc.ch1 < (-20) && rc.ch1 >= (-550))) //小幅
			{
				taget_yaw_a = (rc.mouse_x * 0.007) + rc.ch1 / 200;
				taget_yaw = taget_yaw + taget_yaw_a;
			}
			if((taget_yaw-WH_mpu_yaw)<-180)//配合解决角度跃变
			{
				WH_mpu_yaw=WH_mpu_yaw-360;
			}
			if((taget_yaw-WH_mpu_yaw)>180)
			{
				taget_yaw=taget_yaw-360;
			}
			WH_yaw_angle_pid2(&pid_spd[4],WH_mpu_yaw,taget_yaw,10);
		}
		
		if(dodge_judge==1) //陀螺
		{
			if(dodge_angle>359.999) //解决角度跃变
			{
				dodge_angle=1;
			}
			if(dodge_angle<0.001)
			{
				dodge_angle=359;
			}
			if((rc.mouse_x<(-10)&&rc.key_shift==1)||rc.ch1>550)//大幅
			{
				dodge_yaw_a=(rc.mouse_x*0.03)+rc.ch1/100;
				dodge_angle=dodge_angle+dodge_yaw_a;
			}
			if(rc.mouse_x<(-10)||(rc.ch1>20&&rc.ch1<=550))//小幅
			{
				dodge_yaw_a=(rc.mouse_x*0.015)+rc.ch1/200;
				dodge_angle=dodge_angle+dodge_yaw_a;
			}
			if((rc.mouse_x>10&&rc.key_shift==1)||rc.ch1<(-550))//大幅
			{
				dodge_yaw_a=(rc.mouse_x*0.03)+rc.ch1/100;
				dodge_angle=dodge_angle+dodge_yaw_a;
			}
			if(rc.mouse_x>10||(rc.ch1<(-20)&&rc.ch1>=(-550)))//小幅
			{
				dodge_yaw_a=(rc.mouse_x*0.015)+rc.ch1/200;
				dodge_angle=dodge_angle+dodge_yaw_a;
			}
			if((dodge_angle-WH_mpu_yaw)<-180)//配合解决角度跃变
			{
				WH_mpu_yaw=WH_mpu_yaw-360;
			}
			if((dodge_angle - WH_mpu_yaw) > 180)
			{
				dodge_angle = dodge_angle - 360;
			}
			WH_dodge_attacks_pid1(&pid_spd[4], WH_mpu_yaw, dodge_angle, 0);
		}
		osDelay(10);
	}
}

void WH_flip_poke()
{
//	poke_current=-1700;
//	HAL_Delay(2500);
//	poke_current=0;
//	HAL_Delay(350);
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
