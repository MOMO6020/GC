/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_uart.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */

/**
  * @brief  for enginner
  */

#define rx_buffer_size 39

typedef __packed struct
{
   float    motor[4];     // 16 字节  J0-J3编码器角度数据
   uint16_t joystick_x; 	// 2 	字节  摇杆x值
   uint16_t joystick_y; 	// 2 	字节  摇杆y值
   int8_t   chassis_pump; // 1 	字节  底盘气泵
   int8_t   arm_pump;     // 1 	字节  手臂气泵
   uint8_t  lrc;     			// LRC
}RobotCMD;



/**
  * @brief  remote control information
  */
typedef __packed struct
{
	
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
	
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
	
  /*键盘*/
  int16_t mouse_v;
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int16_t mouse_press_r;
  uint8_t mouse_press_l;
  uint8_t key_w;
  uint8_t key_s;
  uint8_t key_a;
  uint8_t key_d;
  uint8_t key_q;
  uint8_t key_e;
  uint8_t key_shift;
  uint8_t key_ctrl;
	uint8_t key_r;
	uint8_t key_f;
	uint8_t key_g;
	uint8_t key_z;
	uint8_t key_x;
	uint8_t key_c;
	uint8_t key_v;
	uint8_t key_b;
	
}rc_info_t;

typedef __packed struct
{
  
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int16_t mouse_press_r;
  uint8_t mouse_press_l;
  uint8_t key_w;
  uint8_t key_s;
  uint8_t key_a;
  uint8_t key_d;  
  uint8_t key_q;
  uint8_t key_e;
  uint8_t key_shift;
  uint8_t key_ctrl;
	uint8_t key_r;
	uint8_t key_f;
	uint8_t key_g;
	uint8_t key_z;
	uint8_t key_x;
	uint8_t key_c;
	uint8_t key_v;
	uint8_t key_b;
	
}key_board_t;

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef *USARTx , uint16_t Data);

void UART8_Receive_IT(void);
void Robot_Cmd_Queue_init(void);

#endif
