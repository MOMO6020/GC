#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"

uint8_t dbus_buf[DBUS_BUFLEN];
rc_info_t rc;
key_board_t key_t;


QueueHandle_t robot_cmd_queue;
RobotCMD robot_cmd;
uint8_t rx_buffer[rx_buffer_size];
uint8_t rx_data;
uint8_t lrc;
uint8_t rx_index = 0;

void Robot_Cmd_Queue_init(void)
{
	robot_cmd_queue = xQueueCreate(1, sizeof(RobotCMD));
}

void UART8_Receive_IT(void)
{
    HAL_UART_Receive_IT(&huart8, &rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if(huart -> Instance == UART8)
    {
			if ((rx_index == 0 && rx_data != 0xA5) ||
					(rx_index == 1 && rx_data != 0x1e) ||
					(rx_index == 5 && rx_data != 0x02) ||
					(rx_index == 6 && rx_data != 0x03))
			{
				lrc = 0;
				rx_index = 0;
			}
			else
			{
				rx_buffer[rx_index++] = rx_data;
				if (rx_index >= rx_buffer_size - 9)
				{
					if (lrc == rx_data)
					{
						memcpy(&robot_cmd, rx_buffer + 7, sizeof(RobotCMD));
						BaseType_t temp;
						BaseType_t result = xQueueGenericSendFromISR(robot_cmd_queue, &robot_cmd, &temp, queueOVERWRITE);
					}
					lrc = 0;
					rx_index = 0;
				}
				if (rx_index > 7)
				lrc += rx_data;
			}
			HAL_UART_Receive_IT(&huart8, &rx_data, 1);
    }
}



static int uart_receive_dma_no_it(UART_HandleTypeDef *huart , uint8_t *pData , uint32_t Size)
{
	
  uint32_t WHtmp1 = 0;
	
  WHtmp1 = huart -> RxState;
	
	if(WHtmp1 == HAL_UART_STATE_READY)
	{
		if((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
		
		huart -> pRxBuffPtr = pData;
		huart -> RxXferSize = Size;
		huart -> ErrorCode  = HAL_UART_ERROR_NONE;
		
		/* Enable the DMA Stream */
		HAL_DMA_Start(huart -> hdmarx, (uint32_t)&huart -> Instance -> DR, (uint32_t) pData, Size);
		
		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart -> Instance -> CR3, USART_CR3_DMAR);
		
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return((uint16_t)(dma_stream -> NDTR));
}

/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval 
  */
void rc_callback_handler(rc_info_t *rc , uint8_t *buff)
{
  rc -> ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc -> ch1 -= 1024;
  rc -> ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc -> ch2 -= 1024;
  rc -> ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc -> ch3 -= 1024;
  rc -> ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc -> ch4 -= 1024;

  rc -> sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc -> sw2 = (buff[5] >> 4) & 0x0003;
  
  if ((abs(rc -> ch1) > 660) ||
      (abs(rc -> ch2) > 660) ||
      (abs(rc -> ch3) > 660) ||
      (abs(rc -> ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
  }
	
  /*-----------------键盘使用-------------------*/
  rc -> mouse_x = buff[6] | (buff[7] << 8);          //鼠标左右移动
  rc -> mouse_y = buff[8] | (buff[9] << 8);          //鼠标前后移动
  rc -> mouse_z = buff[10] | (buff[11] << 8);        //鼠标滚轮前后滚动
	rc -> mouse_press_l = buff[12];
  rc -> mouse_press_r = buff[13];
  rc -> mouse_v = buff[14] | (buff[15] << 8);        //键盘按键对应字节范围
  
  rc -> key_w 		= (rc -> mouse_v & 0x0001)  >> 0;
  rc -> key_s 		= (rc -> mouse_v & 0x0002)  >> 1;
  rc -> key_a 		= (rc -> mouse_v & 0x0004)  >> 2;
  rc -> key_d 		= (rc -> mouse_v & 0x0008)  >> 3;
  rc -> key_shift = (rc -> mouse_v & 0x0010)  >> 4;
  rc -> key_ctrl  = (rc -> mouse_v & 0x0020)  >> 5;
  rc -> key_q 		= (rc -> mouse_v & 0x0040)  >> 6;
  rc -> key_e 		= (rc -> mouse_v & 0x0080)  >> 7;
  rc -> key_r 		= (rc -> mouse_v & 0x0100)  >> 8;
  rc -> key_f 		= (rc -> mouse_v & 0x0200)  >> 9;
  rc -> key_g 		= (rc -> mouse_v & 0x0400)  >> 10;
  rc -> key_z 		= (rc -> mouse_v & 0x0800)  >> 11;
	rc -> key_x 		= (rc -> mouse_v & 0x01000) >> 12;
	rc -> key_c 		= (rc -> mouse_v & 0x02000) >> 13;
	rc -> key_v 		= (rc -> mouse_v & 0x04000) >> 14;
	rc -> key_b 		= (rc -> mouse_v & 0x08000) >> 15;
	
}

/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* handle received data in idle interrupt */
	if(huart == &DBUS_HUART)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart -> hdmarx);
		
		/* handle dbus data dbus_buf from DMA */
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart -> hdmarx -> Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);	
		}
		
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart -> hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart -> hdmarx);
		
	}
}

/**
  * @brief      callback this function when uart interrupt 
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**
  * @brief   initialize dbus uart device 
  * @param   
  * @retval  
  */

void dbus_uart_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}
