/*
 * comms_manager.h
 *
 *  Created on: Nov 24, 2025
 *      Author: ARPMindTech
 */

#ifndef INC_COMMS_MANAGER_H_
#define INC_COMMS_MANAGER_H_

#include "board_io.h"
#include "stm32c0xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "spi.h"
#include <stdbool.h>

#define RX_BUFFER_SIZE 128 //RS485 Rx BUFFER


void Comms_INIT(void);
void RS485_Send_Package(uint8_t* data, uint16_t lenght);
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart);
uint16_t RS485_Get_Data(uint8_t* data_receive, uint16_t max_len);

HAL_StatusTypeDef SPI_Driver_Write(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI_Driver_Read(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI_Driver_Transfer(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);



#endif /* INC_COMMS_MANAGER_H_ */
