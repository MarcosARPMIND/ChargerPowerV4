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

#define RX_BUFFER_SIZE 128 // RS485 RX DMA ring buffer (bytes)
#define RS485_TX_BUFFER_SIZE 32 // RS485 TX frame copy for the DMA (largest frame: 15 bytes)

/**
 * @brief RS485 link health counters since boot (see CMD_GET_DIAG).
 */
typedef struct {
	uint16_t uart_errors;    /**< UART receive errors (framing/noise/overrun) -- each one aborts the DMA reception */
	uint16_t rx_restarts;    /**< Times the DMA reception had to be restarted */
	uint8_t  tx_recoveries;  /**< Transmissions found stuck and force-aborted (saturates at 255) */
} Comms_Diag_t;


void Comms_INIT(void);
void RS485_Send_Package(uint8_t* data, uint16_t length);
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint16_t RS485_Get_Data(uint8_t* data_receive, uint16_t max_len);
void Comms_Get_Diag(Comms_Diag_t *out);

HAL_StatusTypeDef SPI_Driver_Write(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI_Driver_Read(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI_Driver_Transfer(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);



#endif /* INC_COMMS_MANAGER_H_ */
