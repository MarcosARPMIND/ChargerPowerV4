/**
  ******************************************************************************
  * @file    comms_manager.c
  * @author  Marcos Novo
  * @date    Dec 3, 2025
  * @brief   Communication Manager Implementation.
  *
  * This file handles the high-level logic for:
  * 1. RS-485 Communication (UART + DMA + Ring Buffer).
  *
  ******************************************************************************
  */

#include "comms_manager.h"

/* ============================================================================== */
/* CONFIGURATION MACROS                                                           */
/* ============================================================================== */

/**
 * @brief  DMA Circular Buffer for RX.
 * @note   Data is automatically written here by the DMA controller.
 *         Size is defined by RX_BUFFER_SIZE.
 */
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

/**
 * @brief  Tail Index for the Ring Buffer.
 * @note   Points to the next byte to be processed by the application.
 *         Incremented in RS485_Get_Data().
 */
static volatile uint16_t rx_tail_index = 0;

/**
 * @brief  RS485 Bus Status Flag.
 * @note   True if a transmission is ongoing. Used to prevent collisions.
 */
static volatile bool rs485_busy	=	false;


/* ============================================================================== */
/* RS-485 FUNCTIONS                                                               */
/* ============================================================================== */

/**
 * @brief  Initializes the Communication Interfaces.
 * @note   Sets up RS-485 direction pins, deactivates SPI Chip Selects,
 * and starts the UART DMA Reception in Circular Mode.
 */
void Comms_INIT(void)
{
    /* 1. Set RS-485 to Listener Mode (Default) */
    Board_Set_RS485_DE(RECEIVER);


    /* 3. Start RS-485 RX in Circular DMA Mode */
    /* Note: (uint8_t*) cast prevents warning, buffer must be aligned */
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);

    /* 4. Optimization: Disable Half-Transfer & Transfer-Complete Interrupts for RX.
     * We poll the index manually in Process_Data, so these IRQs are unnecessary overhead. */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC);
}

/**
 * @brief  Sends a data packet via RS-485 using DMA.
 * @param  data   Pointer to the data buffer to send.
 * @param  length Number of bytes to transmit.
 * @note   This function manages the DE (Driver Enable) pin automatically.
 * It returns immediately (Non-Blocking), but checks if bus is free first.
 */
void RS485_Send_Package(uint8_t* data, uint16_t length)
{
    /* Safety Check: Prevent overwriting an ongoing transmission */
    if(rs485_busy == true)
    {
        /* Optional: Implement a software TX Queue here if needed later */
        return;
    }

    rs485_busy = true;

    /* 1. Enable Driver (Talk Mode) */
    Board_Set_RS485_DE(SEND);

    /* 2. Start DMA Transmission */
    if(HAL_UART_Transmit_DMA(&huart2, data, length) != HAL_OK)
    {
        /* Error Handler: Release bus immediately if TX fails to start */
        Board_Set_RS485_DE(RECEIVER);
        rs485_busy = false;
    }
}

/**
 * @brief  UART Transmission Complete Callback.
 * @param  huart Pointer to UART handle.
 * @note   Called by HAL ISR when the last bit has left the Shift Register.
 * Crucial for RS-485 to switch back to RX mode immediately.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        /* CORRECTION: We must switch to RECEIVER, not 'true' (which might be confusing) */
        Board_Set_RS485_DE(RECEIVER);

        /* Release Busy Flag */
        rs485_busy = false;
    }
}

/**
 * @brief  Processes received RS-485 data from the Ring Buffer.
 * @note   Should be called periodically in the main loop.
 * Implements the "Consumer" part of the Producer-Consumer pattern.
 */
uint16_t RS485_Get_Data(uint8_t* data_receive, uint16_t max_len)
{
    /* Calculate current DMA write position (Head) */
    /* Note: __HAL_DMA_GET_COUNTER returns remaining bytes, so we subtract from Size */

	uint16_t data_count = 0;
    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    uint16_t rx_head_index = RX_BUFFER_SIZE - dma_remaining;


    /* Check if there is new data (Head moved away from Tail) */
    if(rx_head_index != rx_tail_index)
    {
        /* Process all pending bytes */
        while(rx_head_index != rx_tail_index)
        {


        	if (data_count >= max_len) {
        	       return data_count;
        	}

        	data_receive[data_count] = rx_buffer[rx_tail_index];
        	data_count++;

            /* Increment Tail with Wrap-Around (Circular Logic) */
            rx_tail_index++;
            if(rx_tail_index >= RX_BUFFER_SIZE)
            {
                rx_tail_index = 0;
            }
        }
        return data_count;
    }
    else{
    	data_count = 0;
    	return data_count;
    }

}




