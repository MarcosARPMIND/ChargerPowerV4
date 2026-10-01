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
#include <string.h>

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

/**
 * @brief  Tick (HAL_GetTick()) when the current transmission started.
 * @note   rs485_busy is only ever cleared by HAL_UART_TxCpltCallback(). If a
 *         transmission is corrupted mid-flight (bus noise/glitch -- e.g.
 *         observed right after cutting mains power to the charger) and that
 *         callback never fires, rs485_busy would otherwise stay stuck true
 *         forever, silently dropping every future response until a manual
 *         reset. This timestamp lets RS485_Send_Package() detect that and
 *         force-recover instead.
 */
static volatile uint32_t rs485_tx_start_tick = 0;

/** Generous timeout for the largest frame this protocol ever sends
 * (~15 bytes) at 115200 baud (~1.3ms) -- anything still "busy" this long
 * after starting is stuck, not just slow. */
#define RS485_TX_TIMEOUT_MS  50u

/**
 * @brief  Private copy of the frame being transmitted.
 * @note   The DMA reads from here for the whole transmission, so the
 *         caller's buffer can be reused as soon as RS485_Send_Package()
 *         returns. Previously the DMA read straight from the caller's static
 *         buffer: a second message built right after the first (e.g. a
 *         command reply followed by a CMD_STATE_NOTIFY in the same loop pass)
 *         overwrote the bytes still being sent -> corrupted frame on the bus.
 */
static uint8_t tx_dma_buffer[RS485_TX_BUFFER_SIZE];

/* Link health counters since boot -- see Comms_Get_Diag() / CMD_GET_DIAG */
static volatile uint16_t uart_error_count = 0;   /* written in HAL_UART_ErrorCallback() (ISR) */
static uint16_t          rx_restart_count = 0;
static uint8_t           tx_recover_count = 0;


/* ============================================================================== */
/* RS-485 FUNCTIONS                                                               */
/* ============================================================================== */

/**
 * @brief  Initializes the Communication Interfaces.
 * @note   Puts the RS-485 transceiver in receive mode
 * and starts the UART DMA Reception in Circular Mode.
 */
void Comms_INIT(void)
{
    /* 1. Transceiver in receive mode (default bus state) */
    Board_Set_RS485_DE(RECEIVER);


    /* 2. Start RS-485 RX in circular DMA mode */
    /* Note: (uint8_t*) cast prevents warning, buffer must be aligned */
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);

    /* 3. Disable the RX DMA half/complete-transfer interrupts:
     * RS485_Get_Data() polls the DMA write index instead. */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC);
}

/**
 * @brief  Sends a data packet via RS-485 using DMA.
 * @param  data   Pointer to the data buffer to send.
 * @param  length Number of bytes to transmit.
 * @note   This function manages the DE (Driver Enable) pin automatically.
 * The frame is copied into tx_dma_buffer, so the caller's buffer is free
 * again on return. If the previous frame is still going out, waits for it
 * (<= ~1.3 ms per frame at 115200 baud) instead of dropping this one.
 */
void RS485_Send_Package(uint8_t* data, uint16_t length)
{
    if (length == 0 || length > RS485_TX_BUFFER_SIZE)
    {
        return;
    }

    /* Previous frame still in flight: wait for HAL_UART_TxCpltCallback(),
     * bounded by RS485_TX_TIMEOUT_MS since that transmission started. */
    while (rs485_busy && (HAL_GetTick() - rs485_tx_start_tick) < RS485_TX_TIMEOUT_MS)
    {
    }

    if(rs485_busy == true)
    {
        /* Stuck: the previous transmission's HAL_UART_TxCpltCallback never
         * fired (corrupted mid-flight). Force the UART/DMA back to idle and
         * recover instead of dropping every response forever. */
        HAL_UART_AbortTransmit(&huart2);
        Board_Set_RS485_DE(RECEIVER);
        rs485_busy = false;
        if (tx_recover_count < UINT8_MAX)
        {
            tx_recover_count++;
        }
    }

    /* Only now, with the DMA idle, is it safe to overwrite its buffer */
    memcpy(tx_dma_buffer, data, length);

    rs485_busy = true;
    rs485_tx_start_tick = HAL_GetTick();

    /* 1. Enable Driver (Talk Mode) */
    Board_Set_RS485_DE(SEND);

    /* 2. Start DMA Transmission */
    if(HAL_UART_Transmit_DMA(&huart2, tx_dma_buffer, length) != HAL_OK)
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
        /* Last bit has left the shift register: back to receive mode */
        Board_Set_RS485_DE(RECEIVER);

        /* Release Busy Flag */
        rs485_busy = false;
    }
}

/**
 * @brief  UART Error Callback.
 * @param  huart Pointer to UART handle.
 * @note   On this HAL, ANY receive error (framing, noise, overrun) while
 *         DMA reception is running is treated as blocking: the HAL aborts
 *         the DMA reception before calling this. With nothing restarting it,
 *         the charger kept transmitting but never heard the Master again
 *         until a reset (seen on the bench: ~2 min of ignored commands and
 *         unacknowledged CMD_STATE_NOTIFY retries). Only counted here --
 *         the restart itself happens in RS485_Get_Data(), from the main
 *         loop, which also owns rx_tail_index.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        if (uart_error_count < UINT16_MAX)
        {
            uart_error_count++;
        }
    }
}

/**
 * @brief  (Re)starts the circular DMA reception from a clean state.
 * @note   Main-loop context only (resets rx_tail_index). Bytes of the frame
 *         being received when the error hit are lost -- the Master times out
 *         on that one request and the next one goes through.
 */
static void RS485_RX_Restart(void)
{
    HAL_UART_AbortReceive(&huart2);   /* no-op if already aborted; clears error flags */
    rx_tail_index = 0;                /* DMA restarts writing at index 0 */
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC);

    if (rx_restart_count < UINT16_MAX)
    {
        rx_restart_count++;
    }
}

/**
 * @brief  Processes received RS-485 data from the Ring Buffer.
 * @note   Should be called periodically in the main loop.
 * Implements the "Consumer" part of the Producer-Consumer pattern.
 */
uint16_t RS485_Get_Data(uint8_t* data_receive, uint16_t max_len)
{
    /* Reception no longer running (aborted by a UART error, see
     * HAL_UART_ErrorCallback())? Restart it before reading -- whatever the
     * cause, the charger must never stay deaf to the Master. */
    if (huart2.RxState != HAL_UART_STATE_BUSY_RX)
    {
        RS485_RX_Restart();
        return 0;
    }

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

/**
 * @brief  Copies the RS485 link health counters (since boot).
 * @param  out Destination (see Comms_Diag_t).
 */
void Comms_Get_Diag(Comms_Diag_t *out)
{
    out->uart_errors   = uart_error_count;
    out->rx_restarts   = rx_restart_count;
    out->tx_recoveries = tx_recover_count;
}




