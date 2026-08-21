/**
  ******************************************************************************
  * @file    app_config.h
  * @author  Marcos Novo
  * @date    Nov 24, 2025
  * @brief   Hardware Abstraction Layer & Pin Mapping Configuration.
  *
  * This file centralizes the mapping between logical names (used in
  * firmware) and physical hardware pins (STM32 HAL).
  * It facilitates hardware revision changes without modifying the
  * core application logic.
  *
  * @version 1.0.0
  ******************************************************************************
  */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

//#include "main.h"
#include "stm32c0xx_hal.h"

/* ============================================================================== */
/* SYSTEM CONFIGURATION                             */
/* ============================================================================== */

/**
 * @brief Hardware Version Identifier
 * Used for conditional compilation if distinct PCB revisions exist.
 */
#define HW_VERSION                  0.0

/* --- ADE7953 RESET PINS (Outputs) --- */
/* Individual hardware reset line per ADE7953 IC */
#define ADE_RESET_1_PORT            GPIOD
#define ADE_RESET_1_PIN             GPIO_PIN_0      /* Reset for ADE Device 1 (Phase 1) */

#define ADE_RESET_2_PORT            GPIOD
#define ADE_RESET_2_PIN             GPIO_PIN_2      /* Reset for ADE Device 2 (Phase 2) */

#define ADE_RESET_3_PORT            GPIOD
#define ADE_RESET_3_PIN             GPIO_PIN_3      /* Reset for ADE Device 3 (Phase 3) */


/* ============================================================================== */
/* COMMUNICATION INTERFACES                             */
/* ============================================================================== */

/* --- RS485 TRANSCEIVER DIRECTION (Output) --- */
/* Mapping for define the direction of the transceiver */
#define RS485_DE_PORT               GPIOA
#define RS485_DE_PIN                GPIO_PIN_1      /* RS485 DE/RE Control Pin (PA1) */

/* --- SPI CHIP SELECTS (Outputs) --- */
/* Mapping for SPI Slaves (Ensure names match connected devices in future) */
#define SPI_CS_1_PORT               GPIOA
#define SPI_CS_1_PIN                GPIO_PIN_4      /* Device 1 CS */

#define SPI_CS_2_PORT               GPIOA
#define SPI_CS_2_PIN                GPIO_PIN_15     /* Device 2 CS */

#define SPI_CS_3_PORT               GPIOA
#define SPI_CS_3_PIN                GPIO_PIN_5      /* Device 3 CS */


/* --- RS485 / UART2 (Modbus/External) --- */
/* Note: DE/RE control is handled by RS485_DE_PORT/PIN above (PA1) */

/* Note: UART pins are configured via CubeMX (.ioc). Defined here for context only. */
#define UART0_RX_PORT               GPIOA
#define UART0_RX_PIN                GPIO_PIN_3
#define UART0_TX_PORT               GPIOA
#define UART0_TX_PIN                GPIO_PIN_2


/* --- UART1 (Debug/Trace) --- */
/* Note: UART pins are configured via CubeMX (.ioc). Defined here for context only. */
#define UART1_RX_PORT               GPIOA
#define UART1_RX_PIN                GPIO_PIN_10
#define UART1_TX_PORT               GPIOA
#define UART1_TX_PIN                GPIO_PIN_9


/* ============================================================================== */
/* SENSORS & MEASUREMENT                              */
/* ============================================================================== */

/* --- RCD (Residual Current Device) --- */
#define RCD_ERROR_PORT              GPIOC
#define RCD_ERROR_PIN               GPIO_PIN_13     /* Input: Fault signal from RCD module */
#define RCD_DEBOUNCE_MS             1              /* Software debounce window (ms) */

#define RCD_TEST_PORT               GPIOD
#define RCD_TEST_PIN                GPIO_PIN_1      /* Output: Triggers RCD self-test */

#define ADE_EXTI_PORT               GPIOA           /* Input: Interrupt Request (IRQ) */
#define ADE_EXTI_PIN                GPIO_PIN_1      /* Configured via NVIC in CubeMX */


/* ============================================================================== */
/* POWER CONTROL                                    */
/* ============================================================================== */

/* --- RELAY COMMANDS (Outputs) --- */
/* Control signals for high-voltage contactors or PCB relays */

#define RELAYS_PORT 				GPIOC
#define RELAYS_PIN  				GPIO_PIN_7      /* Contactor K3 Command (Phase 3) */


/* --- RELAY FEEDBACK (Inputs) --- */
/* Mirror contacts to verify physical state of relays (Safety monitoring) */

#define RELAY_STATE_1_PORT          GPIOB
#define RELAY_STATE_1_PIN           GPIO_PIN_1      /* Feedback from K1 (Phase 1) */

#define RELAY_STATE_2_PORT          GPIOA
#define RELAY_STATE_2_PIN           GPIO_PIN_6      /* Feedback from K2 (Phase 2) */

#define RELAY_STATE_3_PORT          GPIOA
#define RELAY_STATE_3_PIN           GPIO_PIN_7      /* Feedback from K3 (Phase 3) */

#define RELAY_STATE_4_PORT          GPIOB
#define RELAY_STATE_4_PIN           GPIO_PIN_0      /* Feedback from K4 (Neutral) */





/* ============================================================================== */
/* LOGIC LEVEL DEFINITIONS                             */
/* ============================================================================== */

/**
 * @brief Logical definitions for Active High/Low Hardware.
 * Change these definitions if PCB logic is inverted (e.g., Relay driven by PNP transistor).
 */

/* Relays / Outputs */
#define RELAY_ACTIVE_STATE          GPIO_PIN_SET    /* Set to RESET if Active Low */
#define RELAY_INACTIVE_STATE        GPIO_PIN_RESET

/* Inputs / Buttons (assuming pull-up/down config) */
#define INPUT_ACTIVE_STATE          GPIO_PIN_SET    /* Expected state when active */


#endif /* INC_APP_CONFIG_H_ */
