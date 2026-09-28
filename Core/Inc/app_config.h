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
  * @version 1.1.4
  ******************************************************************************
  */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

//#include "main.h"
#include "stm32c0xx_hal.h"
#include "feature_config.h"

/* ============================================================================== */
/* SYSTEM CONFIGURATION                             */
/* ============================================================================== */

/**
 * @brief Hardware Version Identifier
 * Used for conditional compilation if distinct PCB revisions exist.
 */
#define HW_VERSION                  0.0

/* Feature toggles (SYSTEM_PHASES, ENABLE_PP_SENSE, ...) now live in
 * feature_config.h, included above -- this file stays pin-mapping only. */

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

/* --- Relay Feedback Measurement-Circuit Enable (repurposed I2C1 pins) --- */
/* K1/K4 relay feedback (RELAY_STATE_1/4) needs its sense circuit actively
 * powered before the reading is valid -- these were I2C1_SCL/SDA (also
 * labelled UART1_TX/RX above; neither peripheral is actually instantiated
 * in this build, see usart.c/i2c.c), now freed up since the temperature
 * sensor was never implemented. I2C1 is disabled (main.c) so these two
 * pins can drive the measurement-circuit enable lines instead. */
#define K1_MEAS_ENABLE_PORT         GPIOA
#define K1_MEAS_ENABLE_PIN          GPIO_PIN_9      /* was I2C1_SCL */

#define K4_MEAS_ENABLE_PORT         GPIOA
#define K4_MEAS_ENABLE_PIN          GPIO_PIN_10     /* was I2C1_SDA -- unused now, see K2_WELD_TEST_PIN below */

/* TEMPORARY bench-test wiring (K2_WELD_TEST_OVERRIDE, feature_config.h):
 * same physical pin as K4_MEAS_ENABLE_PIN above (PA10 / ex I2C1_SDA / STM
 * pin 32), reused as an input for the phase-2 (K2) weld detector while the
 * hardware fix is pending. Remove once reverted to normal RELAY_STATE_2. */
#define K2_WELD_TEST_PORT           GPIOA
#define K2_WELD_TEST_PIN            GPIO_PIN_10


/* ============================================================================== */
/* SENSORS & MEASUREMENT                              */
/* ============================================================================== */

/* --- RCD (Residual Current Device) --- */
#define RCD_ERROR_PORT              GPIOC
#define RCD_ERROR_PIN               GPIO_PIN_13     /* Input: Fault signal from RCD module */
#define RCD_DEBOUNCE_MS             15             /* Software debounce window (ms) */

#define ADE_EXTI_PORT               GPIOA           /* Input: Interrupt Request (IRQ) */
#define ADE_EXTI_PIN                GPIO_PIN_1      /* Configured via NVIC in CubeMX */

/* --- Temperature Sensor (I2C, over I2C1_SCL/SDA above) --- */
#define TEMP_SENSOR_ALERT_PORT      GPIOD
#define TEMP_SENSOR_ALERT_PIN       GPIO_PIN_1      /* Input: alert/interrupt line from the I2C temperature sensor */

/* --- Proximity Pilot (PP) Sense --- */
#define PP_SENSE_PORT               GPIOA
#define PP_SENSE_PIN                GPIO_PIN_11     /* Input: ADC1_IN11 — measures PP-to-PE resistance (cable current rating) */


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
/* CABLE LOCK ACTUATOR                              */
/* ============================================================================== */

/* --- H-Bridge Driver (Outputs) --- */
/* Drives the connector lock motor; the combination of both pins sets direction (lock/unlock) */
#define LOCK_HBRIDGE_1_PORT         GPIOC
#define LOCK_HBRIDGE_1_PIN          GPIO_PIN_6

#define LOCK_HBRIDGE_2_PORT         GPIOA
#define LOCK_HBRIDGE_2_PIN          GPIO_PIN_12     /* Shares silicon pad with PA10 on this package, see .ioc */

/* --- Lock Feedback (Input) --- */
#define CABLE_LOCK_STATE_PORT       GPIOB
#define CABLE_LOCK_STATE_PIN        GPIO_PIN_10     /* Input: set when the connector lock is engaged */


/* ============================================================================== */
/* USER INTERFACE                                   */
/* ============================================================================== */

/* --- External Buttons (Inputs) --- */
/* Reserved for future emergency-stop buttons */
#define BUTTON_1_EXT_PORT           GPIOB
#define BUTTON_1_EXT_PIN            GPIO_PIN_15

#define BUTTON_2_EXT_PORT           GPIOB
#define BUTTON_2_EXT_PIN            GPIO_PIN_11

/* --- General Purpose LED (Output) --- */
/* Usage not yet defined (status indicator TBD) */
#define GENERAL_LED_PORT            GPIOB
#define GENERAL_LED_PIN             GPIO_PIN_2



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
