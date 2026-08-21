/**
  ******************************************************************************
  * @file    cp.h
  * @author  Marcos Novo
  * @date    Nov 24, 2025
  * @brief   Control Pilot (CP) Signaling Module Header.
  *
  * This file contains the definitions and function prototypes for managing the
  * IEC 61851-1 Control Pilot signal. It handles PWM generation (via Timer)
  * and voltage sensing (via ADC/DMA) to detect EV states.
  *
  ******************************************************************************
  */

#ifndef INC_CP_H_
#define INC_CP_H_

/* ============================================================================== */
/* INCLUDES                                                                       */
/* ============================================================================== */
#include "stm32c0xx_hal.h"
#include "tim.h"
#include "adc.h"
#include "dma.h"
#include <stdbool.h>

/* ============================================================================== */
/* ADC BUFFER SETTINGS                                                            */
/* ============================================================================== */

/**
 * @brief ADC DMA Circular Buffer Size.
 * Defines the number of samples to capture for signal analysis.
 * @note  With a sampling rate of ~14us, 100 samples cover ~1.4ms, ensuring
 * capture of at least one full PWM period (1ms @ 1kHz).
 */
#define ADC_BUFFER_LIMIT    100


/* ============================================================================== */
/* IEC 61851 VOLTAGE THRESHOLDS (ADC RAW)                                         */
/* ============================================================================== */

/**
 * @defgroup CP_Thresholds ADC Raw Thresholds
 * @brief    Thresholds for 12-bit ADC (0-4095) corresponding to IEC 61851 voltages.
 * @note     Values depend on PCB Resistor Divider and VREF (3.3V).
 * Verify these values against the hardware schematic.
 * @{
 */

/* State A: +12V (Not Connected) */
#define STATE_A_RAW_VAL     3572    /* Approx 3.3V at pin */

/* State B: +9V (Connected, EV Ready) */
#define STATE_B_RAW_VAL     3213    /* Approx 2.7V range */

/* State C: +6V (Charging, EV Ready) */
#define STATE_C_RAW_VAL     2855    /* Approx 1.8V range */

/* State D: +3V (Ventilation Required) */
#define STATE_D_RAW_VAL     2497    /* Approx 0.9V range */

/* State E: 0V (Error / No Power) */
#define ZERO_RAW_VAL        2139    /* Near 0V ground */

/* State F: -12V (Error / Diode Fault) */
/* Note: STM32 cannot read negative voltage directly.
   This assumes a clamping circuit or level shift is in place. */
#define NEGATIVE_RAW_VAL    705

/** @} */

/**
 * @defgroup CP_Hysteresis Signal Hysteresis
 * @brief    Tolerance windows to prevent state flickering due to noise.
 * @{
 */
#define TOLERANCE_POS       150     /* Upper/Lower bound for positive peaks */
#define TOLERANCE_NEG       300     /* Upper/Lower bound for negative floor */
/** @} */


#define IS_STATE(reading, target) \
		( (reading >= (target - TOLERANCE_NEG) ) && (reading <= (target + TOLERANCE_POS) ))


/* ============================================================================== */
/* DATA TYPES                                                                     */
/* ============================================================================== */

/**
 * @brief  IEC 61851-1 Charging States.
 * Determines the current status of the EV connection.
 */
typedef enum {
    STATE_A,        /**< +12V: Idle / Not Connected */
    STATE_B,        /**< +9V:  Connected, waiting for EV */
    STATE_C,        /**< +6V:  Charging (PWM Active) */
    STATE_D,        /**< +3V:  Charging with Ventilation (Rare) */
    STATE_E,        /**< 0V:   Error / Short to Earth */
    STATE_F,        /**<  Error / CP Line Fault */
    STATE_UNKNOWN   /**< Signal undefined or unstable */
} CP_State;


/* ============================================================================== */
/* PUBLIC FUNCTION PROTOTYPES                                                     */
/* ============================================================================== */

/**
 * @brief  Initializes the Control Pilot hardware.
 * Starts ADC calibration and sets up DMA in Circular Mode.
 * @note   Must be called before any other CP function.
 */
void CP_ADC_Init(void);

/**
 * @brief  Processes the ADC DMA buffer to find Peak High and Peak Low values.
 * Updates internal static variables with the latest voltage profile.
 */
void CP_Raw_Max_Min(void);

/**
 * @brief  Determines the current IEC 61851 state based on voltage peaks.
 * @retval CP_State The detected state (A, B, C, D, E, or F).
 */
CP_State CP_GetState(void);

/**
 * @brief  Sets the PWM Duty Cycle for the Control Pilot.
 * @param  duty Percentage (0 to 100).
 * - 100%: Force High (State A/B transition)
 * - 53.3% to 6%: Current advertisement (according to formula)
 * - 5%: Digital communication required
 * - 0%: Force Low (State E/F)
 */
void CP_SetDuty(uint16_t duty);

/**
 * @brief  Forces the CP Line to DC HIGH (+12V).
 * Used when the charger is IDLE (State A) or suspending PWM.
 */
void CP_SetLine_High(void);

/**
 * @brief  Forces the CP Line to DC LOW (0V / -12V).
 * Used for Error handling or Reset.
 */
void CP_SetLine_Low(void);

#endif /* INC_CP_H_ */
