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
#include "app_config.h"
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

#if ENABLE_PP_SENSE

/**
 * @brief  Reads the Proximity Pilot (PP) line voltage (PA11 / ADC1_IN11).
 * Used to infer the PP-to-PE resistance (cable current rating coding).
 * @note   Briefly repurposes ADC1 for one polled conversion, then restores
 *         the continuous Control Pilot DMA scan on Channel 0. Call only
 *         occasionally (not from a tight loop) -- not meant for streaming.
 * @return PP line voltage in millivolts (0-3300).
 */
uint16_t PP_Read_mV(void);

/* ============================================================================== */
/* PROXIMITY PILOT (PP) RESISTANCE CODING (IEC 62196-1)                          */
/* ============================================================================== */

/**
 * @defgroup PP_Voltage_Coding Theoretical PP line voltage per coding resistor (mV)
 * Measured at PA11 through this board's PP sense divider.
 * @{
 */
#define PP_V_1500_OHM_mV        2513u   /**< 1500 ohm -> 13A */
#define PP_V_680_OHM_mV         1951u   /**< 680 ohm  -> 20A */
#define PP_V_220_OHM_mV         1052u   /**< 220 ohm  -> 32A */
#define PP_V_100_OHM_mV          578u   /**< 100 ohm  -> 63A three-phase (70A single-phase not handled) */
/** @} */

#define PP_V_TOLERANCE_mV        150u   /**< +/- hysteresis window around each nominal value,
                                              well inside the ~474 mV smallest gap between bands */

#define PP_IS_V(reading, target) \
        ( (reading) >= ((target) - PP_V_TOLERANCE_mV) && (reading) <= ((target) + PP_V_TOLERANCE_mV) )

/**
 * @brief  Cable current rating decoded from the PP line voltage.
 */
typedef enum {
    PP_CURRENT_13A = 0,   /**< ~2513 mV (1500 ohm) */
    PP_CURRENT_20A,       /**< ~1951 mV (680 ohm)  */
    PP_CURRENT_32A,       /**< ~1052 mV (220 ohm)  */
    PP_CURRENT_63A,       /**< ~578 mV (100 ohm), three-phase */
    PP_CURRENT_UNKNOWN    /**< No cable / voltage outside all known bands */
} PP_Current_Rating;

/**
 * @brief  Classifies a PP line voltage reading into a cable current rating.
 * @param  pp_mV Measured PP voltage in millivolts (see PP_Read_mV()).
 * @return PP_Current_Rating (PP_CURRENT_UNKNOWN if it matches no known band).
 */
PP_Current_Rating PP_Classify_Voltage(uint16_t pp_mV);

/**
 * @brief  Converts a PP_Current_Rating to its amp value.
 * @return Amps (13/20/32/63), or 0 for PP_CURRENT_UNKNOWN.
 */
uint8_t PP_Rating_To_Amps(PP_Current_Rating rating);

#endif /* ENABLE_PP_SENSE */

#endif /* INC_CP_H_ */
