/**
  ******************************************************************************
  * @file    cp.c
  * @author  Marcos Novo
  * @date    Nov 25, 2025
  * @brief   Control Pilot (CP) Signal Management Implementation.
  *
  * This file implements the logic to drive the CP PWM signal using STM32 Timers
  * and analyze the voltage feedback using ADC and DMA to determine the
  * Electric Vehicle (EV) connection state according to IEC 61851.
  *
  ******************************************************************************
  */

#include "cp.h"

/* ============================================================================== */
/* PRIVATE VARIABLES                                                              */
/* ============================================================================== */

/**
 * @brief Minimum raw ADC value found in the current buffer.
 * Used to detect the negative part of the PWM (Diode check / State F).
 * Initialized to max 12-bit value to ensure correct min-search logic.
 */
static volatile uint32_t raw_min = 4095;

/**
 * @brief Maximum raw ADC value found in the current buffer.
 * Used to detect the positive peak of the PWM (States A, B, C, D).
 */
static volatile uint32_t raw_max = 0;

/**
 * @brief Circular Buffer for ADC DMA transfers.
 * Stores raw samples from the Control Pilot line.
 * Declared volatile because it is updated by hardware (DMA) in the background.
 */
static volatile uint16_t buffer_dma_raw[ADC_BUFFER_LIMIT];


/* ============================================================================== */
/* FUNCTION IMPLEMENTATION                                                        */
/* ============================================================================== */

/**
 * @brief  Sets the Control Pilot (CP) Duty Cycle for EV Charging (IEC 61851-1).
 * * @param  duty: Value in "permille" (0 to 1000).
 * Example: 266 represents a 26.6% Duty Cycle.
 * * @note   This function uses fixed-point arithmetic to eliminate the need for
 * expensive division operations, ensuring high performance on MCUs
 * without an FPU while maintaining sub-percent precision.
 */
void CP_SetDuty(uint16_t duty)
{
    /* 1. Safety Clamp: Ensure the duty cycle never exceeds 100.0% */
    if(duty > 1000) { duty = 1000; }

    /* 2. Timer Resolution: Dynamically retrieve the total period (ARR + 1).
     * This ensures the duty cycle remains accurate even if the PWM
     * frequency or clock configuration is modified. */
    uint32_t period_ticks = TIM1->ARR + 1;

    /* 3. Fixed-Point Optimization (Bit-Shift Logic):
     * Goal: Calculate (duty * period_ticks) / 1000.
     * * The "Magic": Dividing by 1000 is mathematically equivalent to
     * multiplying by (131 / 131072). Since 131072 is 2^17, we can use
     * the '>> 17' bit-shift operator, which is significantly faster.
     * * Intermediate Cast: We cast to uint64_t to prevent overflow during
     * the multiplication (duty * period_ticks * 131), which can exceed
     * the 4.2 billion limit of a 32-bit integer.
     */
    TIM1->CCR1 = (uint32_t)( ((uint64_t)duty * period_ticks * 131) >> 17 );
}

/**
 * @brief  Forces the Control Pilot line to a static DC HIGH state (+12V).
 * @note   This is achieved by setting the Compare Register (CCR) to a value
 * higher than the Auto-Reload Register (ARR), ensuring the counter
 * never reaches the comparison value to switch the output Low.
 */
void CP_SetLine_High(void)
{
    TIM1->CCR1 = TIM1->ARR + 1;
}

/**
 * @brief  Forces the Control Pilot line to a static DC LOW state (0V / -12V).
 * @note   Setting CCR to 0 ensures the output remains low for the entire cycle.
 */
void CP_SetLine_Low(void)
{
    TIM1->CCR1 = 0;
}

/**
 * @brief  Initializes the ADC and DMA peripheral for CP monitoring.
 * @note   Performs internal ADC calibration for better accuracy before starting.
 */
void CP_ADC_Init(void)
{
    /* Run the automatic self-calibration procedure */
    if(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        /* Calibration Error Handler */
    }

    /* Start ADC in DMA mode (Circular).
     * Note: (uint32_t*) cast is required by HAL API signature, even though
     * the underlying buffer is 16-bit. The DMA configuration in CubeMX
     * (Half-Word alignment) dictates the actual transfer size. */
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_dma_raw, ADC_BUFFER_LIMIT) != HAL_OK)
    {
        /* DMA Start Error Handler */
    }
}

/**
 * @brief  Analyzes the raw DMA buffer to extract signal peaks.
 * @note   Iterates through the entire buffer to find the absolute Minimum and
 * Maximum values of the captured waveform. These globals are then used
 * by GetState functions.
 */
void CP_Raw_Max_Min(void)
{
    /* Reset search variables */
    raw_min = 4096; /* Set above 12-bit max to find any lower value */
    raw_max = 100;  /* Initial noise floor threshold */
    int i = 0;

    /* Iterate through the circular buffer */
    for(i = 0; i < ADC_BUFFER_LIMIT; i++)
    {
        uint32_t sample = buffer_dma_raw[i];

        /* Update Maximum Peak */
        if(sample > raw_max) { raw_max = buffer_dma_raw[i]; }

        /* Update Minimum Peak (Floor) */

        if(sample < raw_min) { raw_min = buffer_dma_raw[i]; }
    }
}

/**
 * @brief  Determines the Control Pilot (CP) state according to IEC 61851.
 * @note   Evaluates both Peak (Positive) and Floor (Negative) voltages to
 * distinguish between DC modes, PWM modes, and Diode Faults.
 * @return CP_State detected (A, B, C, D, E or F).
 */
CP_State CP_GetState(void)
{
    /* 1. Acquire latest signal extrema (Peak and Trough) */
    CP_Raw_Max_Min();

    /* 2. Evaluate Positive Signal Component (Potential IEC State) */
    CP_State detected_top = STATE_UNKNOWN;

    if      (IS_STATE(raw_max, STATE_A_RAW_VAL)) detected_top = STATE_A;
    else if (IS_STATE(raw_max, STATE_B_RAW_VAL)) detected_top = STATE_B;
    else if (IS_STATE(raw_max, STATE_C_RAW_VAL)) detected_top = STATE_C;
    else if (IS_STATE(raw_max, STATE_D_RAW_VAL)) detected_top = STATE_D;
    else if (IS_STATE(raw_max, ZERO_RAW_VAL))    return STATE_E; /* Short to PE detected */
    else if (IS_STATE(raw_max, NEGATIVE_RAW_VAL)) return STATE_F; /* EVSE Fault (-12V DC) */
    else detected_top = STATE_UNKNOWN; /* Signal out of valid IEC ranges */

    /* 3. Validate Negative Signal Component (PWM Integrity)
     * Checks for presence of -12V which confirms valid oscillator and diode.
     */
    bool pwm_negative_ok = IS_STATE(raw_min, NEGATIVE_RAW_VAL);

    /* 4. Check for DC Mode (Steady State)
     * Valid if signal is non-oscillating (Min approx. equal to Max).
     * DC implies PWM is inactive.
     */
    bool dc_mode_ok = (raw_min > ZERO_RAW_VAL) &&
                      (raw_min >= (raw_max - (TOLERANCE_POS + TOLERANCE_NEG)));

    /* --- State Decision Logic --- */

    /* Case 1: Valid PWM Signal Detected
     * Negative floor is within spec (-12V). Return state based on positive peak.
     */
    if (pwm_negative_ok)
    {
        return detected_top;
    }

    /* Case 2: Valid DC Signal Detected
     * Allow States A and B in DC (before PWM negotiation).
     */
    else if (dc_mode_ok)
    {
        if (detected_top == STATE_A || detected_top == STATE_B)
        {
            return detected_top;
        }

        /* Error: State C/D requires active PWM.
         * Detecting 6V/3V in DC implies an invalid vehicle sequence or hardware fault.
         */
        return STATE_F;
    }

    /* Case 3: Signal Integrity Failure — Ambiguous / Transient Reading
     *
     * Reached when the signal is oscillating (pwm_negative_ok == false) AND
     * it is not a valid DC signal (dc_mode_ok == false), but the positive peak
     * also does not match a known state range.
     *
     * This typically happens due to:
     *   - EMI noise spike captured in the DMA buffer
     *   - ADC samples captured mid-cycle during a DMA wrap-around
     *   - Signal temporarily unstable during a physical state transition
     *
     * FIX: Return STATE_UNKNOWN instead of STATE_F.
     * The caller (app_manager) applies a debounce filter and will *ignore*
     * STATE_UNKNOWN readings, preserving the last confirmed valid state.
     * Only a *real* diode fault (consistent negative reading, no valid positive)
     * will produce persistent STATE_F readings and thus trigger FAULT_CAR.
     */
    else
    {
        return STATE_UNKNOWN;
    }
}

#if ENABLE_PP_SENSE

/**
 * @brief  Reads the Proximity Pilot (PP) line voltage on PA11 (ADC1_IN11).
 * @note   ADC1 is otherwise busy running a continuous DMA scan of the CP
 *         line (Channel 0). This briefly stops that scan, takes one polled
 *         conversion on Channel 11, then restores Channel 0 + DMA so the
 *         CP reading resumes normally.
 * @return PP line voltage in millivolts (0-3300, 12-bit ADC, VREF 3.3V).
 */
uint16_t PP_Read_mV(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t raw = 0;

    HAL_ADC_Stop_DMA(&hadc1);

    /* This ADC ORs each configured channel into CHSELR instead of replacing
     * the whole sequence -- Channel 0 (CP) must be explicitly removed
     * (Rank = ADC_RANK_NONE) or it stays in the scan and gets converted
     * first, ahead of Channel 11, on every trigger. */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank    = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank    = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    /* Restore: remove Channel 11, re-enable Channel 0, resume the CP's DMA scan */
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank    = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank    = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_dma_raw, ADC_BUFFER_LIMIT);

    return (uint16_t)(((uint32_t)raw * 3300u) / 4095u);
}

/**
 * @brief  Classifies a PP line voltage reading into a cable current rating.
 * @note   Checked from the smallest resistor (highest current) down, but the
 *         non-overlapping tolerance windows make the order irrelevant.
 */
PP_Current_Rating PP_Classify_Voltage(uint16_t pp_mV)
{
    if (PP_IS_V(pp_mV, PP_V_100_OHM_mV))  { return PP_CURRENT_63A; }
    if (PP_IS_V(pp_mV, PP_V_220_OHM_mV))  { return PP_CURRENT_32A; }
    if (PP_IS_V(pp_mV, PP_V_680_OHM_mV))  { return PP_CURRENT_20A; }
    if (PP_IS_V(pp_mV, PP_V_1500_OHM_mV)) { return PP_CURRENT_13A; }

    return PP_CURRENT_UNKNOWN;
}

/**
 * @brief  Converts a PP_Current_Rating to its amp value.
 */
uint8_t PP_Rating_To_Amps(PP_Current_Rating rating)
{
    switch (rating)
    {
        case PP_CURRENT_13A: return 13u;
        case PP_CURRENT_20A: return 20u;
        case PP_CURRENT_32A: return 32u;
        case PP_CURRENT_63A: return 63u;
        default:             return 0u;
    }
}

#endif /* ENABLE_PP_SENSE */
