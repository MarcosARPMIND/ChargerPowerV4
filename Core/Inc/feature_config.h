/**
  ******************************************************************************
  * @file    feature_config.h
  * @author  Marcos Novo
  * @brief   Compile-Time Feature Flags.
  *
  * This file centralizes every #define that turns a whole feature/code path
  * on or off at compile time (as opposed to app_config.h, which centralizes
  * *pin mapping* for a given piece of hardware).
  *
  * Different products/variants built from this same firmware may not share
  * the same feature set (e.g. a fixed-current charger vs. one that reads the
  * cable's current rating). Instead of hunting through app_manager.c/cp.c
  * for scattered #if blocks, every toggle -- and the reasoning behind it --
  * lives here in one place.
  *
  * Flipping any of these requires a full rebuild (they gate #if blocks
  * resolved by the preprocessor, not runtime checks).
  ******************************************************************************
  */

#ifndef INC_FEATURE_CONFIG_H_
#define INC_FEATURE_CONFIG_H_

/* ============================================================================== */
/* GRID CONFIGURATION                                                             */
/* ============================================================================== */

/**
 * @brief Number of AC phases this build controls and monitors.
 * @values 1 = Single-phase (only K1/phase-1 and K4/neutral are actuated and
 *             checked for relay weld/contact faults; lines 2/3 are still read
 *             from their ADE7953 for information, but not safety-checked).
 *         3 = Three-phase (K1, K2, K3 and K4 all actuated and checked).
 * @where_used app_manager.c: relay verification (APP_Verify_Relays_Open/Closed),
 *             CMD_RELAY_GET reporting, and APP_Safety_Check_Electrical().
 */
#define SYSTEM_PHASES               1

/* ============================================================================== */
/* CABLE / CONNECTOR SENSING                                                     */
/* ============================================================================== */

/**
 * @brief Proximity Pilot (PP) cable current-rating sense.
 * @values 1 = This hardware variant has a PP sensing circuit (PA11 / ADC1_IN11).
 *             Reads the cable's PP-to-PE coding resistor before a charging
 *             session, to cap the offered current to what the cable supports
 *             (1500ohm/13A, 680ohm/20A, 220ohm/32A, 100ohm/63A three-phase).
 *         0 = No PP sensing circuit on this variant -- the maximum current is
 *             fixed/hardcoded (e.g. via CMD_SET_CURRENT or a hardware constant)
 *             and this mechanism is skipped entirely; removes PP_Read_mV(),
 *             PP_Classify_Voltage() and PP_Rating_To_Amps() from the build.
 * @where_used cp.h / cp.c (PP_* functions), main.c (bring-up test readout).
 */
#define ENABLE_PP_SENSE             0

/* ============================================================================== */
/* TEMPORARY HARDWARE WORKAROUNDS                                                 */
/* ============================================================================== */

/**
 * @brief TEMPORARY: ignore the K4 relay feedback pin (PB0 / RELAY_STATE_4).
 * @values 1 = PB0 reads permanently HIGH regardless of the actual relay
 *             state (suspected hardware fault, same symptom observed on
 *             PA6/K2). K4 is excluded from the pre-charge weld check and
 *             from APP_Verify_Relays_Closed()/Open() -- the same way K2/K3
 *             are already excluded under SYSTEM_PHASES=1. This does NOT
 *             stop K4 from being driven, only stops trusting its feedback.
 *         0 = Normal behaviour, K4 feedback trusted like any other relay.
 * @where_used app_manager.c: CHARGING entry weld check, APP_Verify_Relays_*().
 * @todo   Set back to 0 once PB0 is fixed/confirmed on hardware.
 */
#define IGNORE_K4_FEEDBACK          0

#endif /* INC_FEATURE_CONFIG_H_ */
