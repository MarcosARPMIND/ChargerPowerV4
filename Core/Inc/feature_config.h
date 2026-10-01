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
 * @where_used cp.h / cp.c (PP_* functions), app_manager.c (IDLE: PP read on connect).
 */
#define ENABLE_PP_SENSE             1

/* ============================================================================== */
/* RELAY VERIFICATION                                                             */
/* ============================================================================== */

/**
 * @brief Relay contact verification (weld / failed-to-close detection).
 * @values 1 = Normal behaviour. The K1/K4 sense circuit is powered and the
 *             feedback read at three points: before closing (pre-charge weld
 *             check), after closing (APP_Verify_Relays_Closed) and after
 *             opening (APP_Verify_Relays_Open). Any mismatch ->
 *             FAULT_RELAY_CONTACT. K2/K3 only with SYSTEM_PHASES == 3.
 *         0 = BENCH ONLY. None of those checks run and the sense circuit is
 *             never powered automatically; relays are still driven normally
 *             and CMD_RELAY_GET still reads them on demand. A welded contact
 *             goes UNDETECTED (connector stays live after the session) --
 *             app_manager.c emits a #warning so this can't slip into a
 *             release build unnoticed.
 * @note   The K4 feedback workaround (IGNORE_K4_FEEDBACK) is gone: its root
 *         cause was the sense circuit not being powered, now fixed, so K4 is
 *         always verified like K1.
 * @where_used app_manager.c: CHARGING entry (pre-charge check),
 *             APP_Verify_Relays_Closed(), APP_Verify_Relays_Open().
 */
#define ENABLE_RELAY_VERIFICATION   1

/* ============================================================================== */
/* TEMPORARY HARDWARE WORKAROUNDS                                                 */
/* ============================================================================== */

/**
 * @brief Bench-test override for the K2 (phase 2) weld detector.
 * @values 1 = Board_Is_K2_Closed() reads K2_WELD_TEST_PIN (PA10, ex I2C1_SDA,
 *             STM pin 32) instead of the normal RELAY_STATE_2 (PA6). Stand-in
 *             wiring while the phase-2 weld-detector hardware fix is pending.
 *         0 = Normal behaviour, K2 read from RELAY_STATE_2 (PA6) as usual.
 *             Revert to this once the hardware fix lands.
 * @where_used board_io.c: Board_Is_K2_Closed().
 */
#define K2_WELD_TEST_OVERRIDE       1

/* ============================================================================== */
/* RCD NUISANCE-TRIP MITIGATION                                                   */
/* ============================================================================== */

/**
 * @brief Suppress RCD_Fault confirmation around K1/K4 relay-verification reads.
 * @values 1 = Every point that powers the K1/K4 measurement-circuit sense line
 *             (Relay Verification Activation, APP_Verify_Relays_Closed/Open,
 *             CMD_RELAY_GET, CMD_RELAY_SET test path) revokes any RCD trip
 *             and opens a blanking window afterwards (RCD_Monitor_Blank(),
 *             rcd_monitor.c), since that activation has been observed
 *             coupling a ~60ms pulse into the RCD sensor line that is not a
 *             real residual-current event.
 *         0 = No suppression anywhere -- every RCD trip (real or induced by
 *             the sense-circuit activation) is confirmed and reaches
 *             FAULT_RCD normally. Set to 0 to bench-test whether the
 *             nuisance trip still happens without any mitigation in the way.
 * @where_used app_manager.c (the 5 blanking sites).
 */
#define RCD_BLANK_DURING_RELAY_VERIFICATION   0

#endif /* INC_FEATURE_CONFIG_H_ */
