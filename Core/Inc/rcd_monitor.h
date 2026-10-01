/*
 * rcd_monitor.h
 *
 *  Created on: Sep 29, 2026
 *      Author: Marcos Novo
 *
 *  RCD module fault output (PC13: open-collector, idle HIGH, tripped LOW),
 *  sampled every 1 ms from SysTick -- independent of how long the main loop
 *  happens to be blocked.
 */

#ifndef INC_RCD_MONITOR_H_
#define INC_RCD_MONITOR_H_

/* --- Standard Includes --- */
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  RCD line history since RCD_Monitor_Arm(), for diagnostics
 *         (see CMD_GET_RCD_DIAG).
 */
typedef struct {
	uint16_t trip_width_ms;    /**< LOW width of the last confirmed trip, ms -- still growing while trip_ongoing */
	bool     trip_ongoing;     /**< That trip's pulse is still LOW right now */
	bool     trip_recorded;    /**< At least one confirmed trip since boot (trip_width_ms is valid) */
	uint16_t rejected_count;   /**< LOW pulses NOT confirmed as a trip: shorter than RCD_DEBOUNCE_MS, or blanked */
	uint16_t rejected_max_ms;  /**< Longest of those rejected pulses, ms */
} RCD_Diag_t;

/* =================================================================================
 * PUBLIC FUNCTIONS
 * ================================================================================= */

void RCD_Monitor_Tick_1ms(void);
void RCD_Monitor_Arm(void);
void RCD_Monitor_Blank(uint32_t window_ms);
void RCD_Monitor_Get_Diag(RCD_Diag_t *out);

#endif /* INC_RCD_MONITOR_H_ */
