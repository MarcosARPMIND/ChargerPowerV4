/*
 * rcd_monitor.c
 *
 *  Created on: Sep 29, 2026
 *      Author: Marcos Novo
 */

#include "rcd_monitor.h"
#include "main.h"

/* Written by RCD_Monitor_Tick_1ms() (SysTick ISR); the main-loop functions
 * below only touch them inside a critical section. */
static bool     armed           = false;
static bool     blank_active    = false;
static uint32_t blank_until     = 0;      /* HAL tick the blanking window ends at */
static uint16_t low_ms          = 0;      /* Width so far of the LOW pulse in progress, 0 = line HIGH */
static bool     confirmed       = false;  /* Pulse in progress has been confirmed as a trip */
static uint16_t last_trip_ms    = 0;      /* Final width of the last confirmed trip that already ended */
static bool     trip_recorded   = false;
static uint16_t rejected_count  = 0;
static uint16_t rejected_max_ms = 0;

/**
 * @brief  Samples the RCD fault line. Call every 1 ms (SysTick_Handler).
 * @note   A trip is confirmed once the line has been LOW for RCD_DEBOUNCE_MS
 *         *continuously* -- any HIGH sample in between restarts the count --
 *         so EMI spikes are rejected, and no pulse longer than that can be
 *         missed, however long the main loop is blocked (relay verification,
 *         cable lock, ...). Confirmation only sets RCD_Fault; APP_MAIN()
 *         does the actual shutdown.
 * @note   Every LOW pulse is also measured start to end, confirmed or not
 *         (see RCD_Monitor_Get_Diag()).
 */
void RCD_Monitor_Tick_1ms(void){
	if (!armed) {
		return;
	}

	if (blank_active && (int32_t)(HAL_GetTick() - blank_until) >= 0) {
		blank_active = false;
	}

	if (HAL_GPIO_ReadPin(RCD_ERROR_PORT, RCD_ERROR_PIN) == GPIO_PIN_RESET) {
		if (low_ms < UINT16_MAX) {
			low_ms++;
		}
		if (!confirmed && !blank_active && low_ms >= RCD_DEBOUNCE_MS) {
			confirmed = true;
			RCD_Fault = 1;
		}
	}
	else if (low_ms > 0) {
		/* Pulse just ended: file it as a trip or as a rejected pulse */
		if (confirmed) {
			last_trip_ms  = low_ms;
			trip_recorded = true;
		} else {
			if (rejected_count < UINT16_MAX) {
				rejected_count++;
			}
			if (low_ms > rejected_max_ms) {
				rejected_max_ms = low_ms;
			}
		}
		low_ms    = 0;
		confirmed = false;
	}
}

/**
 * @brief  Starts confirming trips. Call once at the end of boot.
 * @note   Nothing is measured or confirmed before this (power-on transients).
 *         A line that is already LOW here is NOT ignored: it starts a fresh
 *         pulse and is confirmed RCD_DEBOUNCE_MS later like any other.
 */
void RCD_Monitor_Arm(void){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	low_ms    = 0;
	confirmed = false;
	RCD_Fault = 0;
	armed     = true;
	__set_PRIMASK(primask);
}

/**
 * @brief  Suppresses trip confirmation for the next window_ms.
 * @note   For RCD_BLANK_DURING_RELAY_VERIFICATION (feature_config.h): callers
 *         blank right after the activity that can induce a nuisance pulse,
 *         so this also revokes a trip already confirmed on the pulse in
 *         progress (clearing RCD_Fault). A line still LOW once the window
 *         ends (persistent -> real) is confirmed then.
 */
void RCD_Monitor_Blank(uint32_t window_ms){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	blank_until  = HAL_GetTick() + window_ms;
	blank_active = true;
	confirmed    = false;
	RCD_Fault    = 0;
	__set_PRIMASK(primask);
}

/**
 * @brief  Copies the RCD line history (consistent snapshot, safe vs the ISR).
 * @param  out Destination; while a confirmed trip is still LOW, trip_width_ms
 *         is the time elapsed so far and trip_ongoing is set.
 */
void RCD_Monitor_Get_Diag(RCD_Diag_t *out){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	out->trip_ongoing    = confirmed;
	out->trip_width_ms   = confirmed ? low_ms : last_trip_ms;
	out->trip_recorded   = trip_recorded || confirmed;
	out->rejected_count  = rejected_count;
	out->rejected_max_ms = rejected_max_ms;
	__set_PRIMASK(primask);
}
