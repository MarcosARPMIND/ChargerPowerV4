/*
 * app_manager.c
 *
 *  Created on: Dec 12, 2025
 *      Author: Marcos Novo
 */


#include "app_manager.h"


static uint16_t counter = 0;

// File-scope state: accessible to all functions in this translation unit,
// but invisible outside app_manager.c (encapsulation via 'static').
static STATE_MACHINE g_nextState = IDLE;
static CP_State      g_CP_STATE  = STATE_A;

static CP_State      g_CP_LAST_STATE = STATE_A;

/* ── CP State Debounce Filter ──────────────────────────────────────────────
 * Problem: CP_GetState() is called every main-loop cycle (~1-2 ms). A single
 * corrupted ADC sample (EMI spike, DMA wrap-around mid-buffer) can produce a
 * momentary STATE_F or STATE_UNKNOWN, which would fire FAULT_CAR immediately.
 *
 * Solution: Require CP_DEBOUNCE_CYCLES consecutive *identical* readings before
 * accepting a new state. STATE_UNKNOWN readings are always discarded — the last
 * confirmed valid state is preserved instead.
 *
 * Timing: 3 cycles × ~2 ms loop ≈ 6 ms max latency — well within the 300 ms
 * IEC 61851 transition window.
 * ─────────────────────────────────────────────────────────────────────────── */
#define CP_DEBOUNCE_CYCLES  3u   /**< Consecutive identical reads to confirm a new CP state */

static CP_State g_CP_CONFIRMED      = STATE_A; /**< Last debounce-verified CP state            */
static CP_State g_CP_CANDIDATE      = STATE_A; /**< Current candidate being counted             */
static uint8_t  g_cp_debounce_count = 0u;      /**< How many consecutive cycles seen candidate  */

static bool     g_waiting_state_ack = false;
static uint32_t g_state_notify_tick = 0;
static uint8_t  g_state_notify_retries = 0;

/* Deferred weld-check variables for CHARGING_COMPLETE state */
static uint32_t charging_complete_entry_tick = 0;
static bool     relay_open_checked = false;

/* IEC 61851 Table A.5: Fault states must persist >= 300ms before recovery */
static uint32_t fault_entry_tick = 0;

/* Cable current rating detected via the PP coding resistor on connect
 * (see case IDLE below). 0 = unknown / no cable currently connected. */
static uint8_t g_cable_max_amps = 0;

// Private Variables for Energy
Energy_Data_t g_energy_line1 = {0};
Energy_Data_t g_energy_line2 = {0};
Energy_Data_t g_energy_line3 = {0};

// Device Status (Error tracking)
DeviceStatus_t g_device_status = {0};

// Private variables for Comms Task
/**
 * @brief Circular Buffer Decoupling
 * Used to "linearize" data received from the DMA circular buffer.
 * This ensures that packets larger than the remaining DMA space or split across
 * multiple DMA interrupts are assembled correctly before parsing.
 */
static uint8_t work_buffer[WORK_BUFFER_SIZE];
static uint16_t work_buffer_len = 0;
//static uint8_t tx_buffer[10]; // Small buffer for responses

static volatile uint16_t duty 		= 0;
static volatile bool autorization 	= false;
static volatile bool session_active	= false;

/* Set by CMD_SET_CURRENT when 'duty' changes; only APP_MAIN() acts on it
 * (see the check right before "Update persistent state" below), so that
 * every CP hardware write stays inside the state machine's own actuation
 * section instead of being called directly from the comms dispatcher. */
static volatile bool g_duty_dirty = false;

// Public Variables (External)
volatile uint8_t flag_read_energy;
volatile uint8_t flag_read_voltage;

volatile uint8_t RCD_Fault = 0;

/* Owned by main.c (set by the EXTI ISR, confirmed by the main-loop debounce).
 * Must be cleared here too whenever we blank a stale RCD trip, or a pending
 * EXTI event from before the blanking survives and re-confirms RCD_Fault on
 * the very next main-loop pass -- silently undoing the blanking. */
extern volatile bool rcd_pending_check;

/* Owned by main.c -- extends RCD blanking forward in time (see the debounce
 * block there). The RCD module's fault pulse can start with some latency
 * relative to the K1/K4 sense-circuit activation edge and lasts ~60ms, so a
 * one-shot clear right when a verification window closes isn't enough --
 * this keeps suppressing confirmation until well past that window. */
extern volatile uint32_t rcd_blank_until_tick;
#define RCD_BLANK_MARGIN_MS  200u  /**< Comfortably > the observed ~60ms RCD pulse width */

#if ENABLE_PP_SENSE
/* Owned by main.c -- debug mirror of the last PP reading, see the IDLE
 * case below. Inspect in the debugger (Watch / Live Expressions) to
 * calibrate PP_V_*_OHM_mV (cp.h) against this board's actual PP circuit. */
extern volatile uint16_t test_pp_mV;
extern volatile uint8_t  test_pp_amps;
#endif


// Private Prototypes
static void APP_Dispatch_Command(RS485_Frame_t* frame);
static void APP_Safe_Shutdown(void);
static void APP_Safety_Check_Electrical(void);
static void APP_Update_Status_LED(STATE_MACHINE state);

/**
 * @brief  Main State Machine Handler for EVSE.
 * @note   Implements control logic according to IEC 61851-1.
 * Hardware actuation is decoupled from state decision logic to prevent
 * relay chattering and redundant bus operations.
 * @param  currentState Pointer to the persistent system state variable.
 */
void APP_MAIN(STATE_MACHINE *currentState) {



    /* 1. Cache current state and acquire inputs */
    g_nextState = *currentState;     // snapshot — worked on locally this cycle

    /* ── CP State Acquisition with Debounce Filter ─────────────────────────
     * Raw reading from ADC/DMA buffer. May occasionally return STATE_UNKNOWN
     * (incoherent/transient sample) or a spurious STATE_F (EMI noise).
     * The debounce logic below ensures only persistent, stable readings update
     * the confirmed state used by the rest of the state machine.
     * ────────────────────────────────────────────────────────────────────── */
    CP_State raw_cp = CP_GetState();

    /* Expected transient: once we've killed the PWM to ask the EV to open S2
     * (CHARGING with autorization/session already false, see the CHARGING
     * case below), CP reads the still-connected state-C divider without any
     * PWM -- which CP_GetState() reports as STATE_F by convention. This is
     * neither a fault nor news: treat it exactly like STATE_UNKNOWN (discard,
     * keep the last confirmed state) so it's never confirmed, never sent to
     * the Master via CMD_STATE_NOTIFY, and the FSM simply waits for the real
     * C->B transition once the EV actually reacts. A genuine CP fault while
     * actually charging (autorization/session both still true) is untouched. */
    if (*currentState == CHARGING && (!autorization || !session_active) && raw_cp == STATE_F)
    {
        raw_cp = STATE_UNKNOWN;
    }

    /* Discard STATE_UNKNOWN entirely — it means the sample was unreadable.
     * Keep the last confirmed state; do not even start a new candidate count. */
    if (raw_cp != STATE_UNKNOWN)
    {
        if (raw_cp == g_CP_CONFIRMED)
        {
            /* Signal is stable at the currently confirmed state.             *
             * Reset candidate tracking so any future change starts fresh.   */
            g_CP_CANDIDATE      = raw_cp;
            g_cp_debounce_count = 0;
        }
        else if (raw_cp == g_CP_CANDIDATE)
        {
            /* Same candidate seen again — increment persistence counter.     *
             * Once we reach CP_DEBOUNCE_CYCLES, promote it to confirmed.    */
            g_cp_debounce_count++;
            if (g_cp_debounce_count >= CP_DEBOUNCE_CYCLES)
            {
                g_CP_CONFIRMED      = raw_cp;
                g_cp_debounce_count = 0;
            }
        }
        else
        {
            /* New, different candidate seen for the first time.             *
             * Start fresh — it must persist before we trust it.            */
            g_CP_CANDIDATE      = raw_cp;
            g_cp_debounce_count = 1;
        }
    }
    /* else: STATE_UNKNOWN → silently discard, g_CP_CONFIRMED unchanged     */

    g_CP_STATE = g_CP_CONFIRMED;  /* Use ONLY the debounce-verified state from here on */

    /* TODO: Execute Grid Voltage and RCD monitoring routines here */

    APP_Safety_Check_Electrical();

    /* ── Safety Overrides (highest priority) ── */
    if(RCD_Fault){
    	RCD_Fault = 0;
    	g_device_status.active_faults |= FAULT_BIT_RCD;
    	g_nextState = FAULT_RCD;
    	goto actuate;
    }

    /* Check grid/overcurrent faults (set by APP_Safety_Check_Electrical) */
    if (g_device_status.active_faults & (FAULT_BIT_OVERVOLT | FAULT_BIT_UNDERVOLT | FAULT_BIT_OVERCURR)) {
        g_nextState = FAULT_GRID;
        goto actuate;
    }



    /* -----------------------------------------------------------
     * SECTION 1: TRANSITION LOGIC (DECISION MAKING)
     * Determines the next state based on inputs.
     * No hardware outputs should be modified in this switch.
     * ----------------------------------------------------------- */
    switch (*currentState) {
        /* NOTE: use g_nextState / g_CP_STATE throughout this function */



        case IDLE:
            /* State A -> State B: Vehicle detected */


        	session_active	= false;
        	autorization 	= false;

            if (g_CP_STATE == STATE_A) {
                CP_SetLine_High();
            }


            if (g_CP_STATE == STATE_B) { //EV present
                /* Cable just connected — read its PP coding resistor once to
                 * cap the current we may offer this session (see CMD_SET_CURRENT). */
#if ENABLE_PP_SENSE
                {
                    uint16_t pp_mV = PP_Read_mV();
                    PP_Current_Rating rating = PP_Classify_Voltage(pp_mV);
                    /* Debug mirror (see main.c) -- inspect in the debugger
                     * (Watch / Live Expressions) to calibrate PP_V_*_OHM_mV
                     * against this board's actual PP pull-up/divider. */
                    test_pp_mV = pp_mV;
                    test_pp_amps = PP_Rating_To_Amps(rating);
                    if (rating == PP_CURRENT_UNKNOWN) {
                        g_nextState = FAULT_CABLE;
                        break;
                    }
                    g_cable_max_amps = PP_Rating_To_Amps(rating);
                }
#else
                g_cable_max_amps = CURRENT_LIMIT_HIGH; /* no PP circuit: cable imposes no extra cap */
#endif
                g_nextState = READY;
                break;
            }

            else if (g_CP_STATE == STATE_A) {
                g_nextState = IDLE;
                break;
            }
            /* IEC 61851: State E — CP short to PE */
            else if (g_CP_STATE == STATE_E) {
                g_nextState = FAULT_CP_SHORT;
                break;
            }
            else{
            	g_nextState = IDLE;
            	break;
            }


        case READY:
            /* State B -> State C: EV requests charging (S2 closed) */

            if (g_CP_STATE == STATE_C && autorization && session_active && duty != 0) {
                g_nextState = CHARGING;
            }
            else if (g_CP_STATE == STATE_C && autorization && !session_active) {
                g_nextState = READY;
            }
            /* State B -> State A: Cable disconnected by user */
            else if (g_CP_STATE == STATE_A) {
                g_nextState = IDLE;
            }
            /* CP Error detected */
            else if (g_CP_STATE == STATE_F) {
                g_nextState = FAULT_CAR;
            }
            /* IEC 61851: State E — CP short to PE */
            else if (g_CP_STATE == STATE_E) {
                g_nextState = FAULT_CP_SHORT;
            }
            break;

        case CHARGING:
            /* State C -> State B: EV stopped charging (S2 open) */
            if (g_CP_STATE == STATE_B) {
                g_nextState = CHARGING_COMPLETE;
            }
            /* State C -> State A: Emergency disconnection (Hot unplug) */
            else if (g_CP_STATE == STATE_A) {
                //send error message
                g_nextState = IDLE;
            }

            else if (g_CP_STATE == STATE_C && (!autorization || !session_active) ) {
                /* If authorisation is lost or session ends during charge:
                 * Turn off PWM to tell EV power is no longer available.
                 * DO NOT transition out yet! Wait for the EV to gracefully
                 * drop load and open S2 (moving the CP to STATE_B).
                 * The transition to CHARGING_COMPLETE will happen naturally
                 * when g_CP_STATE == STATE_B above. */
                duty = 0;
                g_duty_dirty = true;
            }

            /* CP Error detected -- but NOT while we're mid-graceful-stop
             * (see the branch above): once PWM is killed to ask the EV to
             * open S2, CP reads the still-connected state-C divider without
             * any PWM, which our own CP_GetState() reports as STATE_F by
             * definition (DC + 6V-equivalent = invalid per IEC). That's the
             * same transient CHARGING_COMPLETE already tolerates by never
             * checking F at all -- here we only ignore it during this one
             * window (autorization/session already false), a real CP fault
             * while actually charging (both still true) still faults as before. */
            else if (g_CP_STATE == STATE_F && autorization && session_active) {
                g_nextState = FAULT_CAR;
            }
            /* IEC 61851: State E — CP short to PE */
            else if (g_CP_STATE == STATE_E) {
                g_nextState = FAULT_CP_SHORT;
            }
            break;

        case CHARGING_COMPLETE:
            /* State B -> State A: Cable disconnected */
            if (g_CP_STATE == STATE_A) {
                g_nextState = IDLE;
            }
            /* IEC 61851 Fig A.3: Allow EV to resume charging (B<->C cycling).
             * If Master starts a new session while EV is still connected,
             * transition back to READY so B2->C2 can occur again. */
            else if (g_CP_STATE == STATE_B && session_active && autorization) {
                g_nextState = READY;
            }
            /* Note: STATE_F is NOT checked here intentionally.
             * When PWM is killed (DC +12V), the car takes time to open S2.
             * During that window, CP reads +6V DC -> CP_GetState returns STATE_F
             * (State C without PWM = invalid per IEC). This is a transient
             * false alarm. Contactors are already open, so it's safe to wait
             * for the car to release and return to STATE_A naturally.
             */
            break;

        /* Fault Handling States — each fault has its own recovery condition */
        case FAULT_CAR:
            session_active = false;
            /* Recover when backend clears the fault via RS485 */
            /* IEC 61851 Table A.5: minimum 300ms dwell before recovery */
            if (((g_device_status.active_faults & FAULT_BIT_CP_ERROR) == 0 || g_CP_STATE == STATE_A)
                && (HAL_GetTick() - fault_entry_tick) >= 300) {
                g_device_status.active_faults &= ~FAULT_BIT_CP_ERROR;
                g_nextState = IDLE;
            }
            break;

        case FAULT_RCD:
            session_active = false;
            /* Requires: CMD_CLEAR_FAULTS via RS485 */
            /* IEC 61851 Table A.5: minimum 300ms dwell before recovery */
            if ((g_device_status.active_faults & FAULT_BIT_RCD) == 0
                && (HAL_GetTick() - fault_entry_tick) >= 300) {
                g_nextState = IDLE;
            }
            break;

        case FAULT_RELAY_CONTACT:
            session_active = false;
            /* Requires maintenance — only CMD_CLEAR_FAULTS from Master */
            /* IEC 61851 Table A.5: minimum 300ms dwell before recovery */
            if ((g_device_status.active_faults & FAULT_BIT_RELAY) == 0
                && (HAL_GetTick() - fault_entry_tick) >= 300) {
                g_nextState = IDLE;
            }
            break;

        case FAULT_GRID:
            session_active = false;
            /* Recovers when grid fault bits are cleared */
            /* IEC 61851 Table A.5: minimum 300ms dwell before recovery */
            if ((g_device_status.active_faults & (FAULT_BIT_OVERVOLT | FAULT_BIT_UNDERVOLT | FAULT_BIT_OVERCURR)) == 0
                && (HAL_GetTick() - fault_entry_tick) >= 300) {
                g_nextState = IDLE;
            }
            break;

        case FAULT_CP_SHORT:
            session_active = false;
            /* Recover when CP returns to normal (State A) or fault cleared */
            /* IEC 61851 Table A.5: minimum 300ms dwell before recovery */
            if (((g_device_status.active_faults & FAULT_BIT_CP_ERROR) == 0 || g_CP_STATE == STATE_A)
                && (HAL_GetTick() - fault_entry_tick) >= 300) {
            	g_device_status.active_faults &= ~FAULT_BIT_CP_ERROR;
                g_nextState = IDLE;
            }
            break;

        case FAULT_CABLE:
            session_active = false;
            /* Recover when the fault is cleared (CMD_CLEAR_FAULTS) or the
             * cable is unplugged (CP_STATE == STATE_A) -- either way, the
             * next connect attempt re-reads the PP resistor from scratch. */
            if (((g_device_status.active_faults & FAULT_BIT_CABLE) == 0 || g_CP_STATE == STATE_A)
                && (HAL_GetTick() - fault_entry_tick) >= 300) {
                g_device_status.active_faults &= ~FAULT_BIT_CABLE;
                g_nextState = IDLE;
            }
            break;

        default:
            g_nextState = IDLE; /* Failsafe */
            break;
    }

    /* CP state-change notify -- sent after the switch above (not before it)
     * so that a cable just connected (IDLE case, STATE_B) has already had
     * its PP resistor read and g_cable_max_amps updated by the time this
     * builds the frame; sending it earlier reported the stale pre-connect
     * value (0/"unknown") one cycle early. */
    if(g_CP_LAST_STATE != g_CP_STATE){

    	g_CP_LAST_STATE = g_CP_STATE;

    	RS485_Frame_t Transmit_frame;
    	memset(&Transmit_frame, 0, sizeof(Transmit_frame));
    	Transmit_frame.dest_ID = ID_MASTER;
    	Transmit_frame.cmd = CMD_STATE_NOTIFY;
    	Transmit_frame.data_RX[0] = (uint8_t)g_CP_STATE;
    	Transmit_frame.data_RX[1] = g_cable_max_amps; /* PP-derived cable rating, 0 if unknown/no cable */
    	Transmit_frame.len = 2;
    	APP_RS485_Send_Message(&Transmit_frame);

    	g_waiting_state_ack = true;
    	g_state_notify_tick = HAL_GetTick();
    	g_state_notify_retries  = 0;

    }

    /* -----------------------------------------------------------
     * SECTION 2: HARDWARE ACTUATION (ENTRY/EXIT ACTIONS)
     * Only executes when a state transition occurs.
     * Eliminates redundancy and relay chattering.
     * ----------------------------------------------------------- */

   actuate:
    if (g_nextState != *currentState) {

        /* A. EXIT ACTIONS (Cleanup for the state we are leaving) */
        /* Critical Safety: Ensure relays are opened immediately when leaving CHARGING */
        if (*currentState == CHARGING) {
            APP_Safe_Shutdown();

            /* Verify relays actually opened (detect welded contacts)
             * The verification function dynamically polls for up to 300ms
             * to allow AC optocouplers to fully discharge. */
            if (!APP_Verify_Relays_Open(&g_device_status)) {
                g_nextState = FAULT_RELAY_CONTACT;
                /* Note: Don't return — let FAULT entry actions run below */
            }
            /* Note: RCD EMI blanking now lives inside APP_Verify_Relays_Open()
             * itself, right after its relay-measurement circuit window closes. */

            /* Always unlock, even if a relay is welded: CP already told the EV
             * to stop drawing current, and a permanently-stuck cable is worse
             * than the residual risk of unplugging while welded. */
            Board_Unlock_Cable();
        }

        /* B. ENTRY ACTIONS (Configuration for the new state) */
        switch (g_nextState) {
            case IDLE:
                /* Reset Control Pilot to static DC +12V */
                CP_SetLine_High();
                autorization   = false;
                session_active = false;
                g_cable_max_amps = 0; /* cable removed (or never connected) — unknown again */
                duty = 0; /* Forget the last session's current -- a fresh connection
                           * must wait for a new CMD_SET_CURRENT, not inherit whatever
                           * was last requested (READY's entry action applies 'duty'
                           * immediately if it's still nonzero). */
                break;

            case READY:
                /* Enable PWM Oscillator (1kHz) if duty is set by the Master, otherwise hold CP at +9V DC */
                if (duty > 0) {
                    CP_SetDuty(duty);
                } else {
                    CP_SetLine_High();
                }
                break;

            case CHARGING:
                /* Relay Verification Activation: Before closing relays, verify they are
                 * currently OPEN. If any reads as closed before we command them,
                 * it means a welded contact from a previous session.
                 * Settle first -- same reasoning as APP_Verify_Relays_*(): reading
                 * the feedback the instant we enter this state, with no delay at
                 * all, catches transient noise (e.g. from CP switching to PWM
                 * right as this runs) as a false "welded" reading. */
                Board_Enable_Relay_Measurement(CMD_ACTIVATE);
                HAL_Delay(RELAY_MEAS_ENABLE_SETTLING_MS);
                HAL_Delay(RELAY_SETTLING_TIME_MS);

                {
                    bool k1_closed = Board_Is_K1_Closed();
#if !IGNORE_K4_FEEDBACK
                    bool k4_closed = Board_Is_K4_Closed();
#else
                    bool k4_closed = false;
#endif
                    Board_Enable_Relay_Measurement(CMD_DEACTIVATE);

#if RCD_BLANK_DURING_RELAY_VERIFICATION
                    /* EMI Blanking: powering the K1/K4 sense circuit (and the
                     * settling delay above) is the actual moment noise has been
                     * observed coupling into the RCD sensor line -- not the
                     * relay coil switching itself. Clear any RCD fault/pending
                     * check picked up during this verification window so a
                     * routine pre-charge check isn't misread as a real
                     * residual-current event, and extend the blanking window
                     * forward -- the RCD module's ~60ms pulse can start with
                     * some latency, possibly after this point. */
                    RCD_Fault = 0;
                    rcd_pending_check = false;
                    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
                    rcd_blank_until_tick = HAL_GetTick() + RCD_BLANK_MARGIN_MS;
#endif

                    if (k1_closed || k4_closed) {
                        if (k1_closed) g_device_status.relay_errors |= RELAY_ERR_K1_OPEN;
                        if (k4_closed) g_device_status.relay_errors |= RELAY_ERR_K4_OPEN;
                        g_device_status.active_faults |= FAULT_BIT_RELAY;
                        g_nextState = FAULT_RELAY_CONTACT;
                        CP_SetLine_Low();
                        break;
                    }
                }

                /* Lock the connector before energizing -- can't be pulled while live */
                Board_Lock_Cable();

                HAL_Delay(50); /* Allow the actuator to fully engage before energizing relays */
                /* Energize Power Path Contactors */
                Board_Set_Contactors(CMD_ACTIVATE);

                /* Verify relays actually closed
                 * (RCD EMI blanking lives inside APP_Verify_Relays_Closed()
                 * itself now, right after its relay-measurement circuit
                 * window closes -- see the note there.) */
                if (!APP_Verify_Relays_Closed(&g_device_status)) {
                    /* At least one relay failed to close — abort charging */
                    APP_Safe_Shutdown();
                    g_nextState = FAULT_RELAY_CONTACT;
                    CP_SetLine_Low();
                    Board_Unlock_Cable(); /* never actually energized -- no reason to stay locked */
                }
                break;

            case CHARGING_COMPLETE:
                /* Ensure PWM is stopped and CP is safely at DC +12V */
                CP_SetLine_High();
                break;

            case FAULT_CAR:
                /* CP error — set fault bit */
                g_device_status.active_faults |= FAULT_BIT_CP_ERROR;
                APP_Safe_Shutdown();
                CP_SetLine_High();
                fault_entry_tick = HAL_GetTick();
                break;

            case FAULT_CP_SHORT:
                /* State E: CP shorted to PE — set fault bit */
                g_device_status.active_faults |= FAULT_BIT_CP_ERROR;
                APP_Safe_Shutdown();
                CP_SetLine_High();
                fault_entry_tick = HAL_GetTick();
                break;

            case FAULT_RELAY_CONTACT:
                /* Relay error — set fault bit */
                g_device_status.active_faults |= FAULT_BIT_RELAY;
                APP_Safe_Shutdown();
                CP_SetLine_Low();
                fault_entry_tick = HAL_GetTick();
                break;

            case FAULT_CABLE:
                /* PP coding resistor invalid/unreadable — nothing offered to the car */
                g_device_status.active_faults |= FAULT_BIT_CABLE;
                APP_Safe_Shutdown();
                CP_SetLine_High();
                fault_entry_tick = HAL_GetTick();
                break;

            case FAULT_GRID:
            case FAULT_RCD:
                /* Safety Shutdown: Force open contactors and reset CP */
                APP_Safe_Shutdown();
                CP_SetLine_Low();
                fault_entry_tick = HAL_GetTick();
                break;

            default:
                break;
        }

    }

    /* Apply a pending current-limit change from CMD_SET_CURRENT without
     * waiting for a state transition -- covers both "Master sets current
     * while already READY" and "Master adjusts current mid-CHARGING".
     * Kept here, not in the dispatcher, so CP hardware is only ever
     * touched from this function (see Section 2 above). */
    if (g_duty_dirty && (g_nextState == READY || g_nextState == CHARGING)) {
        /* duty==0 must NOT go through CP_SetDuty(): with CCR3=0 that is
         * electrically identical to CP_SetLine_Low(), which reads back as
         * STATE_F (fault) -- not "no current offered, EV still connected".
         * Matches the same duty>0/else split already used in the READY
         * entry action above. */
        if (duty > 0) {
            CP_SetDuty(duty);
        } else {
            CP_SetLine_High();
        }
        g_duty_dirty = false;
    }

    /* Status LED: blinks at a different rate depending on the final state
     * for this cycle -- fast (700ms) on any fault, slower (2s) while
     * charging, off otherwise. */
    APP_Update_Status_LED(g_nextState);

    /* Update persistent state */
    *currentState = g_nextState;
}

/**
 * @brief  Drives the general-purpose LED to reflect charging/fault status.
 * @note   Stateless: derives on/off directly from HAL_GetTick(), so it needs
 *         no extra timer or "last toggle" bookkeeping and self-corrects if
 *         a cycle is skipped or delayed.
 */
static void APP_Update_Status_LED(STATE_MACHINE state) {
    uint32_t period_ms;

    bool is_fault = (state == FAULT_RCD || state == FAULT_RELAY_CONTACT ||
                      state == FAULT_GRID || state == FAULT_CAR ||
                      state == FAULT_CP_SHORT || state == FAULT_CABLE);

    if (is_fault) {
        period_ms = LED_PERIOD_FAULT_MS;
    } else if (state == CHARGING) {
        period_ms = LED_PERIOD_CHARGING_MS;
    } else {
        Board_Set_LED(false);
        return;
    }

    /* Toggles every half-period -- true for the first half of each period,
     * false for the second, giving a symmetric on/off blink. */
    Board_Set_LED(((HAL_GetTick() / (period_ms / 2)) % 2) == 0);
}

// Retorna o número de bytes processados/descartados para que o main possa limpar o buffer
uint16_t APP_RS485_Parser(uint8_t* buffer, uint16_t length, RS485_Frame_t* frame) {
    uint16_t i = 0;

    while (i < length) {
        // 1. Procura STARTBYTE
        if (buffer[i] != STARTBYTE) {
            i++;
            continue; // Continua a procurar
        }

        // 2. Verifica se temos tamanho mínimo para ler o cabeçalho (4 bytes)
        if ((length - i) < 4) {
            // Não temos dados suficientes nem para o cabeçalho.
            // Paramos aqui e dizemos ao main para manter estes bytes para a próxima vez.
            return i;
        }

        uint8_t payload_len = buffer[i + 1];
        uint8_t total_frame_size = 4 + payload_len + 1; // Header + Payload + CRC

        // . Reject oversized frames (corrupted payload_len)
        if (total_frame_size > (DATA_RS485 + 5)) {  // max = header(4) + payload(10) + CRC(1) = 15
            i++;       // skip this fake start byte
            continue;  // try next byte
        }
        // . Wait for complete frame (valid size, but not all bytes received yet)
        if ((length - i) < total_frame_size) {
            return i;
        }
        // 4. Verifica CRC
        uint8_t receive_CRC = buffer[i + total_frame_size - 1];
        uint8_t calculate_CRC = Calculate_CRC(&buffer[i], total_frame_size - 1);

        if (calculate_CRC == receive_CRC) {
            // SUCESSO: Preenche a estrutura
            frame->len = buffer[i + 1];
            frame->dest_ID = buffer[i + 2];
            frame->cmd = buffer[i + 3];

            // Proteção contra overflow do buffer de destino
            uint8_t copy_len = (payload_len > DATA_RS485) ? DATA_RS485 : payload_len;
            memcpy(frame->data_RX, &buffer[i + 4], copy_len);

            // Consumimos este pacote inteiro
            i += total_frame_size;

            // Opcional: Se só queremos processar 1 pacote por ciclo, retornamos aqui
            // Mas é melhor retornar i no fim para indicar tudo o que foi lido.
            // Nota: O main precisa saber que houve SUCESSO para agir sobre o 'frame'.
            // Esta função idealmente retornaria um status enum, ou usaria ponteiros.
            return i; // Retorna até onde lemos (neste caso, assumindo que o main processa o frame imediatamente)
        } else {
            // CRC falhou. O byte 'AA' pode ser dados (lixo).
            // Avançamos apenas 1 byte para tentar encontrar um novo 'AA' válido logo a seguir.
            i++;
        }
    }
    return i; // Retorna total de bytes processados (lixo)
}

/**
 * @brief  Dispatcher for valid RS485 frames.
 * @note   Decouples the Communication Protocol from Business Logic.
 *         This function is called only when a valid, CRC-checked frame addressing
 *         this device (ID_BYTE) is received.
 *
 * @param  frame Pointer to the fully parsed RS485 frame.
 */
static void APP_Dispatch_Command(RS485_Frame_t* frame) {

	RS485_Frame_t Transmit_frame;
	memset(&Transmit_frame, 0, sizeof(Transmit_frame));
    // 1. Security Check: Is the message for me?
    if (frame->dest_ID != ID_BYTE) {
        return;
    }


    // 2. Command Switching
    switch (frame->cmd) {

    	case CMD_HEARTBEAT:
    	{
           	Transmit_frame.dest_ID = ID_MASTER;
            Transmit_frame.cmd = CMD_HEARTBEAT;
            Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
            Transmit_frame.len = 1;
            APP_RS485_Send_Message(&Transmit_frame);
            break;
    	}


        case CMD_AUTH_TRUE:
        {
        	autorization = true;
			//Send ACK
        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_AUTH_TRUE;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.len = 1;
        	APP_RS485_Send_Message(&Transmit_frame);

        	/*
            RS485_Frame_t ack_frame;
            memset(&ack_frame, 0, sizeof(ack_frame));
            ack_frame.dest_ID = 0x00; // Assuming Master ID is 0x00
            ack_frame.cmd = CMD_AUTH_SET;
            ack_frame.len = 1;

            if (frame->data_RX[0] == AUTH_TRUE) {
                autorization = true;
                // Optional: Immediate feedback or state change trigger
                // If in IDLE and CP is connected, this might allow transition to READY in next loop
                Board_SetContactor_K2(CMD_ACTIVATE);
                ack_frame.data_RX[0] = 0xAB; // ACK Auth
               // RS485_Send_Package(ack_frame.data_RX[0], 1);
            }
            else if (frame->data_RX[0] == AUTH_FALSE) {
            	Board_SetContactor_K2(CMD_DEACTIVATE);
                autorization = false;
                ack_frame.data_RX[0] = 0xAC; // NACK Auth
                //RS485_Send_Package(ack_frame.data_RX[0], 1);
            }

            APP_RS485_Send_Message(&ack_frame);
            */
            break;
        }

        case CMD_AUTH_FALSE:
        	autorization = false;
        	duty = 0; /* Kill PWM to signal car */
        	g_duty_dirty = true;
			//Send ACK
        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_AUTH_FALSE;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.len = 1;
        	APP_RS485_Send_Message(&Transmit_frame);

        	break;

        case CMD_RESET:
             // Implement Reset Logic
             HAL_NVIC_SystemReset();
             break;

        case CMD_REPORT:
             // Implement Reporting Logic (e.g., send energy counters)
             break;

        case CMD_SESSION_START:

        	session_active = true;
        	autorization = true;
        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_SESSION_START;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.len = 1;
        	APP_RS485_Send_Message(&Transmit_frame);

        	break;

        case CMD_SESSION_STOP:

        	session_active = false;
        	autorization = false;
        	duty = 0; /* Kill PWM to signal car */
        	g_duty_dirty = true;
        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_SESSION_STOP;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.len = 1;
        	APP_RS485_Send_Message(&Transmit_frame);

        	break;


        case CMD_GET_CP_STATE:

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_GET_CP_STATE;
        	Transmit_frame.data_RX[0] = (uint8_t)g_CP_STATE;
        	Transmit_frame.len = 1;
        	APP_RS485_Send_Message(&Transmit_frame);


        	break;

        case CMD_SET_CURRENT:
        	{
				/* 1. Extract raw current value from frame */
				uint8_t requested_current = frame->data_RX[0];


				/* 2. Clamp the current to hardware limits (Safety First) */
				if (requested_current > CURRENT_LIMIT_HIGH) {
							requested_current = CURRENT_LIMIT_HIGH;
				}
				else if (requested_current < CURRENT_LIMIT_LOW) {
							requested_current = CURRENT_LIMIT_LOW;
				}

#if ENABLE_PP_SENSE
				/* 2b. Never offer more than what the connected cable supports */
				if (g_cable_max_amps > 0 && requested_current > g_cable_max_amps) {
							requested_current = g_cable_max_amps;
				}
#endif

				/* * 3. Calculate Duty Cycle  (0.1% resolution, range 0-1000)
				 * ------------------------------------------------------------------
				 * Formula (IEC 61851-1): Duty(%) = Current(A) / 0.6
				 * To scale for  (0-1000): Duty = (Current / 0.6) * 10
				 * * Fixed-Point Derivation:
				 * The multiplier (10 / 0.6) is approx 16.6667.
				 * To avoid floating point, we use a 10-bit fractional shift (2^10 = 1024):
				 * 16.6667 * 1024 = 17066.66... -> Rounded to 17067 (or 17070 for simplicity).
				 * * Final Fast Math: (Current * 17067) >> 10
				 * This effectively multiplies by 16.667 without using slow division.
				 */
				duty = (uint16_t)(( (uint32_t)requested_current * 17070 ) >> 10);

                /* Don't touch CP hardware from here -- just flag the change.
                 * APP_MAIN() applies it (see near "Update persistent state")
                 * whenever it's actually in READY or CHARGING, on this cycle
                 * or a later one. Covers both a fresh request while READY
                 * and a current-limit change mid-CHARGING (previously a no-op). */
                g_duty_dirty = true;

				//Send ACK
	        	Transmit_frame.dest_ID = ID_MASTER;
	        	Transmit_frame.cmd = CMD_SET_CURRENT;
	        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
	        	Transmit_frame.len = 1;
	        	APP_RS485_Send_Message(&Transmit_frame);

				 /* Optional: Save the current state to a global variable for monitoring */
				 //device_status.active_max_current = requested_current;

        	}

        	break;

        case CMD_RELAY_SET: // activation here only for test purposes

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_RELAY_SET;
        	Transmit_frame.len = 1;

        	if( !(g_device_status.active_faults & FAULT_BIT_RCD) ){

        		Board_Set_Contactors(CMD_ACTIVATE);

#if RCD_BLANK_DURING_RELAY_VERIFICATION
                /* EMI Blanking for test command */
                HAL_Delay(50);
                RCD_Fault = 0;
                rcd_pending_check = false;
                __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
                rcd_blank_until_tick = HAL_GetTick() + RCD_BLANK_MARGIN_MS;
#endif

				//Send ACK
				Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
				APP_RS485_Send_Message(&Transmit_frame);
        	}
        	else {
        		/* Refused: RCD fault active -- tell the Master why instead of
        		 * leaving it to time out with no idea what happened. */
        		Transmit_frame.data_RX[0] = (uint8_t)CMD_NACK;
        		APP_RS485_Send_Message(&Transmit_frame);
        	}

        	break;

        case CMD_RELAY_RESET:

        	Board_Set_Contactors(CMD_DEACTIVATE);
			//Send ACK
        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_RELAY_RESET;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.len = 1;
        	APP_RS485_Send_Message(&Transmit_frame);
        	break;

        case CMD_RELAY_MEAS_SET:
        	/* Bench diagnostics only: forces the K1/K4 sense circuits
        	 * permanently on (payload != 0) or releases the override
        	 * (payload == 0), so K1/K4 feedback can be probed at will
        	 * instead of only during a brief verification pulse. */
        	Board_Force_Relay_Measurement(frame->data_RX[0] != 0);

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_RELAY_MEAS_SET;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.len = 1;
        	APP_RS485_Send_Message(&Transmit_frame);
        	break;

        case CMD_RELAY_GET:
        {
        	/* K1/K4 feedback needs its sense circuit powered before reading. */
        	Board_Enable_Relay_Measurement(CMD_ACTIVATE);
        	HAL_Delay(RELAY_MEAS_ENABLE_SETTLING_MS);

        	if(Board_Is_K1_Closed()){
        		g_device_status.relay_state |= RELAY_ERR_K1_CLOSE;
        	}
        	else{
        		g_device_status.relay_state &=~RELAY_ERR_K1_CLOSE;
        	}

        	/* K2/K3 are read unconditionally here for diagnostics, even though
        	 * SYSTEM_PHASES==1 excludes them from the safety-relevant checks
        	 * (APP_Verify_Relays_Closed/Open) -- otherwise their feedback pins
        	 * are wired but nothing ever reports what they're actually doing. */
        	if(Board_Is_K2_Closed()){
        		g_device_status.relay_state |= RELAY_ERR_K2_CLOSE;
        	}
        	else{
        		g_device_status.relay_state &=~RELAY_ERR_K2_CLOSE;
        	}

        	if(Board_Is_K3_Closed()){
        		g_device_status.relay_state |= RELAY_ERR_K3_CLOSE;
        	}
        	else{
        		g_device_status.relay_state &=~RELAY_ERR_K3_CLOSE;
        	}

        	if(Board_Is_K4_Closed()){
        		g_device_status.relay_state |= RELAY_ERR_K4_CLOSE;
        	}
        	else{
        		g_device_status.relay_state &=~RELAY_ERR_K4_CLOSE;
        	}

        	Board_Enable_Relay_Measurement(CMD_DEACTIVATE);

#if RCD_BLANK_DURING_RELAY_VERIFICATION
        	/* EMI Blanking: same sense-circuit activation as
        	 * APP_Verify_Relays_Closed/Open() -- clear any RCD fault/pending
        	 * check picked up during this bench read so it isn't misread as
        	 * a real residual-current event, and extend the blanking window
        	 * forward in case the RCD module's pulse starts late. */
        	RCD_Fault = 0;
        	rcd_pending_check = false;
        	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
        	rcd_blank_until_tick = HAL_GetTick() + RCD_BLANK_MARGIN_MS;
#endif

			//Send ACK
        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_RELAY_GET;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.data_RX[1] = (uint8_t)g_device_status.relay_state;
        	Transmit_frame.len = 2;
        	APP_RS485_Send_Message(&Transmit_frame);
        	break;
        }


        case CMD_GET_FAULTS:

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_GET_FAULTS;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.data_RX[1] = (uint8_t)g_device_status.active_faults;
        	Transmit_frame.data_RX[2] = (uint8_t)g_device_status.relay_errors;
        	Transmit_frame.len = 3;
        	APP_RS485_Send_Message(&Transmit_frame);
        	break;

        case CMD_CLEAR_FAULTS:


        	g_device_status.active_faults = 0;
        	g_device_status.relay_errors = 0;
        	//Send ACK
        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_CLEAR_FAULTS;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
        	Transmit_frame.data_RX[1] = (uint8_t)g_device_status.active_faults;
			Transmit_frame.data_RX[2] = (uint8_t)g_device_status.relay_errors;
        	Transmit_frame.len = 3;
        	APP_RS485_Send_Message(&Transmit_frame);
        	break;


        case CMD_GET_VOLTAGE:

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_GET_VOLTAGE;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;


        	uint16_t v1 = (uint16_t)(g_energy_line1.vrms_mV / 100);
        	Transmit_frame.data_RX[1] = (uint8_t)(v1>>8); //MSB
        	Transmit_frame.data_RX[2] = (uint8_t)(v1 & 0xFF);//LSB

        	uint16_t v2 = (uint16_t)(g_energy_line2.vrms_mV / 100);
        	Transmit_frame.data_RX[3] = (uint8_t)(v2>>8); //MSB
        	Transmit_frame.data_RX[4] = (uint8_t)(v2 & 0xFF);//LSB

        	uint16_t v3 = (uint16_t)(g_energy_line3.vrms_mV / 100);
        	Transmit_frame.data_RX[5] = (uint8_t)(v3>>8); //MSB
        	Transmit_frame.data_RX[6] = (uint8_t)(v3 & 0xFF);//LSB

        	Transmit_frame.len = 7;
        	APP_RS485_Send_Message(&Transmit_frame);

        	break;


        case CMD_GET_CURRENT:

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_GET_CURRENT;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;

        	uint16_t i1 = (uint16_t)(g_energy_line1.current_mA / 100);
        	Transmit_frame.data_RX[1] = (uint8_t)(i1>>8); //MSB
        	Transmit_frame.data_RX[2] = (uint8_t)(i1 & 0xFF);//LSB

        	uint16_t i2 = (uint16_t)(g_energy_line2.current_mA / 100);
        	Transmit_frame.data_RX[3] = (uint8_t)(i2>>8); //MSB
        	Transmit_frame.data_RX[4] = (uint8_t)(i2 & 0xFF);//LSB

        	uint16_t i3 = (uint16_t)(g_energy_line3.current_mA / 100);
        	Transmit_frame.data_RX[5] = (uint8_t)(i3>>8); //MSB
        	Transmit_frame.data_RX[6] = (uint8_t)(i3 & 0xFF);//LSB

        	Transmit_frame.len = 7;
        	APP_RS485_Send_Message(&Transmit_frame);


        	break;


        case CMD_GET_POWER:

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_GET_POWER;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;

        	uint16_t p1 = (uint16_t)(g_energy_line1.active_power_W / 100);
        	Transmit_frame.data_RX[1] = (uint8_t)(p1>>8); //MSB
        	Transmit_frame.data_RX[2] = (uint8_t)(p1 & 0xFF);//LSB

        	uint16_t p2 = (uint16_t)(g_energy_line2.active_power_W / 100);
        	Transmit_frame.data_RX[3] = (uint8_t)(p2>>8); //MSB
        	Transmit_frame.data_RX[4] = (uint8_t)(p2 & 0xFF);//LSB

        	uint16_t p3 = (uint16_t)(g_energy_line3.active_power_W / 100);
        	Transmit_frame.data_RX[5] = (uint8_t)(p3>>8); //MSB
        	Transmit_frame.data_RX[6] = (uint8_t)(p3 & 0xFF);//LSB

        	Transmit_frame.len = 7;
        	APP_RS485_Send_Message(&Transmit_frame);


        	break;

        case CMD_GET_ENERGY:

        	Transmit_frame.dest_ID = ID_MASTER;
        	Transmit_frame.cmd = CMD_GET_ENERGY;
        	Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;

        	uint16_t e1 = (uint16_t)(g_energy_line1.session_energy_Wh / 100);
        	Transmit_frame.data_RX[1] = (uint8_t)(e1>>8); //MSB
        	Transmit_frame.data_RX[2] = (uint8_t)(e1 & 0xFF);//LSB

        	uint16_t e2 = (uint16_t)(g_energy_line2.session_energy_Wh / 100);
        	Transmit_frame.data_RX[3] = (uint8_t)(e2>>8); //MSB
        	Transmit_frame.data_RX[4] = (uint8_t)(e2 & 0xFF);//LSB

        	uint16_t e3 = (uint16_t)(g_energy_line3.session_energy_Wh / 100);
        	Transmit_frame.data_RX[5] = (uint8_t)(e3>>8); //MSB
        	Transmit_frame.data_RX[6] = (uint8_t)(e3 & 0xFF);//LSB

        	Transmit_frame.len = 7;
        	APP_RS485_Send_Message(&Transmit_frame);

        	break;


        case CMD_STATE_NOTIFY:
            /* Se recebermos o comando STATE_NOTIFY e o payload for um ACK */
            if (frame->len > 0 && frame->data_RX[0] == CMD_ACK) {
                g_waiting_state_ack = false; // Sucesso: Cancelamos o timeout/retransmissão
            }
            break;


        default:
            // Unknown command
            break;


    }
}

/**
 * @brief  Main Communication Logic Task.
 * @note   This function implements a "Producer-Consumer" pattern for UART/RS485 data.
 *         It performs 4 key steps:
 *         1. FETCH: Moves raw bytes from DMA (Hardware) to Work Buffer (Application).
 *         2. PARSE: Scans the Work Buffer for valid packets (Start Byte + Length + CRC).
 *         3. DISPATCH: If a valid packet is found, executes the corresponding command.
 *         4. CLEAN: Shifts the buffer to discard processed data and prepare for new bytes.
 *
 * @warning Must be called periodically in the main loop (e.g., every 10ms or in a FreeRTOS task).
 */
void APP_Comms_Task(void) {
    // 1. Fetch new data from DMA into Work Buffer
    uint16_t space_left = WORK_BUFFER_SIZE - work_buffer_len;
    if (space_left > 0) {
        uint16_t bytes_read = RS485_Get_Data(&work_buffer[work_buffer_len], space_left);
        work_buffer_len += bytes_read;
    }

    // 2. Process Data if available
    if (work_buffer_len > 0) {
        RS485_Frame_t frame_recebida;
        memset(&frame_recebida, 0, sizeof(frame_recebida));

        // Parse buffer
        uint16_t bytes_processed = APP_RS485_Parser(work_buffer, work_buffer_len, &frame_recebida);

        // 3. Dispatch Valid Frame
        // Note: APP_RS485_Parser logic in this file returns 'i' (bytes processed).
        // It doesn't explicitly return "Success/Fail" easily without checking if a frame was filled.
        // We check if cmd is not 0 (assuming 0 is not a valid cmd) or by a flag.
        // Given existing logic, if bytes_processed > 0 AND we have a valid header/crc check inside...
        // Actually, the current Parser returns 'i' which increments on success.
        // We added a basic check: if frame_recebida.cmd is set (assuming 0 initialization).

        if (bytes_processed > 0 && frame_recebida.cmd != 0) {
            APP_Dispatch_Command(&frame_recebida);
        }

        // 4. Buffer Cleanup (Shift Left)
        if (bytes_processed > 0) {
            uint16_t remaining = work_buffer_len - bytes_processed;
            if (remaining > 0) {
                memmove(work_buffer, &work_buffer[bytes_processed], remaining);
            }
            work_buffer_len = remaining;
        }
    }

    /*
    RS485_Frame_t Transmit_frame;
    memset(&Transmit_frame, 0, sizeof(Transmit_frame));
   	Transmit_frame.dest_ID = ID_MASTER;
    Transmit_frame.cmd = CMD_HEARTBEAT;
    Transmit_frame.data_RX[0] = (uint8_t)CMD_ACK;
    Transmit_frame.len = 1;
    APP_RS485_Send_Message(&Transmit_frame);
    */

    if (g_waiting_state_ack) {
           if ((HAL_GetTick() - g_state_notify_tick) > STATE_NOTIFY_TIMEOUT_MS) {

               if (g_state_notify_retries < STATE_NOTIFY_MAX_RETRIES) {
                   // Tenta retransmitir
                   g_state_notify_retries++;
                   g_state_notify_tick = HAL_GetTick(); // Reinicia o timer

                   RS485_Frame_t notify;
                   memset(&notify, 0, sizeof(notify));
                   notify.dest_ID    = ID_MASTER;
                   notify.cmd        = CMD_STATE_NOTIFY;
                   notify.data_RX[0] = (uint8_t)g_CP_STATE; // Reenvia o estado atual
                   notify.data_RX[1] = g_cable_max_amps;
                   notify.len = 2;
                   APP_RS485_Send_Message(&notify);
               } else {
                   // Esgotaram-se as tentativas - Declarar perda de comunicação!
                   g_waiting_state_ack = false;
                   //g_state_notify_retries = 0;

                   // Opcional: Levantar flag de erro de comunicação
                   // g_device_status.active_faults |= FAULT_BIT_COMM;
                   // (Se desejares que o EVSE passe a um estado de falha quando perde ligação)
               }
           }
       }
}


/**
 * @brief  Energy Flag Task Set.
 * @note   Set the flag for the energy Task function.
 *
 */
void APP_Energy_Flag_Set(void){
	flag_read_energy = 1;
}

/**
 * @brief  Voltage Flag Task Set.
 * @note   Set the flag for the voltage and current Task function.
 *
 */
void APP_Voltage_Flag_Set(void){
	flag_read_voltage = 1;
}


/**
 * @brief  Energy Monitoring Task.
 * @note   Reads critical registers (Vrms, Power, Energy) from the ADE7953.
 *         The execution is gated by a timer flag (flag_read_energy) to prevent
 *         flooding the SPI bus and blocking the main loop excessively.
 */
void APP_Energy_Task(void) {



	if(flag_read_voltage){

		flag_read_voltage = 0;


		//solução temporária para teste
	    static bool calibrating = false;
	    static uint32_t cal_result1 = 0;
	    static uint32_t cal_result2 = 0;
	    static uint32_t cal_result3 = 0;
	    if (calibrating) {
	        if (ade7953_calibrate_vrms(ADE_DEVICE_3, 100, &cal_result3)) {
	            //calibrating = false;
	            // cal_result → lê no debugger ou envia por RS485
	        }
	        if (ade7953_calibrate_vrms(ADE_DEVICE_2, 100, &cal_result2)) {
	            //calibrating = false;
	            // cal_result → lê no debugger ou envia por RS485
	        }
	        if (ade7953_calibrate_vrms(ADE_DEVICE_1, 100, &cal_result1)) {
	            calibrating = false;
	            // cal_result → lê no debugger ou envia por RS485
	        }
	        return;
	    }
        /* Line 1 */
        g_energy_line1.vrms_mV        = ade7953_get_vrms_mV(ADE_DEVICE_1);
        g_energy_line1.current_mA     = ade7953_get_irms_mA(ADE_DEVICE_1);
        
        /* Line 2 */
        g_energy_line2.vrms_mV        = ade7953_get_vrms_mV(ADE_DEVICE_2);
        g_energy_line2.current_mA     = ade7953_get_irms_mA(ADE_DEVICE_2);
        
        /* Line 3 */
        g_energy_line3.vrms_mV        = ade7953_get_vrms_mV(ADE_DEVICE_3);
        g_energy_line3.current_mA     = ade7953_get_irms_mA(ADE_DEVICE_3);
	}

	if(flag_read_energy){

		flag_read_energy = 0;
        
        /* Line 1 */
        g_energy_line1.active_power_W = ade7953_get_active_power_mW(ADE_DEVICE_1);
        g_energy_line1.freq_dec_Hz 	  = ade7953_get_line_frequency(ADE_DEVICE_1);
        
        /* Line 2 */
        g_energy_line2.active_power_W = ade7953_get_active_power_mW(ADE_DEVICE_2);
        g_energy_line2.freq_dec_Hz 	  = ade7953_get_line_frequency(ADE_DEVICE_2);
        
        /* Line 3 */
        g_energy_line3.active_power_W = ade7953_get_active_power_mW(ADE_DEVICE_3);
        g_energy_line3.freq_dec_Hz 	  = ade7953_get_line_frequency(ADE_DEVICE_3);
	}


}

/**
 * @brief  Centralized Relay Shutdown.
 * @note   Opens all 4 contactors. Used as a single point of actuation
 *         to avoid code duplication and ensure consistency.
 */
static void APP_Safe_Shutdown(void) {
	Board_Set_Contactors(CMD_DEACTIVATE);
}

/**
 * @brief  Verifies that all contactors have physically CLOSED after activation.
 * @param  status Pointer to DeviceStatus_t. On failure, the corresponding
 *         close-error bits [3:0] are SET in status->relay_errors.
 * @return true if ALL relays confirmed closed, false if any failed.
 * @note   Waits RELAY_SETTLING_TIME_MS before reading feedback pins.
 */
bool APP_Verify_Relays_Closed(DeviceStatus_t *status) {

    bool all_ok = false;

    /* K1/K4 feedback needs its sense circuit powered before any of the
     * reads below (including the polling loop) are valid. */
    Board_Enable_Relay_Measurement(CMD_ACTIVATE);
    HAL_Delay(RELAY_MEAS_ENABLE_SETTLING_MS);

    /* Dynamic Polling (Max 250ms): Sluggish AC Optocouplers or slow mechanical
     * latches take time. We loop every 10ms to check if they have finally closed.
     * We instantly break out of the loop and resume the main program the millisecond
     * they all report success, avoiding static heavy delays!
     */
    for (int wait = 0; wait < 30; wait++) {
        HAL_Delay(10);
        all_ok = true;
        
        if (!Board_Is_K1_Closed()) all_ok = false;
#if SYSTEM_PHASES == 3
        if (!Board_Is_K2_Closed()) all_ok = false;
        if (!Board_Is_K3_Closed()) all_ok = false;
#endif
#if !IGNORE_K4_FEEDBACK
        if (!Board_Is_K4_Closed()) all_ok = false;
#endif

        if (all_ok) break; /* They all successfully closed! Proceed immediately. */
    }

    /* Clear previous close errors, preserve open errors */
    status->relay_errors &= ~RELAY_ERR_CLOSE_MASK;
    HAL_Delay(50);

    /* Final decisive read: start clean so a stale timeout from the polling
     * loop above can't fail this check when the relays are actually fine
     * by now -- only an actual closed-check failure below may set false. */
    all_ok = true;

    /* If they never closed after 250ms, register the faults */
    if (!Board_Is_K1_Closed()) {
        status->relay_errors |= RELAY_ERR_K1_CLOSE;
        all_ok = false;
    }
    
#if SYSTEM_PHASES == 3
    if (!Board_Is_K2_Closed()) {
        status->relay_errors |= RELAY_ERR_K2_CLOSE;
        all_ok = false;
    }
    if (!Board_Is_K3_Closed()) {
        status->relay_errors |= RELAY_ERR_K3_CLOSE;
        all_ok = false;
    }
#endif
        
#if !IGNORE_K4_FEEDBACK
    if (!Board_Is_K4_Closed()) {
        status->relay_errors |= RELAY_ERR_K4_CLOSE;
        all_ok = false;
    }
#endif

    Board_Enable_Relay_Measurement(CMD_DEACTIVATE);

#if RCD_BLANK_DURING_RELAY_VERIFICATION
    /* EMI Blanking: powering the K1/K4 sense circuit for this whole
     * verification window is what has been observed coupling noise into
     * the RCD sensor line -- not the contactor coils switching. Clear any
     * RCD fault/pending-check picked up during it so a routine relay
     * verification isn't misread as a real residual-current event, and
     * extend the blanking window forward -- the RCD module's ~60ms pulse
     * can start with some latency, possibly after this point. */
    RCD_Fault = 0;
    rcd_pending_check = false;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    rcd_blank_until_tick = HAL_GetTick() + RCD_BLANK_MARGIN_MS;
#endif

    return all_ok;
}

/**
 * @brief  Verifies that all contactors have physically OPENED after deactivation.
 * @param  status Pointer to DeviceStatus_t. On failure (welded contact), the
 *         corresponding open-error bits [7:4] are SET in status->relay_errors.
 * @return true if ALL relays confirmed open, false if any is still closed (welded).
 * @note   Waits RELAY_SETTLING_TIME_MS before reading feedback pins.
 */
bool APP_Verify_Relays_Open(DeviceStatus_t *status) {

    bool all_ok = false;

    /* K1/K4 feedback needs its sense circuit powered before any of the
     * reads below (including the polling loop) are valid. */
    Board_Enable_Relay_Measurement(CMD_ACTIVATE);
    HAL_Delay(RELAY_MEAS_ENABLE_SETTLING_MS);

    /* Dynamic Polling (Max 250ms): When opening, AC sensing Optocouplers
     * have huge smoothing capacitors that can take up to 100~200ms to fully 
     * discharge down to 0V. We poll every 10ms until they all report OPEN.
     */
    for (int wait = 0; wait < 30; wait++) {
        HAL_Delay(10);
        all_ok = true;
        
        if (Board_Is_K1_Closed()) all_ok = false;
#if SYSTEM_PHASES == 3
        if (Board_Is_K2_Closed()) all_ok = false;
        if (Board_Is_K3_Closed()) all_ok = false;
#endif
#if !IGNORE_K4_FEEDBACK
        if (Board_Is_K4_Closed()) all_ok = false;
#endif

        if (all_ok) break; /* They all successfully opened (discharged)! */
    }

    /* Clear previous open errors, preserve close errors */
    status->relay_errors &= ~RELAY_ERR_OPEN_MASK;
    //HAL_Delay(100);

    /* Final decisive read: start clean so a stale timeout from the polling
     * loop above can't fail this check when the relays are actually fine
     * by now -- only an actual still-closed reading below may set false. */
    all_ok = true;

    /* If they never discharged/opened after 250ms, register the welded fault */
    if (Board_Is_K1_Closed()) {
        status->relay_errors |= RELAY_ERR_K1_OPEN;
        all_ok = false;
    }
#if SYSTEM_PHASES == 3
    if (Board_Is_K2_Closed()) {
        status->relay_errors |= RELAY_ERR_K2_OPEN;
        all_ok = false;
    }
    if (Board_Is_K3_Closed()) {
        status->relay_errors |= RELAY_ERR_K3_OPEN;
        all_ok = false;
    }
#endif
#if !IGNORE_K4_FEEDBACK
    if (Board_Is_K4_Closed()) {
        status->relay_errors |= RELAY_ERR_K4_OPEN;
        all_ok = false;
    }
#endif

    Board_Enable_Relay_Measurement(CMD_DEACTIVATE);

#if RCD_BLANK_DURING_RELAY_VERIFICATION
    /* EMI Blanking: powering the K1/K4 sense circuit for this whole
     * verification window is what has been observed coupling noise into
     * the RCD sensor line -- not the contactor coils switching. Clear any
     * RCD fault/pending-check picked up during it so a routine relay
     * verification isn't misread as a real residual-current event, and
     * extend the blanking window forward -- the RCD module's ~60ms pulse
     * can start with some latency, possibly after this point. */
    RCD_Fault = 0;
    rcd_pending_check = false;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    rcd_blank_until_tick = HAL_GetTick() + RCD_BLANK_MARGIN_MS;
#endif

    return all_ok;
}




/**
 * @brief  Checks grid voltage and current against safety thresholds (3-phase).
 * @note   Reads cached values from g_energy_line1/2/3.
 *         Sets fault bits in g_device_status.active_faults if any phase exceeds limits.
 *         A single phase out of range is enough to trigger the fault.
 */
static void APP_Safety_Check_Electrical(void) {
    /* Array of pointers for compact iteration over active phases */
    Energy_Data_t *lines[] = {
#if SYSTEM_PHASES == 3
        &g_energy_line1,
        &g_energy_line2,
		&g_energy_line3

#else
		&g_energy_line1
#endif

    };
    
    /* Dynamically calculate the number of lines to check based on array size */
    uint8_t num_lines = sizeof(lines) / sizeof(lines[0]);

    for (uint8_t i = 0; i < num_lines; i++) {
        uint32_t vrms = lines[i]->vrms_mV;
        uint32_t irms = lines[i]->current_mA;
        /* Overvoltage — any phase above 253V */
        if (vrms > GRID_OVERVOLT_THRESH_mV) {
            g_device_status.active_faults |= FAULT_BIT_OVERVOLT;
        }
        /* Undervoltage — any phase below 207V (ignore 0 = no reading yet) */
        if (vrms < GRID_UNDERVOLT_THRESH_mV && vrms > 0) {
            g_device_status.active_faults |= FAULT_BIT_UNDERVOLT;
        }
        /* Overcurrent — any phase above 35A */
        if (irms > OVERCURRENT_THRESH_mA) {
            g_device_status.active_faults |= FAULT_BIT_OVERCURR;
        }
    }
}


uint8_t Calculate_CRC(uint8_t* buffer_d, uint8_t frame_len){
	uint8_t sum = 0;

	for(int i = 0; i< frame_len; i++){
		sum+=buffer_d[i];
	}
	return sum;
}

void APP_RS485_Send_Message(RS485_Frame_t* tx_frame){

	// static é bom para o DMA, mas cuidado com reentrância (não chamar em IT e Main ao mesmo tempo)
	    static uint8_t temp_buffer[DATA_RS485 + 5];
	    uint8_t calculated_crc = 0;

	    // Proteção contra overflow de payload
	    if (tx_frame->len > DATA_RS485) {
	        return;
	    }

	    temp_buffer[0] = STARTBYTE;
	    temp_buffer[1] = tx_frame->len;
	    temp_buffer[2] = tx_frame->dest_ID;
	    temp_buffer[3] = tx_frame->cmd;

	    if (tx_frame->len > 0) {
	        // Assume-se que tx_frame->data_RX contém os dados a enviar (ou muda para data_TX)
	        memcpy(&temp_buffer[4], tx_frame->data_RX, tx_frame->len);
	    }

	    // Calcula CRC sobre Header + Dados
	    calculated_crc = Calculate_CRC(temp_buffer, tx_frame->len + 4);
	    temp_buffer[tx_frame->len + 4] = calculated_crc;

	    // Aguarda um curto período para permitir que o Raspberry Pi liberte o bus (Turnaround Delay)
	    // Como o Pi corre um OS não-RealTime, a latência do GPIO mudar para modo Receção pode demorar uns milissegundos.
	    HAL_Delay(10);

	    // Envia: Header(4) + Data(len) + CRC(1) = len + 5
	    RS485_Send_Package(temp_buffer, tx_frame->len + 5);
}


//only for test
void APP_TEST_Contactor_Monitor(void){

	Board_Set_Contactors(CMD_ACTIVATE);


	  bool k1_state;
	  bool k2_state;
	  bool k3_state;
	  bool k4_state;


	  k1_state = Board_Is_K1_Closed();
	  k2_state = Board_Is_K2_Closed();
	  k3_state = Board_Is_K3_Closed();
	  k4_state = Board_Is_K4_Closed();

	  HAL_Delay(1000);
	Board_Set_Contactors(CMD_DEACTIVATE);



	  k1_state = Board_Is_K1_Closed();
	  k2_state = Board_Is_K2_Closed();
	  k3_state = Board_Is_K3_Closed();
	  k4_state = Board_Is_K4_Closed();
}
