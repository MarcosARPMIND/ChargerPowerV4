/*
 * app_manager.h
 *
 *  Created on: Dec 12, 2025
 *      Author: ARPMindTech
 */

#ifndef INC_APP_MANAGER_H_
#define INC_APP_MANAGER_H_

#include "cp.h"
#include "board_io.h"
#include "energy_meter.h"
#include "stm32c0xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define STARTBYTE 			0xAA

#define ID_BYTE				0x01 //change this in case the charger has two ports
#define ID_MASTER			0x04

//-----------------------------Commands

// --- Session Control ---

#define CMD_SESSION_START   0x11  // Start charging session
#define CMD_SESSION_STOP    0x12  // Stop charging session
#define CMD_SESSION_STATUS  0x13  // Request current session status
#define CMD_SESSION_INFO    0x14  // Session info (duration, energy)
#define CMD_STATE_NOTIFY    0x15  // Unsolicited state change notification (MCU → Master)

// --- System Control ---
#define CMD_RESET           0x20  // Reset the system

// --- Metering / Report ---
#define CMD_REPORT          0x30  // General status report
#define CMD_GET_VOLTAGE     0x31  // Read voltage (V)
#define CMD_GET_CURRENT     0x32  // Read current (A)
#define CMD_GET_POWER       0x33  // Instantaneous power (kW)
#define CMD_GET_ENERGY      0x34  // Session energy (kWh)
#define CMD_GET_TEMP        0x35  // Connector / EVSE temperature
#define CMD_GET_METER_ALL   0x36  // All electrical values at once
#define CMD_GET_FREQ		0x37

// --- State Machine ---
#define CMD_SET_STATE       0x40  // Force state machine state
#define CMD_GET_CP_STATE    0x41  // Get current state (IEC 61851: A/B/C/D/E/F)
//#define CMD_SET_MAX_CURRENT 0x42  // Set maximum PWM current limit
//#define CMD_GET_MAX_CURRENT 0x43  // Get configured maximum current
#define CMD_SET_CURRENT		0x45

// --- Faults & Diagnostics ---
#define CMD_GET_FAULTS      0x50  // List active faults
#define CMD_CLEAR_FAULTS    0x51  // Clear faults
#define CMD_GET_DIAG        0x52  // General diagnostics (FW version, uptime, etc.)

// --- Hardware Actuation ---
#define CMD_RELAY_SET       0x60  // Set contactor / relay state
#define CMD_RELAY_GET       0x61  // Get current relay state
#define CMD_LOCK_SET        0x62  // Lock / unlock cable actuator
#define CMD_LOCK_GET        0x63  // Get cable lock state
#define CMD_RELAY_RESET     0x64  // Set contactor / relay state

// --- Heartbeat / ACK ---
#define CMD_HEARTBEAT       0x70  // Ping / alive check
#define CMD_ACK             0xF0  // Generic acknowledgement
#define CMD_NACK            0xF1  // Negative acknowledgement (command error)

// --- Auth Response Values ---
#define CMD_AUTH_TRUE       0x01
#define CMD_AUTH_FALSE      0x05

// --- Generic Response Status ---
#define RESP_OK             0x00  // Command executed successfully
#define RESP_ERROR          0xFF  // Generic error
#define RESP_BUSY           0xFE  // System busy (e.g. session already active)
#define RESP_NOT_AUTH       0xFD  // Command not authorised

// --- Active Fault Bitmask (uint8_t active_faults) ---
//
//   Bit 7     Bit 6   Bit 5   Bit 4    Bit 3    Bit 2     Bit 1     Bit 0
//  [RSVD]   [COMM] [CP_ERR][RELAY] [UNDERVOLT][OVERVOLT][OVERCURR][ RCD ]
//
//  Multiple faults can be active simultaneously.
//  Example: RCD + Overcurrent → active_faults = 0b 0000 0011 = 0x03
//
#define FAULT_BIT_RCD        (1U << 0)  /* Bit 0: Residual Current Device tripped */
#define FAULT_BIT_OVERCURR   (1U << 1)  /* Bit 1: Overcurrent detected */
#define FAULT_BIT_OVERVOLT   (1U << 2)  /* Bit 2: Grid overvoltage */
#define FAULT_BIT_UNDERVOLT  (1U << 3)  /* Bit 3: Grid undervoltage */
#define FAULT_BIT_RELAY      (1U << 4)  /* Bit 4: Relay error (see relay_errors for detail) */
#define FAULT_BIT_CP_ERROR   (1U << 5)  /* Bit 5: Control Pilot signal error */
#define FAULT_BIT_COMM       (1U << 6)  /* Bit 6: Communication timeout */
//                           (1U << 7)  /* Bit 7: Reserved for future use */


#define DATA_RS485			10	  // Maximum data lenght for transmisson

#define CURRENT_LIMIT_HIGH 	33    //Amps
#define CURRENT_LIMIT_LOW  	6     //Amps

#define WORK_BUFFER_SIZE 256


/* Grid Voltage Limits (millivolts) — EN 50160 tolerance ±10% of 230V */
#define GRID_OVERVOLT_THRESH_mV   253000   /* 253V = 230V + 10% */
#define GRID_UNDERVOLT_THRESH_mV  207000   /* 207V = 230V - 10% */
/* Overcurrent Limit (milliamps) — matches CURRENT_LIMIT_HIGH with margin */
#define OVERCURRENT_THRESH_mA     35000    /* 35A — above 33A hard limit */

/* System Configuration */
#define SYSTEM_PHASES             1        /* Set to 1 for Single-Phase, 3 for Three-Phase */

/* ============================================================================== */
/* RELAY FEEDBACK VERIFICATION                                                    */
/* ============================================================================== */

/**
 * @brief Settling time (ms) to wait after actuating a relay before reading feedback.
 * Typical mechanical relays need 5-20ms to fully commute.
 */
#define RELAY_SETTLING_TIME_MS  50

/**
 * @brief Relay Error Bitmask Layout (uint8_t relay_errors)
 *
 *   Bit 7   Bit 6   Bit 5   Bit 4   Bit 3   Bit 2   Bit 1   Bit 0
 *  [K4_OPN][K3_OPN][K2_OPN][K1_OPN][K4_CLS][K3_CLS][K2_CLS][K1_CLS]
 *   \__________0xF0__________/       \__________0x0F__________/
 *     OPEN errors (welded)              CLOSE errors
 *
 *  Bits [3:0] = Close error (relay did not close when commanded)
 *  Bits [7:4] = Open error  (relay did not open — welded contact)
 *
 *  Example: K2 failed to close + K4 welded → relay_errors = 0b 1000 0010 = 0x82
 *
 *  MASK usage (group operations):
 *    (relay_errors & CLOSE_MASK) → true if ANY relay failed to close
 *    (relay_errors & OPEN_MASK)  → true if ANY relay is welded
 *    relay_errors &= ~CLOSE_MASK → clears close errors, preserves open errors
 *    relay_errors &= ~OPEN_MASK  → clears open errors, preserves close errors
 */
#define RELAY_ERR_K1_CLOSE   (1U << 0)  /* Bit 0: K1 failed to close */
#define RELAY_ERR_K2_CLOSE   (1U << 1)  /* Bit 1: K2 failed to close */
#define RELAY_ERR_K3_CLOSE   (1U << 2)  /* Bit 2: K3 failed to close */
#define RELAY_ERR_K4_CLOSE   (1U << 3)  /* Bit 3: K4 failed to close */
#define RELAY_ERR_K1_OPEN    (1U << 4)  /* Bit 4: K1 welded (stuck closed) */
#define RELAY_ERR_K2_OPEN    (1U << 5)  /* Bit 5: K2 welded (stuck closed) */
#define RELAY_ERR_K3_OPEN    (1U << 6)  /* Bit 6: K3 welded (stuck closed) */
#define RELAY_ERR_K4_OPEN    (1U << 7)  /* Bit 7: K4 welded (stuck closed) */

#define RELAY_ERR_CLOSE_MASK  0x0F   /* 0000 1111 — selects any close error */
#define RELAY_ERR_OPEN_MASK   0xF0   /* 1111 0000 — selects any open/welded error */

#define STATE_NOTIFY_TIMEOUT_MS 5000
#define STATE_NOTIFY_MAX_RETRIES 3





typedef enum{

	IDLE=0,
	READY,
	CHARGING,
	CHARGING_COMPLETE,
	FAULT_RCD,
	FAULT_RELAY_CONTACT,
	FAULT_GRID,
	FAULT_CAR,
	FAULT_CP_SHORT     /* State E: CP short to PE (0V) — IEC 61851 Table A.5 */

}STATE_MACHINE;

typedef struct{
	uint8_t dest_ID;
	uint8_t cmd;
	uint8_t len;
	uint8_t data_RX[DATA_RS485]; //adjust lenght
}RS485_Frame_t;

/**
 * @brief  Device Status Structure.
 * @note   Centralizes runtime status and error tracking for diagnostics/reporting.
 */
typedef struct {
	uint8_t relay_state;
	uint8_t relay_errors;       /**< Bitmask: [7:4] open/welded errors, [3:0] close errors */
	uint8_t active_faults;      /**< Bitmask: all active faults (FAULT_BIT_RCD, etc.) */
} DeviceStatus_t;


void APP_RS485_Send_Message(RS485_Frame_t* tx_frame);
uint16_t APP_RS485_Parser(uint8_t* buffer, uint16_t length, RS485_Frame_t* frame);
uint8_t Calculate_CRC(uint8_t* buffer_d, uint8_t frame_len);


void APP_Comms_Task(void);
void APP_Energy_Flag_Set(void);
void APP_Voltage_Flag_Set(void);
void APP_Energy_Task(void);
void APP_MAIN(STATE_MACHINE *currentState);

/* Relay Verification */
bool APP_Verify_Relays_Closed(DeviceStatus_t *status);
bool APP_Verify_Relays_Open(DeviceStatus_t *status);

void APP_TEST_Contactor_Monitor(void);


#endif /* INC_APP_MANAGER_H_ */
