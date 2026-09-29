/*
 * board_io.h
 *
 *  Created on: Nov 24, 2025
 *      Author: Marcos Novo
 */

#ifndef INC_BOARD_IO_H_
#define INC_BOARD_IO_H_

#include "app_config.h"
#include <stdbool.h>

typedef enum {
	SEND=0,
	RECEIVER
}DE_STATE;

typedef enum{
	CMD_ACTIVATE=0,
	CMD_DEACTIVATE
}PIN_STATE;

typedef enum{
	OPEN=0,
	CLOSE,
	OFF
}ACTUATOR_STATE;

void Board_init(void);

void Board_ADE_Reset_1(PIN_STATE state);
void Board_ADE_Reset_2(PIN_STATE state);
void Board_ADE_Reset_3(PIN_STATE state);

void Board_Set_SPI_CS_1(PIN_STATE state);
void Board_Set_SPI_CS_2(PIN_STATE state);
void Board_Set_SPI_CS_3(PIN_STATE state);

void Board_Set_RS485_DE(DE_STATE state);

/* Relay-coil MOSFET economizer duty cycle (0-1000 = 0.0%-100.0%) on PC7/TIM3_CH2 */
#define RELAY_PULLIN_DUTY_PERMILLE   1000    /* 100% — full force to pull the armature in */
#define RELAY_HOLD_DUTY_PERMILLE     300     /* 30%  — reduced current once contact is closed */
#define RELAY_PULLIN_TIME_MS        150      /* Blind pull-in window before dropping to hold duty */

void Board_Set_Contactors(PIN_STATE state);
void Board_Set_Relay_Coil_Duty(uint16_t duty_permille);

void Board_Set_Actuator(ACTUATOR_STATE state);

ACTUATOR_STATE Board_Get_Actuator_State(void);

/* Cable lock: drives the H-bridge for one pulse, then de-energizes it */
void Board_Lock_Cable(void);
void Board_Unlock_Cable(void);

/* General-purpose status LED */
void Board_Set_LED(bool on);


bool Board_Is_K1_Closed(void);
bool Board_Is_K2_Closed(void);
bool Board_Is_K3_Closed(void);
bool Board_Is_K4_Closed(void);


#endif /* INC_BOARD_IO_H_ */
