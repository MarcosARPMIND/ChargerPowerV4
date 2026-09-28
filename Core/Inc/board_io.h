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

void Board_Set_Contactors(PIN_STATE state);

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

/* K1/K4 feedback needs their sense circuit powered before the reading is
 * valid -- CMD_ACTIVATE enables both, CMD_DEACTIVATE disables both. Call
 * before reading Board_Is_K1_Closed()/Board_Is_K4_Closed() (after the
 * settling delay), and disable again once done reading. */
void Board_Enable_Relay_Measurement(PIN_STATE state);

/* Bench diagnostics: force the K1/K4 measurement circuits permanently on
 * (or release the override) via the Master -- see CMD_RELAY_MEAS_SET. */
void Board_Force_Relay_Measurement(bool force_on);


#endif /* INC_BOARD_IO_H_ */
