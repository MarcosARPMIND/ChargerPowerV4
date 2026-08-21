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

void Board_init(void);

void Board_ADE_Reset_1(PIN_STATE state);
void Board_ADE_Reset_2(PIN_STATE state);
void Board_ADE_Reset_3(PIN_STATE state);

void Board_Set_SPI_CS_1(PIN_STATE state);
void Board_Set_SPI_CS_2(PIN_STATE state);
void Board_Set_SPI_CS_3(PIN_STATE state);

void Board_Set_RS485_DE(DE_STATE state);

void Board_Set_Contactors(PIN_STATE state);


bool Board_Is_K1_Closed(void);
bool Board_Is_K2_Closed(void);
bool Board_Is_K3_Closed(void);
bool Board_Is_K4_Closed(void);

void Board_SetRCD(bool state);




#endif /* INC_BOARD_IO_H_ */
