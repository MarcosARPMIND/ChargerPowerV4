/*
 * board_io.c
 *
 *  Created on: Nov 25, 2025
 *      Author: ARPMindTech
 */


#include "board_io.h"
#include "stm32c0xx_hal.h"

/**
 * @brief  Initializes Board I/O Logic.
 * @note   This function can be used to set default states for pins if not handled by HAL_GPIO_Init.
 *         Currently reserved for future expansion.
 */

void Board_init(void){

}

/**
 * @brief  Controls the Hardware Reset Pin for ADE7953 Device 1 (Phase 1).
 * @param  state CMD_ACTIVATE (Low) to Reset, CMD_DEACTIVATE (High) to Run.
 */
void Board_ADE_Reset_1(PIN_STATE state){
	switch(state){
		case CMD_ACTIVATE:
			HAL_GPIO_WritePin(ADE_RESET_1_PORT,ADE_RESET_1_PIN,GPIO_PIN_RESET);
			break;

		case CMD_DEACTIVATE:
			HAL_GPIO_WritePin(ADE_RESET_1_PORT,ADE_RESET_1_PIN,GPIO_PIN_SET);
			break;
	}
}

/**
 * @brief  Controls the Hardware Reset Pin for ADE7953 Device 2 (Phase 2).
 * @param  state CMD_ACTIVATE (Low) to Reset, CMD_DEACTIVATE (High) to Run.
 */
void Board_ADE_Reset_2(PIN_STATE state){
	switch(state){
		case CMD_ACTIVATE:
			HAL_GPIO_WritePin(ADE_RESET_2_PORT,ADE_RESET_2_PIN,GPIO_PIN_RESET);
			break;

		case CMD_DEACTIVATE:
			HAL_GPIO_WritePin(ADE_RESET_2_PORT,ADE_RESET_2_PIN,GPIO_PIN_SET);
			break;
	}
}

/**
 * @brief  Controls the Hardware Reset Pin for ADE7953 Device 3 (Phase 3).
 * @param  state CMD_ACTIVATE (Low) to Reset, CMD_DEACTIVATE (High) to Run.
 */
void Board_ADE_Reset_3(PIN_STATE state){
	switch(state){
		case CMD_ACTIVATE:
			HAL_GPIO_WritePin(ADE_RESET_3_PORT,ADE_RESET_3_PIN,GPIO_PIN_RESET);
			break;

		case CMD_DEACTIVATE:
			HAL_GPIO_WritePin(ADE_RESET_3_PORT,ADE_RESET_3_PIN,GPIO_PIN_SET);
			break;
	}
}

/**
 * @brief  Controls Chip Select for SPI Device 1.
 * @param  state CMD_ACTIVATE (Low) or CMD_DEACTIVATE (High).
 */
void Board_Set_SPI_CS_1(PIN_STATE state){
	// SPI Device is activate when CS PIN LOW
	switch(state){
		case CMD_ACTIVATE:
			HAL_GPIO_WritePin(SPI_CS_1_PORT,SPI_CS_1_PIN,GPIO_PIN_RESET);
			break;

		case CMD_DEACTIVATE:
			HAL_GPIO_WritePin(SPI_CS_1_PORT,SPI_CS_1_PIN,GPIO_PIN_SET);
			break;

		default:
			HAL_GPIO_WritePin(SPI_CS_1_PORT,SPI_CS_1_PIN,GPIO_PIN_SET);
			break;
	}
}

/**
 * @brief  Controls Chip Select for SPI Device 2.
 * @param  state CMD_ACTIVATE (Low) or CMD_DEACTIVATE (High).
 */
void Board_Set_SPI_CS_2(PIN_STATE state){
	// SPI Device is activate when CS PIN LOW
	switch(state){
		case CMD_ACTIVATE:
			HAL_GPIO_WritePin(SPI_CS_2_PORT,SPI_CS_2_PIN,GPIO_PIN_RESET);
			break;

		case CMD_DEACTIVATE:
			HAL_GPIO_WritePin(SPI_CS_2_PORT,SPI_CS_2_PIN,GPIO_PIN_SET);
			break;

		default:
			HAL_GPIO_WritePin(SPI_CS_2_PORT,SPI_CS_2_PIN,GPIO_PIN_SET);
			break;
	}
}

/**
 * @brief  Controls Chip Select for SPI Device 3.
 * @param  state CMD_ACTIVATE (Low) or CMD_DEACTIVATE (High).
 */
void Board_Set_SPI_CS_3(PIN_STATE state){
	// SPI Device is activate when CS PIN LOW
	switch(state){
		case CMD_ACTIVATE:
			HAL_GPIO_WritePin(SPI_CS_3_PORT,SPI_CS_3_PIN,GPIO_PIN_RESET);
			break;

		case CMD_DEACTIVATE:
			HAL_GPIO_WritePin(SPI_CS_3_PORT,SPI_CS_3_PIN,GPIO_PIN_SET);
			break;

		default:
			HAL_GPIO_WritePin(SPI_CS_3_PORT,SPI_CS_3_PIN,GPIO_PIN_SET);
			break;
	}
}

/**
 * @brief  Controls RS485 Transceiver Direction.
 * @param  state SEND (TX Enable) or RECEIVER (RX Enable).
 */
void Board_Set_RS485_DE(DE_STATE state){

	switch(state){
		case SEND:
			HAL_GPIO_WritePin(RS485_DE_PORT,RS485_DE_PIN,GPIO_PIN_SET);
			break;

		case RECEIVER:
			HAL_GPIO_WritePin(RS485_DE_PORT,RS485_DE_PIN,GPIO_PIN_RESET);
			break;

		default:
			HAL_GPIO_WritePin(RS485_DE_PORT,RS485_DE_PIN,GPIO_PIN_RESET);
			break;
	}

}
/**
 * @brief  Controls Power Contactors.
 * @param  state CMD_ACTIVATE (Close Relays) or CMD_DEACTIVATE (Open Relays).
 */
void Board_Set_Contactors(PIN_STATE state){
	switch(state){
		case CMD_ACTIVATE:
			HAL_GPIO_WritePin(RELAYS_PORT,RELAYS_PIN,GPIO_PIN_SET);
			break;

		case CMD_DEACTIVATE:
			HAL_GPIO_WritePin(RELAYS_PORT,RELAYS_PIN,GPIO_PIN_RESET);
			break;

		default:
			HAL_GPIO_WritePin(RELAYS_PORT,RELAYS_PIN,GPIO_PIN_RESET);
			break;
	}
}



/**
 * @brief  Reads the physical state of Contactor K1 (Mirror Contact).
 * @return true if Closed (Conducting), false if Open.
 */
bool Board_Is_K1_Closed(void){
	return (HAL_GPIO_ReadPin(RELAY_STATE_1_PORT,RELAY_STATE_1_PIN) == GPIO_PIN_SET);
}
/**
 * @brief  Reads the physical state of Contactor K2 (Mirror Contact).
 * @return true if Closed (Conducting), false if Open.
 */
bool Board_Is_K2_Closed(void){
	return (HAL_GPIO_ReadPin(RELAY_STATE_2_PORT,RELAY_STATE_2_PIN) == GPIO_PIN_SET);
}
/**
 * @brief  Reads the physical state of Contactor K3 (Mirror Contact).
 * @return true if Closed (Conducting), false if Open.
 */
bool Board_Is_K3_Closed(void){
	return (HAL_GPIO_ReadPin(RELAY_STATE_3_PORT,RELAY_STATE_3_PIN) == GPIO_PIN_SET);
}
/**
 * @brief  Reads the physical state of Contactor K4 (Mirror Contact).
 * @return true if Closed (Conducting), false if Open.
 */
bool Board_Is_K4_Closed(void){
	return (HAL_GPIO_ReadPin(RELAY_STATE_4_PORT,RELAY_STATE_4_PIN) == GPIO_PIN_SET);
}

/**
 * @brief  Controls the RCD (Residual Current Device) Test Output.
 * @param  state true to TRIGGER test (create fault), false to idle.
 */
void Board_SetRCD(bool state){
	HAL_GPIO_WritePin(RCD_TEST_PORT,RCD_TEST_PIN, state);
}
