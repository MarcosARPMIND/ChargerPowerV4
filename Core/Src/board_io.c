/*
 * board_io.c
 *
 *  Created on: Nov 25, 2025
 *      Author: ARPMindTech
 */


#include "board_io.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"

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
 * @brief  Sets the PWM duty driving the relay-coil MOSFET (PC7 / TIM3_CH2).
 * @param  duty_permille 0-1000 (0.0% to 100.0%), clamped.
 * @note   Normal code goes through Board_Set_Contactors() (pull-in, then hold).
 *         Public only for the economizer bench test in main.c.
 */
void Board_Set_Relay_Coil_Duty(uint16_t duty_permille){
	if (duty_permille > 1000) { duty_permille = 1000; }

	uint32_t period_ticks = TIM3->ARR + 1;
	TIM3->CCR2 = (uint32_t)(((uint64_t)duty_permille * period_ticks) / 1000);
}

/**
 * @brief  Controls Power Contactors via the relay-coil MOSFET (PC7 / TIM3_CH2, PWM).
 * @param  state CMD_ACTIVATE (Close relays: pull-in at 100%, then hold at 30%)
 *               or CMD_DEACTIVATE (Open relays: cut coil current).
 * @note   PC7 is an alternate-function PWM pin, not a plain GPIO output —
 *         this drives TIM3->CCR2 directly instead of HAL_GPIO_WritePin.
 */
void Board_Set_Contactors(PIN_STATE state){
	switch(state){
		case CMD_ACTIVATE:
			Board_Set_Relay_Coil_Duty(RELAY_PULLIN_DUTY_PERMILLE);
			HAL_Delay(RELAY_PULLIN_TIME_MS);
			Board_Set_Relay_Coil_Duty(RELAY_HOLD_DUTY_PERMILLE);
			break;

		case CMD_DEACTIVATE:
		default:
			Board_Set_Relay_Coil_Duty(0);
			break;
	}
}
/**
 * @brief  Controls the Cable Lock Actuator (H-Bridge).
 * @param  state OPEN (Unlock) or CLOSE (Lock).
 * @note   The combination of both H-Bridge pins sets the direction.
 */

void Board_Set_Actuator(ACTUATOR_STATE state){
	switch(state){
		case CLOSE:
			HAL_GPIO_WritePin(LOCK_HBRIDGE_1_PORT,LOCK_HBRIDGE_1_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LOCK_HBRIDGE_2_PORT,LOCK_HBRIDGE_2_PIN,GPIO_PIN_RESET);
			break;

		case OPEN:
			HAL_GPIO_WritePin(LOCK_HBRIDGE_1_PORT,LOCK_HBRIDGE_1_PIN,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LOCK_HBRIDGE_2_PORT,LOCK_HBRIDGE_2_PIN,GPIO_PIN_SET);
			break;

		case OFF:
			HAL_GPIO_WritePin(LOCK_HBRIDGE_1_PORT,LOCK_HBRIDGE_1_PIN,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LOCK_HBRIDGE_2_PORT,LOCK_HBRIDGE_2_PIN,GPIO_PIN_RESET);
			break;
	}
}

/** 
 * @brief  Reads the physical state of the Cable Lock Actuator (H-Bridge feedback).
 * @return CLOSE if the lock is engaged, OPEN if it is disengaged.
 */

ACTUATOR_STATE Board_Get_Actuator_State(void){

	if(HAL_GPIO_ReadPin(CABLE_LOCK_STATE_PORT, CABLE_LOCK_STATE_PIN) == GPIO_PIN_SET){
		return CLOSE;
	}
	else{
		return OPEN;
	}

}

/* Drive time for the lock H-bridge -- tune to the actuator's actual travel time */
#define CABLE_LOCK_PULSE_MS   500

/**
 * @brief  Locks the cable: drives the actuator CLOSED, then de-energizes it.
 * @note   Call BEFORE energizing the power contactors (see APP_MAIN CHARGING
 *         entry action), so the connector can't be pulled while live.
 */
void Board_Lock_Cable(void){
	Board_Set_Actuator(CLOSE);
	HAL_Delay(CABLE_LOCK_PULSE_MS);
	Board_Set_Actuator(OFF);
}

/**
 * @brief  Unlocks the cable: drives the actuator OPEN, then de-energizes it.
 * @note   Call AFTER the power contactors are confirmed open (see APP_MAIN
 *         CHARGING exit action) -- never unlock while the relays might still
 *         be welded closed.
 */
void Board_Unlock_Cable(void){
	Board_Set_Actuator(OPEN);
	HAL_Delay(CABLE_LOCK_PULSE_MS);
	Board_Set_Actuator(OFF);
}

/**
 * @brief  Drives the general-purpose status LED.
 * @param  on true = LED on, false = LED off.
 */
void Board_Set_LED(bool on){
	HAL_GPIO_WritePin(GENERAL_LED_PORT, GENERAL_LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
