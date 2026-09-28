/*
 * temp_sensor.c
 *
 *  Created on: Sep 28, 2026
 *      Author: Marcos Novo
 */

#include "temp_sensor.h"

/**
 * @brief Conf value written by LM75B_INIT().
 * - Normal mode (not shutdown): converts every ~100 ms.
 * - OS comparator mode: the OS level simply follows Temp vs Tos/Thyst, so it
 *   can be polled at any time. Interrupt mode is unusable here -- ANY register
 *   read (including our periodic temperature reads) resets it.
 * - OS active LOW: same as the power-on default, so a sensor brown-out/reset
 *   never silently inverts the meaning of the pin.
 * - Fault queue 4: ~400 ms (4 conversions) past a threshold before OS changes,
 *   to filter EMI from relay/contactor switching.
 */
#define LM75B_CONF_VALUE   (LM75B_CONF_FAULT_Q_4)

/**
 * @brief  Writes a Tos/Thyst set-point register.
 * @param  reg     LM75B_REG_TOS or LM75B_REG_THYST.
 * @param  temp_C  Threshold in whole degrees Celsius.
 * @note   Set-points are 9-bit two's complement, 0.5 C/LSB, left-aligned in
 *         bits [15:7] (Tables 11/12) -- so the MSByte is exactly the integer
 *         temperature in two's complement, and the LSByte is 0 for whole degrees.
 */
static HAL_StatusTypeDef lm75b_write_setpoint(uint8_t reg, int8_t temp_C){
	uint8_t buf[2] = { (uint8_t)temp_C, 0x00 };
	return HAL_I2C_Mem_Write(&hi2c1, LM75B_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, LM75B_I2C_TIMEOUT_MS);
}

/**
 * @brief  Detects and configures the LM75B.
 * @param  tos_C    Overtemperature threshold (C): OS asserts above this.
 * @param  thyst_C  Hysteresis threshold (C): OS releases below this. Must be < tos_C.
 * @return HAL_OK if the sensor answered and the configuration was read back correctly.
 * @note   Call after MX_I2C1_Init(). On failure the sensor (if present at all)
 *         keeps its power-on defaults: comparator mode, OS active LOW,
 *         Tos 80 C, Thyst 75 C, fault queue 1.
 */
HAL_StatusTypeDef LM75B_INIT(int8_t tos_C, int8_t thyst_C){
	HAL_StatusTypeDef status;
	uint8_t conf = LM75B_CONF_VALUE;
	int32_t discard;

	/* Datasheet 7.6: Tos <= Thyst leaves the OS output state undefined */
	if (thyst_C >= tos_C) {
		return HAL_ERROR;
	}

	/* An absent sensor NACKs the address here -- no separate probe needed */
	status = HAL_I2C_Mem_Write(&hi2c1, LM75B_ADDR, LM75B_REG_CONF, I2C_MEMADD_SIZE_8BIT, &conf, 1, LM75B_I2C_TIMEOUT_MS);
	if (status != HAL_OK) return status;

	status = lm75b_write_setpoint(LM75B_REG_TOS, tos_C);
	if (status != HAL_OK) return status;

	status = lm75b_write_setpoint(LM75B_REG_THYST, thyst_C);
	if (status != HAL_OK) return status;

	/* Read back Conf: confirms the device actually latched the config, not just ACKed it */
	conf = 0xFF;
	status = HAL_I2C_Mem_Read(&hi2c1, LM75B_ADDR, LM75B_REG_CONF, I2C_MEMADD_SIZE_8BIT, &conf, 1, LM75B_I2C_TIMEOUT_MS);
	if (status != HAL_OK) return status;
	if ((conf & LM75B_CONF_USED_MASK) != LM75B_CONF_VALUE) return HAL_ERROR;

	/* Datasheet 7.4.1: the first temperature reading after power-up is wrong -- discard it */
	return lm75b_get_temp_mC(&discard);
}

/**
 * @brief  Reads the current temperature.
 * @param  temp_mC Output, milli-degrees Celsius (0.125 C resolution). Untouched on failure.
 * @return HAL status of the I2C transaction.
 * @note   Temp is 11-bit two's complement, 0.125 C/LSB, left-aligned in bits
 *         [15:5] (Table 9). With the 5 LSBs masked off, dividing the signed
 *         16-bit word by 32 is exact -- and, unlike >>, well-defined for
 *         negative values.
 */
HAL_StatusTypeDef lm75b_get_temp_mC(int32_t *temp_mC){
	uint8_t buf[2];
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, LM75B_ADDR, LM75B_REG_TEMP, I2C_MEMADD_SIZE_8BIT, buf, 2, LM75B_I2C_TIMEOUT_MS);
	if (status != HAL_OK) return status;

	int16_t raw = (int16_t)((((uint16_t)buf[0] << 8) | buf[1]) & 0xFFE0);
	*temp_mC = (int32_t)(raw / 32) * 125;
	return HAL_OK;
}

/**
 * @brief  Reads the LM75B OS (overtemperature shutdown) output.
 * @return true while the temperature is above Tos and has not yet dropped
 *         below Thyst (comparator mode, active LOW -- see LM75B_CONF_VALUE).
 * @note   This is the sensor's own hardware thermostat: it keeps working even
 *         if the I2C bus is dead. OS is open-drain and needs a pull-up.
 */
bool lm75b_is_os_active(void){
	return (HAL_GPIO_ReadPin(TEMP_SENSOR_ALERT_PORT, TEMP_SENSOR_ALERT_PIN) == GPIO_PIN_RESET);
}
