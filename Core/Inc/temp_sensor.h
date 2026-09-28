/*
 * temp_sensor.h
 *
 *  Created on: Sep 28, 2026
 *      Author: Marcos Novo
 *
 *  NXP LM75B digital temperature sensor + thermal watchdog.
 *  I2C1: PA9 = SCL (pin 29), PA10 = SDA (pin 32). OS output: PD1 (pin 39).
 *  Based on LM75B Datasheet Rev. 6.1.
 */

#ifndef INC_TEMP_SENSOR_H_
#define INC_TEMP_SENSOR_H_


/* --- Standard Includes --- */
#include <stdint.h>
#include <stdbool.h>

/* --- Hardware Abstraction Includes --- */
#include "i2c.h"
#include "app_config.h"


/* --- Protocol Constants --- */
/**
 * @brief 7-bit slave address: fixed 1001 + A2 A1 A0 pins (datasheet Table 4).
 * 0x48 = A2..A0 all tied to GND -- change if the PCB straps them differently.
 */
#define LM75B_ADDR_7BIT          0x48
#define LM75B_ADDR               (LM75B_ADDR_7BIT << 1)   /* HAL expects the address left-aligned */

/**
 * @brief I2C Timeout definition.
 * At 100 kHz a 2-byte register read (addr + pointer + restart + addr + 2 data)
 * takes ~0.5 ms. 10 ms leaves ample margin without stalling the main loop for
 * long if the bus is stuck (the LM75B itself releases SDA after 75-200 ms).
 */
#define LM75B_I2C_TIMEOUT_MS     10

/* =================================================================================
 * REGISTER MAP
 * Pointer values, datasheet Table 5
 * ================================================================================= */
#define LM75B_REG_TEMP           0x00  // Temperature (16-bit, read-only, 11-bit data)
#define LM75B_REG_CONF           0x01  // Configuration (8-bit)
#define LM75B_REG_THYST          0x02  // Hysteresis threshold (16-bit, 9-bit data)
#define LM75B_REG_TOS            0x03  // Overtemperature shutdown threshold (16-bit, 9-bit data)

/* --- Conf register bits (datasheet Table 8) --- */
#define LM75B_CONF_SHUTDOWN      (1U << 0)  // 1 = shutdown, 0 = normal (converts every 100 ms)
#define LM75B_CONF_OS_INT        (1U << 1)  // 1 = OS interrupt mode, 0 = comparator (thermostat)
#define LM75B_CONF_OS_POL_HIGH   (1U << 2)  // 1 = OS active HIGH, 0 = active LOW
#define LM75B_CONF_FAULT_Q_1     (0U << 3)  // OS fault queue: consecutive out-of-limit
#define LM75B_CONF_FAULT_Q_2     (1U << 3)  // conversions needed before OS changes state
#define LM75B_CONF_FAULT_Q_4     (2U << 3)
#define LM75B_CONF_FAULT_Q_6     (3U << 3)
#define LM75B_CONF_USED_MASK     0x1F       // B[7:5] reserved

typedef struct {
	int32_t temp_mC;      /**< Last good reading, milli-degrees Celsius (0.125 C resolution) */
	bool    valid;        /**< false until the first good read, or after TEMP_MAX_READ_ERRORS consecutive failures */
	bool    configured;   /**< LM75B_INIT() succeeded: Conf/Tos/Thyst written and verified */
	bool    os_active;    /**< OS pin: above Tos and not yet back below Thyst (hardware thermostat) */
	uint8_t read_errors;  /**< Consecutive failed reads (saturates at 255) */
} Temp_Data_t;

/* =================================================================================
 * PUBLIC FUNCTIONS
 * ================================================================================= */

// Initialization
HAL_StatusTypeDef LM75B_INIT(int8_t tos_C, int8_t thyst_C);

// Readings
HAL_StatusTypeDef lm75b_get_temp_mC(int32_t *temp_mC);

// Status Checks
bool lm75b_is_os_active(void);

#endif /* INC_TEMP_SENSOR_H_ */
