/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|h_bridge_2_Pin
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level -- PA9, K1/K4 measurement-circuit
     enable (active HIGH): default RESET/inactive, only driven HIGH around
     a relay-feedback check (see Board_Enable_Relay_Measurement). PA10 is
     no longer an output -- see K2_WELD_TEST_OVERRIDE below. */
  HAL_GPIO_WritePin(GPIOA, K1_MEAS_ENABLE_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(general_LED_GPIO_Port, general_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(h_bridge_1_GPIO_Port, h_bridge_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  /* RCD module output is open-collector: idle/OK = pulled HIGH, tripped =
   * pulled LOW (0V). Trigger on the falling edge (the trip itself) rather
   * than the rising edge (which only fires on recovery); PULLUP keeps the
   * line at a defined HIGH level so it doesn't float (and pick up noise)
   * whenever the module's transistor is off. */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA5 h_bridge_2_Pin
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|h_bridge_2_Pin
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 -- K2/K3 relay feedback (RELAY_STATE_2/3)
     Pull-down: an open contact must read a clean LOW, not float and pick up
     noise (see relay-feedback investigation -- false "closed" readings). */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 -- K4/K1 relay feedback (RELAY_STATE_4/1)
     Pull-down for the same reason as PA6/PA7 above. Both are only valid
     while the K1/K4 sense circuit is powered (Board_Enable_Relay_Measurement)
     -- without it PB0 read permanently HIGH, which is what the old
     IGNORE_K4_FEEDBACK workaround was hiding. */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 -- K1/K4 relay measurement-circuit enable
     (repurposed I2C1_SCL, see app_config.h). Plain push-pull output, driven
     by Board_Enable_Relay_Measurement() around each relay-feedback check --
     I2C1 is disabled (main.c) so nothing else drives this pin. Single pin
     now enables both K1/K4 sense circuits (hardware revision); PA10
     (K4_MEAS_ENABLE_PIN, ex I2C1_SDA) is no longer driven -- see below, it
     is temporarily repurposed as an input for K2_WELD_TEST_OVERRIDE. */
  GPIO_InitStruct.Pin = K1_MEAS_ENABLE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 -- TEMPORARY (K2_WELD_TEST_OVERRIDE,
     feature_config.h) K2 phase-2 weld-detector input, bench-test stand-in
     for RELAY_STATE_2 (PA6) while the hardware fix is pending. Pull-down
     for the same reason as the other relay-feedback inputs -- an open
     contact must read a clean LOW, not float and pick up noise. */
  GPIO_InitStruct.Pin = K2_WELD_TEST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : cable_lock_Pin button_2_ext_Pin
                           PB12 PB13 PB14 button_1_ext_Pin */
  GPIO_InitStruct.Pin = cable_lock_Pin|button_2_ext_Pin
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|button_1_ext_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : general_LED_Pin */
  GPIO_InitStruct.Pin = general_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(general_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : h_bridge_1_Pin */
  GPIO_InitStruct.Pin = h_bridge_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(h_bridge_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : os_temp_Pin */
  GPIO_InitStruct.Pin = os_temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(os_temp_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
