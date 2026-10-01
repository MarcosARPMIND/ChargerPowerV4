/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "board_io.h"
#include "cp.h"
#include "comms_manager.h"
#include "energy_meter.h"
#include "app_manager.h"
#include <stdbool.h>
#include "stm32c0xx_it.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TIM17 ticks every 100 ms: voltage/current every tick, power/frequency every 10 s */
#define ENERGY_READ_EVERY_TICKS     100u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Persistent charger state, owned here and advanced by APP_MAIN() */
STATE_MACHINE currentState = IDLE;

/* TIM17 ticks since the last energy read (see HAL_TIM_PeriodElapsedCallback) */
static volatile uint8_t tim17_ticks = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  MX_IWDG_Init(); /* ~8.2s timeout (Prescaler/64, Reload 4095, LSI~32kHz) -- see iwdg.c */
  /* I2C1 not initialised: on this board PA9 (ex SCL) enables the K1/K4 relay
   * measurement circuit and PA10 (ex SDA) is the K2 weld-detector test input
   * -- see K1_MEAS_ENABLE_PIN / K2_WELD_TEST_PIN in app_config.h. */
  //MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  CP_ADC_Init();
  Comms_INIT();

  HAL_Delay(1000);
  HAL_TIM_Base_Start(&htim14);
  HAL_TIM_Base_Start_IT(&htim17);



  /* --- Hardware Reset for each ADE7953 (individual reset lines) --- */
  Board_ADE_Reset(ADE_DEVICE_1, CMD_ACTIVATE);
  Board_ADE_Reset(ADE_DEVICE_2, CMD_ACTIVATE);
  Board_ADE_Reset(ADE_DEVICE_3, CMD_ACTIVATE);
  HAL_Delay(15);
  Board_ADE_Reset(ADE_DEVICE_1, CMD_DEACTIVATE);
  Board_ADE_Reset(ADE_DEVICE_2, CMD_DEACTIVATE);
  Board_ADE_Reset(ADE_DEVICE_3, CMD_DEACTIVATE);
  HAL_Delay(100);
  Board_Set_SPI_CS(ADE_DEVICE_1, CMD_DEACTIVATE);
  Board_Set_SPI_CS(ADE_DEVICE_2, CMD_DEACTIVATE);
  Board_Set_SPI_CS(ADE_DEVICE_3, CMD_DEACTIVATE);
  ADE7953_INIT(ADE_DEVICE_1);
  ADE7953_INIT(ADE_DEVICE_2);
  ADE7953_INIT(ADE_DEVICE_3);

  ade7953_enable_reset_on_read(ADE_DEVICE_1);
  ade7953_enable_reset_on_read(ADE_DEVICE_2);
  ade7953_enable_reset_on_read(ADE_DEVICE_3);

  /* Start RCD trip confirmation only now: anything on the line during
   * power-on is ignored. A line still LOW at this point is not -- see
   * RCD_Monitor_Arm(). */
  RCD_Monitor_Arm();

  CP_SetLine_High();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* -----------------------------------------------------------
   * MAIN APPLICATION LOOP -- task-based cyclic executive
   * 1. Comms task:    RS485 intake, command dispatch, notify retries.
   * 2. State machine: EVSE logic (CP signalling, contactors, cable lock),
   *                   including the safety overrides (RCD, grid faults).
   * 3. Energy task:   ADE7953 readings, gated by the TIM17 flags.
   *
   * RCD trips are confirmed asynchronously by rcd_monitor.c (SysTick ISR)
   * and acted upon by APP_MAIN() with top priority over any state.
   * ----------------------------------------------------------- */
  while (1)
  {
      HAL_Delay(5);

      APP_Comms_Task();
      APP_MAIN(&currentState);
      APP_Energy_Task();

      HAL_IWDG_Refresh(&hiwdg);
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Timer period-elapsed callback -- paces APP_Energy_Task().
 * @note   TIM17 fires every 100 ms: Vrms/Irms are refreshed on every tick
 *         (fast enough for the grid/overcurrent checks in APP_MAIN()), active
 *         power and line frequency every ENERGY_READ_EVERY_TICKS (10 s).
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM17) {
    	APP_Voltage_Flag_Set();

        tim17_ticks++;
        if (tim17_ticks >= ENERGY_READ_EVERY_TICKS) {
            tim17_ticks = 0;
            APP_Energy_Flag_Set();
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
