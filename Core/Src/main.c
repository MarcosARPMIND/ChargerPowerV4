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
#define WORK_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t timer_seconds = 0;

extern volatile uint8_t RCD_Fault;

/* RCD Debounce Variables (non-static: shared with ISR in stm32c0xx_it.c) */
volatile uint32_t rcd_trigger_time  = 0;
volatile bool     rcd_pending_check = false;

STATE_MACHINE currentState = IDLE;


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
int main(void)
{

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
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


/*
  Board_Set_Contactors(CMD_DEACTIVATE);
  Board_Set_Contactors(CMD_ACTIVATE);
  HAL_Delay(1000);
  Board_Set_Contactors(CMD_DEACTIVATE);
*/
  
  //uint8_t state = HAL_GPIO_ReadPin(RELAY_STATE_1_PORT, RELAY_STATE_1_PIN);
  //uint8_t state1 = HAL_GPIO_ReadPin(RELAY_STATE_4_PORT, RELAY_STATE_4_PIN);

  /* Clear transient EXTI triggers that occur during power-on */
  RCD_Fault = 0;
  rcd_pending_check = false;
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
  
  CP_SetLine_High();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* -----------------------------------------------------------
   * MAIN APPLICATION LOOP
   *
   * Architecture: Task-Based Cyclic Executive
   * 1. Comms Task: Handles RS485/UART data intake and command dispatching.
   * 2. State Machine: Manages EVSE logic (CP signaling, Contactor control).
   * 3. Background Tasks: Energy metering, Safety monitoring.
   *
   *
   * ----------------------------------------------------------- */

  //APP_TEST_Contactor_Monitor();

  //volatile  uint32_t raw_A = ade7953_Read_Reg(ADE_DEVICE_1, 0x031A, 4);  // IRMSA
  //volatile  uint32_t raw_B = ade7953_Read_Reg(ADE_DEVICE_1, 0x031B, 4);  // IRMSB

  while (1)
  {




      // 0. RCD Debounce Validation
      if (rcd_pending_check) {
          if ((HAL_GetTick() - rcd_trigger_time) >= RCD_DEBOUNCE_MS) {
              /* Debounce window elapsed — re-read the physical pin */
              if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
                  /* Pin still HIGH after 30ms → confirmed real RCD fault */
                  RCD_Fault = 1;
              }
              rcd_pending_check = false;
          }
      }

      // 1. Process Communications
	  APP_Comms_Task();

      // 2. Main State Machine
      APP_MAIN(&currentState);

      // 3. Energy Monitoring
      APP_Energy_Task();

  //    HAL_IWDG_Refresh(&hiwdg);


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
 * @brief  EXTI Callback — RCD Fault Trigger (Debounced)
 * @note   Only records the timestamp of the trigger. Actual fault confirmation
 *         happens in the main loop after RCD_DEBOUNCE_MS has elapsed and the
 *         pin is re-read to filter out EMI noise spikes.
 */
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
    	RCD_Fault = 1;
    }
}
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM17) {
        // Esta flag dispara a cada 1 segundo (útil para RMS rápido/Proteção)
    	APP_Voltage_Flag_Set();

        timer_seconds++;
        if (timer_seconds >= 100) {
            timer_seconds = 0;
            // Esta flag dispara a cada 10 segundos (para Energia)
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
