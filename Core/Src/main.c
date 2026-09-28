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

/* RCD Blanking Window (non-static: set from app_manager.c's relay
 * verification routines). Bench testing found the RCD module's fault
 * pulse can start with some latency relative to the K1/K4 sense-circuit
 * activation edge -- and lasts ~60ms -- so clearing RCD_Fault/rcd_pending_check
 * only once, right when the verification window closes, isn't reliable:
 * the pulse can still be in flight, or hasn't even started yet, at that
 * exact instant. This timestamp instead suppresses *any* confirmation
 * (see the debounce block below) until well after it, regardless of when
 * exactly within the window the pulse actually occurs. */
volatile uint32_t rcd_blank_until_tick = 0;

STATE_MACHINE currentState = IDLE;



/* --- PC7 / TIM3_CH2 — relay-coil MOSFET economizer test ---
 * Edit this value live in the debugger (Watch: right-click -> Set Value,
 * or Debug Console: -exec set var test_mosfet_duty_permille = 300) to see
 * the effect on the relay/MOSFET without recompiling.
 * Range: 0-1000 (0.0% - 100.0%). Shares TIM3's 1 kHz base with the CP
 * signal on CH3 -- only the duty is independent, not the frequency.
 */

/* Cable lock actuator feedback — volatile so it survives -Og and can be
 * watched with Live Watch/Live Expressions without halting at a breakpoint
 * (a breakpoint stops the whole CPU, so the H-bridge would stop moving too). */
volatile ACTUATOR_STATE actuator_state = OPEN;

#if ENABLE_PP_SENSE
/* Proximity Pilot (PP) line voltage, PA11/ADC1_IN11 -- see PP_Read_mV() in cp.c */
volatile uint16_t test_pp_mV = 0;
volatile uint8_t  test_pp_amps = 0;   /* 13/20/32/63, or 0 if no cable/unknown */
#endif /* ENABLE_PP_SENSE */

/* --- Relay feedback diagnostic (temporary) ---
 * Reads K1-K4 mirror contacts BEFORE commanding the relays, then again once
 * closed, then again once re-opened -- to isolate whether any relay
 * (K4 in particular) reads closed when it shouldn't (wiring/pull config on
 * RELAY_STATE_x). Runs once at boot, before the main loop. Inspect in the
 * debugger (Watch / Live Expressions). Remove once the wiring is confirmed.
 */
volatile bool diag_k1_before, diag_k2_before, diag_k3_before, diag_k4_before;
volatile bool diag_k1_closed, diag_k2_closed, diag_k3_closed, diag_k4_closed;
volatile bool diag_k1_after,  diag_k2_after,  diag_k3_after,  diag_k4_after;

/* TEMP: same "closed" read as diag_k1/k4_closed above, but taken ~2s later
 * instead of ~250ms after commanding the relay closed -- isolates whether
 * the K1/K4 measurement circuit just needs longer than expected to settle
 * (if these read true while diag_k1/k4_closed above read false, it's a
 * timing issue, not a wiring/logic one). Remove once settling time is
 * confirmed and RELAY_MEAS_ENABLE_SETTLING_MS is set accordingly. */
volatile bool diag_k1_closed_slow, diag_k4_closed_slow;

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
  /* I2C1 disabled: PA9/PA10 (SCL/SDA) are now the K1/K4 relay
   * measurement-circuit enable lines -- see K1/K4_MEAS_ENABLE_PIN in
   * app_config.h. The temperature sensor this bus was reserved for was
   * never implemented. */
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



  


  /* Clear transient EXTI triggers that occur during power-on */
  RCD_Fault = 0;
  rcd_pending_check = false;
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);

  CP_SetLine_High();

  /* --- Relay feedback diagnostic (temporary) ---
   * K1/K4 sense circuits need Board_Enable_Relay_Measurement(CMD_ACTIVATE)
   * before their reading means anything (see board_io.c) -- enabled once
   * for this whole one-shot boot sequence, disabled again at the end. */

  // Board_Enable_Relay_Measurement(CMD_ACTIVATE);
  // HAL_Delay(RELAY_MEAS_ENABLE_SETTLING_MS);

  // Board_Set_Contactors(CMD_DEACTIVATE);   /* baseline: force open first */
  // HAL_Delay(100);
  // diag_k1_before = Board_Is_K1_Closed();
  // diag_k2_before = Board_Is_K2_Closed();
  // diag_k3_before = Board_Is_K3_Closed();
  // diag_k4_before = Board_Is_K4_Closed();

  // Board_Set_Contactors(CMD_ACTIVATE);     /* pull-in 150ms -> 30% hold, see board_io.c */
  // HAL_Delay(100);                          /* extra settling beyond the pull-in/hold above */
  // diag_k1_closed = Board_Is_K1_Closed();
  // diag_k2_closed = Board_Is_K2_Closed();
  // diag_k3_closed = Board_Is_K3_Closed();
  // diag_k4_closed = Board_Is_K4_Closed();


  // Board_Enable_Relay_Measurement(CMD_DEACTIVATE);

  // diag_k1_closed = Board_Is_K1_Closed();
  // diag_k2_closed = Board_Is_K2_Closed();
  // diag_k3_closed = Board_Is_K3_Closed();
  // diag_k4_closed = Board_Is_K4_Closed();

  // Board_Set_Contactors(CMD_DEACTIVATE); 

  // diag_k1_closed = Board_Is_K1_Closed();
  // diag_k2_closed = Board_Is_K2_Closed();
  // diag_k3_closed = Board_Is_K3_Closed();
  // diag_k4_closed = Board_Is_K4_Closed();



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



  while (1)
  {


      // 0. RCD Debounce Validation
#if RCD_BLANK_DURING_RELAY_VERIFICATION
      if (rcd_pending_check && HAL_GetTick() < rcd_blank_until_tick) {
          /* Still inside a relay-verification blanking window (see
           * rcd_blank_until_tick) -- this edge is attributable to the K1/K4
           * sense-circuit activation, not a real residual-current event. */
          rcd_pending_check = false;
      }
#endif
      if (rcd_pending_check) {
          if ((HAL_GetTick() - rcd_trigger_time) >= RCD_DEBOUNCE_MS) {
              /* Debounce window elapsed — re-read the physical pin.
               * RCD module output is open-collector: idle/OK = pulled HIGH,
               * tripped = pulled LOW (0V). */
              if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
                  /* Pin still LOW after RCD_DEBOUNCE_MS → confirmed real RCD fault */
                  RCD_Fault = 1;
              }
              rcd_pending_check = false;
          }
      }

      /* RCD_Fault (once confirmed above) is handled inside APP_MAIN() itself,
       * with top priority over any state -- it opens the contactors, unlocks
       * the cable once relays are confirmed open, and transitions to
       * FAULT_RCD. No need to react to it here too. */


      HAL_Delay(5);

      // --- Normal operation (disabled during bring-up test) ---
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
