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

/* Relay-coil economizer bench test (temporary, see USER CODE 2).
 * 1 = runs once at boot, before the main loop, to tune the RELAY_* limits
 * in board_io.h. Keep at 0 in anything pushed/flashed for normal use.
 * Needs mains on the relays' line side (the K1 feedback optocoupler only
 * sees a closed contact with AC across it) and NO vehicle connected: the
 * output is live every time the relays close. */
#define RELAY_COIL_TEST              0
#define RELAY_TEST_STEP_PERMILLE     10      /* Hold sweep step: 1.0% duty */
#define RELAY_TEST_DWELL_MS          1000    /* Per step: coil settling + optocoupler charge/discharge (100-200 ms) */
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



/* --- PC7 / TIM3_CH2 — relay-coil MOSFET economizer test ---
 * Edit this value live in the debugger (Watch: right-click -> Set Value,
 * or Debug Console: -exec set var test_mosfet_duty_permille = 300) to see
 * the effect on the relay/MOSFET without recompiling.
 * Range: 0-1000 (0.0% - 100.0%). TIM3 runs at 20 kHz (above audible, keeps
 * the coil current smooth) and is no longer shared with the CP, which is on
 * TIM1_CH1 at its own 1 kHz -- the two frequencies are independent.
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

#if RELAY_COIL_TEST
/* Economizer bench test results -- read in the debugger once test_relay_status != 0 */
volatile uint8_t  test_relay_status        = 0;  /* 0 running, 1 done, 2 K1 never closed even at 100% (mains on line side?),
                                                    3 K1 still reads closed with the coil off (welded / feedback stuck) */
volatile uint16_t test_relay_duty_now      = 0;  /* Coil duty currently applied (live progress) */
volatile uint16_t test_relay_min_hold      = 0;  /* Lowest hold duty at which K1 stayed closed */
volatile uint16_t test_relay_dropout       = 0;  /* First duty at which K1 opened (0 = held all the way down) */
volatile uint16_t test_relay_min_pullin_ms = 0;  /* Shortest 100% pulse that closed K1 and held at RELAY_HOLD_DUTY_PERMILLE (0 = none) */
#endif /* RELAY_COIL_TEST */

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
  //MX_IWDG_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* CP, PA8 -- also sets TIM1's MOE, without which the advanced timer drives no output */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* Relay-coil economizer, PC7 */
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

  /* --- LM75B temperature sensor (I2C1: PA9 SCL / PA10 SDA, OS on PD1) ---
   * Well past the sensor's first ~100ms conversion by now (HAL_Delay above). */
  APP_Temp_Init();

#if RELAY_COIL_TEST
  /* --- Relay-coil economizer bench test (temporary) ---
   * A) Hold sweep: close at 100%, then lower the hold duty in steps from
   *    RELAY_HOLD_DUTY_PERMILLE until K1 drops out.
   * B) Pull-in sweep: shortest 100% pulse that still closes K1 before
   *    dropping to RELAY_HOLD_DUTY_PERMILLE.
   * Runs before the RCD/EXTI clear below, so relay-switching EMI picked up
   * by the RCD line during the test is discarded there. */
  {
    static const uint16_t pullin_ms[] = { 10, 20, 30, 50, 75, 100, 150 };
    uint16_t duty;

    /* A) Full duty for the whole dwell: tells "no feedback at all" apart
     *    from "hold too weak" before the sweep starts. */
    test_relay_duty_now = RELAY_PULLIN_DUTY_PERMILLE;
    Board_Set_Relay_Coil_Duty(RELAY_PULLIN_DUTY_PERMILLE);
    HAL_Delay(RELAY_TEST_DWELL_MS);

    if (!Board_Is_K1_Closed()) {
      test_relay_status = 2;
    } else {
      for (duty = RELAY_HOLD_DUTY_PERMILLE; duty >= RELAY_TEST_STEP_PERMILLE; duty -= RELAY_TEST_STEP_PERMILLE) {
        test_relay_duty_now = duty;
        Board_Set_Relay_Coil_Duty(duty);
        HAL_Delay(RELAY_TEST_DWELL_MS);
        if (!Board_Is_K1_Closed()) {
          test_relay_dropout = duty;
          break;
        }
        test_relay_min_hold = duty;
      }
    }

    /* B) Each attempt starts fully open, with the optocoupler discharged */
    for (uint8_t i = 0; test_relay_status == 0 && i < sizeof(pullin_ms) / sizeof(pullin_ms[0]); i++) {
      test_relay_duty_now = 0;
      Board_Set_Relay_Coil_Duty(0);
      HAL_Delay(RELAY_TEST_DWELL_MS);
      if (Board_Is_K1_Closed()) {
        test_relay_status = 3;
        break;
      }

      test_relay_duty_now = RELAY_PULLIN_DUTY_PERMILLE;
      Board_Set_Relay_Coil_Duty(RELAY_PULLIN_DUTY_PERMILLE);
      HAL_Delay(pullin_ms[i]);
      test_relay_duty_now = RELAY_HOLD_DUTY_PERMILLE;
      Board_Set_Relay_Coil_Duty(RELAY_HOLD_DUTY_PERMILLE);
      HAL_Delay(RELAY_TEST_DWELL_MS);
      if (Board_Is_K1_Closed()) {
        test_relay_min_pullin_ms = pullin_ms[i];
        break;
      }
    }

    test_relay_duty_now = 0;
    Board_Set_Relay_Coil_Duty(0);    /* never leave the relays energized */
    if (test_relay_status == 0) {
      test_relay_status = 1;
    }
  }
#endif /* RELAY_COIL_TEST */


  /* Clear transient EXTI triggers that occur during power-on */
  RCD_Fault = 0;
  rcd_pending_check = false;
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);

  CP_SetLine_High();

  // /* --- Relay feedback diagnostic (temporary) --- */
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

  // Board_Set_Contactors(CMD_DEACTIVATE);   /* open again -- don't leave energized */
  // HAL_Delay(300);                          /* let AC optocouplers discharge, same margin as APP_Verify_Relays_Open() */
  // diag_k1_after = Board_Is_K1_Closed();
  // diag_k2_after = Board_Is_K2_Closed();
  // diag_k3_after = Board_Is_K3_Closed();
  // diag_k4_after = Board_Is_K4_Closed();

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

      /* RCD_Fault (once confirmed above) is handled inside APP_MAIN() itself,
       * with top priority over any state -- it opens the contactors, unlocks
       * the cable once relays are confirmed open, and transitions to
       * FAULT_RCD. No need to react to it here too. */


      HAL_Delay(100);

      // --- Normal operation (disabled during bring-up test) ---
       APP_Comms_Task();
       APP_MAIN(&currentState);
       APP_Energy_Task();
       APP_Temp_Task();

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
