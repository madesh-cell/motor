/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  PAD_STATE_IDLE    = 0,
  PAD_STATE_RUNNING = 1,
  PAD_STATE_FAULT   = 2
} Pad_State_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Raw counts per 1 mmHg, fitted to a gauge (about +/-3 mmHg gauge accuracy).
 * Valid for this sensor module only; each new sensor needs its own value. */
#define PAD_COUNTS_PER_MMHG            (75000)

/* Calibration mode: once the pump has stopped at the current target, each
 * button press raises the target by one step and continues from there. */
#define PAD_CAL_STEP_COUNTS            (1000000)   /* first target and each step   */
#define PAD_CAL_MAX_TARGET_COUNTS      (5000000)   /* highest target allowed       */
#define PAD_CAL_LIMIT_MARGIN_COUNTS    (500000)    /* fault above target + this    */

/* Hard cap in raw counts, independent of every other setting. */
#define PAD_ABSOLUTE_MAX_DELTA_COUNTS  (6000000)

#define PAD_PUMP_ON_TIMEOUT_MS         (60000U)
#define PAD_BASELINE_SAMPLES           (8U)
#define PAD_BUTTON_DEBOUNCE_MS         (30U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Short delay for the HX710B clock pulses (about 1-2 us at 16 MHz). */
static void SensorDelay(void)
{
  volatile uint32_t count = 20U;

  while (count > 0U)
  {
    count--;
  }
}

/* Read one 24-bit raw sample from the HX710B.
 * DOUT = PA9, SCK = PC7. Returns false if no sample arrives in 500 ms. */
static bool HX710B_ReadRaw(int32_t *value)
{
  uint32_t start_ms = HAL_GetTick();
  uint32_t raw      = 0U;
  uint8_t  i;

  /* DOUT goes LOW when a new sample is ready. */
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start_ms) > 500U)
    {
      return false;
    }
  }

  /* 24 data bits + 1 extra pulse (selects the 10 Hz input for the next read). */
  for (i = 0U; i < 25U; i++)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    SensorDelay();

    if (i < 24U)
    {
      raw <<= 1U;

      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
      {
        raw |= 1U;
      }
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    SensorDelay();
  }

  /* Sign-extend the 24-bit two's complement value. */
  if ((raw & 0x00800000U) != 0U)
  {
    raw |= 0xFF000000U;
  }

  *value = (int32_t)raw;
  return true;
}

/* Send a text string over the USB serial port. */
static void Pad_Print(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

/* Pump ON/OFF through TB6612FNG AIN1 (PB4). Green LED mirrors the pump. */
static void Pad_SetPump(bool pump_on)
{
  GPIO_PinState level = pump_on ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, level);
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, level);
}

/* Average several resting samples to find the zero point. */
static bool Pad_ReadBaseline(int32_t *baseline)
{
  int64_t sum = 0;
  int32_t sample = 0;
  uint8_t i;

  /* Discard the first sample; it can be stale after power-up. */
  if (!HX710B_ReadRaw(&sample))
  {
    return false;
  }

  for (i = 0U; i < PAD_BASELINE_SAMPLES; i++)
  {
    if (!HX710B_ReadRaw(&sample))
    {
      return false;
    }

    sum += sample;
  }

  *baseline = (int32_t)(sum / (int64_t)PAD_BASELINE_SAMPLES);
  return true;
}

/* Returns true once per button press (button is active LOW on PB5). */
static bool Pad_ButtonPressed(bool *was_pressed)
{
  bool is_low    = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
  bool new_press = false;

  if (is_low && !(*was_pressed))
  {
    HAL_Delay(PAD_BUTTON_DEBOUNCE_MS);

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      new_press = true;
    }
  }

  *was_pressed = is_low;
  return new_press;
}

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Pad_State_e pad_state          = PAD_STATE_IDLE;
  int32_t     baseline           = 0;
  int32_t     raw_value          = 0;
  int32_t     delta              = 0;
  int32_t     target_counts      = PAD_CAL_STEP_COUNTS;
  bool        pump_is_on         = false;
  uint32_t    pump_on_since_ms   = 0U;
  bool        button_was_pressed = false;
  char        message[96];
  int         message_len;

  Pad_SetPump(false);

  /* Tube must be open to air while the baseline is measured. */
  if (Pad_ReadBaseline(&baseline))
  {
    message_len = snprintf(message, sizeof(message), "baseline=%ld  (press button to START)\r\n", (long)baseline);
    Pad_Print(message);
  }
  else
  {
    pad_state = PAD_STATE_FAULT;
    Pad_Print("FAULT: no sensor data at start-up\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    bool button_press = Pad_ButtonPressed(&button_was_pressed);
    bool sensor_ok    = HX710B_ReadRaw(&raw_value);

    if (button_press)
    {
      if (pad_state == PAD_STATE_IDLE)
      {
        pad_state  = PAD_STATE_RUNNING;
        pump_is_on = false;
        Pad_Print("START\r\n");
      }
      else if ((pad_state == PAD_STATE_RUNNING) && (!pump_is_on))
      {
        /* Pump has stopped at the target: raise the target by one step. */
        if ((target_counts + PAD_CAL_STEP_COUNTS) <= PAD_CAL_MAX_TARGET_COUNTS)
        {
          target_counts += PAD_CAL_STEP_COUNTS;
          message_len = snprintf(message, sizeof(message),
                                 "TARGET UP: %ld counts\r\n", (long)target_counts);
          Pad_Print(message);
        }
        else
        {
          Pad_Print("MAX TARGET REACHED\r\n");
        }
      }
      else
      {
        /* Pump is running, or FAULT: stop and return to IDLE. */
        pad_state = PAD_STATE_IDLE;
        Pad_SetPump(false);
        pump_is_on = false;
        Pad_Print("STOP\r\n");
      }
    }

    if (pad_state == PAD_STATE_RUNNING)
    {
      if (!sensor_ok)
      {
        /* No valid reading means we cannot know the pressure: stop. */
        Pad_SetPump(false);
        pump_is_on = false;
        pad_state  = PAD_STATE_FAULT;
        Pad_Print("FAULT: no sensor data\r\n");
      }
      else
      {
        delta = raw_value - baseline;

        if ((delta >= (target_counts + PAD_CAL_LIMIT_MARGIN_COUNTS)) ||
            (delta >= PAD_ABSOLUTE_MAX_DELTA_COUNTS))
        {
          Pad_SetPump(false);
          pump_is_on = false;
          pad_state  = PAD_STATE_FAULT;
          Pad_Print("FAULT: pressure over limit\r\n");
        }
        else if (delta < target_counts)
        {
          if (!pump_is_on)
          {
            Pad_SetPump(true);
            pump_is_on       = true;
            pump_on_since_ms = HAL_GetTick();
          }
          else if ((HAL_GetTick() - pump_on_since_ms) > PAD_PUMP_ON_TIMEOUT_MS)
          {
            Pad_SetPump(false);
            pump_is_on = false;
            pad_state  = PAD_STATE_FAULT;
            Pad_Print("FAULT: target not reached in time\r\n");
          }
        }
        else
        {
          /* Target reached: pump off, keep watching the pressure. */
          Pad_SetPump(false);
          pump_is_on = false;
        }
      }
    }
    else
    {
      /* IDLE or FAULT: pump is always off. */
      Pad_SetPump(false);
      pump_is_on = false;
    }

    if (sensor_ok)
    {
      /* Estimated pressure in tenths of mmHg (integer math, no floats). */
      int32_t mmhg_tenths = (int32_t)(((int64_t)(raw_value - baseline) * 10) / PAD_COUNTS_PER_MMHG);
      char    sign        = (mmhg_tenths < 0) ? '-' : ' ';
      int32_t abs_tenths  = (mmhg_tenths < 0) ? -mmhg_tenths : mmhg_tenths;

      message_len = snprintf(message, sizeof(message),
                             "state=%d tgt=%ld raw=%ld delta=%ld ~%c%ld.%ld mmHg pump=%d\r\n",
                             (int)pad_state, (long)target_counts, (long)raw_value,
                             (long)(raw_value - baseline), sign,
                             (long)(abs_tenths / 10), (long)(abs_tenths % 10),
                             (int)pump_is_on);
      Pad_Print(message);
    }
  }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
