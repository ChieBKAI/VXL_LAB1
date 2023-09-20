/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display7SEG(int num, int vertical) {
    // Define the 7-segment patterns for each digit
    const uint8_t segmentPatterns[] = {
        0b11000000, // 0
        0b11111001, // 1
        0b10100100, // 2
        0b10110000, // 3
        0b10011001, // 4
        0b10010010, // 5
        0b10000010, // 6
        0b11111000, // 7
        0b10000000, // 8
        0b10010000  // 9
    };

    // Ensure the input number is within the valid range (0-9)
    if (num < 0 || num > 9) {
        // Handle invalid input
        return;
    }
    if (vertical == 1) {
        // Set the GPIO pins according to the 7-segment pattern for the digit
        HAL_GPIO_WritePin(SEG1_A_GPIO_Port, SEG1_A_Pin, (segmentPatterns[num] >> 0) & 1);
        HAL_GPIO_WritePin(SEG1_B_GPIO_Port, SEG1_B_Pin, (segmentPatterns[num] >> 1) & 1);
        HAL_GPIO_WritePin(SEG1_C_GPIO_Port, SEG1_C_Pin, (segmentPatterns[num] >> 2) & 1);
        HAL_GPIO_WritePin(SEG1_D_GPIO_Port, SEG1_D_Pin, (segmentPatterns[num] >> 3) & 1);
        HAL_GPIO_WritePin(SEG1_E_GPIO_Port, SEG1_E_Pin, (segmentPatterns[num] >> 4) & 1);
        HAL_GPIO_WritePin(SEG1_F_GPIO_Port, SEG1_F_Pin, (segmentPatterns[num] >> 5) & 1);
        HAL_GPIO_WritePin(SEG1_G_GPIO_Port, SEG1_G_Pin, (segmentPatterns[num] >> 6) & 1);
    }
    if (vertical == 0) {
        // Set the GPIO pins according to the 7-segment pattern for the digit
        HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, (segmentPatterns[num] >> 0) & 1);
        HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, (segmentPatterns[num] >> 1) & 1);
        HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, (segmentPatterns[num] >> 2) & 1);
        HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, (segmentPatterns[num] >> 3) & 1);
        HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, (segmentPatterns[num] >> 4) & 1);
        HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, (segmentPatterns[num] >> 5) & 1);
        HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, (segmentPatterns[num] >> 6) & 1);
    }
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int green_time = 300; // stage 1
  int yellow_time = 200; // stage 2
  int red_time = 500; // stage 3
  int count = -1;
  int count_vertical = -1;
  int stage = 4;
  while (1)
  {
      if (stage == 1) {
          if (count <= 0) {
              count = yellow_time;
              stage = 2;
              HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 0); // Turn on yellow 1
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1); // Turn off green 1
          }
      }
      if (stage == 2) {
          if (count <= 0) {
              count = red_time;
              count_vertical = green_time;
              stage = 3;
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0); // Turn on red 1
              HAL_GPIO_WritePin(LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, 0); // Turn on green 2
              HAL_GPIO_WritePin(LED_RED_2_GPIO_Port, LED_RED_2_Pin, 1); // Turn off red 2
              HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 1); // Turn off yellow 1
          }
      }
      if (stage == 3) {
    	  if (count_vertical <= 0) {
              count_vertical = yellow_time;
              stage = 4;
              HAL_GPIO_WritePin(LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, 1); // Turn off green 2
              HAL_GPIO_WritePin(LED_YELLOW_2_GPIO_Port, LED_YELLOW_2_Pin, 0); // Turn on yellow 2
          }
      }
      if (stage == 4) {
          if (count <= 0) {
              count = green_time;
              count_vertical = red_time;
              stage = 1;
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);
              HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 1);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
              HAL_GPIO_WritePin(LED_RED_2_GPIO_Port, LED_RED_2_Pin, 0);
              HAL_GPIO_WritePin(LED_YELLOW_2_GPIO_Port, LED_YELLOW_2_Pin, 1);
              HAL_GPIO_WritePin(LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, 1);
          }
      }
      display7SEG(count/100 + 1, 0);
      display7SEG(count_vertical/100 + 1, 1);
      count = count - 1;
      count_vertical = count_vertical - 1;
      HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG1_A_Pin|SEG1_B_Pin|SEG1_C_Pin|SEG1_D_Pin
                          |SEG1_E_Pin|SEG1_F_Pin|SEG1_G_Pin|LED_RED_Pin
                          |LED_YELLOW_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|LED_RED_2_Pin
                          |LED_YELLOW_2_Pin|LED_GREEN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG1_A_Pin SEG1_B_Pin SEG1_C_Pin SEG1_D_Pin
                           SEG1_E_Pin SEG1_F_Pin SEG1_G_Pin LED_RED_Pin
                           LED_YELLOW_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = SEG1_A_Pin|SEG1_B_Pin|SEG1_C_Pin|SEG1_D_Pin
                          |SEG1_E_Pin|SEG1_F_Pin|SEG1_G_Pin|LED_RED_Pin
                          |LED_YELLOW_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_C_Pin SEG_D_Pin
                           SEG_E_Pin SEG_F_Pin SEG_G_Pin LED_RED_2_Pin
                           LED_YELLOW_2_Pin LED_GREEN_2_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|LED_RED_2_Pin
                          |LED_YELLOW_2_Pin|LED_GREEN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
