/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

  void display7SEG1(int num) {
        // Set = turn off, Reset = turn on
        switch (num) {
            case 0:
                HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_RESET);
                break;
            case 1:
                HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                break;
            case 2:
                HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_SET);
                break;
            case 3:
                HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_RESET);
                break;
            case 4:
                HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                break;
            case 5:
                HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_RESET);
                              break;
                          case 6:
                              HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_SET);
                              HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_RESET);
                              break;
                          case 7:
                              HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_SET);
                              HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_SET);
                              HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_SET);
                              HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_SET);
                              HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                              HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                              break;
                          case 8:
                			  HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                			  break;
                		  case 9:
                			  HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_SET);
                			  HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_RESET);
                			  HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_RESET);
                			  break;
                		  default:
                			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_All, GPIO_PIN_SET);
                			  break;
                	  }
                  }


		void display7SEG2(int num) {
		  // Set = turn off, Reset = turn on
		  switch (num) {
			  case 0:
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_RESET);
				  break;
			  case 1:
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  break;
			  case 2:
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_SET);
				  break;
			  case 3:
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_RESET);
				  break;
			  case 4:
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  break;
			  case 5:
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_RESET);
				  break;
			  case 6:
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_RESET);
				  break;
			  case 7:
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  break;
			  case 8:
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  break;
			  case 9:
				  HAL_GPIO_WritePin(l_GPIO_Port, l_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(h_GPIO_Port, h_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(i_GPIO_Port, i_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(j_GPIO_Port, j_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(m_GPIO_Port, m_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(n_GPIO_Port, n_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(k_GPIO_Port, k_Pin, GPIO_PIN_RESET);
				  break;
			  default:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_All, GPIO_PIN_SET);
				  break;
			}
		  }


		void controlTrafficLights1(int redState, int yellowState, int greenState)
		{

		  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin, redState);
		  HAL_GPIO_WritePin(GPIOA, LED_YELLOW_Pin, yellowState);
		  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin, greenState);


		}

		void controlTrafficLights2(int redState, int yellowState, int greenState)
		  {

		  HAL_GPIO_WritePin(GPIOA, LED_RED_1_Pin, redState);
		  HAL_GPIO_WritePin(GPIOA, LED_YELLOW_1_Pin, yellowState);
		  HAL_GPIO_WritePin(GPIOA, LED_GREEN_1_Pin, greenState);

		  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter_1 = 6; // Countdown cho led 1 ( Luc dau den xanh )
  int counter_2 = 9; // Countdown cho led 2 ( Luc dau den do )
  int status_run1 = 0;  // 0: xanh, 1: vang, 2: do
  int status_run2 = 2;  //Bat dau voi do


  // Cập nhật vòng lặp while với trạng thái đèn giao thông hợp lý
  while (1)
  {
      display7SEG1(counter_1);
      display7SEG2(counter_2);
      HAL_Delay(500);  // Chỉnh delay nếu cần

      /* Điều khiển đèn giao thông hướng 1 */
      if (status_run1 == 0) { // Đèn xanh cho hướng 1
          controlTrafficLights1(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);  // Bật đèn xanh cho hướng 1
          controlTrafficLights2(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);  // Bật đèn đỏ cho hướng 2

          if (counter_1 == 0) {
              status_run1 = 1;  // Chuyển sang đèn vàng
              counter_1 = 4;    // Thời gian bật đèn vàng
          }
      }
      else if (status_run1 == 1) { // Đèn vàng cho hướng 1
          controlTrafficLights1(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);  // Bật đèn vàng cho hướng 1
          controlTrafficLights2(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);  // Đèn đỏ vẫn bật cho hướng 2

          if (counter_1 == 0) {
              status_run1 = 2;  // Chuyển sang đèn đỏ
              counter_1 = 9;    // Thời gian bật đèn đỏ
              status_run2 = 0;  // Bắt đầu đèn xanh cho hướng 2
              counter_2 = 6;    // Thời gian đèn xanh cho hướng 2
          }
      }
      else if (status_run1 == 2) { // Đèn đỏ cho hướng 1
          controlTrafficLights1(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);  // Bật đèn đỏ cho hướng 1
          controlTrafficLights2(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);  // Bật đèn xanh cho hướng 2

          if (counter_1 == 0) {
              status_run1 = 0;  // Quay lại đèn xanh cho hướng 1
              counter_1 = 6;    // Thời gian đèn xanh cho hướng 1
          }
      }

      /* Điều khiển đèn giao thông hướng 2 */
      if (status_run2 == 0) { // Đèn xanh cho hướng 2
          controlTrafficLights2(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);  // Bật đèn xanh cho hướng 2
          controlTrafficLights1(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);  // Đèn đỏ cho hướng 1

          if (counter_2 == 0) {
              status_run2 = 1;  // Chuyển sang đèn vàng cho hướng 2
              counter_2 = 4;    // Thời gian bật đèn vàng
          }
      }
      else if (status_run2 == 1) { // Đèn vàng cho hướng 2
          controlTrafficLights2(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);  // Bật đèn vàng cho hướng 2
          controlTrafficLights1(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);  // Đèn đỏ vẫn bật cho hướng 1

          if (counter_2 == 0) {
              status_run2 = 2;  // Chuyển sang đèn đỏ cho hướng 2
              counter_2 = 9;    // Thời gian bật đèn đỏ
              status_run1 = 0;  // Quay lại đèn xanh cho hướng 1
              counter_1 = 6;    // Thời gian đèn xanh cho hướng 1
          }
      }
      else if (status_run2 == 2) { // Đèn đỏ cho hướng 2
          controlTrafficLights2(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);  // Bật đèn đỏ cho hướng 2
      }

      /* Giảm bộ đếm */
      if (counter_1 >= 0) {
          counter_1--;
      }

      if (counter_2 >= 0) {
          counter_2--;
      }
  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_RED_1_Pin
                          |LED_YELLOW_1_Pin|LED_GREEN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a_Pin|b_Pin|c_Pin|k_Pin
                          |l_Pin|m_Pin|n_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|h_Pin
                          |i_Pin|j_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin LED_GREEN_Pin LED_RED_1_Pin
                           LED_YELLOW_1_Pin LED_GREEN_1_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_RED_1_Pin
                          |LED_YELLOW_1_Pin|LED_GREEN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a_Pin b_Pin c_Pin k_Pin
                           l_Pin m_Pin n_Pin d_Pin
                           e_Pin f_Pin g_Pin h_Pin
                           i_Pin j_Pin */
  GPIO_InitStruct.Pin = a_Pin|b_Pin|c_Pin|k_Pin
                          |l_Pin|m_Pin|n_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|h_Pin
                          |i_Pin|j_Pin;
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
