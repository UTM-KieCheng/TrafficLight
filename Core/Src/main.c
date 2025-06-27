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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void TrafficLight_Control(void);
void Set_Lights_Direction(uint8_t Count, GPIO_PinState red, GPIO_PinState yellow, GPIO_PinState green);
void Light_Sequency(uint8_t Count, uint32_t GreenDelayTime);
/* USER CODE BEGIN PFP */
uint16_t redPins[4]    			= {TL_Pin_Red_1, TL_Pin_Red_2, TL_Pin_Red_3, TL_Pin_Red_4};
uint16_t yellowPins[4] 			= {TL_Pin_Yellow_1, TL_Pin_Yellow_2, TL_Pin_Yellow_3, TL_Pin_Yellow_4};
uint16_t greenPins[4]  			= {TL_Pin_Green_1, TL_Pin_Green_2, TL_Pin_Green_3, TL_Pin_Green_4};
uint16_t sensor1Pins[4] 			= {IR_Sensor1_Pin_1, IR_Sensor1_Pin_2, IR_Sensor1_Pin_3, IR_Sensor1_Pin_4};
uint16_t sensor2Pins[4] 		= {IR_Sensor2_Pin_1, IR_Sensor2_Pin_2, IR_Sensor2_Pin_3, IR_Sensor2_Pin_4};
uint8_t starvation[4] 			= {0, 0, 0, 0};          // For S, N, E, W
uint8_t starvationResetActive 	= 0;    // Flag for forced serve mode
uint32_t GreenDelayTime			= 0;
uint8_t Count					= 0;
uint8_t u8IRDetected		 	= 0;
uint8_t u8ActualCount			= 0;
bool bStarvation				= false;
uint8_t STARVATION_THRESHOLD 	= 3;
uint32_t u32LONGBLINKING	  	= 3000;
uint32_t u32SHORTBLINKING	  	= 1500;
uint32_t u32YellowDelayTime		= 2000;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  TrafficLight_Control(); // run selected traffic light logic
  }

}
  /* USER CODE END 3 */


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Set_Lights_Direction(uint8_t Count, GPIO_PinState red, GPIO_PinState yellow, GPIO_PinState green)
{
	if(red == GPIO_PIN_RESET)
	{
		GPIOC->ODR &= ~redPins[Count];
	}
	else if(red == GPIO_PIN_SET)
	{
		GPIOC->ODR |= redPins[Count];
	}

	if(yellow == GPIO_PIN_RESET)
	{
		GPIOC->ODR &= ~yellowPins[Count];
	}
	else if(yellow == GPIO_PIN_SET)
	{
		GPIOC->ODR |= yellowPins[Count];
	}

	if(green == GPIO_PIN_RESET)
	{
		GPIOC->ODR &= ~greenPins[Count];
	}
	else if(green == GPIO_PIN_SET)
	{
		GPIOC->ODR |= greenPins[Count];
	}
}

void Light_Sequency(uint8_t Count, uint32_t GreenDelayTime)
{
	Set_Lights_Direction(Count, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
	Set_Lights_Direction(Count, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); // Green
	HAL_Delay(GreenDelayTime);

	Set_Lights_Direction(Count, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); // Yellow
	HAL_Delay(u32YellowDelayTime);

	Set_Lights_Direction(Count, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET); // Red
	HAL_Delay(u32YellowDelayTime);

	starvation[Count] = starvation[Count] + 1;		// Increase starvation value for this direction
}


void TrafficLight_Control(void)
{
	GPIO_PinState s1 = GPIOB->IDR & sensor1Pins[Count];
	GPIO_PinState s2 = GPIOB->IDR & sensor2Pins[Count];

	if(bStarvation == true)
	{
		if(u8ActualCount == Count)
		{
			for (int i = 0; i < 4; i++)
			{
				starvation[i] = 0;
			}
			bStarvation = false;
		}
		else
		{
			if(s1 == GPIO_PIN_RESET && s2 == GPIO_PIN_RESET)
			{
				Light_Sequency(Count,u32LONGBLINKING);
			}

			else
			{
				Light_Sequency(Count,u32SHORTBLINKING);
			}
		}
	}
	else
	{
		// Check if at least one of first sensors is active
		for (int i = 0; i < 4; i++)
		{
			if ((GPIOB->IDR & sensor1Pins[i]) == GPIO_PIN_RESET ||
					(GPIOB->IDR & sensor2Pins[i]) == GPIO_PIN_RESET)
			{
				u8IRDetected = u8IRDetected + 1;
			}
		}

		// All first sensors inactive → simple 15s
		if (u8IRDetected == 0)
		{
			Light_Sequency(Count,u32SHORTBLINKING);
		}

//		else if (u8IRDetected == 4)
//		{
//			// All first sensors active → check second sensor
//			if (s2 == GPIO_PIN_RESET)
//			{
//				Light_Sequency(Count,u32LONGBLINKING);
//			}
//
//			else
//			{
//				Light_Sequency(Count,u32SHORTBLINKING);
//			}
//		}

		// Condition: 1–3 first sensors active
		else
		{
			if (s1 == GPIO_PIN_RESET && s2 == GPIO_PIN_RESET)
			{
				Light_Sequency(Count,u32LONGBLINKING);
				if(starvation[Count] >= STARVATION_THRESHOLD)
				{
					bStarvation = true;
					u8ActualCount = Count;
				}
			}

			else if (s1 == GPIO_PIN_RESET || s2 == GPIO_PIN_RESET)
			{

				Light_Sequency(Count,u32SHORTBLINKING);
				if(starvation[Count] >= STARVATION_THRESHOLD)
				{
					bStarvation = true;
					u8ActualCount = Count;
				}
			}
		}
	}
	u8IRDetected = 0; 	//Reset Input Reading
	Count = (Count + 1) % 4; // next direction
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  // Configure PC0 (Red), PA1 (Yellow), PA3 (Green) as Output
//  GPIO_InitStruct.Pin =		GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2|
//		  	  	  	  	  	GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5|
//							GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8|
//							GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  // Configure PB0 (Sensor) as Input
//  GPIO_InitStruct.Pin = 	GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
//					  	  	GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;  // For push-button. If using IR sensor, adjust as needed.
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Enable GPIOB and GPIOC clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; // RCC->AHB1ENR |= (1 << 1) | (1 << 2);

  // Set PC0–PC11 as output, push-pull, no-pull
  GPIOC->MODER  |=  0x555555;       // Set to output mode (01)
  GPIOC->OTYPER &= ~(0x0FFF);       // Push-pull (0)
  GPIOC->PUPDR  &= ~(0xFFFFFF);     // No pull-up/down (00)

  // Set PB0–PB7 as input with pull-up
  GPIOB->MODER &= ~(0xFFFF);        // Input mode (00)
  GPIOB->PUPDR &= ~(0xFFFF);        // Clear PUPDR
  GPIOB->PUPDR |=  0x5555;          // Set pull-up (01)

  GPIOC->ODR |= GPIO_PIN_0;			// TL_Pin_Red_1
  GPIOC->ODR |= GPIO_PIN_3;			// TL_Pin_Red_2
  GPIOC->ODR |= GPIO_PIN_6;			// TL_Pin_Red_3
  GPIOC->ODR |= GPIO_PIN_9;			// TL_Pin_Red_4
  GPIOC->ODR &= ~GPIO_PIN_1;		// TL_Pin_Yellow_1
  GPIOC->ODR &= ~GPIO_PIN_4;		// TL_Pin_Yellow_2
  GPIOC->ODR &= ~GPIO_PIN_7;		// TL_Pin_Yellow_3
  GPIOC->ODR &= ~GPIO_PIN_10;		// TL_Pin_Yellow_4
  GPIOC->ODR &= ~GPIO_PIN_2;		// TL_Pin_Green_1
  GPIOC->ODR &= ~GPIO_PIN_5;		// TL_Pin_Green_2
  GPIOC->ODR &= ~GPIO_PIN_8;		// TL_Pin_Green_3
  GPIOC->ODR &= ~GPIO_PIN_11;		// TL_Pin_Green_4
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Red_1, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Red_2, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Red_3, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Red_4, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Yellow_1, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Yellow_2, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Yellow_3, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Yellow_4, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Green_1, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Green_2, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Green_3, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TL_GPIO_Port, TL_Pin_Green_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
