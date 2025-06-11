/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#define TL_Pin_Red_1			GPIO_PIN_0
#define TL_Pin_Red_2			GPIO_PIN_3
#define TL_Pin_Red_3			GPIO_PIN_6
#define TL_Pin_Red_4			GPIO_PIN_9
#define TL_Pin_Yellow_1		GPIO_PIN_1
#define TL_Pin_Yellow_2		GPIO_PIN_4
#define TL_Pin_Yellow_3		GPIO_PIN_7
#define TL_Pin_Yellow_4		GPIO_PIN_10
#define TL_Pin_Green_1		GPIO_PIN_2
#define TL_Pin_Green_2		GPIO_PIN_5
#define TL_Pin_Green_3		GPIO_PIN_8
#define TL_Pin_Green_4		GPIO_PIN_11
#define IR_Sensor1_Pin_1				GPIO_PIN_0
#define IR_Sensor1_Pin_2				GPIO_PIN_1
#define IR_Sensor1_Pin_3				GPIO_PIN_2
#define IR_Sensor1_Pin_4				GPIO_PIN_3
#define IR_Sensor2_Pin_1				GPIO_PIN_4
#define IR_Sensor2_Pin_2				GPIO_PIN_5
#define IR_Sensor2_Pin_3				GPIO_PIN_6
#define IR_Sensor2_Pin_4				GPIO_PIN_7
#define IR_Sensor_Port 					GPIOB
#define TL_GPIO_Port 		GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
