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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ToF_nRST_Pin GPIO_PIN_13
#define ToF_nRST_GPIO_Port GPIOC
#define ENC2_A_Pin GPIO_PIN_0
#define ENC2_A_GPIO_Port GPIOA
#define ENC2_B_Pin GPIO_PIN_1
#define ENC2_B_GPIO_Port GPIOA
#define MOT1_FW_Pin GPIO_PIN_6
#define MOT1_FW_GPIO_Port GPIOA
#define ACC_INT1_Pin GPIO_PIN_4
#define ACC_INT1_GPIO_Port GPIOC
#define ACC_INT2_Pin GPIO_PIN_0
#define ACC_INT2_GPIO_Port GPIOB
#define MOT1_REV_Pin GPIO_PIN_1
#define MOT1_REV_GPIO_Port GPIOB
#define MOT2_Imes_Pin GPIO_PIN_2
#define MOT2_Imes_GPIO_Port GPIOB
#define BT_nRST_Pin GPIO_PIN_12
#define BT_nRST_GPIO_Port GPIOB
#define MOT1_Imes_Pin GPIO_PIN_15
#define MOT1_Imes_GPIO_Port GPIOB
#define ENC1_B_Pin GPIO_PIN_8
#define ENC1_B_GPIO_Port GPIOA
#define ENC1_A_Pin GPIO_PIN_9
#define ENC1_A_GPIO_Port GPIOA
#define MOT2_REV_Pin GPIO_PIN_5
#define MOT2_REV_GPIO_Port GPIOB
#define MOT2_FW__Pin GPIO_PIN_7
#define MOT2_FW__GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
