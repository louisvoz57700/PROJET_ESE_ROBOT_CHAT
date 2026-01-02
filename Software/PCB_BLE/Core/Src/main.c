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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "ipcc.h"
#include "lptim.h"
#include "usart.h"
#include "rf.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timers.h"
#include "cmsis_os2.h"
#include "app_common.h"
#include <string.h>
#include "app_common.h"
#include "ble.h"
#include "ble_types.h"
#include "ble_const.h"
#include "ble_gap_aci.h"
#include "ble_gatt_aci.h"
#include "hci_tl.h"
#include "hrs_app.h"
#include "VL53L0X.h"
#include "stdio.h"
#include "sensor.h"
#include "moteur.h"
#include "LIDAR.h"
// ---------- Prototypes ----------
void SensorTask(void *argument);
void BleTask(void *argument);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern osThreadId_t sensorTaskHandle;
extern osThreadId_t odomTaskHandle;
extern osThreadId_t lidarTaskHandle;
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t TOFTaskHandle;
extern osThreadId_t comTaskHandle;

HAL_StatusTypeDef ID_SENSOR;
uint16_t position ;

extern Position pos ;

extern uint16_t distance;
extern statInfo_t_VL53L0X distanceStr;
System_state state = {0};
TOF capteur_tof ;
#define LPTIM_PERIOD_10MS 2500

volatile uint8_t dma_byte_buf[DMA_BYTE_BUF_SIZE];
Pos vue[N_ANGLES];  // tableau final des distances en mm
Cluster clusters[MAX_CLUSTERS];

int* last_count_g = 0;
int* last_count_d = 0;

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
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
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
	char msgBuffer[52];
	for (int i = 0; i < 52; i++) {
		msgBuffer[i] = ' ';
	}
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_LPTIM1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */
  ID_SENSOR = ADXL343_Init();
  if (ID_SENSOR == HAL_OK)
  {
	  for (int i = 0;i<10;i++)
	  {
		   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		   //HAL_Delay(300);
	  }
  }

  ID_SENSOR = TCA9548A_Init();
    if (ID_SENSOR != HAL_OK)
    {
  	  for (int i = 0;i<20;i++)
  	  {
  		   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
  		   //HAL_Delay(150);
  	  }
    }


  //Initialise the 4th TOF
  INIT_TOFS(&hi2c1);
  ///////
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Active TRC
  DWT->CYCCNT = 0;                                  // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // Active CYCCNT
  //Motor_Init();
  //Motor_Forward(500);
 // Motor_Ramp_L(&state, 600, 4);


  if (HAL_LPTIM_Counter_Start_IT(&hlptim1,LPTIM_PERIOD_10MS) != HAL_OK)
  {
      Error_Handler();
  }
  Encoder_Init();
  HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)dma_byte_buf, DMA_BYTE_BUF_SIZE);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Init code for STM32_WPAN */
  MX_APPE_Init();
  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
    }
}
extern osSemaphoreId_t i2cSem;
extern osSemaphoreId_t chockSem;
extern osSemaphoreId_t UARTSem;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == LPUART1)
    {
  	  osSemaphoreRelease(UARTSem);
    }
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim->Instance == LPTIM1)
    {
    	  //Control_Speed(10.0f,0.01);

    	//Update_Encoder_Vitesse(pos,&last_count_d,&last_count_g);

    	/*
    	// PID gauche
		float u_g = PID_Update(&pid_vitesse_G, pid_vitesse_G, pos.Vitesse_G, 0.001f);
		int pwm_g = (int)(500 + u_g * 500);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_g);

		// PID droit
		float u_d = PID_Update(&pid_vitesse_D, pid_vitesse_D, pos.Vitesse_D, 0.001f);
		int pwm_d = (int)(500 + u_d * 500);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_d);
		*/
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) // Vérifie que c'est bien le pin attendu
    {
    	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    	osSemaphoreRelease(chockSem);


    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
    	//osSemaphoreRelease(i2cSem);
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
