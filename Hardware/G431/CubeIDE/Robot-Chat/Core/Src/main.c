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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "bm71_driver.h"
#include "usb_vcp.h"
//#include "adxl345.h"
//#include "sensor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Defines ADXL345 SPI */
#define ADXL345_SPI             hspi1
#define ADXL345_CS_GPIO         GPIOA
#define ADXL345_CS_PIN          GPIO_PIN_4        // Change avec ta broche ACC_nCS
#define ADXL345_READ_BIT        0x80
#define ADXL345_MULTIBYTE_BIT   0x40
#define REG_DATA_FORMAT			0x31

/* ADXL345 SPI commands */
#define MULTI_BIT           0x40

/* Registres importants */
#define REG_DEVID               0x00
#define REG_DATA_FORMAT         0x31
#define REG_POWER_CTL           0x2D
#define REG_THRESH_TAP          0x1D
#define REG_DUR                 0x21
#define REG_TAP_AXES            0x2A
#define REG_INT_ENABLE          0x2E
#define REG_INT_MAP             0x2F
#define REG_INT_SOURCE          0x30

// Définition des Pin de nRST et CFG du BT
#define BM71_CFG_GPIO_Port   GPIOA
#define BM71_CFG_Pin         GPIO_PIN_10
#define BM71_nRST_GPIO_Port  GPIOC
#define BM71_nRST_Pin        GPIO_PIN_10

// mapping baudrate
#define BAUDRATE_INDEX_115200 0x05
#define BAUDRATE_INDEX_921600 0x08
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* CS control macros */
#define CS_LOW()   HAL_GPIO_WritePin(ADXL345_CS_GPIO, ADXL345_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(ADXL345_CS_GPIO, ADXL345_CS_PIN, GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t adxl345_devid;

// FreeRTOS objects
QueueHandle_t ble_queue_handle;


//User Variables
/* Variables */
volatile uint8_t tap_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Envoyer une réponse via USB CDC
void send_response(const char *msg)
{
	CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
}

void bm71_hw_init(void) //Trop spécifique à la board pour être mis dans le fichier de driver
{
	/* GPIO config */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);   // P2_0 = HIGH → mode application
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

	/* Reset propre */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_Delay(5);                                          // OK ici : HAL_Delay marche
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(50);                                        // Attente stabilisation ( datasheet = 50 ms max )
}


void USB_VCP_LauncherTask(void *argument)
{
	(void)argument;

	vTaskDelay(1000);

	/* Crée la vraie tâche qui gère le shell USB */
	if (xTaskCreate(usb_vcp_task, "USB_VCP_Shell", 1024, NULL, 5, NULL)!=pdPASS){
		usb_vcp_printf("Error Creating Task USB_VCP_Shell \r\n");
		Error_Handler();
	}

	/* Initialise la queue interne du module VCP */
	usb_vcp_init();

	/* On se suicide → plus de tâche inutile */
	vTaskDelete(NULL);
}

void BM71_LauncherTask(void *argument)
{
	bm71_t *bm = (bm71_t*)argument;

	vTaskDelay(pdMS_TO_TICKS(800));   // laisse le BM71 se réveiller

	xTaskCreate(BLEParseTask, "BM71_Parser", 768,  bm, tskIDLE_PRIORITY + 2, &bm->parser_task_handle);
	xTaskCreate(BLEShellTask,   "BM71_Shell",  512, bm, tskIDLE_PRIORITY + 4, NULL);

	vTaskDelete(NULL);
}


// Envoie une commande BLEDK3 brute
void SendCmd(uint8_t opcode, const uint8_t* data, uint8_t len)
{
	uint8_t pkt[258];
	pkt[0] = opcode;
	pkt[1] = len;
	if (len) memcpy(&pkt[2], data, len);
	HAL_UART_Transmit(&huart3, pkt, len+2, 1000);
}

// Fonction principale de configuration BM71
// Paramètres : name (ex. "MonBM71"), passkey_str (ex. "123456"), baud_index (ex. 0x05 pour 115200)
void Configurer_BM71(const char* name, const char* passkey_str, uint8_t baud_index) {
	// 1. UART en 115200 + flow control OFF (obligatoire en mode config)
	huart3.Init.BaudRate = 115200;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_DeInit(&huart3);
	HAL_UART_Init(&huart3);

	// 2. Entrée mode config
	HAL_GPIO_WritePin(BM71_CFG_GPIO_Port, BM71_CFG_Pin, GPIO_PIN_RESET);   // CFG = LOW
	HAL_GPIO_WritePin(BM71_nRST_GPIO_Port, BM71_nRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BM71_nRST_GPIO_Port, BM71_nRST_Pin, GPIO_PIN_SET);
	HAL_Delay(300);   // attendre boot complet

	// 3. CHANGER LE NOM → opcode 0x0D (pas 0x08, pas 0x54)
	const char* nouveau_nom = "Cattee";          // ← change ici ce que tu veux
	uint8_t payload[34];
	payload[0] = (uint8_t)strlen(nouveau_nom);
	memcpy(&payload[1], nouveau_nom, strlen(nouveau_nom));

	SendCmd(0x0D, payload, strlen(nouveau_nom) + 1);   // ← 0x0D magique
	HAL_Delay(200);

	// 4. SAUVEGARDE EN FLASH
	SendCmd(0xFF, NULL, 0);
	HAL_Delay(600);

	// 5. Sortie mode config + reset final
	HAL_GPIO_WritePin(BM71_CFG_GPIO_Port, BM71_CFG_Pin, GPIO_PIN_SET);     // CFG = HIGH
	HAL_Delay(50);
	HAL_GPIO_WritePin(BM71_nRST_GPIO_Port, BM71_nRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BM71_nRST_GPIO_Port, BM71_nRST_Pin, GPIO_PIN_SET);

}
static void ADXL345_Write(uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};
	CS_LOW();
	HAL_SPI_Transmit(&ADXL345_SPI, buf, 2, HAL_MAX_DELAY);
	CS_HIGH();
}

uint8_t ADXL345_Read(uint8_t reg)
{
	uint8_t tx =  reg | ADXL345_READ_BIT;
	uint8_t rx = 0;

	CS_LOW();
	HAL_SPI_Transmit(&ADXL345_SPI, &tx, 1, 10);
	while (HAL_SPI_GetState(&ADXL345_SPI) != HAL_SPI_STATE_READY) { __NOP(); }  // safe et ultra-court
	HAL_SPI_Receive(&ADXL345_SPI, &rx, 1, 10);
 	CS_HIGH();

	return rx;
}

static void ADXL345_Init_SingleTap(void)
{
	ADXL345_Write(REG_DATA_FORMAT,  0x08);     // ±4g, full res
	ADXL345_Write(REG_THRESH_TAP,   0x35);     // ~3.3g → ajuste 0x30-0x50 selon sensibilité voulue
	ADXL345_Write(REG_DUR,          0x30);     // 30 ms max
	ADXL345_Write(0x2B,             0x00);     // Window = 0 → désactive double tap
	ADXL345_Write(REG_TAP_AXES,     0x07);     // X+Y+Z
	ADXL345_Write(REG_INT_MAP,      0x00);     // Single Tap → INT1
	ADXL345_Write(REG_INT_ENABLE,   0x40);     // active uniquement Single Tap
	ADXL345_Write(REG_DATA_FORMAT,   0x40);     // active uniquement Single Tap
	ADXL345_Write(REG_POWER_CTL,    0x08);     // mode mesure
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	// Initialisations
	MX_USB_Device_Init();

	bm71_hw_init();

	CS_HIGH();               // nCS inactif au repos
	ADXL345_Init_SingleTap();

	//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	//	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 20);


	//send_response("PWM Started\n\r");

	//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);


	//------------------------------------------------------------
	//Initialisation des objets FreeRTOS
	//------------------------------------------------------------
	//	ble_queue_handle = xQueueCreate(32, sizeof(bm71_event_t));
	//
	//	bm71_t *bm = bm71_create(&huart3, ble_queue_handle);
	//	bm71_start(bm);           // ← tout part tout seul : parser + shell

	//	QueueHandle_t robotEvents = xQueueCreate(16, sizeof(RobotEvent_t));

	//------------------------------------------------------------
	//Initialisation des handles
	//------------------------------------------------------------
	//	sensors = Sensors_Create(
	//	        g_adxl,                     // ton handle ADXL345
	//	        tof_fl, tof_fr, tof_rl, tof_rr,   // tes 4 handles VL53Lxx
	//	        robotEvents
	//	    );
	//
	//	    Sensors_StartTask(sensors, 5);   // priorité très haute
	//
	//------------------------------------------------------------
	//Création des Taches FreeRTOS
	//------------------------------------------------------------

	/* Accelerometre */
	//	if(xTaskCreate(SensorTask,  "Sensor", 256, robotEvents , 5, NULL) != pdPASS){
	//		Error_Handler();
	//	}

	/* Module BLE */
	//		if(xTaskCreate(BM71_LauncherTask,  "BM71_Launch", 256, bm, 5, NULL)!=pdPASS){
	//			Error_Handler();
	//		}
	/* VCP USB */
			if(xTaskCreate(USB_VCP_LauncherTask, "VCP_Launch", 256, NULL, 7, NULL)!=pdPASS){
				Error_Handler();
			}

	vTaskStartScheduler();
  /* USER CODE END 2 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ACC_INT1_Pin)
		{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
