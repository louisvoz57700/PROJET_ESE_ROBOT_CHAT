/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file : main.c
 * @brief : Main program body - VERSION NETTOYÉE ET OPTIMISÉE
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
#include <stdio.h>
#include "ble.h"
#include "VL53L0X.h"
#include "moteur.h"
#include "oled.h"
#include "sensor.h"
#include "lidar.h"
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
/* Bitmaps OLED */
extern const uint8_t bitmap_data[1024];
extern const uint8_t bitmap_data_mouse[1024];
extern const uint8_t bitmap_data_cat[1024];

/* Variables partagées */
static uint8_t current_bitmap = 0;
volatile int distance = 0;

/* Buffers ADC */
#define ADC_BUFFER_SIZE 2
uint16_t adc_buffer[ADC_BUFFER_SIZE];
Measure m1 = {0};

/* Synchronisation */
osMutexId_t i2c_mutex = NULL;
volatile bool oled_initialized = false;
volatile bool vl53l0x_initialized = false;

/* Handles globaux (accessibles depuis les ISR) */
osThreadId_t sensors_handle = NULL;
osThreadId_t motor_handle   = NULL;
osThreadId_t screen_handle  = NULL;
osThreadId_t lidar_handle = NULL;
osThreadId_t scenario_handle=NULL;

/* Variables de distance (Globales) */
volatile int dist_gauche   = -1; // Canal 0
volatile int dist_devant   = -1; // Canal 1
volatile int dist_droite   = -1; // Canal 2
volatile int dist_derriere = -1; // Canal 3

extern Pos vue[N_ANGLES]; // Défini dans lidar.c
volatile uint32_t sensor_debug_counter = 0;//test pour vérifier l'entrée dans les taches

/* Structure pour le VL53L0X */
statInfo_t_VL53L0X distanceStr;

extern Pos vue[N_ANGLES];

/* USER CODE BEGIN PV */

volatile Target_t current_target = {0};

// --- PARAMÈTRES DE COMBAT (Globaux) ---
#define ALIGN_TOLERANCE     10.0f
#define CHARGE_CHUNK        400.0f  // Distance de charge aveugle (40cm)
#define ATTACK_BONUS_DIST   100.0f  // Bonus pour l'impact
#define ATTACK_SPEED        2000.0f  // Vitesse de charge
#define ROTATION_SPEED      1000.0f  // Vitesse de rotation

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* * Tâche 1 : Capteurs (ADC + I2C)
 * Priorité : ÉLEVÉE (55)
 * Stratégie : Lit l'ADC à chaque cycle, I2C décimé, PAUSE pour éviter famine CPU.
 */
void Task_Sensors(void *argument)
{
	UNUSED(argument);
	uint8_t loop_divider = 0;

	/* Init ADC */
	INIT_ADC(adc_buffer, ADC_BUFFER_SIZE);

	/* Attente Init VL53L0X */
	while (!vl53l0x_initialized) osDelay(10);

	/* Lancement initial DMA */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);

	for (;;)
	{
		/* 1. Attente des données (ADC) - Métronome */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		/* Compteur de debug pour vérifier que la tâche tourne */
		sensor_debug_counter++;

		/* 2. Traitements Rapides (Tension/Courant) */
		UPDATE_V_I(&m1, adc_buffer);

		/* 3. Gestion I2C (Lente) - Décimation 1/20 */
		loop_divider++;
		if (loop_divider >= 20)
		{
			loop_divider = 0;

			/* On prend le Mutex pour toute la durée de la transaction I2C */
			if (osMutexAcquire(i2c_mutex, 20) == osOK)
			{
				/* A. Boucle sur les 4 canaux du Multiplexeur */
				for(uint8_t ch = 0; ch < 4; ch++)
				{

					TCA9548A_SelectChannel(ch);

					/* 2. Lire la distance */
					uint16_t meas = readRangeSingleMillimeters(&distanceStr);
					int val_mm = -1;

					/* 3. Vérifier validité */
					if (!timeoutOccurred() && meas < 8190) {
						val_mm = (int)meas;
					}

					/* 4. Assigner à la variable nommée correspondante */
					/* ADAPTEZ L'ORDRE ICI SELON VOTRE CÂBLAGE */
					switch(ch) {
					case 1: dist_devant   = val_mm; break;
					case 0: dist_gauche   = val_mm; break;
					case 2: dist_droite   = val_mm; break;
					case 3: dist_derriere = val_mm; break;
					}
				}

				ADXL343_ReadXYZ(&m1);

				osMutexRelease(i2c_mutex);
			}

		}

		/* Pause CPU */
		osDelay(10);

		/* Relance ADC */
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
	}
}

/* Tâche 2 : Moteurs + Tap
 * Priorité : MAXIMALE (56)
 * Comportement :
 * - Lit les encodeurs en permanence (toutes les 10ms).
 * - Vérifie s'il y a eu une notification (Tap) sans bloquer indéfiniment.
 */
/* Tâche 2 : Moteurs - Stratégie "LOCK & CHARGE" */
void Task_MotorControl(void *argument)
{
	UNUSED(argument);
	uint8_t int_source = 0;
	bool robot_running = true;
	Target_t target;

	Motor_Init_System();

	for (;;)
	{
		// 1. Gestion TAP (Start/Stop)
		// On garde un timeout court (5ms) pour être réactif si le robot est à l'arrêt
		uint32_t wait_time = robot_running ? 0 : 5;
		uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, wait_time);

		if (ulNotificationValue > 0)
		{
			if (osMutexAcquire(i2c_mutex, 50) == osOK) {
				ADXL343_ReadReg(0x30, &int_source);
				osMutexRelease(i2c_mutex);
			}
			if (int_source & 0x40) {
				robot_running = !robot_running;
				if (!robot_running) Motor_Stop_Both();
			}
		}

		// 2. Logique de Combat
		if (robot_running)
		{
			// On récupère la dernière cible connue
			portENTER_CRITICAL();
			target = current_target;
			portEXIT_CRITICAL();

			if (target.detected)
			{
				float angle_relatif = target.angle;
				// Normalisation simple si besoin (-180 / +180)
				if (angle_relatif > 180.0f) angle_relatif -= 360.0f;
				if (angle_relatif < -180.0f) angle_relatif += 360.0f;

				// --- CAS 1 : ALIGNEMENT (SNIPER) ---
				// Si on n'est pas en face, on tourne.
				// Ici on "regarde souvent" (boucle rapide) pour être précis.
				if (fabsf(angle_relatif) > ALIGN_TOLERANCE)
				{
					// On tourne vers la cible
					Robot_Rotation(ROTATION_SPEED, -angle_relatif);

					// Petite pause pour stabiliser la mécanique avant de re-mesurer
					osDelay(100);
				}

				else
				{
					float dist_to_go = target.dist;

					// Si l'ennemi est loin (> 40cm), on fait un "Saut" aveugle de 40cm.
					// C'est ça qui supprime les à-coups : pendant ce mouvement,
					// le robot ignore le LiDAR et utilise son PID interne fluide.
					if (dist_to_go > CHARGE_CHUNK)
					{
						Robot_Translation(ATTACK_SPEED, CHARGE_CHUNK);
					}
					else
					{
						// L'ennemi est proche (< 40cm).
						// C'est l'assaut final. On vise la distance + 10cm pour l'impact.
						Robot_Translation(ATTACK_SPEED + 100, dist_to_go + 100.0f);

						// Après un impact, on peut reculer un peu ou s'arrêter
						// Robot_Translation(ATTACK_SPEED, -100.0f); // Recul tactique ?
					}

					// Après une grosse charge (aveugle), on marque une micro-pause
					// pour laisser le LiDAR reprendre ses esprits et rafraîchir la position.
					osDelay(50);
				}
			}
			else
			{
				// Pas de cible : Arrêt ou Recherche
				Motor_Stop_Both();
				osDelay(100); // On regarde moins souvent quand on cherche
			}
		}
	}
}

/* * Tâche 3 : Écran OLED
 * Priorité : NORMALE (24)
 * Met à jour l'écran seulement quand demandé (Tap) ou init.
 */
void Task_Screen(void *argument)
{
	UNUSED(argument);

	/* Initialisation OLED protégée */
	osDelay(100);
	osMutexAcquire(i2c_mutex, osWaitForever);
	OLED_Init();
	OLED_Clear();
	OLED_DisplayBitmap(bitmap_data);
	oled_initialized = true;
	osMutexRelease(i2c_mutex);

	for (;;)
	{
		/* Dort jusqu'à ce qu'un Tap (Task_Motor) nous réveille */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		/* Dessin protégé */
		if (osMutexAcquire(i2c_mutex, osWaitForever) == osOK)
		{
			// Pas de OLED_Clear() pour éviter le scintillement
			if (current_bitmap == 0)
				OLED_DisplayBitmap(bitmap_data);
			else if (current_bitmap == 1)
				OLED_DisplayBitmap(bitmap_data_mouse);
			else
				OLED_DisplayBitmap(bitmap_data_cat);

			osMutexRelease(i2c_mutex);
		}
	}
}
/* Tâche Lidar : Traite les données UART DMA */
void Task_Lidar(void *argument)
{
	UNUSED(argument);

	/* Démarrage DMA Circulaire */
	YD_Start_UART_DMA();

	for (;;)
	{
		/* 1. Parsing des données brutes (fonction bloquante tant qu'il y a des paquets) */
		// Remplit le tableau 'vue' avec {distance, angle}
		YD_Parser_MainLoop(vue);

		/* 2. Segmentation en objets (Clusters) */
		// Analyse le tableau 'vue' pour trouver des groupes de points
		segment_points(vue, N_ANGLES);

		/* 3. Identification de la cible prioritaire */
		Target_t local_target;
		LIDAR_Get_Closest_Cluster(&local_target);

		/* 4. Mise à jour de la variable globale pour la tâche Moteur */
		portENTER_CRITICAL();
		current_target = local_target;
		portEXIT_CRITICAL();

		/* Pause pour libérer le CPU (Scan lidar typique ~10-15Hz) */
		osDelay(20);
	}
}

//void Task_Scenario(void *argument)
//{
//    UNUSED(argument);
//
//    osDelay(10000);
//
//    Encoder_Update_Debug();
//
//
//    Robot_Translation(2500, 500.0f);
//    Encoder_Update_Debug();
//    osDelay(500);
//
//    Robot_Rotation(700.0f, -105.0f);
//    osDelay(500);
//
//
//    Robot_Translation(2500, 500.0f);
//    osDelay(500);
//
//    Robot_Rotation(700.0f, -105.0f);
//    osDelay(500);
//
//    Robot_Translation(2500, 500.0f);
//    Encoder_Update_Debug();
//    osDelay(500);
//
//    Robot_Rotation(700.0f, -105.0f);
//    osDelay(500);
//
//    Robot_Translation(2500, 500.0f);
//    osDelay(500);
//
//    Robot_Rotation(700.0f, -105.0f);
//    osDelay(500);
//
//    vTaskDelete(NULL);
//
//}
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
	/* Init scheduler */

	osKernelInitialize();

	i2c_mutex = osMutexNew(NULL);
	if (i2c_mutex == NULL) { Error_Handler(); }

	osThreadAttr_t attr = {0};


	/* 1. TAP + MOTEURS – PRIORITÉ LA PLUS HAUTE */
	attr.stack_size = 1536;
	attr.priority   = osPriorityRealtime;      // 56 → MAX
	attr.name       = "MotorCtrl";
	motor_handle    = osThreadNew(Task_MotorControl, NULL, &attr);

	/* 2. CAPTEURS – Juste en dessous */
	attr.stack_size = 1280;
	attr.priority   = osPriorityRealtime7;     // 55
	attr.name       = "Sensors";
	sensors_handle  = osThreadNew(Task_Sensors, NULL, &attr);

	/* 3. ÉCRAN – Priorité normale */
	attr.stack_size = 1536;
	attr.priority   = osPriorityNormal;        // 24
	attr.name       = "Screen";
	screen_handle   = osThreadNew(Task_Screen, NULL, &attr);

	//4  Tâche Lidar
	attr.name       = "LidarTask";
	attr.stack_size = 1280; // Un peu de marge pour le parsing
	attr.priority   = osPriorityHigh; // Moins critique que les moteurs
	lidar_handle    = osThreadNew(Task_Lidar, NULL, &attr);

	//	//5 scénario prédéfinit ex: carré
	//	attr.name       = "Scenarion";
	//	attr.stack_size = 1024;
	//	attr.priority   = osPriorityAboveNormal;
	//	scenario_handle = osThreadNew(Task_Scenario, NULL, &attr);

	//pour juste avoir une tache
	//	//5 scénario prédéfinit ex: carré
	//	osThreadAttr_t scenario_attr = {
	//	    .name = "Scenario",
	//	    .stack_size = 1024,
	//	    .priority = osPriorityNormal,
	//
	//	};
	//	osThreadNew(Task_Scenario, NULL, &scenario_attr);

	/* --- INITIALISATION MATERIELLES --- */
	ADXL343_Init();
	TCA9548A_Init();
	TCA9548A_SelectChannel(0);

	// IMPORTANT : Initialisation des Moteurs (Timers + Sécurité MOE)
	Motor_Init_System();

	I2C_ResetBus(&hi2c1);

	if (initVL53L0X(1, &hi2c1))
	{
		setSignalRateLimit(0.25f);
		setVcselPulsePeriod(VcselPeriodPreRange, 10);
		setVcselPulsePeriod(VcselPeriodFinalRange, 14);
		setMeasurementTimingBudget(300000UL);
		vl53l0x_initialized = true;
	}



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
			|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
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
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
/* Callback ADC */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(sensors_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* Callback interruption ADXL343 (tap) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)  // ADXL343 INT1
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(motor_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
