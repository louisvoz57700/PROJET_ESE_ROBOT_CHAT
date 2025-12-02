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
#include <stdio.h>
#include "ble.h"
#include "ble_types.h"
#include "ble_const.h"
#include "ble_gap_aci.h"
#include "ble_gatt_aci.h"
#include "hci_tl.h"
#include "hrs_app.h"
#include "VL53L0X.h"
#include "moteur.h"
#include "oled.h"        // AJOUT pour les fonctions OLED


// ---------- Prototypes ----------
void Task_default(void *argument);
void Task_odom(void *argument);
void Task_lidar(void *argument);
void Task_com(void *argument);
void Task_sensor(void *argument);
void task_screen(void *argument);
void task_ScanI2C(void *argument);
void task_tof(void *argument);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId_t sensor_handle;
osThreadId_t lidar_handle;
osThreadId_t com_handle;
osThreadId_t odom_handle;
osThreadId_t default_handle;

float angle;
int distance;

#define ADC_BUFFER_SIZE 2
uint16_t adc_buffer[ADC_BUFFER_SIZE];

#ifndef MAX_I2C_DEVICES
#define MAX_I2C_DEVICES 16
#endif

Measure m1;

osSemaphoreId_t sem_sensor;
osSemaphoreId_t sem_3axis;
osMutexId_t i2cMutex;

HAL_StatusTypeDef ID_SENSOR;

statInfo_t_VL53L0X distanceStr;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t i2c_device_count = 0;
uint8_t i2c_detected_addresses[MAX_I2C_DEVICES];
bool i2c_scan_completed = false;

extern const uint8_t bitmap_data[1024];       // mouse_cat_bitmap_128x64
extern const uint8_t bitmap_data_mouse[1024]; // mouse_bitmap_128x64
extern const uint8_t bitmap_data_cat[1024];   // cat_bitmap
static uint8_t current_bitmap = 0;            // 0 = mouse_cat, 1 = mouse, 2 = cat

osMutexId_t i2c_mutex = NULL;
volatile bool oled_initialized = false;

char listeI2C[256] = {0}; // String list of detected addresses (e.g., "0x3C, 0x70, ")

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Task_default(void *argument)
{
    UNUSED(argument);

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        Target_angle(500, 180);
        osDelay(10);
    }
}

void Task_odom(void *argument)
{
    UNUSED(argument);
    for (;;)
    {
        osDelay(1);
    }
}

void Task_lidar(void *argument)
{
    UNUSED(argument);
    for (;;)
    {
        osDelay(1);
    }
}

void Task_com(void *argument)
{
    UNUSED(argument);
    for (;;)
    {
        osDelay(10);
    }
}

void Task_sensor(void *argument)
{
    UNUSED(argument);
    uint8_t source;

    INIT_ADC(&adc_buffer, ADC_BUFFER_SIZE);

    for (;;)
    {
        if (distance > 500)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ADXL343_ReadReg(0x30, &source);
        UPDATE_V_I(&m1, &adc_buffer);
        osDelay(1);
    }
}

void task_screen(void *argument)
{
    UNUSED(argument);

    // Initialiser l'OLED une seule fois au démarrage de la tâche
    osDelay(100);
    osMutexAcquire(i2c_mutex, osWaitForever);
    OLED_Init();
    oled_initialized = true;
    osMutexRelease(i2c_mutex);

    for (;;)
    {
        osMutexAcquire(i2c_mutex, osWaitForever);
        OLED_Clear();
        current_bitmap = (current_bitmap + 1) % 3;

        if (current_bitmap == 0)
        {
            OLED_DisplayBitmap(bitmap_data);
        }
        else if (current_bitmap == 1)
        {
            OLED_DisplayBitmap(bitmap_data_mouse);
        }
        else
        {
            OLED_DisplayBitmap(bitmap_data_cat);
        }

        osMutexRelease(i2c_mutex);
        osDelay(10000);
    }
}

void task_ScanI2C(void *argument)
{
    UNUSED(argument);
    HAL_StatusTypeDef result;
    uint8_t i;
    char temp[10];

    // Attendre que l'OLED soit initialisé avant de scanner
    while (!oled_initialized)
    {
        osDelay(10);
    }

    osDelay(200);

    i2c_device_count = 0;
    i2c_scan_completed = false;
    memset(listeI2C, 0, sizeof(listeI2C));

    osMutexAcquire(i2c_mutex, osWaitForever);
    for (i = 1; i < 128 && i2c_device_count < MAX_I2C_DEVICES; i++)
    {
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 10);
        if (result == HAL_OK)
        {
            i2c_detected_addresses[i2c_device_count++] = i;
            sprintf(temp, "0x%02X, ", i);
            strcat(listeI2C, temp);
        }
    }
    osMutexRelease(i2c_mutex);

    i2c_scan_completed = true;
    osThreadTerminate(osThreadGetId());
}

void task_tof(void *argument)
{
    UNUSED(argument);
    for (;;)
    {
        osDelay(10);
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
    char msgBuffer[52];
    for (int i = 0; i < 52; i++)
    {
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
    /* Init scheduler */
    osKernelInitialize();

    /* Mutex I2C - APRÈS osKernelInitialize() */
    i2c_mutex = osMutexNew(NULL);
    if (i2c_mutex == NULL)
    {
        Error_Handler();
    }

    /* Création des tâches avec des stack sizes optimisées */
    osThreadAttr_t attr = {0};

    // Tâche SCREEN
    attr.stack_size = 1536;
    attr.name = "SCREEN";
    attr.priority = osPriorityHigh;
    osThreadId_t screen_handle = osThreadNew(task_screen, NULL, &attr);
    if (screen_handle == NULL)
    {
        Error_Handler();
    }

    // Tâche ScanI2C
    attr.stack_size = 768;
    attr.name = "ScanI2C";
    attr.priority = osPriorityAboveNormal;
    osThreadId_t scan_handle = osThreadNew(task_ScanI2C, NULL, &attr);
    if (scan_handle == NULL)
    {
        Error_Handler();
    }

    // Tâche TOF
    attr.stack_size = 768;
    attr.name = "TOF";
    attr.priority = osPriorityNormal;
    osThreadId_t tof_handle = osThreadNew(task_tof, NULL, &attr);
    if (tof_handle == NULL)
    {
        Error_Handler();
    }

    // Initialisation ADXL343
    ID_SENSOR = ADXL343_Init();
    if (ID_SENSOR == HAL_OK)
    {
        for (int i = 0; i < 10; i++)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
        }
    }

    // Initialisation TCA9548A
    ID_SENSOR = TCA9548A_Init();
    if (ID_SENSOR != HAL_OK)
    {
        for (int i = 0; i < 20; i++)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
        }
    }

    HAL_StatusTypeDef status;
    status = TCA9548A_SelectChannel(1);

    /*
    // Initialisation VL53L0X (commenté)
    initVL53L0X(1, &hi2c1);
    setSignalRateLimit(200);
    setVcselPulsePeriod(VcselPeriodPreRange, 10);
    setVcselPulsePeriod(VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(300 * 1000UL);
    */

    Encoder_Init();

    /* USER CODE END 2 */

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

    /** Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI1 |
                                       RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_MSI;
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

    /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2 |
                                  RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

    /** Initializes the peripherals clock */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS | RCC_PERIPHCLK_RFWAKEUP;
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(sensor_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        uint8_t source;
        ADXL343_ReadReg(0x2A, &source);
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(default_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
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
