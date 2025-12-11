/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L0X.h"
#include "sensor.h"
#include "moteur.h"
#include "lidar.h"
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
osThreadId_t ControlLoopTaskHandle; //
osThreadId_t odomTaskHandle;
osThreadId_t lidarTaskHandle;
osThreadId_t MainLogicTaskHandle; //
osThreadId_t SafetyTaskHandle; //
osThreadId_t comTaskHandle;

extern ADC_HandleTypeDef hadc1;
extern System_state state ;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SAFETY_TASK_PERIOD_MS 10

uint16_t distance;

Position pos = {0};

#define ADC_BUFFER_SIZE 2
uint16_t adc_buffer[ADC_BUFFER_SIZE];

Measure m1;
Position robot;

osSemaphoreId_t sem_sensor;
osSemaphoreId_t sem_3axis;
osMutexId_t i2cMutex;
osMutexId_t motorMutex;



extern Pos vue[N_ANGLES];
osMutexId_t myMutexHandle;
const osMutexAttr_t myMutexAttr = {
  .name = "myMutex"
};

osMutexId_t motorMutexHandle;
const osMutexAttr_t motorAttr = {
  .name = "motorMutex"
};

osSemaphoreId_t i2cSem;
osSemaphoreId_t chockSem;
osSemaphoreId_t UARTSem;


extern TOF capteur_tof ;

float angle ;

uint32_t t1;
uint32_t t2;

statInfo_t_VL53L0X distanceStr;

float mean_velo = 0;

extern float velocity;

#define ALPHA  0.95
#define TARGET 10.0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Task_main_loop(void *argument);
void Task_odom(void *argument);
void Task_lidar(void *argument);
void Task_com(void *argument);
void Task_control_loop(void *argument);
void Task_safety(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t MainLogicTaskAttr = {
      .name = "MainLogic",
      .priority = osPriorityAboveNormal,
      .stack_size = 512   // CMSIS expects bytes
  };

  MainLogicTaskHandle = osThreadNew(Task_main_loop, NULL, &MainLogicTaskAttr);
  if (MainLogicTaskHandle == NULL) {
      Error_Handler();
  }

  const osThreadAttr_t odomTaskAttr = {
   	 .name = "Task_odom",
  	 .priority = osPriorityAboveNormal,
  	 .stack_size = 512
  };
  odomTaskHandle = osThreadNew(Task_odom, NULL, &odomTaskAttr);
  if (odomTaskHandle == NULL) {
 	 Error_Handler();
  }

  const osThreadAttr_t lidarTaskAttr = {
     	 .name = "Task_lidar",
    	 .priority = osPriorityHigh,
    	 .stack_size = 1536
  };
  lidarTaskHandle = osThreadNew(Task_lidar, NULL, &lidarTaskAttr);
  if (lidarTaskHandle == NULL) {
 	 Error_Handler();
  }

  const osThreadAttr_t comTaskAttr = {
     	 .name = "Task_com",
    	 .priority = osPriorityHigh,
    	 .stack_size = 512
  };
  comTaskHandle = osThreadNew(Task_com, NULL, &comTaskAttr);
  if (comTaskHandle == NULL) {
 	 Error_Handler();
  }

  const osThreadAttr_t ControlLoopTaskAttr = {
     	 .name = "Task_control_loop",
    	 .priority = osPriorityHigh,
    	 .stack_size = 512
  };
  ControlLoopTaskHandle = osThreadNew(Task_control_loop, NULL, &ControlLoopTaskAttr);
  if (ControlLoopTaskHandle == NULL) {
 	 Error_Handler();
  }

  const osThreadAttr_t SafetyTaskAttr = {
     	 .name = "Task_safety",
    	 .priority = osPriorityRealtime,
    	 .stack_size = 1536
  };
  SafetyTaskHandle = osThreadNew(Task_safety, NULL, &SafetyTaskAttr);
  if (SafetyTaskHandle == NULL) {
    	 Error_Handler();
  }

  myMutexHandle = osMutexNew(&myMutexAttr);
  i2cSem = osSemaphoreNew(1, 1, NULL);
  chockSem = osSemaphoreNew(1, 0, NULL);
  UARTSem = osSemaphoreNew(1, 0, NULL);
  motorMutexHandle = osMutexNew(&motorAttr);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void Task_safety(void *argument)
{
    UNUSED(argument);
    uint8_t source;
    INIT_ADC(&adc_buffer,ADC_BUFFER_SIZE);

    // Remise à zéro du compteur au début
    for (;;)
    {
    	//cycles = DWT->CYCCNT;
    	//---------MESURE TOF---------//
        osMutexAcquire(i2cSem, pdMS_TO_TICKS(200));
        t1 = DWT->CYCCNT;
    	mesure_TOF(&capteur_tof);
    	t2 = DWT->CYCCNT - t1;
        osMutexRelease(i2cSem);
        //------------------//

        if (capteur_tof.front > 200)
        {
        	state.space = 1 ;
        	Motor_Reverse_R(&state,200);
        	Motor_Reverse_L(&state,200);

        }
        else if (capteur_tof.back > 200)
        {
        	state.space = 2 ;
        	Motor_Reverse_R(&state,200);
        	Motor_Reverse_L(&state,200);

        }
        else
        {
        	/*
        	temps = DWT->CYCCNT-cycles;
            Control_Speed(TARGET,temps);
            cycles = DWT->CYCCNT;
            */
        	Motor_Forward_R(&state,300);
        	Motor_Forward_L(&state,300);
        	state.space = 0 ;
        }
        /*
		if (state.space > 0)
		{
			Motor_Forward_R(&state,100);
		}
		*/


	  mean_velo =  ALPHA * mean_velo + (1-ALPHA)*velocity;

		//temps = DWT->CYCCNT-cycles;



		osDelay(20);



    }
}

void Task_main_loop(void *argument)
{
    UNUSED(argument);
    // Remise à zéro du compteur au début
    for (;;)
    {
    	osDelay(10);

    }

    // Ici la tâche peut continuer ou se terminer
}
void Task_odom(void *argument)
{
  UNUSED(argument);
  for (;;)
  {

   osDelay(10);
  }
}

void Task_lidar(void *argument)
{
  UNUSED(argument);
  for (;;)
  {

	  osMutexAcquire(UARTSem, pdMS_TO_TICKS(200));
	  YD_Parser_MainLoop(&vue);
	  segment_points(&vue, 360);
	  osMutexRelease(UARTSem);
	  osDelay(10);
	  //osMutexAcquire(i2cSem, pdMS_TO_TICKS(200));
      //UPDATE_V_I(&m1,&adc_buffer);
      //osMutexRelease(i2cSem);


	  //YD_Parser_MainLoop(&vue);



  }
}

void Task_com(void *argument)
{

  UNUSED(argument);
  for (;;)
  {
	    //osDelay(10);
  }
}

void Task_control_loop(void *argument)
{
  UNUSED(argument);
  uint8_t source;
  TickType_t lastWakeTime = xTaskGetTickCount();
  for (;;)
  {   if (state.space == 0)
  	  {
	  	  //Motor_Forward(400);
  	  }


	  pos.angle_r = Encoder_UpdateAngle(&htim2);
	  pos.angle_l =Encoder_UpdateAngle(&htim1);
	  vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
  }
}
/* USER CODE END Application */

