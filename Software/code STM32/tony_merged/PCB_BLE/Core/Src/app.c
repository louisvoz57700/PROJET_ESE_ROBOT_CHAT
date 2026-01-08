/*
 * app.c
 *
 *  Created on: Dec 9, 2025
 *      Author: knn64
 */


#include "app.h"
#include "main.h"

#include "capteurs/multiplexer.h"
#include "capteurs/sensor.h"
#include "actionneurs/oled.h"
#include "capteurs/VL53L0X.h"

#include "tasks/task_control.h"
#include "tasks/task_FSM.h"
#include "tasks/task_sensor.h"
#include "tasks/task_comm.h"

#define BASE_PRIO 25

/* Variables partagées */
ToF_t tof;
LidarData_t lidarData;
oled_handle_t hOled;

/* Synchronisation */
extern osMessageQueueId_t QLidar;
extern osMessageQueueId_t QFSM;

osMutexId_t i2c_mutex = NULL;

/* Handles globaux (accessibles depuis les ISR) */
osThreadId_t task_lidar_handle = NULL;
osThreadId_t task_ToF_handle = NULL;
osThreadId_t task_fsm_handle   = NULL;
osThreadId_t task_action_handle  = NULL;
osThreadId_t task_comm_handle = NULL;

void app_init()
{
	/* --- Synchro FreeRTOS ---*/
	i2c_mutex = osMutexNew(NULL);
	if (i2c_mutex == NULL)  Error_Handler();

	/* --- INITIALISATION MATERIELLES --- */
	/* Accelerometre */
	ADXL343_Init();

	/* MUX I2C */
	TCA9548A_Init();

	/* Initialisation des ToF */
	tof.i2c_mutex = i2c_mutex;


	INIT_TOFS(&hi2c1); // Passer le mutex aussi comme pour OLED

	/* Initialisation du oled */
	OLED_Init(&MUX_I2C_HANDLER, i2c_mutex);

}

/* -------- Création des Threads -----------*/
void app_task_init()
{
	osThreadAttr_t attr = {0};

	/* 1. Tâche de Contrôle des Actionneurs (TaskControl) */
	// Priorité: BASE_PRIO + 5 (La plus haute)
	// Rôle: Exécuter la boucle de contrôle en temps réel (asservissement, PID) pour les moteurs et actionneurs.
	attr.stack_size = 1536;
	attr.priority   = BASE_PRIO + 5;
	attr.name       = "Actionneurs";
	task_action_handle   = osThreadNew(TaskControl, NULL, &attr);

	/* 2. Tâche de Décision / Machine à États Finis (TaskFSM) */
	// Priorité: BASE_PRIO + 4 (Haute)
	// Rôle: Implémenter la logique principale du robot. Prend les données des capteurs (sensorData) et décide des commandes à envoyer à la tâche des actionneurs.
	attr.stack_size = 1280;
	attr.priority   = BASE_PRIO + 4;
	attr.name       = "Decision";
	task_fsm_handle  = osThreadNew(TaskFSM, (void *) &i2c_mutex  , &attr);

	/* 3. Tâche d'Acquisition Capteur Lidar (TaskLidar) */
	// Priorité: BASE_PRIO + 3 (Moyenne-Haute)
	// Rôle: Gérer la communication avec le capteur Lidar et acquérir les données de télémétrie à long rayon (cartographie, détection lointaine).
	attr.stack_size = 1536;
	attr.priority   = BASE_PRIO + 3;
	attr.name       = "Lidar";
	task_lidar_handle = osThreadNew(TaskLidar, (void *) &lidarData, &attr);

	/* 4. Tâche d'Acquisition Capteur ToF (TaskToF) */
	// Priorité: BASE_PRIO + 2 (Moyenne)
	// Rôle: Gérer les capteurs de proximité Time-of-Flight, souvent pour la détection d'obstacles immédiats ou le recalage.
	attr.stack_size = 1536;
	attr.priority   = BASE_PRIO + 2;
	attr.name       = "ToF";
	task_ToF_handle = osThreadNew(TaskToF, NULL, &attr);

	/* 5. Tâche de Communication (TaskCommuncation) */
	// Priorité: BASE_PRIO + 1 (La plus basse)
	// Rôle: Gérer les communications externes (série, réseau, debug) pour la télémétrie ou les commandes à distance (commData).
	attr.stack_size = 1536;
	attr.priority   = BASE_PRIO +  1;
	attr.name       = "Communication";
	osThreadNew(TaskCommunication, (void*)&hOled, &attr);
}


// ==== Callback d'interruptions =====
/* Mesure de puissance globale du systeme */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//	if (hadc->Instance == ADC1)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		vTaskNotifyGiveFromISR(task_sensor_handle, &xHigherPriorityTaskWoken);
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}

/* Callback interruption ADXL343 (tap) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)  // ADXL343 INT1
	{
		BaseType_t xhptw = pdFALSE;
		xTaskNotifyFromISR(task_fsm_handle, FSM_NOTIF_TAP_DETECTED, eSetValueWithOverwrite, &xhptw);
		portYIELD_FROM_ISR(xhptw);
	}
}


