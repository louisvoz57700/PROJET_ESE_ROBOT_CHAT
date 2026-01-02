#include "tasks/task_sensor.h"
#include "capteurs/VL53L0X.h"
#include "adc.h"
#include "app.h"
#include "cmsis_os2.h"

/* Buffers ADC */
#define ADC_BUFFER_SIZE 2
uint16_t adc_buffer[ADC_BUFFER_SIZE];
Measure m1 = {0};

/* Synchronisation */
osMessageQueueId_t QLidar;

/* Structure pour le VL53L0X */
statInfo_t_VL53L0X distanceStr;

volatile uint32_t sensor_debug_counter = 0; //test pour vérifier l'entrée dans les taches

/* * Tâche 1 : Capteurs (ADC + I2C)
 * Priorité : Haute (4
 * Stratégie : Lit l'ADC à chaque cycle, I2C décimé, PAUSE pour éviter famine CPU.
 */
void TaskToF(void *argument)
{
	ToF_t *p = (ToF_t *)argument;

	//	/* Init ADC */
	//	INIT_ADC(adc_buffer, ADC_BUFFER_SIZE);

	/* Attente Init VL53L0X */
	while (!p->vl53l0x_initialized) osDelay(10);

	//	/* Lancement initial DMA */
	//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);

	for (;;)
	{
		/* 1. Attente des données (ADC) - Métronome */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		/* Compteur de debug pour vérifier que la tâche tourne */
		sensor_debug_counter++;

		/* 2. Traitements Rapides (Tension/Courant) */
		if (osMutexAcquire(p->i2c_mutex, 20) == osOK)
		{
			mesure_TOF(p);
			osMutexRelease(p->i2c_mutex);
		}


		/* Pause CPU */
		osDelay(10);

		//		/* Relance ADC */
		//		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
	}
}


/* Tâche Lidar : Traite les données UART DMA */
void TaskLidar(void *argument)
{
	LidarData_t * lidarData = (LidarData_t *) argument;

	/* 1. Init : Démarrage réception DMA circulaire */
	YD_Start_UART_DMA();

	for (;;)
	{
		/* 1. Parsing des données brutes (fonction bloquante tant qu'il y a des paquets) */
		// Remplit le tableau 'vue' avec {distance, angle}
		YD_Parser_MainLoop(lidarData->vue);

		/* 2. Segmentation en objets (Clusters) */
		// Analyse le tableau 'vue' pour trouver des groupes de points
		segment_points(lidarData->vue, N_ANGLES);

		/* 3. Identification de la cible prioritaire */
		Pos local_target;
		LIDAR_Get_Closest_Cluster(&local_target);

		/* 4. Mise à jour de la variable globale pour la tâche Moteur */
		osMessageQueuePut(QLidar, &local_target, 0U, 0U);

		/* Pause pour libérer le CPU (Scan lidar typique ~10-15Hz) */
		osDelay(20);
	}
}
