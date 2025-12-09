#include "tasks/task_sensor.h"
#include "capteurs/VL53L0X.h"
#include "adc.h"
#include "app.h"

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
	uint8_t loop_divider = 0;

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
		//UPDATE_V_I(&(p->meas), adc_buffer);

		/* 3. Traitements Rapides (Tension/Courant) */

		/* 4. Gestion I2C (Lente) - Décimation 1/20 */
		loop_divider++;
		if (loop_divider >= 20)
		{
			loop_divider = 0;

			/* On prend le Mutex pour toute la durée de la transaction I2C */
			if (osMutexAcquire(p->i2c_mutex, 20) == osOK)
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
					case 1: p->dist_devant   = val_mm; break;
					case 0: p->dist_gauche   = val_mm; break;
					case 2: p->dist_droite   = val_mm; break;
					case 3: p->dist_derriere = val_mm; break;
					}
				}

				//				ADXL343_ReadXYZ(&(p->meas));

				osMutexRelease(p->i2c_mutex);
			}

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
		/* 2. Parsing des données */
		/* Cette fonction retourne immédiatement si pas assez de données,
           ou boucle tant qu'il y a des paquets à traiter. */
		YD_Parser_MainLoop(lidarData->vue, 0); // TODO < --------------- Passer un pointeur de Lidar_Frame à la tache

		/* 3. (Optionnel) Clustering */
		/* Décommentez si vous voulez calculer les objets */
		// segment_points(vue, N_ANGLES);

		/* 4. Pause pour libérer le CPU */
		osDelay(20);
	}
}
