/*
 * task_control.c
 *
 *  Created on: Dec 9, 2025
 *      Author: knn64
 */


#include "cmsis_os.h"
#include <stdint.h>
#include "queue.h"

#include "tasks/task_control.h"
#include "tasks/task_comm.h"

extern osMessageQueueId_t QFSM;
OdomData_t Robot_pos;


OdomData_t Robot_pos; // <--- Position du robot
/* Tâche 2 : Controle des moteurs
 * Priorité : MAXIMALE 5
 * Comportement :
 * -
 */
void TaskControl(void *argument)
{
	osMutexId_t i2c_mutex = (osMutexId_t) argument;
	uint8_t int_source = 0;

	/* 1. Init Système */
	Motor_Init_System();

	for (;;)
	{
		/* 2. Mise à jour continue pour le debug */
		Encoder_Update_Debug();

		/* 3. Attente Notification (Tap) */
		uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, 10);

		if (ulNotificationValue > 0)
		{
//			/* Lecture I2C ADXL343 */
//			if (osMutexAcquire(i2c_mutex, 100) == osOK)
//			{
//				ADXL343_ReadReg(0x30, &int_source);
//				osMutexRelease(i2c_mutex);
//			}

//			/* Action sur Tap */
//			if (int_source & 0x40) // Tap
//			{
//				current_bitmap = (current_bitmap + 1) % 3;
//				xTaskNotifyGive(task_comm_handle);
//
//				// 1. Avance de 200 mm
//				/* Le détail du calcul pour info :
//					   Avec vos roues de 49mm et codeurs 897.6 ticks/tour :
//					   - 1 tour de roue = 49 * PI ~= 154 mm
//					   - 1500 ticks/s = 1500 / 897.6 ~= 1.67 tours/s
//					   - Vitesse = 1.67 * 154 mm ~= 257 mm/s (soit 25.7 cm/s)
//				 */
//				Robot_Translation(1500, 200.0f);
//				osDelay(500);
//
//				// Pause pour stabiliser (utilise osDelay, ne bloque pas le CPU)
//				osDelay(500);
//
//				// 2. Tourne à Droite de 90 degrés
//				// Vitesse réduite (1200) pour la précision en rotation
//				Robot_Rotation(1200.0f, -90.0f);
//				osDelay(500);
//
//				// 3. Avance de 20 cm
//				Robot_Translation(1500.0f, 200.0f);
//				osDelay(500);
//
//				// 4. Demi-tour (180 degrés)
//				Robot_Rotation(1200.0f, 180.0f);
//				osDelay(1000);
//
//				/* Fin de la séquence : Attente longue avant de recommencer */
//				osDelay(5000);
//			}
//			int_source = 0;
		}
	}
}

