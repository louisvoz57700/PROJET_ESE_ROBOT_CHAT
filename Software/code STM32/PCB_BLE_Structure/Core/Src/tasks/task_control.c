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

OdomData_t Robot_pos; // <--- Position du robot
/* Tâche 2 : Controle des moteurs
 * Priorité : MAXIMALE 5
 * Comportement :
 * -
 */
void TaskControl(void *argument)
{
	EncoderDebug_t * g_encoder;

	/* 1. Init Système */
	Motor_Init_System();

	for (;;)
	{
		/* 2. Mise à jour continue pour le debug */
		g_encoder = Encoder_Update_Debug();
		Robot_pos.robot_x = g_encoder->x_mm;
		Robot_pos.robot_y = g_encoder->y_mm;
		Robot_pos.robot_heading = g_encoder->theta_deg;
		/* 3. Attente Notification (Tap) */






	}
}

