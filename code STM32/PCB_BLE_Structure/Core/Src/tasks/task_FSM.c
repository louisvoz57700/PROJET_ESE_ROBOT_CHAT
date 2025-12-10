/*
 * task_FSM.c
 *
 *  Created on: Dec 3, 2025
 *      Author: knn64
 */

#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include <stdbool.h>

#include "tasks/task_FSM.h"
#include "tasks/task_sensor.h"

typedef enum {
	STATE_IDLE,
	STATE_SEARCH,
	STATE_CHASE,
	STATE_EVADE,
	STATE_TAG,
	STATE_HIT
} RobotState;

static RobotState currentState = STATE_IDLE;
static RobotState previousState = STATE_IDLE;

static float r, theta;               // coordonnées polaires cible
static float lidar_min;              // distance minimale LiDAR

extern osMessageQueueId_t QLidar;

extern OdomData_t Robot_pos;
osMessageQueueId_t QFSM;

static void chase_target(float r, float theta){

}

static void evade()
{

}

static void signal_tag(){

}

static void signal_hit(){

}

static void SWITCH_TO_EVADE(){}

static void obstacle_avoid(uint32_t side){
	static int current_side = 0;
	if (side != 0xF0) current_side = side;
	switch (side) {
	case(FSM_NOTIF_ToF_LEFT) :
						break;
	case(FSM_NOTIF_ToF_RIGHT) :
						break;
	case(FSM_NOTIF_ToF_FRONT) :
						break;
	case(FSM_NOTIF_ToF_REAR) :
						break;
	default :
		break;
	}

}

/* * Tâche 3 : Prise de décision
 * Priorité : Haute
 * - Envoie les coordonées successives à suivre à la partie controle en tenant compte du temps entre chaque envoi (10 ms)
 */
void TaskFSM(void *argument)
{
	UNUSED(argument);

	LidarFrame frame;
	uint32_t notif = 0;
	bool targetDetected, obstacleDetected = false;

	for(;;)
	{
		// Récupérer données LiDAR si dispo
		if (xQueueReceive(QLidar, &frame, 0) == pdPASS) {
			r = frame.target_r;
			theta = frame.target_theta;
			targetDetected = frame.target_detected;
			lidar_min = frame.lidar_min;
			obstacleDetected = frame.obstacle_detected;
		}

		// Notification d'interruption :
		if (xTaskNotifyWait(0, 0, &notif, 0) == pdTRUE)
		{
			if (notif == FSM_NOTIF_TAP_DETECTED)
			{
				// Vérifier qu’un vrai objet est proche
				if (lidar_min < 0.20f)
				{
					// INTERPRÉTATION DU TAP SELON L'ÉTAT COURANT
					switch(currentState) {
					case STATE_CHASE:
						currentState = STATE_TAG;   // on l'a touché
						break;

					case STATE_SEARCH:
					case STATE_EVADE:
						currentState = STATE_HIT;   // on s’est fait toucher
						break;

					default:
						// IDLE, etc. → ignorer
						break;
					}
				}
			}
			if (notif & 0x000000F0){
				obstacle_avoid(notif);
			}
		}


		switch(currentState)
		{
		case STATE_SEARCH:
			if (targetDetected)
				currentState = STATE_CHASE;

			if (obstacleDetected)
				obstacle_avoid(0xF0);
			break;

		case STATE_CHASE:
			chase_target(r, theta);

			if (r < 0.25f)
				currentState = STATE_TAG;

			if (!targetDetected)
				currentState = STATE_SEARCH;

			if (obstacleDetected)
				SWITCH_TO_EVADE();
			break;

		case STATE_EVADE:
			evade();
			currentState = previousState;
			break;

		case STATE_TAG:
			signal_tag();
			currentState = STATE_IDLE;
			break;

		case STATE_HIT:
			signal_hit();
			currentState = STATE_IDLE;
			break;
		default :
			break;

			// etc.
		}

		osDelay(pdMS_TO_TICKS(5)); // Haute réactivité (200 Hz)
	}
}



