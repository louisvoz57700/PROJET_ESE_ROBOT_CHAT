/*
 * task_FSM.h
 *
 * Created on: Dec 3, 2025
 * Author: knn64
 */

#ifndef INC_TASK_FSM_H_
#define INC_TASK_FSM_H_

#include "cmsis_os.h" // Indispensable pour osThreadId_t
#include <stdbool.h>  // Pour _Bool ou bool

/* --- MASQUES DE NOTIFICATION --- */
#define FSM_NOTIF_TAP_DETECTED 0x0000000F
#define FSM_NOTIF_ToF_LEFT     0x00000010
#define FSM_NOTIF_ToF_RIGHT    0x00000020
#define FSM_NOTIF_ToF_FRONT    0x00000030
#define FSM_NOTIF_ToF_REAR     0x00000040

/* --- CONSTANTES PARTAGÉES --- */
#define TABLE_WIDTH  200.0f   // cm
#define TABLE_HEIGHT 400.0f   // cm
#define CHECK_SIZE   30.0f    // cm
#define SECURITY_DISTANCE 10.0f

/* --- DÉFINITION GLOBALE DE L'ENUM (La clé du problème) --- */
typedef enum
{
	STATE_IDLE,
	STATE_SEARCH,
	STATE_CHASE,
	STATE_EVADE,
	STATE_TAG,
	STATE_HIT,
	STATE_RECOVERY
} RobotState;

/* --- VARIABLE PARTAGÉE --- */
extern volatile RobotState currentState;

/* Handle de la tache pour recevoir des notifications */
extern osThreadId_t task_fsm_handle;

/* --- PROTOTYPES --- */
void TaskFSM(void *argument);
void FSM_GetEvadeInfo(float *heading, int *checkpoint, _Bool *ready);

#endif /* INC_TASK_FSM_H_ */
