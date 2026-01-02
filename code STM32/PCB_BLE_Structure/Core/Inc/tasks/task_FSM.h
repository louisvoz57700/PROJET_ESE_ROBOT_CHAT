/*
 * task_FSM.h
 *
 *  Created on: Dec 3, 2025
 *      Author: knn64
 */

#ifndef INC_TASK_FSM_H_
#define INC_TASK_FSM_H_

#define FSM_NOTIF_TAP_DETECTED 0x0000000F
#define FSM_NOTIF_ToF_LEFT 0x00000010
#define FSM_NOTIF_ToF_RIGHT 0x00000020
#define FSM_NOTIF_ToF_FRONT 0x00000030
#define FSM_NOTIF_ToF_REAR 0x00000040

#define TABLE_WIDTH  200.0f   // 2 m
#define TABLE_HEIGHT 400.0f   // 4 m
#define CHECK_SIZE   30.0f    // cm
#define SECURITY_DISTANCE 10.0f


/* Handle de la tache pour recevoir des notifications */
extern osThreadId_t task_fsm_handle;

void TaskFSM(void *argument);

/* Evade status: heading is degree in robot frame (0 = front), checkpoint is 0..3, ready==true when prediction available */
void FSM_GetEvadeInfo(float *heading, int *checkpoint, _Bool *ready);

#endif /* INC_TASK_FSM_H_ */
