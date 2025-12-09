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

/* Handle de la tache pour recevoir des notifications */
extern osThreadId_t task_fsm_handle;

void TaskFSM(void *argument);

#endif /* INC_TASK_FSM_H_ */
