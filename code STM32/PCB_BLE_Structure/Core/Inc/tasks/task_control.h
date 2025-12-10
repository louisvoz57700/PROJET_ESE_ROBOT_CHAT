/*
 * task_control.h
 *
 *  Created on: Dec 9, 2025
 *      Author: knn64
 */

#ifndef INC_TASKS_TASK_CONTROL_H_
#define INC_TASKS_TASK_CONTROL_H_

#include "actionneurs/moteur.h"


typedef struct {
	Measure meas;
	/* Odometrie*/
	float robot_x;
	float robot_y;
	float robot_heading;   // degrés

} OdomData_t;

/* Handle de la tache pour recevoir des notifications */
extern osThreadId_t task_control_handle;

void TaskControl(void *argument);

#endif /* INC_TASKS_TASK_CONTROL_H_ */
