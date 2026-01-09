/*
 * app.h
 *
 *  Created on: Dec 9, 2025
 *      Author: knn64
 */

#ifndef INC_APP_H_
#define INC_APP_H_


#include "cmsis_os.h"
#include "timers.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "tasks/task_FSM.h"
#include "tasks/task_sensor.h"
#include "tasks/task_comm.h"
#include "tasks/task_control.h"

#define TOF_I2C_HANDLER hi2c1

void app_init();
void app_task_init();

#endif /* INC_APP_H_ */
