/*
 * task_comm.h
 *
 *  Created on: Dec 3, 2025
 *      Author: knn64
 */

#ifndef INC_TASK_COMM_H_
#define INC_TASK_COMM_H_

#include "actionneurs/oled.h"
#include "stdbool.h"
#include "cmsis_os2.h"

/* Handle de la tache pour recevoir des notifications */
extern osThreadId_t task_comm_handle;

/* Bitmaps OLED */
extern const uint8_t bitmap_data[1024];
extern const uint8_t bitmap_data_mouse[1024];
extern const uint8_t bitmap_data_cat[1024];

void TaskCommuncation(void *argument);
#endif /* INC_TASK_COMM_H_ */
