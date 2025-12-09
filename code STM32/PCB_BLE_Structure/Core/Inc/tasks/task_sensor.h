/*
 * task_sensor.h
 *
 *  Created on: Dec 2, 2025
 *      Author: knn64
 */

#ifndef INC_TASK_SENSOR_H_
#define INC_TASK_SENSOR_H_

#include "capteurs/sensor.h"
#include "cmsis_os2.h"

/* Handle des taches pour recevoir des notifications */
extern osThreadId_t task_lidar_handle ;
extern osThreadId_t task_ToF_handle;

typedef struct {
	Measure meas;
	/* Odometrie*/
	float robot_x;
	float robot_y;
	float robot_heading;   // degrés

} OdomData_t;

typedef struct {
	/* Lidar */
	float distance_obstacle;
	float target_x;
	float target_y;
	Pos vue[N_ANGLES];
} LidarData_t;

typedef struct {
    float target_r;       // distance ennemi détecté (si détecté)
    float target_theta;   // angle ennemi
    bool target_detected;

    float lidar_min;      // distance la plus courte autour du robot
    bool obstacle_detected;
} LidarFrame;

void TaskToF(void *argument);
void TaskLidar(void *argument);

#endif /* INC_TASK_SENSOR_H_ */
