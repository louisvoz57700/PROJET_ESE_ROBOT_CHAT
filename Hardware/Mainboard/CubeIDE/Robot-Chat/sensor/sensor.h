/*
 * sensor.h
 *
 *  Created on: Nov 25, 2025
 *      Author: knn64
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "adxl345.h"          // ton driver ADXL345 déjà vu

// ---------------------------------------------------------------------------
// Événements sémantiques envoyés à la FSM (8 bits → ultra rapide)
// ---------------------------------------------------------------------------
typedef enum {
    EV_NO_EVENT = 0,

    // ADXL345
    EV_BUMP_FRONT,
    EV_BUMP_REAR,
    EV_SIDE_KICK_LEFT,
    EV_SIDE_KICK_RIGHT,
    EV_USER_DOUBLE_TAP,
    EV_ROBOT_LIFTED,          // free-fall
    EV_ROBOT_CLIMBING,        // pente > 18°
    EV_ROBOT_DESCENDING,

    // ToF (4 capteurs)
    EV_OBSTACLE_FRONT_LEFT,   // < 150 mm
    EV_OBSTACLE_FRONT_RIGHT,
    EV_OBSTACLE_REAR_LEFT,
    EV_OBSTACLE_REAR_RIGHT,
    EV_OBSTACLE_VERY_CLOSE,   // < 80 mm → urgence

    EV_SENSORS_MAX
} RobotEvent_t;

// ---------------------------------------------------------------------------
// Handle opaque de la tâche capteurs
// ---------------------------------------------------------------------------
typedef struct SensorsContext_t SensorsHandle_t;

// Création / destruction
SensorsHandle_t* Sensors_Create(ADXL345_Handle_t *adxl,
                                void *tof1, void *tof2, void *tof3, void *tof4,   // tes handles ToF
                                QueueHandle_t event_queue);

void Sensors_Delete(SensorsHandle_t *handle);

// Lancement de la tâche (une seule fois)
void Sensors_StartTask(SensorsHandle_t *handle, UBaseType_t priority);

#endif // SENSORS_H
