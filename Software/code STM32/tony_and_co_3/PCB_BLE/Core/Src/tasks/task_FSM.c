/*
 * task_FSM.c
 *
 * Created on: Dec 3, 2025
 * Author: knn64
 * Updated: Master Logic - Fixed Headers
 */

#include "tasks/task_FSM.h"     // Contient maintenant l'Enum RobotState
#include "tasks/task_control.h"
#include "tasks/task_comm.h"
#include "tasks/task_sensor.h"

#include "actionneurs/moteur.h"
#include "capteurs/lidar.h"
#include "capteurs/sensor.h"
#include "capteurs/sensor.h"

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <math.h>

/* --- VARIABLES GLOBALES --- */
// On initialise la variable globale ici
volatile RobotState currentState = STATE_IDLE;

uint8_t int_source = 0;

/* --- VARIABLES EXTERNES --- */
extern osMessageQueueId_t QLidar;
extern volatile Target_t current_target;
extern osThreadId_t task_comm_handle;
extern OdomData_t Robot_pos;

/* --- CONSTANTES LOCALES --- */
#define CLIFF_BACKUP_DIST   -120.0f // Recul en mm après détection vide
// CHECK_SIZE est déjà dans task_FSM.h

/* --- FONCTIONS PRIVÉES --- */

static void obstacle_avoid(uint32_t side)
{
    Motor_Stop_Both();
    osDelay(50);

    switch (side)
    {
    case (FSM_NOTIF_ToF_FRONT):
        Robot_Translation(1000.0f, CLIFF_BACKUP_DIST);
        osDelay(100);
        Robot_Rotation(1000.0f, 135.0f);
        break;

    case (FSM_NOTIF_ToF_REAR):
        Robot_Translation(1000.0f, -CLIFF_BACKUP_DIST);
        break;

    case (FSM_NOTIF_ToF_LEFT):
        Robot_Rotation(1000.0f, -90.0f);
        Robot_Translation(1000.0f, 100.0f);
        break;

    case (FSM_NOTIF_ToF_RIGHT):
        Robot_Rotation(1000.0f, 90.0f);
        Robot_Translation(1000.0f, 100.0f);
        break;
    }

    currentState = STATE_SEARCH;
}

static void evade(void)
{
    Robot_Rotation(1500.0f, 170.0f);
    Robot_Translation(2000.0f, 300.0f);
    currentState = STATE_SEARCH;
}

void TaskFSM(void *argument)
{
    osMutexId_t i2c_mutex = (osMutexId_t) argument;
    uint32_t notif = 0;
    for (;;)
    {
        // Timeout court (10ms)
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notif, 10) == pdTRUE)
        {
            if (notif == FSM_NOTIF_TAP_DETECTED)
            {


                if (int_source == 1) // Single Tap
                {
                    if (currentState == STATE_IDLE) {
                        currentState = STATE_SEARCH;
                    } else if (currentState == STATE_CHASE) {
                        currentState = STATE_TAG;
                    } else {
                        currentState = STATE_HIT;
                    }
                    int_source = 0;

                }
            }
            else if (notif & (FSM_NOTIF_ToF_FRONT | FSM_NOTIF_ToF_REAR | FSM_NOTIF_ToF_LEFT | FSM_NOTIF_ToF_RIGHT))
            {
                obstacle_avoid(notif & 0xF0);
                continue;
            }
        }

        switch (currentState)
        {
        case STATE_IDLE:
            Motor_Stop_Both();
            osDelay(100);
            break;

        case STATE_SEARCH:
            taskENTER_CRITICAL();
            bool seen = current_target.detected;
            taskEXIT_CRITICAL();

            if (seen) {
                currentState = STATE_CHASE;
            } else {
                Robot_Rotation(600.0f, 45.0f);
                osDelay(200);
            }
            break;

        case STATE_CHASE:
            if (!current_target.detected) {
                osDelay(500);
                if (!current_target.detected) currentState = STATE_SEARCH;
            }
            osDelay(50);
            break;

        case STATE_EVADE:
            evade();
            break;

        case STATE_TAG:
            xTaskNotify(task_comm_handle, COMM_NOTIF_CAT, eSetValueWithOverwrite);
            Motor_Stop_Both();
            osDelay(1000);
            currentState = STATE_SEARCH;
            break;

        case STATE_HIT:
            xTaskNotify(task_comm_handle, COMM_NOTIF_MOUSE, eSetValueWithOverwrite);
            Robot_Rotation(800.0f, 20.0f);
            Robot_Rotation(800.0f, -20.0f);
            currentState = STATE_SEARCH;
            break;

        default:
            currentState = STATE_IDLE;
            break;
        }
    }
}
