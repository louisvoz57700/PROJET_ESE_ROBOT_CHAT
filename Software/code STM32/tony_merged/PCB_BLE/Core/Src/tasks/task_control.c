/*
 * task_control.c
 *
 * Created on: Dec 9, 2025
 * Author: knn64
 * Updated: Slave to FSM
 */

#include "tasks/task_control.h"
#include "tasks/task_FSM.h" // INDISPENSABLE : Pour voir RobotState et STATE_CHASE

#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdbool.h>

#include "actionneurs/moteur.h"
#include "capteurs/lidar.h"
#include "capteurs/sensor.h"

/* --- PARAMÈTRES DE COMBAT --- */
#define ALIGN_TOLERANCE     15.0f
#define CHARGE_CHUNK        400.0f
#define ATTACK_BONUS_DIST   100.0f
#define ATTACK_SPEED        2000.0f
#define ROTATION_SPEED      1000.0f

/* --- VARIABLES EXTERNES --- */
extern volatile Target_t current_target;
/* currentState est déclaré "extern" via task_FSM.h */

void TaskControl(void *argument)
{
    UNUSED(argument);
    Target_t target;

    /* 1. Init Système Moteur */
    Motor_Init_System();

    for (;;)
    {
        // On vérifie si c'est à nous de jouer
        if (currentState == STATE_CHASE)
        {
            taskENTER_CRITICAL();
            target = current_target;
            taskEXIT_CRITICAL();

            if (target.detected)
            {
                float angle_relatif = target.angle;

                if (angle_relatif > 180.0f) angle_relatif -= 360.0f;
                if (angle_relatif < -180.0f) angle_relatif += 360.0f;

                if (fabsf(angle_relatif) > ALIGN_TOLERANCE)
                {
                    Robot_Rotation(ROTATION_SPEED, -angle_relatif);
                    osDelay(50);
                }
                else
                {
                    float dist_to_go = target.dist;
                    if (dist_to_go > CHARGE_CHUNK)
                    {
                        Robot_Translation(ATTACK_SPEED, CHARGE_CHUNK);
                    }
                    else
                    {
                        Robot_Translation(ATTACK_SPEED + 200.0f, dist_to_go + ATTACK_BONUS_DIST);
                    }
                    osDelay(50);
                }
            }
            else
            {
                Motor_Stop_Both();
                osDelay(100);
            }
        }
        else
        {
            // Pas en CHASE -> On laisse la main au FSM
            osDelay(20);
        }
    }
}
