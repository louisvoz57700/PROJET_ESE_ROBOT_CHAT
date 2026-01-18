/*
 * sensor.c
 *
 *  Created on: Nov 25, 2025
 *      Author: knn64
 */


#include "sensor.h"
//#include "vl53l0x_api.h"        // ou vl53l1x, vl53l4cd… selon tes capteurs
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Contexte privé (opaque)
// ---------------------------------------------------------------------------
struct SensorsContext_t {
    // Accéléromètre
    ADXL345_Handle_t   *adxl;

    // 4 ToF (on garde void* → compatible avec tous les drivers ST)
    void               *tof[4];
    uint16_t            last_dist_mm[4];

    // Files d'événements
    QueueHandle_t       event_queue;

    // Filtre pitch/roll
    float               pitch_filtered;
    uint32_t            last_inclinaison_tick;
};

// Queue interne : données brutes de l'ADXL postées par l'ISR
static QueueHandle_t rawAdxlQueue = NULL;

#define RAW_QUEUE_LENGTH    8
#define RAW_ITEM_SIZE       8       // 6 octets data + 2 octets timestamp hi

// ---------------------------------------------------------------------------
// ISR ADXL345 (à mettre dans ton fichier d'interruptions)
// ---------------------------------------------------------------------------
void ADXL345_ISR_Handler(void)
{
    static uint8_t burst[6];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Lecture ultra-rapide des 6 octets accélération (FIFO du ADXL345)
    ADXL345_BurstRead(g_adxl_global, 0x32, burst, 6);  // DATA_X0 → DATA_Z1

    uint8_t payload[RAW_ITEM_SIZE];
    memcpy(payload, burst, 6);
    uint16_t tick_hi = (uint16_t)(xTaskGetTickCountFromISR() >> 16);
    payload[6] = tick_hi >> 8;
    payload[7] = tick_hi & 0xFF;

    xQueueSendFromISR(rawAdxlQueue, payload, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ---------------------------------------------------------------------------
// Création du contexte
// ---------------------------------------------------------------------------
SensorsHandle_t* Sensors_Create(ADXL345_Handle_t *adxl,
                                void *tof1, void *tof2, void *tof3, void *tof4,
                                QueueHandle_t event_queue)
{
    SensorsHandle_t *ctx = (SensorsHandle_t*)pvPortMalloc(sizeof(SensorsContext_t));
    if (!ctx) return NULL;

    ctx->adxl = adxl;
    ctx->tof[0] = tof1;  ctx->tof[1] = tof2;
    ctx->tof[2] = tof3;  ctx->tof[3] = tof4;
    ctx->event_queue = event_queue;
    ctx->pitch_filtered = 0.0f;
    ctx->last_inclinaison_tick = 0;
    memset(ctx->last_dist_mm, 0xFF, sizeof(ctx->last_dist_mm));

    // Création de la queue interne pour l'ISR
    if (!rawAdxlQueue)
        rawAdxlQueue = xQueueCreate(RAW_QUEUE_LENGTH, RAW_ITEM_SIZE);

    return ctx;
}

void Sensors_Delete(SensorsHandle_t *handle)
{
    if (handle) vPortFree(handle);
}

// ---------------------------------------------------------------------------
// Tâche unique qui gère TOUT : ADXL + 4 ToF
// ---------------------------------------------------------------------------
static void Sensors_Task(void *pvParameters)
{
    SensorsHandle_t *ctx = (SensorsHandle_t*)pvParameters;
    uint8_t raw[RAW_ITEM_SIZE];
    RobotEvent_t ev = EV_NO_EVENT;

    for(;;)
    {
        // 1. Traitement ADXL (prioritaire)
        while (xQueueReceive(rawAdxlQueue, raw, 0) == pdTRUE)
        {
            int16_t ax = ((int16_t)raw[1] << 8) | raw[0];
            int16_t ay = ((int16_t)raw[3] << 8) | raw[2];
            int16_t az = ((int16_t)raw[5] << 8) | raw[4];

            uint8_t int_src = 0;
            ADXL345_ReadReg(ctx->adxl, 0x30, &int_src);

            if (int_src & 0x10) { ev = EV_ROBOT_LIFTED; }                                   // FREEFALL
            else if (int_src & 0x20) { ev = EV_USER_DOUBLE_TAP; }                           // DOUBLE TAP
            else if (int_src & 0x40) {                                                              // SINGLE TAP
                if (az > 1300)                  ev = EV_BUMP_FRONT;
                else if (az < -1300)            ev = EV_BUMP_REAR;
                else if (ax > 1400)             ev = EV_SIDE_KICK_LEFT;
                else if (ax < -1400)            ev = EV_SIDE_KICK_RIGHT;
            }

            // Filtre d'inclinaison toutes les 25 ms
            if (xTaskGetTickCount() - ctx->last_inclinaison_tick >= 25) {
                float pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;
                ctx->pitch_filtered = ctx->pitch_filtered * 0.85f + pitch * 0.15f;

                if (ctx->pitch_filtered > 20.0f)        ev = EV_ROBOT_CLIMBING;
                else if (ctx->pitch_filtered < -20.0f)  ev = EV_ROBOT_DESCENDING;

                ctx->last_inclinaison_tick = xTaskGetTickCount();
            }

            if (ev != EV_NO_EVENT) {
                xQueueSend(ctx->event_queue, &ev, 0);
                ev = EV_NO_EVENT;
            }
        }

        // 2. Lecture des 4 ToF (polling rapide, tous les 15 ms)
        for (int i = 0; i < 4; i++) {
            if (!ctx->tof[i]) continue;

            uint16_t dist_mm = 8191;  // valeur "out of range" officielle ST
            VL53L0X_GetDistance(ctx->tof[i], &dist_mm);   // ou VL53L1X, etc.

            // Hystérésis 20 mm pour éviter le bruit
            if (abs((int)dist_mm - (int)ctx->last_dist_mm[i]) > 20) {
                ctx->last_dist_mm[i] = dist_mm;

                if (dist_mm < 80) {
                    ev = EV_OBSTACLE_VERY_CLOSE;
                } else if (dist_mm < 150) {
                    switch(i) {
                        case 0: ev = EV_OBSTACLE_FRONT_LEFT;  break;
                        case 1: ev = EV_OBSTACLE_FRONT_RIGHT; break;
                        case 2: ev = EV_OBSTACLE_REAR_LEFT;    break;
                        case 3: ev = EV_OBSTACLE_REAR_RIGHT;   break;
                    }
                }
                if (ev != EV_NO_EVENT) {
                    xQueueSend(ctx->event_queue, &ev, 0);
                    ev = EV_NO_EVENT;
                }
            }
        }

        // Petite pause pour ne pas monopoliser le CPU
        vTaskDelay(pdMS_TO_TICKS(8));  // ~125 Hz global
    }
}

// ---------------------------------------------------------------------------
// Lancement de la tâche
// ---------------------------------------------------------------------------
void Sensors_StartTask(SensorsHandle_t *handle, UBaseType_t priority)
{
    xTaskCreate(Sensors_Task, "Sensors", 640, handle, priority, NULL);
}

