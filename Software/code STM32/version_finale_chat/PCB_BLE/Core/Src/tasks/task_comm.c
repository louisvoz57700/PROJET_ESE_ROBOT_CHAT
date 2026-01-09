/*
 * task_comm.c
 *
 * Created on: Dec 3, 2025
 * Author: knn64
 * Updated: Merged Screen & Comm logic
 */

#include "tasks/task_comm.h"
#include "cmsis_os.h"
#include "actionneurs/oled.h"      // Assure-toi que c'est le bon header pour OLED_Init/Display
#include "main.h"      // Pour UNUSED si besoin

/* --- EXTERNES (Bitmaps définis dans main.c ou oled_assets.c) --- */
extern const uint8_t bitmap_data[1024];       // Logo par défaut
extern const uint8_t bitmap_data_mouse[1024]; // Image Souris
extern const uint8_t bitmap_data_cat[1024];   // Image Chat

/* --- CONSTANTES LOCALES --- */
#define ROLE_DEFAULT 0
#define ROLE_MOUSE   1
#define ROLE_CAT     2

/**
 * @brief Fonction interne pour dessiner selon le rôle actuel
 * @param p : Pointeur vers la structure de gestion OLED
 */
static void drawRoleOLED(oled_handle_t * p)
{
    /* Protection I2C via le Mutex passé dans la structure */
    if (osMutexAcquire(p->i2c_mutex, osWaitForever) == osOK)
    {
        // On évite OLED_Clear() complet pour réduire le scintillement
        // On écrase simplement la RAM graphique avec le nouveau bitmap

        if (p->current_bitmap == ROLE_DEFAULT) {
            OLED_DisplayBitmap(bitmap_data);
        }
        else if (p->current_bitmap == ROLE_MOUSE) {
            OLED_DisplayBitmap(bitmap_data_mouse);
        }
        else if (p->current_bitmap == ROLE_CAT) {
            OLED_DisplayBitmap(bitmap_data_cat);
        }

        osMutexRelease(p->i2c_mutex);
    }
}

/* * Tâche 4 : Communication & Affichage
 * Priorité : Basse/Normale
 * Rôle :
 * 1. Initialise l'écran OLED.
 * 2. Attend des notifications (soit du BLE, soit d'autres tâches).
 * 3. Met à jour l'écran en conséquence.
 */
void TaskCommunication(void *argument)
{
    /* On récupère la structure passée en argument lors de la création de la tâche */
    oled_handle_t * p = (oled_handle_t *)argument;

    /* --- 1. INITIALISATION OLED --- */
    osDelay(100); // Petite pause pour laisser la tension monter

    if (osMutexAcquire(p->i2c_mutex, osWaitForever) == osOK)
    {
        OLED_Clear();
        OLED_DisplayBitmap(bitmap_data);
        osMutexRelease(p->i2c_mutex);
    }

    uint32_t notif = 0;

    /* --- 2. BOUCLE PRINCIPALE --- */
    for (;;)
    {
        /* Attente infinie d'une notification.
         * La valeur des bits de notification est stockée dans 'notif'.
         * 0xFFFFFFFF nettoie tous les bits après lecture.
         */
        xTaskNotifyWait(0, 0xFFFFFFFF, &notif, portMAX_DELAY);

        /* Analyse de la notification reçue */
        /* Note: On utilise des if ou switch selon comment tu définis les bits */

        switch(notif) {
            case COMM_NOTIF_CAT:
                p->current_bitmap = ROLE_CAT;
                drawRoleOLED(p);
                break; // IMPORTANT : ne pas oublier le break

            case COMM_NOTIF_MOUSE:
                p->current_bitmap = ROLE_MOUSE;
                drawRoleOLED(p);
                break;

            default:
                // Notification inconnue ou Tap générique (rafraîchissement forcé)
                drawRoleOLED(p);
                break;
        }
    }
}
