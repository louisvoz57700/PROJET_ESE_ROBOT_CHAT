/*
 * task_comm.c
 *
 *  Created on: Dec 3, 2025
 *      Author: knn64
 */


#include "tasks/task_comm.h"
#include "cmsis_os.h"

#define ROLE_CAT 2
#define ROLE_MOUSE 1

static void drawRoleOLED (oled_handle_t * p){

	/* Dessin protégé */
	if (osMutexAcquire(p->i2c_mutex, osWaitForever) == osOK)
	{
		// Pas de OLED_Clear() pour éviter le scintillement
		if (p->current_bitmap == 0)
			OLED_DisplayBitmap(bitmap_data);
		else if (p->current_bitmap == 1)
			OLED_DisplayBitmap(bitmap_data_mouse);
		else
			OLED_DisplayBitmap(bitmap_data_cat);

		osMutexRelease(p->i2c_mutex);
	}
}

//Fonction de traitement des infos reçues en BLE


/* * Tâche 4 : Communcation
 * Priorité : Basse 2
 * - Met à jour l'écran seulement quand demandé (Tap) ou init.
 * - Gestion de la liaison
 */
void TaskCommuncation(void *argument)
{
	oled_handle_t * p = (oled_handle_t *)argument;
	/* Initialisation OLED protégée */
	OLED_Clear();
	OLED_DisplayBitmap(bitmap_data);
	uint32_t notif = 0;
	for (;;)
	{
		xTaskNotifyWait(0x00, 0x00, &notif, portMAX_DELAY);

		switch(notif) {
		case(COMM_NOTIF_CAT) :
			p->current_bitmap = ROLE_CAT;
			drawRoleOLED(p);
		case(COMM_NOTIF_MOUSE) :
			p->current_bitmap = ROLE_MOUSE;
			drawRoleOLED(p);
		}

	}
}

