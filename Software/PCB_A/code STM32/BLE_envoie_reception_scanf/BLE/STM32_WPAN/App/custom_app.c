/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file App/custom_app.c
  * @author MCD Application Team
  * @brief Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* myService */
  uint8_t               Charnotify_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* myService */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* myService */
    case CUSTOM_STM_CHARWRITE_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHARWRITE_WRITE_EVT */
      printf("Données reçues: %s\r\n", pNotification->DataTransfered.pPayload);
      /* USER CODE END CUSTOM_STM_CHARWRITE_WRITE_EVT */
      break;

    case CUSTOM_STM_CHARNOTIFY_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHARNOTIFY_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Charnotify_Notification_Status = 1; // Notifications activées
      printf("Notifications activées pour charNotify\r\n");
      /* USER CODE END CUSTOM_STM_CHARNOTIFY_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CHARNOTIFY_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHARNOTIFY_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Charnotify_Notification_Status = 0; // Notifications désactivées
      printf("Notifications désactivées pour charNotify\r\n");
      /* USER CODE END CUSTOM_STM_CHARNOTIFY_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      printf("Notification envoyée avec succès\r\n");
      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  Custom_App_Context.Charnotify_Notification_Status = 0; // Initialisation à désactivé
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* myService */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void SendNotification(void) {
  /* USER CODE BEGIN SendNotification */
  if (Custom_App_Context.Charnotify_Notification_Status) {
    char *message = customMessage; // Utiliser le message personnalisé ou par défaut
    uint8_t len = strlen(message);
    if (len > 0) {
      memcpy(NotifyCharData, message, len + 1); // Inclure \0
      printf("\rDonnées envoyées (hex): ");
      for (int i = 0; i < len + 1; i++) {
        printf("%02x ", NotifyCharData[i]);
      }
      printf("\n");
      tBleStatus status = Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_CHARNOTIFY, (uint8_t*)NotifyCharData, len + 1);
      if (status == BLE_STATUS_SUCCESS) {
        printf("\r Envoi: %s\r\n", message);
        printf("Notification envoyée avec succès, longueur: %d\r\n", len + 1);
      } else {
        printf("Échec de l'envoi, statut: 0x%x\r\n", status);
      }
    }
    messageReady = 0; // Réinitialiser après envoi
  } else {
    printf("Notifications non activées\r\n");
  }
  /* USER CODE END SendNotification */
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
