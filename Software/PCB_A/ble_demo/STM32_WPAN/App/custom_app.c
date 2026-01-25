/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"
#include <string.h>
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
static uint8_t last_received_value = 0;  // Store the last received value
// Buffer to store the last received message (for debugging) - made extern for main.c access
uint8_t last_received_message[32];
uint16_t last_received_length = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* myService */
static void Custom_Charnotify_Update_Char(void);
static void Custom_Charnotify_Send_Notification(void);

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
      {
        uint8_t *data_ptr = pNotification->DataTransfered.pPayload;
        uint8_t len   = pNotification->DataTransfered.Length;

        /* CRITICAL: Copy data immediately to avoid buffer reuse issues */
        /* The event buffer pointer may become invalid, so copy first! */
        if (len > 0)
        {
          uint8_t copy_len = (len <= sizeof(last_received_message)) ? len : sizeof(last_received_message);
          memcpy(last_received_message, data_ptr, copy_len);
          last_received_length = len;  // Store original length
          
          // Null terminate for easier string viewing in debugger (if space available)
          if (copy_len < sizeof(last_received_message))
          {
            last_received_message[copy_len] = '\0';
          }
          
          // Now use the copied data for all operations
          data_ptr = last_received_message;
        }

        /* First LED (PA2): Indicate that data is being received */
        if (len > 0)
        {
          uint8_t loop_len = (len <= sizeof(last_received_message)) ? len : sizeof(last_received_message);
          for (int i = 0; i < loop_len; i++)
          {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, data_ptr[i]);
            HAL_Delay(100);
          }
        }

        /* Cat/mouse checking moved to main while loop */
      }
      /* USER CODE END CUSTOM_STM_CHARWRITE_WRITE_EVT */
      break;

    case CUSTOM_STM_CHARNOTIFY_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHARNOTIFY_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_CHARNOTIFY_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CHARNOTIFY_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHARNOTIFY_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_CHARNOTIFY_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

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
__USED void Custom_Charnotify_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Charnotify_UC_1*/

  /* USER CODE END Charnotify_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CHARNOTIFY, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Charnotify_UC_Last*/

  /* USER CODE END Charnotify_UC_Last*/
  return;
}

void Custom_Charnotify_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Charnotify_NS_1*/

  /* USER CODE END Charnotify_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CHARNOTIFY, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Charnotify_NS_Last*/

  /* USER CODE END Charnotify_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/**
 * @brief  Send data via BLE notify characteristic
 * @param  pPayload: Pointer to data to send
 * @param  size: Size of data to send (max 512 bytes)
 * @retval tBleStatus: Status of the operation
 */
tBleStatus Custom_APP_Send_Data(uint8_t *pPayload, uint8_t size)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  
  if (pPayload != NULL && size > 0 && size <= 512)
  {
    /* Copy payload to the notification buffer */
    memcpy(NotifyCharData, pPayload, size);
    
    /* Update the characteristic value */
    ret = Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_CHARNOTIFY, (uint8_t *)NotifyCharData, size);
  }
  
  return ret;
}

/**
 * @brief  Send a single character via BLE notify characteristic
 * @param  data: Character to send
 * @retval tBleStatus: Status of the operation
 */
tBleStatus Custom_APP_Send_Char(uint8_t data)
{
  return Custom_APP_Send_Data(&data, 1);
}

/**
 * @brief  Send position data via BLE notify characteristic
 * @param  x: Robot X position
 * @param  y: Robot Y position
 * @param  angle: Robot angle
 * @param  x_t: Target X position
 * @param  y_t: Target Y position
 * @retval tBleStatus: Status of the operation
 */
tBleStatus Custom_APP_Send_Position(uint8_t x, uint8_t y, uint8_t angle, uint8_t x_t, uint8_t y_t)
{
  uint8_t payload[5];
  payload[0] = x;
  payload[1] = y;
  payload[2] = angle;
  payload[3] = x_t;
  payload[4] = y_t;
  
  return Custom_APP_Send_Data(payload, 5);
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
