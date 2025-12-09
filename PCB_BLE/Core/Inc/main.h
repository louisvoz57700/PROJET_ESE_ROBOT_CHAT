/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct{
	float courant ;
	float tension ;
	float A_X;
	float A_Y;
	float A_Z;
} Measure;

typedef struct {
	float X;
	float Y;
	float orientation;
	float angle_r;
	float angle_l;
	float Depl_R;
	float Depl_L;
	float delta_S;
	float delta_theta;
	float Vitesse_D;
	float Vitesse_G

} Position;

typedef struct {
	uint16_t front;
	uint16_t back;
	uint16_t right;
	uint16_t left;

} TOF;

typedef struct {
	uint16_t PWM_R;
	uint16_t PWM_L;
	uint8_t chock;
	uint8_t space ; // 0 = RAS , 1 = devant, 2 = derriere, 3 = droite, 4 = gauche
} System_state;


typedef struct {
    uint16_t num_samples;   // Nombre de samples
    float start_angle;      // Angle de départ (en degrés ou radians)
    float end_angle;        // Angle de fin (en degrés ou radians)
    float precision;
} AngleData;

typedef struct {
	uint16_t distance ;
    float X ;
    float Y;        // Angle de fin (en degrés ou radians)
    float Angle;
} Pos;

typedef struct {
    uint8_t active;        // 0 = vide, 1 = utilisé
    uint16_t start_idx;    // index du premier point du cluster
    uint16_t end_idx;      // index du dernier point
    float x;             // barycentre X (fix-point ou int)
    float y;             // barycentre Y
    uint16_t size;         // nombre de points
    uint8_t ttl;   // time-to-live
} Cluster;



void Task_TOF(void *argument);
void Task_default(void *argument);
void Task_odom(void *argument);
void Task_lidar(void *argument);
void Task_com(void *argument);
void Task_sensor(void *argument);
HAL_StatusTypeDef VL53L0X_Read(uint16_t *buffer);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
