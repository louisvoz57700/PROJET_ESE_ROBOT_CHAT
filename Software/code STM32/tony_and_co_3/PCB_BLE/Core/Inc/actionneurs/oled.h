/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file oled.h
  * @brief Header for oled.c module (SSD1306 OLED driver)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Your Name.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h" // à adapter suivant carte
#include "cmsis_os2.h"

/* Handler structure -------------------------------------------------------- */
typedef struct {
	I2C_HandleTypeDef i2c;
	osMutexId_t i2c_mutex;
	uint8_t current_bitmap;
}oled_handle_t;


/* Exported variables --------------------------------------------------------*/
extern const uint8_t bitmap_data[1024];      // mouse_cat_bitmap_128x64
extern const uint8_t bitmap_data_mouse[1024]; // mouse_bitmap_128x64
extern const uint8_t bitmap_data_cat[1024];

/* Exported functions prototypes ---------------------------------------------*/
oled_handle_t OLED_Init(I2C_HandleTypeDef *i2c, osMutexId_t i2c_mutex);
void OLED_Clear(void);
void OLED_DisplayChar(char c, uint8_t page, uint8_t col);
void OLED_DisplayString(const char* str, uint8_t page, uint8_t col);
void OLED_DisplayBitmap(const uint8_t* bitmap);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
