/*
 * sensor.h
 *
 * Created on: Oct 28, 2025
 * Author: louisvoz
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include "capteurs/VL53L0X.h"
#include "capteurs/lidar.h"
#include "capteurs/multiplexer.h"
#include "cmsis_os2.h"

typedef struct{
	float courant ;
	float tension ;
	float A_X;
	float A_Y;
	float A_Z;
} Measure;


/* Prototypes ADC & Maths */
void INIT_ADC(uint16_t *adc_buffer, uint16_t size);
void UPDATE_V_I(Measure *m1, uint16_t *adc_buffer);

/* Prototypes ADXL343 */
HAL_StatusTypeDef ADXL343_Init(void);
HAL_StatusTypeDef ADXL343_ReadXYZ(Measure *m1);
HAL_StatusTypeDef ADXL343_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ADXL343_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ADXL343_EnableSingleTap(void);
void check_single_tap(uint8_t * ret);
/* Prototypes TCA9548A (Multiplexeur) */
void TCA9548A_Init(void);
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel);

/* Prototypes VL53L0X */
void ToF_Init(ToF_t *tof);

/* Prototypes Utils */
void I2C_ResetBus(I2C_HandleTypeDef *hi2c);

#endif /* INC_SENSOR_H_ */
