/*
 * sensor.h
 *
 * Created on: Oct 28, 2025
 * Author: louisvoz
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"

/* Prototypes ADC & Maths */
void INIT_ADC(uint16_t *adc_buffer, uint16_t size);
void UPDATE_V_I(Measure *m1, uint16_t *adc_buffer);

/* Prototypes ADXL343 */
HAL_StatusTypeDef ADXL343_Init(void);
HAL_StatusTypeDef ADXL343_ReadXYZ(Measure *m1);
HAL_StatusTypeDef ADXL343_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ADXL343_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ADXL343_EnableSingleTap(void);

/* Prototypes TCA9548A (Multiplexeur) */
void TCA9548A_Init(void);
void TCA9548A_SelectChannel(uint8_t channel);

/* Prototypes Utils */
void I2C_ResetBus(I2C_HandleTypeDef *hi2c);

#endif /* INC_SENSOR_H_ */
