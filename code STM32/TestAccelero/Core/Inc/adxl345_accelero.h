/*
 * adxl345_accelerom.h
 *
 *  Created on: Oct 1, 2025
 *      Author: kennystflr
 */

#ifndef INC_ADXL345_ACCELEROM_H_
#define INC_ADXL345_ACCELEROM_H_

#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>

// Définitions des pins (à configurer selon votre projet)
#define ADXL345_CS_LOW()  HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET)
#define ADXL345_CS_HIGH() HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_SET)
#define ADXL345_INT1_Pin  GPIO_PIN_X  // Remplacer par la pin réelle (ex: GPIO_PIN_2)
#define ADXL345_INT2_Pin  GPIO_PIN_Y  // Remplacer par la pin réelle (ex: GPIO_PIN_3)

// Sensibilité de l'ADXL345 en g/LSB (pour ±16g, full resolution)
#define ADXL345_SENSITIVITY_G_PER_LSB  0.0039f

// Structures pour les données
typedef struct {
    float roll;   // En degrés
    float pitch;  // En degrés
} ADXL345_AxesData;

typedef struct {
    uint8_t single_tap_detected;
    uint8_t double_tap_detected;
} ADXL345_TapData;

// Prototypes des fonctions
void ADXL345_Init(void);
void ADXL345_Calibrate(uint8_t num_samples);  // Nouvelle fonction de calibration
ADXL345_AxesData ADXL345_GetAxesData(void);
ADXL345_TapData ADXL345_GetTapData(void);

#endif // ADXL345_H

#endif /* INC_ADXL345_ACCELEROM_H_ */
