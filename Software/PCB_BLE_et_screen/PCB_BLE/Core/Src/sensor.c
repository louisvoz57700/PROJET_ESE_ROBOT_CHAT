/*
 * sensor.c
 *
 *  Created on: Oct 28, 2025
 *      Author: louisvoz
 */


#include "main.h"
#include "sensor.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;  // ton handle I2C
#define ADXL343_REG_BW_RATE  0x2C
#define ADXL343_DEVID_VALUE  0xE5

#define ADXL343_ADDR        (0x53 << 1)  // I2C address shifted for STM32 HAL
#define ADXL343_REG_DEVID   0x00
#define ADXL343_REG_POWERCTL 0x2D
#define ADXL343_REG_DATAX0  0x32
#define ADXL343_REG_DATA_FORMAT 0x31
#define ALPHA 0.05

#define ADXL343_MG2G_MULTIPLIER (0.0039f) // 4mg/LSB in full resolution mode (±2g)


void INIT_ADC(uint16_t *adc_buffer,uint16_t size)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
   {
	   Error_Handler();  // en cas d’échec
   }
  HAL_ADC_Start_DMA(&hadc1, (uint16_t*)adc_buffer, size);
}

void UPDATE_V_I(Measure *m1,uint16_t *adc_buffer)
{
		m1->tension = ALPHA * ((float)adc_buffer[1] * (3.3f / 4095.0f) * 3)+ (1-ALPHA)*(m1->tension);
		m1->courant = ALPHA * ((float)adc_buffer[0] * 2.0 * (3.3f / (4095.0f))) + (1-ALPHA)*(m1->courant);
		if (ADXL343_ReadXYZ(m1) != HAL_OK)
		{
			Error_Handler();
		}


}

HAL_StatusTypeDef ADXL343_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, ADXL343_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL343_ReadReg(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c1, ADXL343_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1, HAL_MAX_DELAY);
}

/* Lecture de plusieurs octets (utilisée pour lire les axes) */
HAL_StatusTypeDef ADXL343_ReadMulti(uint8_t reg, uint8_t *buffer, uint16_t size)
{
    return HAL_I2C_Mem_Read(&hi2c1, ADXL343_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buffer, size, HAL_MAX_DELAY);
}
HAL_StatusTypeDef ADXL343_Init(void)
{
    uint8_t id = 0;

    // Vérifie la présence du périphérique
    if (HAL_I2C_IsDeviceReady(&hi2c1, ADXL343_ADDR, 3, 100) != HAL_OK)
    {
        return HAL_ERROR;  // Périphérique non détecté
    }

    // Lecture du Device ID (doit être 0xE5)
    if (ADXL343_ReadReg(ADXL343_REG_DEVID, &id) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (id != ADXL343_DEVID_VALUE)
    {
        return HAL_ERROR;  // Mauvais périphérique
    }


    // Mettre le capteur en veille avant config
    if (ADXL343_WriteReg(ADXL343_REG_POWERCTL, 0x00) != HAL_OK)
        return HAL_ERROR;

    // Configurer la plage ±2g, mode full resolution
    if (ADXL343_WriteReg(ADXL343_REG_DATA_FORMAT, 0x08) != HAL_OK)
        return HAL_ERROR;

    // Définir la fréquence d’échantillonnage à 100 Hz
    if (ADXL343_WriteReg(ADXL343_REG_BW_RATE, 0x0A) != HAL_OK)
        return HAL_ERROR;

    // Activer la mesure (bit Measure = 1)
    if (ADXL343_WriteReg(ADXL343_REG_POWERCTL, 0x08) != HAL_OK)
        return HAL_ERROR;
    if (ADXL343_EnableSingleTap() != HAL_OK)
    	return HAL_ERROR;
    uint8_t source;
    ADXL343_ReadReg(0x30, &source); // Lire INT_SOURCE
    return HAL_OK;
}

HAL_StatusTypeDef ADXL343_ReadXYZ(Measure *m1)
{
    uint8_t buffer[6];
    int16_t rawX, rawY, rawZ;
    uint8_t INT;

    if (ADXL343_ReadMulti(ADXL343_REG_DATAX0, buffer, 6) != HAL_OK)
        return HAL_ERROR;
    ADXL343_WriteReg(0x2F, 0b10111111);
    if (ADXL343_ReadReg(0x2F, &INT)!=HAL_OK)
    	return HAL_ERROR;



    rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
    rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
    rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);

    /* Convertir en g */
    m1->A_X = ALPHA * (rawX * ADXL343_MG2G_MULTIPLIER) + m1->A_X * (1-ALPHA);
    m1->A_Y = ALPHA * (rawY * ADXL343_MG2G_MULTIPLIER) + m1->A_Y * (1-ALPHA);
    m1->A_Z = ALPHA * (rawZ * ADXL343_MG2G_MULTIPLIER) + m1->A_Z * (1-ALPHA);

    return HAL_OK;
}

// Seuil et durée (valeurs typiques)
#define ADXL343_THRESH_TAP 0xFF  // 48 -> ~0.48 g
#define ADXL343_DUR        0xFF   // 16 -> ~10ms
#define ADXL343_LATENT     0x00   // 0 pour single tap
#define ADXL343_TAP_AXES   0b00001110   // X,Y,Z

HAL_StatusTypeDef ADXL343_EnableSingleTap(void)
{
    // Configurer le seuil
    if (ADXL343_WriteReg(0x1D, ADXL343_THRESH_TAP) != HAL_OK) return HAL_ERROR;
    if (ADXL343_WriteReg(0x21, ADXL343_DUR) != HAL_OK) return HAL_ERROR;
    if (ADXL343_WriteReg(0x22, ADXL343_LATENT) != HAL_OK) return HAL_ERROR;

    // Activer les axes
    if (ADXL343_WriteReg(0x2A, ADXL343_TAP_AXES) != HAL_OK) return HAL_ERROR;

    // Activer l’interruption single tap
    if (ADXL343_WriteReg(0x2E, 0x40) != HAL_OK) return HAL_ERROR; // bit 6 = single tap

    // Mapper l’interruption sur INT1 ou INT2 si nécessaire
    if (ADXL343_WriteReg(0x2F, 0x00) != HAL_OK) return HAL_ERROR; // INT1

    return HAL_OK;
}

void I2C_ResetBus(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1️⃣ Désactive le périphérique I2C
    __HAL_I2C_DISABLE(hi2c);
    HAL_I2C_DeInit(hi2c);

    // 2️⃣ Configure PB8 (SCL) et PB9 (SDA) en GPIO Output Open-Drain
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3️⃣ Génère quelques cycles sur SCL pour libérer SDA
    for (int i = 0; i < 10; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // SCL haut
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL bas
        HAL_Delay(1);
    }

    // 4️⃣ Relâche SDA et SCL
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

    // 5️⃣ Reconfigure PB8/PB9 en mode I2C AF
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // AF pour I2C1 sur PB8/PB9
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 6️⃣ Réinitialise et réactive I2C1
    HAL_I2C_Init(hi2c);
    __HAL_I2C_ENABLE(hi2c);
}
