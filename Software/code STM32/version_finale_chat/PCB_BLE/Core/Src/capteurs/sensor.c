/*
 * sensor.c
 *
 * Created on: Oct 28, 2025
 * Author: louisvoz
 */

#include "main.h"
#include "capteurs/sensor.h"
#include "cmsis_os.h"

/* Handles externes */
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

extern uint8_t int_source;

/* Adresses et Registres ADXL343 */
#define ADXL343_ADDR            (0x53 << 1) // 0xA6
#define ADXL343_DEVID_VALUE     0xE5
#define ADXL343_REG_DEVID       0x00
#define ADXL343_REG_BW_RATE     0x2C
#define ADXL343_REG_POWERCTL    0x2D
#define ADXL343_REG_DATA_FORMAT 0x31
#define ADXL343_REG_DATAX0      0x32

#define ADXL343_REG_INT_SOURCE    0x30

/* Paramètres Tap */
#define ADXL343_THRESH_TAP      0xFF // ~0.48 g
#define ADXL343_DUR             0xFF // ~10ms
#define ADXL343_LATENT          0x00
#define ADXL343_TAP_AXES        0b00001110

/* Adresse TCA9548A (Multiplexeur I2C) */
#define TCA9548A_ADDR           (0x70 << 1)

/* Constantes calcul */
#define ALPHA 0.05f
#define ADXL343_MG2G_MULTIPLIER 0.0039f

/* Prototype interne (pour éviter warning implicit declaration) */
HAL_StatusTypeDef ADXL343_EnableSingleTap(void);


/* ========================================================================== */
/* ADC                                     */
/* ========================================================================== */

void INIT_ADC(uint16_t *adc_buffer, uint16_t size)
{
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}
	// Correction du warning de pointeur ici : (uint32_t*)
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, size);
}

// Fonction purement mathématique (plus d'appel I2C ici)
void UPDATE_V_I(Measure *m1, uint16_t *adc_buffer)
{
	float raw_courant = (float)adc_buffer[0];
	float raw_tension = (float)adc_buffer[1];

	const float k_tension = (3.3f / 4095.0f) * 3.0f;
	const float k_courant = (3.3f / 4095.0f) * 2.0f;

	m1->tension = ALPHA * (raw_tension * k_tension) + (1.0f - ALPHA) * m1->tension;
	m1->courant = ALPHA * (raw_courant * k_courant) + (1.0f - ALPHA) * m1->courant;
}

/* ========================================================================== */
/* ADXL343                                   */
/* ========================================================================== */

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

HAL_StatusTypeDef ADXL343_ReadMulti(uint8_t reg, uint8_t *buffer, uint16_t size)
{
	return HAL_I2C_Mem_Read(&hi2c1, ADXL343_ADDR, reg,
			I2C_MEMADD_SIZE_8BIT, buffer, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL343_EnableSingleTap(void)
{
	if (ADXL343_WriteReg(0x1D, ADXL343_THRESH_TAP) != HAL_OK) return HAL_ERROR;
	if (ADXL343_WriteReg(0x21, ADXL343_DUR) != HAL_OK) return HAL_ERROR;
	if (ADXL343_WriteReg(0x22, ADXL343_LATENT) != HAL_OK) return HAL_ERROR;
	if (ADXL343_WriteReg(0x2A, ADXL343_TAP_AXES) != HAL_OK) return HAL_ERROR;
	if (ADXL343_WriteReg(0x2E, 0x40) != HAL_OK) return HAL_ERROR; // Enable Single Tap INT
	if (ADXL343_WriteReg(0x2F, 0x00) != HAL_OK) return HAL_ERROR; // Map to INT1

	return HAL_OK;
}

HAL_StatusTypeDef ADXL343_Init(void)
{
	uint8_t id = 0;

	if (HAL_I2C_IsDeviceReady(&hi2c1, ADXL343_ADDR, 3, 100) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (ADXL343_ReadReg(ADXL343_REG_DEVID, &id) != HAL_OK) return HAL_ERROR;
	if (id != ADXL343_DEVID_VALUE) return HAL_ERROR;

	if (ADXL343_WriteReg(ADXL343_REG_POWERCTL, 0x00) != HAL_OK) return HAL_ERROR;
	if (ADXL343_WriteReg(ADXL343_REG_DATA_FORMAT, 0x08) != HAL_OK) return HAL_ERROR;
	if (ADXL343_WriteReg(ADXL343_REG_BW_RATE, 0x0A) != HAL_OK) return HAL_ERROR;
	if (ADXL343_WriteReg(ADXL343_REG_POWERCTL, 0x08) != HAL_OK) return HAL_ERROR;

	if (ADXL343_EnableSingleTap() != HAL_OK) return HAL_ERROR;

	// Lecture pour effacer les flags initiaux
	uint8_t source;
	ADXL343_ReadReg(0x30, &source);

	return HAL_OK;
}

HAL_StatusTypeDef ADXL343_ReadXYZ(Measure *m1)
{
	uint8_t buffer[6];
	int16_t rawX, rawY, rawZ;
	uint8_t INT;

	if (ADXL343_ReadMulti(ADXL343_REG_DATAX0, buffer, 6) != HAL_OK)
		return HAL_ERROR;

	// Note: Ecriture/Lecture du registre INT_MAP (0x2F) ici - logique spécifique conservée
	ADXL343_WriteReg(0x2F, 0b10111111);
	if (ADXL343_ReadReg(0x2F, &INT) != HAL_OK)
		return HAL_ERROR;

	rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
	rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
	rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);

	m1->A_X = ALPHA * (rawX * ADXL343_MG2G_MULTIPLIER) + m1->A_X * (1 - ALPHA);
	m1->A_Y = ALPHA * (rawY * ADXL343_MG2G_MULTIPLIER) + m1->A_Y * (1 - ALPHA);
	m1->A_Z = ALPHA * (rawZ * ADXL343_MG2G_MULTIPLIER) + m1->A_Z * (1 - ALPHA);

	return HAL_OK;
}

/* ========================================================================== */
/* ToF                                 */
/* ========================================================================== */
//void ToF_Init(ToF_t *tof){
//	tof->left= -1;
//	tof->right = -1;
//	tof->front = -1;
//	tof->back = -1;
//	tof->vl53l0x_initialized = false;

//	I2C_ResetBus(&(tof->i2c));

//	if (initVL53L0X(1, &(tof->i2c)))
//	{
//		setSignalRateLimit(0.25f);
//		setVcselPulsePeriod(VcselPeriodPreRange, 10);
//		setVcselPulsePeriod(VcselPeriodFinalRange, 14);
//		setMeasurementTimingBudget(300000UL);
//		tof->vl53l0x_initialized = true;
//	}

//}

/* ========================================================================== */
/* I2C UTILS                                   */
/* ========================================================================== */

void I2C_ResetBus(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_I2C_DISABLE(hi2c);
	HAL_I2C_DeInit(hi2c);

	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	for (int i = 0; i < 10; i++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		osDelay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		osDelay(1);
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_I2C_Init(hi2c);
	__HAL_I2C_ENABLE(hi2c);
}

void check_single_tap(uint8_t * ret){
	ADXL343_ReadReg(ADXL343_REG_INT_SOURCE, ret);
}
