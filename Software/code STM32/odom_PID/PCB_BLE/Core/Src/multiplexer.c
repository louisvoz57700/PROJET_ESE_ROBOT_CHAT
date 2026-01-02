/*
 * multiplexer.c
 *
 *  Created on: Oct 29, 2025
 *      Author: louisvoz
 */

#include "main.h"
#include "multiplexer.h"

extern I2C_HandleTypeDef hi2c1;

#define TCA9548A_ADDR 0x70 << 1

/**
  * @brief  Initialise le TCA9548A (reset, vérification)
  */
HAL_StatusTypeDef TCA9548A_Init(void)
{
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    //HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    //HAL_Delay(10);

    // Test si le périphérique répond
    if (HAL_I2C_IsDeviceReady(&hi2c1, TCA9548A_ADDR, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Désactive tous les canaux par sécurité
    uint8_t disable = 0x00;
    return HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &disable, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Sélectionne un canal du TCA9548A (0 à 7)
  * @param  channel : numéro du canal à activer
  */
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel)
{
    if (channel > 7)
        return HAL_ERROR;

    uint8_t command = (1 << channel);
    return HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &command, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Désactive tous les canaux du TCA9548A
  */
HAL_StatusTypeDef TCA9548A_DisableAll(void)
{
    uint8_t command = 0x00;
    return HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &command, 1, HAL_MAX_DELAY);
}
