/*
 * moteur.c
 *
 *  Created on: Nov 7, 2025
 *      Author: louisvoz
 */

#include "main.h"

extern TIM_HandleTypeDef htim16;

extern TIM_HandleTypeDef htim1; // Timer configuré en mode Encodeur


int32_t encoder_position = 0;
int32_t last_count = 0;

#define ENCODER_PPR 224.4  // impulsions par tour
#define WHEEL_DIAMETER_MM 60.0f

float Encoder_GetAngleDeg(void)
{
    return (( (float)__HAL_TIM_GET_COUNTER(&htim1) * 360.0f ) / (ENCODER_PPR * 4));

}

float Encoder_GetDistanceMM(void)
{
    return (Encoder_GetPosition() * (3.14159f * WHEEL_DIAMETER_MM)) / ENCODER_PPR;
}

void Encoder_Init(void)
{
    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL)!= HAL_OK)
    	{
    	Error_Handler();
    	}
}

int Encoder_GetPosition(void)
{
    int32_t new_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
    int32_t diff = new_count - last_count;

    // Gestion du débordement (overflow)
    if (diff > 32767)       diff -= 65536;
    else if (diff < -32768) diff += 65536;

    encoder_position += diff;
    last_count = new_count;

    return encoder_position;
}

void Motor_Init(void)
{
    // Démarrer les sorties PWM complémentaires
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
}

void Motor_SetSpeed(uint16_t duty)
{
    // duty entre 0 et htim1.Init.Period
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, duty);
}

void Motor_Forward(uint16_t duty)
{
    // CH1 actif, CH1N inactif
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1);
    Motor_SetSpeed(duty);
}

void Motor_Reverse(uint16_t duty)
{
    // CH1N actif, CH1 inactif
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
    Motor_SetSpeed(duty);
}

void Motor_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1);
}

void Target_angle (int speed,int angle)
{
	float angle_0 = Encoder_GetAngleDeg();
	float angle_1 = angle_0 ;
	while(angle_1-angle_0 < angle)
	{
		Motor_Forward(speed-1.5*(angle_1-angle_0));
		angle_1 = Encoder_GetAngleDeg();
		//angle = angle_1-angle_0;
	}
	Motor_Reverse(100);
}
