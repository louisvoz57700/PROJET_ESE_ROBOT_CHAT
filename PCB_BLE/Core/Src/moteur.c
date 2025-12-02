/*
 * moteur.c
 *
 *  Created on: Nov 7, 2025
 *      Author: louisvoz
 */

#include "main.h"
#include "moteur.h"
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17; // Timer configuré en mode Encodeur

extern TIM_HandleTypeDef htim1; // Timer configuré en mode Encodeur
extern TIM_HandleTypeDef htim2; // Timer configuré en mode Encodeur

int32_t encoder_position = 0;
int32_t last_count = 0;

#define WHEEL_DIAMETER_MM 60.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Définition et initialisation des états internes à zéro
PID_t pid_vitesse_G = {0};
PID_t pid_vitesse_D = {0};


float Encoder_GetAngleDeg(TIM_HandleTypeDef* htim)
{
    return (( (float)__HAL_TIM_GET_COUNTER(htim) * 360.0f ) / (ENCODER_PPR * 4));

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
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL)!= HAL_OK)
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

void Motor_SetSpeed(TIM_HandleTypeDef* htim,uint16_t duty)
{
    // duty entre 0 et htim1.Init.Period
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty);

}

void Motor_Forward(uint16_t duty)
{
	// CH1N actif, CH1 inactif
	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);

	Motor_SetSpeed(&htim16,duty);
	Motor_SetSpeed(&htim17,duty);
}

void Motor_Reverse(uint16_t duty)
{


    // CH1 actif, CH1N inactif
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);

	Motor_SetSpeed(&htim16,duty);
	Motor_SetSpeed(&htim17,duty);
}

void Motor_Stop(System_state* state)
{
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);
}

void Target_angle (int speed,int angle)
{
	float angle_0 = Encoder_GetAngleDeg(&htim1);
	float angle_1 = angle_0 ;
	while(angle_1-angle_0 < angle)
	{
		Motor_Forward(speed-1.5*(angle_1-angle_0));
		angle_1 = Encoder_GetAngleDeg(&htim1);
		//angle = angle_1-angle_0;
	}
	Motor_Reverse(100);
}

float PID_Update(PID_t* pid, float setpoint, float measurement, float Ts)
{
    // erreur
    float error = setpoint - measurement;

    // intégrale + anti-windup
    pid->integral += error * Ts;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    // dérivée (simple)
    float derivative = (error - pid->last_error) / Ts;
    pid->last_error = error;

    // sortie PID
    float output = pid->Kp*error + pid->Ki*pid->integral + pid->Kd*derivative;

    // saturation de la sortie
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}

void PID_init_params() {
    // 1. Initialisation des paramètres de G
    pid_vitesse_G.Kp = 0.3f;
    pid_vitesse_G.Ki = 20.0f;
    pid_vitesse_G.Kd = 0.0f;
    pid_vitesse_G.out_min = -1.0f;
    pid_vitesse_G.out_max =  1.0f;

    // 2. Copie des paramètres de G vers D (la copie bit à bit fonctionne ici)
    pid_vitesse_D = pid_vitesse_G;

    // Assurez-vous que les états internes sont bien à zéro après la copie
    // Si l'initialisation {0} dans la déclaration n'a pas suffi :
    pid_vitesse_D.integral = 0.0f;
    pid_vitesse_D.last_error = 0.0f;
}

void Update_Encoder_Vitesse(Position* pos,uint32_t last_count_d,uint32_t last_count_g)
{
	const float puls_per_rev = ENCODER_PPR * 4.0f;   // résolution quadrature
	const float Ts_inv = 1000.0f;                   // fréquence ISR : 1 kHz

	/* ---- MOTEUR DROIT ---- */
	uint32_t pos_now_d = __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t diff_d = pos_now_d - last_count_d;

	// gestion overflow 16 bits
	if (diff_d >  32768) diff_d -= 65536;
	if (diff_d < -32768) diff_d += 65536;

	last_count_d = pos_now_d;

	float rev_per_s_d = (float)diff_d * Ts_inv / puls_per_rev;
	pos->Vitesse_D = rev_per_s_d * 2.0f * (float)M_PI;

	/* ---- MOTEUR GAUCHE ---- */
	int32_t pos_now_g = __HAL_TIM_GET_COUNTER(&htim2);
	int32_t diff_g = pos_now_g - last_count_g;

	// gestion overflow 16 bits
	if (diff_g >  32768) diff_g -= 65536;
	if (diff_g < -32768) diff_g += 65536;

	last_count_g = pos_now_g;

	float rev_per_s_g = (float)diff_g * Ts_inv / puls_per_rev;
	pos->Vitesse_G = rev_per_s_g * 2.0f * (float)M_PI;
}

void control_motor(float u_d)
{

}
