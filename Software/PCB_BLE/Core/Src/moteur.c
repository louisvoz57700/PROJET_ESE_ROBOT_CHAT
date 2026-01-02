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

#define ALPHA 0.1
// Définition et initialisation des états internes à zéro
PID_t pid_vitesse_G = {0};
PID_t pid_vitesse_D = {0};

float delta;

extern System_state state;

PID_t pidSpeed = {
    .Kp = 350.0f,
    .Ki = 200.0f,
    .Kd = 0.0f,
    .integrator = 0.0f,
    .prevError = 0.0f,
    .outMin = 0.0f,  // PWM min
    .outMax = 990.0f    // PWM max
};

float error ;

uint16_t duty;

float velocity;

int32_t last_cnt = 0;
float angle1 = 0;

float Encoder_UpdateAngle(TIM_HandleTypeDef* htim)
{
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(htim);
    int32_t diff = cnt - last_cnt;

    // gérer le dépassement (overflow)
    if(diff > 32768)       diff -= 65536;
    else if(diff < -32768) diff += 65536;

    last_cnt = cnt;

    angle1 += ((float)diff * 360.0f) / (ENCODER_PPR * 4);
    return angle1;
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

void Motor_Forward_R(System_state* etat,uint16_t duty)
{
	// CH1N actif, CH1 inactif
	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);


	Motor_SetSpeed(&htim16,duty);

}

void Motor_Forward_L(System_state* etat ,uint16_t duty)
{
	// CH1N actif, CH1 inactif
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);

	Motor_SetSpeed(&htim17,duty);
}

void Motor_Reverse_R(System_state* etat,uint16_t duty)
{


    // CH1 actif, CH1N inactif
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1);


	Motor_SetSpeed(&htim16,duty);
}

void Motor_Reverse_L(System_state* etat,uint16_t duty)
{


    // CH1 actif, CH1N inactif
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);

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
	float angle_0 = Encoder_UpdateAngle(&htim1);
	float angle_1 = angle_0 ;
	while(angle_1-angle_0 < angle)
	{
		Motor_Forward(speed-1.5*(angle_1-angle_0));
		angle_1 = Encoder_UpdateAngle(&htim1);
		//angle = angle_1-angle_0;
	}
	Motor_Reverse(100);
}



float Encoder_GetAngularVelocityRad(TIM_HandleTypeDef* htim, int temps)
{
    static float previousAngle = 0.0f;

    float dt = temps * (1 / 64000000.0) ;

    float currentAngle = Encoder_UpdateAngle(htim);
    delta = -(currentAngle - previousAngle);

    // Gestion du passage 359° -> 0° ou 0° -> 359°
    if (delta > 180.0f)  delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    // Convertit degré → radian
    float velocityRad = fabs((delta * 3.1415f / 180.0f) * (1 / dt));

    previousAngle = currentAngle;
    return velocityRad;    // rad/s
}

void Target_Velocity(int temps)
{

}

float PID_Compute(PID_t *pid, float target, float measurement, float dt)
{

    error = (float)( target - measurement);

    if (fabs(error)<0.1)
    {
    	error = 0;
    }
    // Intégrale
    pid->integrator += pid->Ki * error * dt;

    // Anti-windup
    if(pid->integrator > pid->outMax) pid->integrator = pid->outMax;
    if(pid->integrator < pid->outMin) pid->integrator = pid->outMin;

    // Dérivée
    float derivative = (error - pid->prevError) / dt;
    pid->prevError = error;

    // PID
    float output = pid->Kp * error + pid->integrator + pid->Kd * derivative;

    // Saturation
    if(output > pid->outMax) output = pid->outMax;
    if(output < pid->outMin) output = pid->outMin;

    return output;
}

void Control_Speed(float targetRadPerSec,int temps)
{
    // Lecture vitesse moteur
    velocity = Encoder_GetAngularVelocityRad(&htim1, temps);  // rad/s

    // dt = temps * (1/64000000)
    float dt = temps * (1 / 64000000.0f);

    // PID
    float pidOut = PID_Compute(&pidSpeed, targetRadPerSec, velocity, dt);

    // Conversion PID vers PWM
    duty = (uint16_t)fabs(pidOut);

    if(duty > pidSpeed.outMax) duty = pidSpeed.outMax;

    // Commande moteur
    if(pidOut > 0) {
        Motor_Forward_R(&state,duty);
        if (duty >500 && velocity < 0.1)
        {
        	duty = duty - 200;
        	Motor_Forward_R(&state,duty);
        }
    } else if(pidOut < 0) {
        Motor_Reverse_R(&state,duty);
    } else {
        // Stop
        Motor_Forward_R(&state,000);
    }
}


