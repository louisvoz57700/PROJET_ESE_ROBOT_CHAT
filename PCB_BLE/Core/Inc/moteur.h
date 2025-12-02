/*
 * moteur.h
 *
 *  Created on: Nov 7, 2025
 *      Author: louisvoz
 */

#ifndef INC_MOTEUR_H_
#define INC_MOTEUR_H_

// Fonctions Encodeur
void Encoder_Init(void);
int  Encoder_GetPosition(void);
float Encoder_GetAngleDeg(TIM_HandleTypeDef* htim);
float Encoder_GetDistanceMM(void);

// Fonctions moteurs
void Motor_Init(void);
void Motor_SetSpeed(TIM_HandleTypeDef* htim,uint16_t duty);
void Motor_Forward(uint16_t duty);
void Motor_Reverse(uint16_t duty);
void Motor_Stop(System_state* state);

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float last_error;
    float out_min;
    float out_max;
} PID_t;

extern PID_t pid_vitesse_G;
extern PID_t pid_vitesse_D;


#define ENCODER_PPR 224.4  // impulsions par tour


// Fonction de rotation sur place
void Target_angle(int speed, int angle);
#endif /* INC_MOTEUR_H_ */
