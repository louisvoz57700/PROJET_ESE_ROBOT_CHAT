/*
 * moteur.h
 *
 *  Created on: Nov 7, 2025
 *      Author: louisvoz
 */

#ifndef INC_MOTEUR_H_
#define INC_MOTEUR_H_

void Motor_Init(void);
void Motor_SetSpeed(uint16_t duty);
void Motor_Forward(uint16_t duty);
void Motor_Reverse(uint16_t duty);
void Motor_Stop(void);
float Encoder_GetAngleDeg(void);
float Encoder_GetDistanceMM(void);
void Encoder_Init(void);
int Encoder_GetPosition(void);

#endif /* INC_MOTEUR_H_ */
