#ifndef INC_MOTEUR_H_
#define INC_MOTEUR_H_

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

/* --- CONFIG ROBOT --- */
#define WHEEL_DIAMETER_MM    49.0f
#define WHEEL_BASE_MM        162.0f
#define ENCODER_PPR          224.4f
#define TICKS_PER_REV        (ENCODER_PPR * 4.0f) // ~897.6
#define MM_PER_TICK          ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REV)

/* --- CONFIG MOTEUR --- */
#define PWM_ARR_MAX          1000
#define PWM_MIN              150  // Zone morte
#define PWM_MAX_SAFE         900  // Limite sécurité
#define MOTOR_TIMEOUT_MS     10000 // Timeout 10s

/* --- STRUCTURE DEBUG --- */
typedef struct {
	// Compteurs bruts encodeurs
	uint32_t cnt_left_raw;
	uint32_t cnt_right_raw;

	// Totaux accumulés (en ticks)
	int32_t  total_left;
	int32_t  total_right;

	// Position odométrie
	float    x_mm;
	float    y_mm;
	float    theta_deg;

	// Vitesses mesurées (ticks/s) - MIS À JOUR EN TEMPS RÉEL
	float    speed_L_meas;
	float    speed_R_meas;

	// === AJOUTS POUR STM32CUBEMONITOR ===
	float debug_target_speed; // Consigne envoyée au PID (L)
	float debug_pwm_output;   // Sortie du PID (L)
} EncoderDebug_t;

extern volatile EncoderDebug_t g_DebugInfo;

/* --- PROTOTYPES PRINCIPAUX --- */
void Motor_Init_System(void);
void Motor_Stop_Both(void);
EncoderDebug_t * Encoder_Update_Debug(void);

// Fonctions de déplacement avec PID vitesse
bool Robot_Translation(int speed, float distance_mm);
bool Robot_Rotation(int speed, float angle_deg);

// Fonctions utiles debug
float Get_Total_Distance_mm(void);
void Reset_Odometry(void);

#endif /* INC_MOTEUR_H_ */
