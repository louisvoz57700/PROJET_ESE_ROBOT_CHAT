/* moteur.h */
#ifndef INC_MOTEUR_H_
#define INC_MOTEUR_H_

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

/* --- DIMENSIONS ROBOT (MISES À JOUR) --- */
#define WHEEL_DIAMETER_MM    49.0f    // Diamètre roue
#define WHEEL_BASE_MM        162.0f   // Entraxe (Distance entre les roues)

/* --- CONFIG ENCODEUR --- */
#define ENCODER_PPR          224.4f
#define TICKS_PER_REV        (ENCODER_PPR * 4.0f) // ~897.6 ticks/tour
#define MM_PER_TICK          ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REV)

/* --- CONFIG MOTEUR --- */
#define PWM_ARR_MAX          1000
#define PWM_MIN              200
#define PWM_MAX              999
#define MOTOR_TIMEOUT_MS     10000
#define STALL_CHECK_MS       500

/* --- DEBUG --- */
typedef struct {
    uint32_t cnt_left_raw;
    uint32_t cnt_right_raw;
    int32_t  total_left;
    int32_t  total_right;
    float    x_mm;
    float    y_mm;
    float    theta_deg;
} EncoderDebug_t;

extern volatile EncoderDebug_t g_DebugInfo;

/* --- PROTOTYPES --- */
void Motor_Init_System(void);
void Encoder_Update_Debug(void);
void Motor_Stop_Both(void);

// --- NOUVELLES FONCTIONS DE COMMANDE ---
// Avance ou recule d'une distance en mm
bool Robot_Translation(int speed, float distance_mm);

// Tourne sur place d'un angle en degrés (+ = Gauche, - = Droite)
bool Robot_Rotation(int speed, float angle_deg);

#endif /* INC_MOTEUR_H_ */
