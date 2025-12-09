/* ===================================================================================
 * Fichier : moteur.c
 * Description : PID + FeedForward (Anti-Stall) + Filtre Léger
 * =================================================================================== */

#include "moteur.h"
#include <math.h>
#include <stdlib.h>
#include "cmsis_os.h"

/* ==================== HANDLES EXTERNES ==================== */
extern TIM_HandleTypeDef htim17; // PWM GAUCHE
extern TIM_HandleTypeDef htim2;  // Encodeur GAUCHE
extern TIM_HandleTypeDef htim16; // PWM DROIT
extern TIM_HandleTypeDef htim1;  // Encodeur DROIT

/* ==================== CONFIGURATION ==================== */
#define FILTER_SIZE 2
#define STATIC_OFFSET 100 // Démarrage
#define FF_RATIO 0.15f    // Moins violent à haute vitesse

/* ==================== STRUCTURE PID ==================== */
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral_sum;
	float prev_error;
	float out_max;
	float out_min;
} PID_Controller_t;

/* ==================== VARIABLES GLOBALES ==================== */
volatile EncoderDebug_t g_DebugInfo = {0};

/* PID DOUX (Le FeedForward fait le travail de force, le PID ajuste) */
PID_Controller_t pid_left  = {0.7f, 0.0f, 0.0f, 0.0f, 0.0f, PWM_ARR_MAX, -PWM_ARR_MAX};
PID_Controller_t pid_right = {0.7f, 0.0f, 0.0f, 0.0f, 0.0f, PWM_ARR_MAX, -PWM_ARR_MAX};

/* Variables internes Odométrie & Filtre */
static uint32_t last_cnt_L = 0;
static uint32_t last_cnt_R = 0;
static int32_t prev_total_L = 0;
static int32_t prev_total_R = 0;

static float speed_L_buffer[FILTER_SIZE] = {0};
static float speed_R_buffer[FILTER_SIZE] = {0};
static uint8_t filter_idx = 0;

/* ==================== FONCTIONS BAS NIVEAU ==================== */

void Motor_Init_System(void) {
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	__HAL_TIM_MOE_ENABLE(&htim16);
	__HAL_TIM_MOE_ENABLE(&htim17);

	Motor_Stop_Both();
}

static void Set_Motor_PWM(TIM_HandleTypeDef *htim, int pwm_value) {
	if (pwm_value > PWM_ARR_MAX) pwm_value = PWM_ARR_MAX;
	if (pwm_value < -PWM_ARR_MAX) pwm_value = -PWM_ARR_MAX;

	// Petite zone morte logicielle
	if (abs(pwm_value) < 50 && pwm_value != 0) pwm_value = 0;

	uint16_t duty = (uint16_t)abs(pwm_value);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty);

	if (pwm_value > 0) {
		HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	} else if (pwm_value < 0) {
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
	} else {
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
	}
}

void Motor_Left_Control(int speed) { Set_Motor_PWM(&htim17, -speed); }
void Motor_Right_Control(int speed) { Set_Motor_PWM(&htim16, -speed); }
void Motor_Stop_Both(void) { Motor_Left_Control(0); Motor_Right_Control(0); }

/* ==================== ODOMÉTRIE + FILTRE ==================== */

void Encoder_Update_Debug(void) {
	uint32_t curr_L = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t curr_R = __HAL_TIM_GET_COUNTER(&htim1);

	g_DebugInfo.cnt_left_raw = curr_L;
	g_DebugInfo.cnt_right_raw = curr_R;

	int32_t diff_L = (int32_t)(curr_L - last_cnt_L);
	int32_t diff_R = (int32_t)(curr_R - last_cnt_R);

	if (diff_L < -32768) diff_L += 65536; else if (diff_L > 32768) diff_L -= 65536;
	if (diff_R < -32768) diff_R += 65536; else if (diff_R > 32768) diff_R -= 65536;

	g_DebugInfo.total_left -= diff_L;
	g_DebugInfo.total_right -= diff_R;

	last_cnt_L = curr_L;
	last_cnt_R = curr_R;

    // Mise à jour positions X,Y (Code odométrie standard conservé)
    int32_t d_ticks_L = g_DebugInfo.total_left - prev_total_L;
    int32_t d_ticks_R = g_DebugInfo.total_right - prev_total_R;
    prev_total_L = g_DebugInfo.total_left;
    prev_total_R = g_DebugInfo.total_right;

    float d_left_mm = (float)d_ticks_L * MM_PER_TICK;
    float d_right_mm = (float)d_ticks_R * MM_PER_TICK;
    float d_center = (d_left_mm + d_right_mm) / 2.0f;
    float d_theta = (d_right_mm - d_left_mm) / WHEEL_BASE_MM;

    float current_theta_rad = g_DebugInfo.theta_deg * (3.14159265f / 180.0f);
    float avg_theta = current_theta_rad + (d_theta / 2.0f);

    g_DebugInfo.x_mm += d_center * cosf(avg_theta);
    g_DebugInfo.y_mm += d_center * sinf(avg_theta);
    g_DebugInfo.theta_deg += d_theta * (180.0f / 3.14159265f);
}

// Fonction de calcul de vitesse avec FILTRE
static void Encoder_Calculate_Speed(float dt) {
	int32_t pos_L_before = g_DebugInfo.total_left;
	int32_t pos_R_before = g_DebugInfo.total_right;

	Encoder_Update_Debug();

	int32_t dL = g_DebugInfo.total_left - pos_L_before;
	int32_t dR = g_DebugInfo.total_right - pos_R_before;

	if (dt > 0.0001f) {
		// 1. Vitesse instantanée
		float raw_L = (float)dL / dt;
		float raw_R = (float)dR / dt;

		// 2. Buffer circulaire
		speed_L_buffer[filter_idx] = raw_L;
		speed_R_buffer[filter_idx] = raw_R;

		// 3. Moyenne
		float sum_L = 0, sum_R = 0;
		for(int i=0; i<FILTER_SIZE; i++) {
			sum_L += speed_L_buffer[i];
			sum_R += speed_R_buffer[i];
		}

		g_DebugInfo.speed_L_meas = sum_L / (float)FILTER_SIZE;
		g_DebugInfo.speed_R_meas = sum_R / (float)FILTER_SIZE;

		filter_idx = (filter_idx + 1) % FILTER_SIZE;
	}
}

static float PID_Compute(PID_Controller_t *pid, float target, float measured, float dt) {
	float error = target - measured;
	float P = pid->Kp * error;

	pid->integral_sum += error * dt;
	// Saturation Intégrale pour éviter windup
	if (pid->integral_sum > 5000) pid->integral_sum = 5000;
	if (pid->integral_sum < -5000) pid->integral_sum = -5000;

	float I = pid->Ki * pid->integral_sum;
	float D = pid->Kd * (error - pid->prev_error) / dt;
	pid->prev_error = error;

	float output = P + I + D;

	// Clamp PID output (on le limite pour qu'il n'écrase pas le FeedForward)
	if (output > 400.0f) output = 400.0f;
	if (output < -400.0f) output = -400.0f;

	return output;
}

/* ==================== MOTEUR MOVE PID + FEEDFORWARD ==================== */

static bool Motor_Move_PID(float target_speed_ticks, float target_deg_L, float target_deg_R)
{
	// Reset PID & Filtres
	pid_left.integral_sum = 0; pid_left.prev_error = 0;
	pid_right.integral_sum = 0; pid_right.prev_error = 0;
	for(int i=0; i<FILTER_SIZE; i++) { speed_L_buffer[i]=0; speed_R_buffer[i]=0; }

	Encoder_Update_Debug();
	float start_ticks_L = (float)g_DebugInfo.total_left;
	float start_ticks_R = (float)g_DebugInfo.total_right;

	// Calcul distances cibles
	float dist_ticks_L = (target_deg_L / 360.0f) * TICKS_PER_REV;
	float dist_ticks_R = (target_deg_R / 360.0f) * TICKS_PER_REV;
	float abs_dist_L = fabsf(dist_ticks_L);
	float abs_dist_R = fabsf(dist_ticks_R);
	float max_dist = (abs_dist_L > abs_dist_R) ? abs_dist_L : abs_dist_R;

	int sign_L = (dist_ticks_L >= 0) ? 1 : -1;
	int sign_R = (dist_ticks_R >= 0) ? 1 : -1;

	if (max_dist < 5.0f) return true;

	uint32_t start_time = HAL_GetTick();
	uint32_t prev_loop_time = HAL_GetTick();

	while (1)
	{
		uint32_t now = HAL_GetTick();
		float dt = (float)(now - prev_loop_time) / 1000.0f;
		if (dt <= 0.0001f) { osDelay(1); continue; }
		prev_loop_time = now;

		Encoder_Calculate_Speed(dt);

		float current_dist_L = fabsf((float)g_DebugInfo.total_left - start_ticks_L);
		float current_dist_R = fabsf((float)g_DebugInfo.total_right - start_ticks_R);
		float progress = (current_dist_L + current_dist_R) / (abs_dist_L + abs_dist_R);

		if (progress >= 1.0f) {
			Motor_Stop_Both();
			// Reset Debug pour graphe propre
			g_DebugInfo.debug_target_speed = 0;
			g_DebugInfo.debug_pwm_output = 0;
			return true;
		}
		if (now - start_time > MOTOR_TIMEOUT_MS) {
			Motor_Stop_Both();
			return false;
		}

		// --- 1. PROFIL DE VITESSE (Trapèze) ---
		float speed_base = target_speed_ticks;
		if (progress < 0.2f) speed_base = target_speed_ticks * (progress / 0.2f);
		else if (progress > 0.8f) speed_base = target_speed_ticks * ((1.0f - progress) / 0.2f);

		if(speed_base < 300.0f) speed_base = 300.0f; // Vitesse min maintenue

		float setpoint_L = speed_base * (abs_dist_L / max_dist) * sign_L;
		float setpoint_R = speed_base * (abs_dist_R / max_dist) * sign_R;

		// --- 2. FEED FORWARD (LA CLÉ DU SUCCÈS) ---
		// On injecte direct une commande proportionnelle à la vitesse + un offset pour la friction
		float ff_L = setpoint_L * FF_RATIO;
		float ff_R = setpoint_R * FF_RATIO;

		// Ajout du BOOST statique (Offset) si on veut bouger
		if (setpoint_L > 0) ff_L += STATIC_OFFSET; else if (setpoint_L < 0) ff_L -= STATIC_OFFSET;
		if (setpoint_R > 0) ff_R += STATIC_OFFSET; else if (setpoint_R < 0) ff_R -= STATIC_OFFSET;

		// --- 3. PID (Correction fine) ---
		float pid_L = PID_Compute(&pid_left, setpoint_L, g_DebugInfo.speed_L_meas, dt);
		float pid_R = PID_Compute(&pid_right, setpoint_R, g_DebugInfo.speed_R_meas, dt);

		// --- 4. TOTAL ---
		float total_L = ff_L + pid_L;
		float total_R = ff_R + pid_R;

		// Debug
		g_DebugInfo.debug_target_speed = setpoint_L;
		g_DebugInfo.debug_pwm_output = total_L;

		Motor_Left_Control((int)total_L);
		Motor_Right_Control((int)total_R);

		osDelay(10);
	}
}

bool Robot_Translation(int speed, float distance_mm) {
	float wheel_deg = (distance_mm / (WHEEL_DIAMETER_MM * 3.14159265f)) * 360.0f;
	return Motor_Move_PID((float)speed, wheel_deg, wheel_deg);
}

bool Robot_Rotation(int speed, float angle_deg) {
	float ratio_robot = WHEEL_BASE_MM / WHEEL_DIAMETER_MM;
	float wheel_deg = angle_deg * ratio_robot;
	return Motor_Move_PID((float)speed, -wheel_deg, wheel_deg);
}
