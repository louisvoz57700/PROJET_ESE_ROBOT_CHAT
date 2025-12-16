/*
 * task_FSM.c
 *
 *  Created on: Dec 3, 2025
 *      Author: knn64
 */

#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include <stdbool.h>
#include <math.h>

#include "tasks/task_FSM.h"
#include "tasks/task_sensor.h"
#include "tasks/task_control.h"

typedef enum
{
	STATE_IDLE,
	STATE_SEARCH,
	STATE_CHASE,
	STATE_EVADE,
	STATE_TAG,
	STATE_HIT
} RobotState;

static RobotState currentState = STATE_IDLE;
static RobotState previousState = STATE_IDLE;

static float r, theta;	// coordonnées polaires cible
static float lidar_min; // distance minimale LiDAR

extern osMessageQueueId_t QLidar;

extern OdomData_t Robot_pos;
osMessageQueueId_t QFSM;

static void chase_target(float r, float theta)
{
}

/* Getter pour que d'autres tâches puissent consulter l'état d'évitement */
void FSM_GetEvadeInfo(float *heading, int *checkpoint, _Bool *ready)
{
	if (heading)
		*heading = last_evade_heading;
	if (checkpoint)
		*checkpoint = last_chosen_checkpoint;
	if (ready)
		*ready = evade_ready;
}

static Zone checkpoints[4] = {
	{0 + SECURITY_DISTANCE, CHECK_SIZE + SECURITY_DISTANCE, 0 + SECURITY_DISTANCE, CHECK_SIZE + SECURITY_DISTANCE},													 // Bas-gauche
	{TABLE_WIDTH - CHECK_SIZE - SECURITY_DISTANCE, TABLE_WIDTH - SECURITY_DISTANCE, 0 + SECURITY_DISTANCE, CHECK_SIZE + SECURITY_DISTANCE},							 // Bas-droite
	{0 + SECURITY_DISTANCE, CHECK_SIZE + SECURITY_DISTANCE, TABLE_HEIGHT - CHECK_SIZE - SECURITY_DISTANCE, TABLE_HEIGHT - SECURITY_DISTANCE},						 // Haut-gauche
	{TABLE_WIDTH - CHECK_SIZE - SECURITY_DISTANCE, TABLE_WIDTH - SECURITY_DISTANCE, TABLE_HEIGHT - CHECK_SIZE - SECURITY_DISTANCE, TABLE_HEIGHT - SECURITY_DISTANCE} // Haut-droite
};

/* Centres des checkpoints : { x, y } */
static float checkpoint_centers[4][2] = {
	{SECURITY_DISTANCE + (CHECK_SIZE) / 2.0f, SECURITY_DISTANCE + (CHECK_SIZE) / 2.0f},								/* Bas-gauche */
	{TABLE_WIDTH - SECURITY_DISTANCE - (CHECK_SIZE) / 2.0f, SECURITY_DISTANCE + (CHECK_SIZE) / 2.0f},				/* Bas-droite */
	{SECURITY_DISTANCE + (CHECK_SIZE) / 2.0f, TABLE_HEIGHT - SECURITY_DISTANCE - (CHECK_SIZE) / 2.0f},				/* Haut-gauche */
	{TABLE_WIDTH - SECURITY_DISTANCE - (CHECK_SIZE) / 2.0f, TABLE_HEIGHT - SECURITY_DISTANCE - (CHECK_SIZE) / 2.0f} /* Haut-droite */
};

/* Evade state exported variables (internal storage) */
static volatile float last_evade_heading = 0.0f; /* radians in robot frame */
static volatile int last_chosen_checkpoint = -1; /* -1 = none, 0..3 valid */
static volatile _Bool evade_ready = 0;

static void evade(void)
{
	/*
	 * Prédiction de la trajectoire du robot à éviter.
	 * - r, theta : coordonnées polaires du cible dans le repère du robot (r en m, theta en degrés)
	 * - On garde un petit historique de positions (repère robot) pour estimer la vitesse
	 * - On prédit la position future et calcule une direction d'évitement (heading en rad)
	 *
	 * Remarques / hypothèses :
	 * - theta est en radians (si ce n'est pas le cas, il faut convertir)
	 * - la fréquence d'appel de TaskFSM est ~200 Hz (délai de 5 ms) — on utilise cela
	 *   implicitement pour l'échelle temporelle entre échantillons.
	 */

	enum
	{
		EV_HIST = 30
	}; /* nombre d'échantillons dans l'historique */
	static float hist_x[EV_HIST];
	static float hist_y[EV_HIST];
	static int hist_idx = 0;
	static int hist_count = 0;
	const float sample_dt = 0.005f; /* 5 ms entre itérations (approx) */
	const int predict_steps = 40;	/* nombre d'échantillons en avant pour prédiction */

	/* Convertir cible polaire (repère robot) -> cartésien (m)
		theta est fourni en degrés : convertir en radians pour cosf/sinf */
	float theta_rad = theta * (M_PI / 180.0f);
	float tx = r * cosf(theta_rad);
	float ty = r * sinf(theta_rad);

	/* Pousser dans l'historique */
	hist_x[hist_idx] = tx;
	hist_y[hist_idx] = ty;
	hist_idx = (hist_idx + 1) % EV_HIST;
	if (hist_count < EV_HIST)
		hist_count++;

	/* Si on n'a pas assez d'échantillons, on ne prédit pas ; on ne bouge pas */
	float pred_x = tx;
	float pred_y = ty;

	if (hist_count < 20)
	{
		/* pas encore de prédiction fiable */
		evade_ready = 0;
		last_chosen_checkpoint = -1;
		return;
	}

	if (hist_count >= 20)
	{
		/* oldest index (approx) */
		int oldest = (hist_idx) % EV_HIST; /* hist_idx pointe vers la prochaine écriture */
		float ox = hist_x[oldest];
		float oy = hist_y[oldest];

		/* Estimation vitesse moyenne sur la fenêtre (m/s) */
		float dt = sample_dt * (float)(hist_count - 1);
		if (dt <= 0.0f)
			dt = sample_dt;
		float vx = (tx - ox) / dt;
		float vy = (ty - oy) / dt;

		/* prédiction sur predict_steps échantillons */
		pred_x = tx + vx * (predict_steps * sample_dt);
		pred_y = ty + vy * (predict_steps * sample_dt);
	}

	/* Convertir position prédite du repère robot -> monde (x,y en mêmes unités que Robot_pos)
	   Robot_pos.robot_heading est en degrés dans `OdomData_t` : convertir en rad */
	float phi = Robot_pos.robot_heading * (M_PI / 180.0f);
	float pred_world_x = Robot_pos.robot_x + pred_x * cosf(phi) - pred_y * sinf(phi);
	float pred_world_y = Robot_pos.robot_y + pred_x * sinf(phi) + pred_y * cosf(phi);

	/* Choisir le checkpoint le plus éloigné de la position prédite (pour s'enfuir)
	   checkpoint_centers[][] est en coordonnées de table (monde) */
	int best_idx = -1;
	float best_dist2 = -1.0f;
	for (int i = 0; i < 4; ++i)
	{
		float dx = checkpoint_centers[i][0] - pred_world_x;
		float dy = checkpoint_centers[i][1] - pred_world_y;
		float d2 = dx * dx + dy * dy;
		if (d2 > best_dist2)
		{
			best_dist2 = d2;
			best_idx = i;
		}
	}

	if (best_idx < 0)
	{
		evade_ready = 0;
		last_chosen_checkpoint = -1;
		return;
	}

	/* Calculer heading d'évitement : cap vers le checkpoint choisi, exprimé en repère robot */
	float cp_x = checkpoint_centers[best_idx][0];
	float cp_y = checkpoint_centers[best_idx][1];
	float dxw = cp_x - Robot_pos.robot_x;
	float dyw = cp_y - Robot_pos.robot_y;
	/* monde -> robot frame: x_r = cos(phi)*dx + sin(phi)*dy ; y_r = -sin(phi)*dx + cos(phi)*dy */
	float dxr = cosf(phi) * dxw + sinf(phi) * dyw;
	float dyr = -sinf(phi) * dxw + cosf(phi) * dyw;
	float heading_to_cp = atan2f(dyr, dxr);

	last_evade_heading = heading_to_cp;
	last_chosen_checkpoint = best_idx;
	evade_ready = 1;

	/* TODO: ici appeler l'API de contrôle pour appliquer `last_evade_heading`.
	   Exemple (à implémenter dans task_control) :
		 Control_SetHeading(evade_heading);
	   Ou remplir une structure/message et la poster sur une queue vers TaskControl. */
}

static void signal_tag()
{
}

static void signal_hit()
{
}

static void SWITCH_TO_EVADE() {}

static void obstacle_avoid(uint32_t side)
{
	static int current_side = 0;
	if (side != 0xF0)
		current_side = side;
	switch (side)
	{
	case (FSM_NOTIF_ToF_LEFT):
		break;
	case (FSM_NOTIF_ToF_RIGHT):
		break;
	case (FSM_NOTIF_ToF_FRONT):
		break;
	case (FSM_NOTIF_ToF_REAR):
		break;
	default:
		break;
	}
}

/* * Tâche 3 : Prise de décision
 * Priorité : Haute
 * - Envoie les coordonées successives à suivre à la partie controle en tenant compte du temps entre chaque envoi (10 ms)
 */
void TaskFSM(void *argument)
{
	UNUSED(argument);

	LidarFrame frame;
	uint32_t notif = 0;
	bool targetDetected, obstacleDetected = false;
	bool danger_detected = false;

	for (;;)
	{
		// --- A. RÉFLEXE ANTI-CHUTE (Priorité absolue) ---
		// On vérifie d'abord si on est en danger. Si oui, on agit et on ignore le combat.

		// Cas 1 : Vide DEVANT
		if (front > CLIFF_THRESHOLD)
		{
			Motor_Stop_Both();					 // 1. On pile
			Robot_Translation(1200.0f, -120.0f); // 2. On recule de 15cm vite
			Robot_Rotation(1000.0f, 90.0f);		 // 3. Demi-tour (presque complet)
			danger_detected = true;
		}
		// Cas 2 : Vide DERRIÈRE
		else if (back > CLIFF_THRESHOLD)
		{
			Motor_Stop_Both();
			Robot_Translation(1500.0f, 100.0f); // On avance de 15cm pour s'éloigner du bord
			danger_detected = true;
		}
		// Cas 3 : Vide à GAUCHE
		else if (left > CLIFF_THRESHOLD)
		{
			Motor_Stop_Both();
			Robot_Rotation(1000.0f, -90.0f);	// On tourne à DROITE (angle négatif) pour s'éloigner
			Robot_Translation(1000.0f, 100.0f); // Et on avance un peu
			danger_detected = true;
		}
		// Cas 4 : Vide à DROITE
		else if (right > CLIFF_THRESHOLD)
		{
			Motor_Stop_Both();
			Robot_Rotation(1000.0f, 90.0f); // On tourne à GAUCHE (angle positif)
			Robot_Translation(1000.0f, 100.0f);
			danger_detected = true;
		}

		if (danger_detected)
		{
			osDelay(200); // Laisse le temps aux capteurs de se stabiliser après la secousse
			continue;     // On recommence la boucle (on ne charge pas l'ennemi si on est au bord !)
		}

		// Récupérer données LiDAR si dispo
		if (xQueueReceive(QLidar, &frame, 0) == pdPASS)
		{
			r = frame.target_r;
			theta = frame.target_theta;
			targetDetected = frame.target_detected;
			lidar_min = frame.lidar_min;
			obstacleDetected = frame.obstacle_detected;
		}

		// Notification d'interruption :
		if (xTaskNotifyWait(0, 0, &notif, 0) == pdTRUE)
		{
			if (notif == FSM_NOTIF_TAP_DETECTED)
			{
				// Vérifier qu’un vrai objet est proche
				if (lidar_min < 0.20f)
				{
					// INTERPRÉTATION DU TAP SELON L'ÉTAT COURANT
					switch (currentState)
					{
					case STATE_CHASE:
						currentState = STATE_TAG; // on l'a touché
						break;

					case STATE_SEARCH:
					case STATE_EVADE:
						currentState = STATE_HIT; // on s’est fait toucher
						break;

					default:
						// IDLE, etc. → ignorer
						break;
					}
				}
			}
			if (notif & 0x000000F0)
			{
				obstacle_avoid(notif);
			}
		}

		switch (currentState)
		{
		case STATE_SEARCH:
			if (targetDetected)
				currentState = STATE_CHASE;

			if (obstacleDetected)
				obstacle_avoid(0xF0);
			break;

		case STATE_CHASE:
			chase_target(r, theta);

			if (r < 0.25f)
				currentState = STATE_TAG;

			if (!targetDetected)
				currentState = STATE_SEARCH;

			if (obstacleDetected)
				SWITCH_TO_EVADE();
			break;

		case STATE_EVADE:
			evade();
			currentState = previousState;
			break;

		case STATE_TAG:
			signal_tag();
			currentState = STATE_IDLE;
			break;

		case STATE_HIT:
			signal_hit();
			currentState = STATE_IDLE;
			break;
		default:
			break;

			// etc.
		}

		osDelay(pdMS_TO_TICKS(5)); // Haute réactivité (200 Hz)
	}
}
