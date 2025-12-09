/*
 * lidar.h
 *
 * Created on: Nov 25, 2025
 * Author: Antle
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* --- VARIABLES GLOBALES ACCESSIBLES (DEBUG & DATA) --- */
/* Compteur incrémenté à chaque trame valide reçue (Heartbeat) */
extern volatile uint32_t lidar_frame_count;

/* --- DÉFINITIONS ET CONFIGURATION --- */
#define FACTEUR              2
#define MAX_SAMPLES_PER_PKT  200U
#define N_ANGLES             (360 * FACTEUR)

/* Limites de la zone de détection (en mm) */
#define X_MAX                600
#define Y_MAX                1500
#define MAX_RANGE_LIDAR      500

/* Paramètres de clustering */
#define MAX_POINTS           720
#define MAX_CLUSTERS         20
#define DIST_THRESHOLD       300     // 30 cm
#define MERGE_THRESHOLD      100.0f  // Fusion si < 10 cm
#define MIN_VALID_DISTANCE   50u     // Ignore les points trop proches (< 5 cm)
#define MIN_CLUSTER_DIST     20.0f   // Ignore bruit au centre
#define CLUSTER_MARGIN       2
#define CLUSTER_PERSISTENCE  3

/* Buffer DMA */
#define DMA_BYTE_BUF_SIZE    300U

/* Constante PI si non définie */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* --- STRUCTURES --- */

/* Structure d'un point lidar */
typedef struct {
    float distance;
    float Angle;
    float X;
    float Y;
} Pos;

/* Structure d'un objet détecté (Cluster) */
typedef struct {
    uint16_t start_idx;
    uint16_t end_idx;
    uint16_t size;
    float x;
    float y;
    uint8_t active;
} Cluster;

/* Déclaration externe du tableau pour qu'il soit visible dans le main */
extern Pos vue[N_ANGLES];

/* --- PROTOTYPES PUBLICS --- */

/* Démarre le DMA sur LPUART1 */
void YD_Start_UART_DMA(void);

/* Fonction principale de parsing (à appeler dans la boucle/tâche) */
void YD_Parser_MainLoop(Pos *vue, uint16_t dummy);

/* Fonction de segmentation (création des clusters) */
void segment_points(Pos *points, uint16_t n_points);

#endif /* INC_LIDAR_H_ */
