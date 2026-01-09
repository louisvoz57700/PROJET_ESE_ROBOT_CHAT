/*
 * lidar.h
 *
 * Created on: Nov 25, 2025
 * Author: Antle
 * Updated: Ajout de Target_t pour compilation task_control
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* --- DÉFINITIONS ET CONFIGURATION --- */
#define FACTEUR 1
#define MAX_SAMPLES_PER_PKT  200U      // max samples par trame
#define N_ANGLES   (360 * FACTEUR)     // 0.1° resolution pour 0°-360°
#define X_MAX 600                      // Taille maximale en X en mm
#define Y_MAX 1500                     // Taille maximale en Y en mm
#define MAX_RANGE_LIDAR 200            // Cercle de 25 cm
#define MAX_POINTS      (360 * FACTEUR)// exemple : 1 tour LiDAR = 720 mesures
#define MAX_CLUSTERS       20
#define DIST_THRESHOLD     300     // 30 cm en mm
#define MERGE_THRESHOLD    100.0f  // si barycentres < 100 mm => fusion
#define MIN_VALID_DISTANCE 50u     // distance minimale (mm) pour considérer un point valide
#define MIN_CLUSTER_DIST   20.0f   // ignorer barycentre à moins de 20 mm de l'origine
#define DMA_BYTE_BUF_SIZE    300U  // taille du buffer DMA circulaire
#define CLUSTER_MARGIN 2
#define CLUSTER_PERSISTENCE 3


/* Constante PI si non définie */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* --- STRUCTURES --- */

typedef struct {
    uint16_t num_samples;   // Nombre de samples
    float start_angle;      // Angle de départ (en degrés ou radians)
    float end_angle;        // Angle de fin (en degrés ou radians)
    float precision;
} AngleData;

/* Structure d'un point lidar brut */
typedef struct {
    float distance;
    float Angle;
    float X;
    float Y;
    uint8_t detected; // 1 ou 0
} Pos;

/* --- AJOUT MISSING : Structure cible pour task_control --- */
typedef struct {
    float dist;      // Attention: minuscule 'dist' pour matcher task_control.c
    float angle;     // Attention: minuscule 'angle' pour matcher task_control.c
    bool detected;   // true si cible trouvée
} Target_t;

// Structure Cluster
typedef struct {
    uint16_t start_idx;
    uint16_t end_idx;
    uint16_t size;
    float x, y;
    float angle_center;
    uint16_t dist_min;
    uint8_t active;
    float angle;
} Cluster;


/* --- PROTOTYPES PUBLICS --- */

/* Démarre le DMA sur LPUART1 */
void YD_Start_UART_DMA(void);

/* Fonction principale de parsing (à appeler dans la boucle/tâche) */
void YD_Parser_MainLoop(Pos *vue);

/* Fonction de segmentation (création des clusters) */
void segment_points(Pos *points, uint16_t n_points);

/* --- CORRECTION PROTOTYPE --- */
/* Anciennement : void LIDAR_Get_Closest_Cluster(Pos *target); */
/* Doit prendre Target_t* car c'est ce qui est passé dans task_control */
void LIDAR_Get_Closest_Cluster(Target_t *target);

#endif /* INC_LIDAR_H_ */
