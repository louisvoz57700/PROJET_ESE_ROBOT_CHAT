/* LIDAR.h */
#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// --- STRUCTURE POLAIRE PURE ---
typedef struct {
	uint16_t distance; // mm
	float Angle;       // degrés
} Pos;

// Structure Cible
typedef struct {
    float angle;      // degrés
    float dist;       // mm
    uint8_t detected; // 1 ou 0
} Target_t;

// Structure Cluster
typedef struct {
    uint16_t start_idx;
    uint16_t end_idx;
    uint16_t size;
    float angle_center;
    uint16_t dist_min;
    uint8_t active;
} Cluster;

// --- CONFIGURATION ---
#define FACTEUR 1
#define N_ANGLES (360 * FACTEUR)
#define MAX_SAMPLES_PER_PKT 200U
#define MAX_CLUSTERS 20
#define DMA_BYTE_BUF_SIZE 512 // Augmenté un peu pour sécurité (puissance de 2 préférée)

// Filtres
#define MIN_VALID_DISTANCE 100 // mm
#define MAX_RANGE_LIDAR 2500   // mm
#define DIST_THRESHOLD 300     // mm (Seuil saut pour cluster)

// Prototypes
void YD_Start_UART_DMA(void);
void YD_Parser_MainLoop(Pos *vue);
void segment_points(Pos *points, uint16_t n_points);
void LIDAR_Get_Closest_Cluster(Target_t *target);

// Correction angulaire (à ajuster selon tes tests)

#define LIDAR_ANGLE_OFFSET  75.0f

#endif /* INC_LIDAR_H_ */
