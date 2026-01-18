/* LIDAR.h */
#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// --- STRUCTURES ---

typedef struct {
    uint16_t distance; // mm
    float Angle;       // degrés
    float X;           // mm (Coordonnée cartésienne)
    float Y;           // mm (Coordonnée cartésienne)
} Pos;

typedef struct {
    float x;           // Barycentre X
    float y;           // Barycentre Y
    float angle;       // Angle moyen
    uint16_t size;     // Nombre de points
    uint16_t dist_min; // Distance minimale au robot (pour l'attaque)
    uint8_t active;    // 1 = valide
} Cluster;

typedef struct {
    float angle;
    float dist;
    uint8_t detected;
} Target_t;

// --- CONFIGURATION ---
#define FACTEUR 1
#define N_ANGLES (360 * FACTEUR)
#define MAX_CLUSTERS 20
#define DMA_BYTE_BUF_SIZE 512

// Seuils de segmentation (Ceux de ton ami)
#define MIN_VALID_DISTANCE 50   // mm
#define MAX_RANGE_LIDAR 2500    // mm
#define DIST_THRESHOLD 300      // mm (Saut pour séparer deux objets)
#define MERGE_THRESHOLD 150.0f  // mm (Si deux clusters sont plus proches que ça, on fusionne)
#define MIN_CLUSTER_DIST 20.0f  // mm (Filtre bruit central)

// Prototypes
void YD_Start_UART_DMA(void);
void YD_Parser_MainLoop(Pos *vue);
void segment_points(Pos *points, uint16_t n_points);
void LIDAR_Get_Closest_Cluster(Target_t *target);

#endif /* INC_LIDAR_H_ */
