/*
 * LIDAR.h
 *
 *  Created on: Oct 1, 2025
 *      Author: louisvoz
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_
////////
#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/////////
void YD_Start_UART_DMA(void);
void YD_Parser_MainLoop(Pos *vue);

static uint32_t get_dma_write_index(void);
static void parser_process_available_bytes(void);
static bool try_parse_frame_from_stream(uint32_t *consumed_bytes);
static bool validate_checksum(const uint8_t *data, uint32_t len, uint16_t cs);
static void on_new_frame(uint16_t start_ang_raw, uint16_t end_ang_raw,
                         const uint16_t *raw_samples, uint16_t sample_count);

///////////////////DEFINE/////////////////

#define FACTEUR 1
#define MAX_SAMPLES_PER_PKT  200U      // max samples par trame
#define N_ANGLES   (360 * FACTEUR)     // 0.1° resolution pour 0°-360°
#define X_MAX 600// Taille maximale en X en mm
#define Y_MAX 1500  // Taille maximale en Y en mm
#define MAX_RANGE_LIDAR 200 // Cercle de 25 cm
#define M_PI             3.14159265358979323846f
#define MAX_POINTS      (360 * FACTEUR)   // exemple : 1 tour LiDAR = 720 mesures
#define MAX_CLUSTERS       20
#define DIST_THRESHOLD     300     // 30 cm en mm
#define MERGE_THRESHOLD    100.0f  // si barycentres < 100 mm => fusion
#define MIN_VALID_DISTANCE 50u     // distance minimale (mm) pour considérer un point valide
#define MIN_CLUSTER_DIST   20.0f   // ignorer barycentre à moins de 20 mm de l'origine
#define DMA_BYTE_BUF_SIZE    300U      // taille du buffer DMA circulaire
#define CLUSTER_MARGIN 2
#define CLUSTER_PERSISTENCE 3




#endif /* INC_LIDAR_H_ */

