/*
 * LIDAR.c
 *
 *  Created on: Oct 1, 2025
 *      Author: louisvoz
 */


//////////////////INCLUDE/////////////////
#include "LIDAR.h"
#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

///////////////////VARIABLES///////////////

uint16_t compteur = 0;
float remplissage = 0.0;
#define RX_BUF_SIZE 360
// Buffer DMA circulaire
uint8_t dma_rx_buf[RX_BUF_SIZE];
volatile uint16_t old_pos = 0;
static volatile uint32_t dma_read_idx = 0;
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern volatile uint8_t dma_byte_buf[DMA_BYTE_BUF_SIZE];


////DEF CLUSTER
extern Cluster clusters[MAX_CLUSTERS];
uint8_t cluster_count = 0;

/* Démarrage DMA circulaire */
void YD_Start_UART_DMA(void)
{
    HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)dma_byte_buf, DMA_BYTE_BUF_SIZE);
}

/* Retourne l'index d'écriture actuel du DMA */
static uint32_t get_dma_write_index(void)
{
	return (DMA_BYTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx)) % DMA_BYTE_BUF_SIZE;}


static bool validate_checksum(const uint8_t *data, uint32_t len, uint16_t cs)
{
    uint16_t acc = 0;

    // XOR de tous les bytes du frame (CT + LSN + FSA + LSA + samples) sauf le checksum
    for (uint32_t i = 0; i < len; i += 2) {
        uint16_t val = data[i] | (i+1 < len ? ((uint16_t)data[i+1] << 8) : 0);
        acc ^= val;
    }

    return acc == cs;
}

/* Parser principal : à appeler dans la boucle principale */
void YD_Parser_MainLoop(Pos *vue)
{
    static const uint32_t MIN_FRAME_SIZE = 10;
    uint32_t write_idx = get_dma_write_index();

    while (dma_read_idx != write_idx) {
    	uint32_t available = (write_idx + DMA_BYTE_BUF_SIZE - dma_read_idx) % DMA_BYTE_BUF_SIZE; // + DMA_BYTE car il est circulaire
        if (available < MIN_FRAME_SIZE) {
            return; // Not enough data for even a header
        }

        uint8_t b0 = dma_byte_buf[dma_read_idx];
        uint8_t b1 = dma_byte_buf[(dma_read_idx + 1) % DMA_BYTE_BUF_SIZE];

        // Search for sync bytes
        if (b0 != 0xAA || b1 != 0x55) {
            dma_read_idx = (dma_read_idx + 1) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        // Get LSN
        uint8_t lsn = dma_byte_buf[(dma_read_idx + 3) % DMA_BYTE_BUF_SIZE]; // le % sert à reboucler
        if (lsn == 0 || lsn > MAX_SAMPLES_PER_PKT) {
            dma_read_idx = (dma_read_idx + 2) % DMA_BYTE_BUF_SIZE; // Header corrompu , on avance de 2 pour skip le header
            continue;
        }

        uint32_t frame_len = 10 + lsn * 2;
        if (available < frame_len) {
            return; // Not a full frame yet
        }

        // Read the full frame into a temporary buffer
        uint8_t frame_buf[10 + MAX_SAMPLES_PER_PKT * 2];
        for (uint32_t i = 0; i < frame_len; i++) {
            frame_buf[i] = dma_byte_buf[(dma_read_idx + i) % DMA_BYTE_BUF_SIZE];
        }
        // Angle de début et de fin
        double start_deg = (((uint16_t)frame_buf[5] << 8 | ((uint16_t)frame_buf[4])) >> 1 ) / 64.0;
        double end_deg = (((uint16_t)frame_buf[7] << 8 | ((uint16_t)frame_buf[6])) >> 1 ) / 64.0;

        // Calcul du step entre chaque mesure
        double angle_diff = end_deg - start_deg;
        if (angle_diff < 0) {
            angle_diff += 360.0;
        }
        double step = angle_diff / (lsn - 1.0); // -1 car on prend pas en compte la première valeur

        for (uint16_t i = 0; i < lsn; i++) {


            // On en déduit les distance
            uint16_t raw_dist = (uint16_t)frame_buf[10 + i * 2] | ((uint16_t)frame_buf[11 + i * 2] << 8);
            uint16_t distance_mm = raw_dist / 4;

            double correction_angle = atanf(21.8f*((155.3f - distance_mm)/(155.3f + distance_mm)));
            double angle = start_deg + step * i;
			double rad = (angle - 90.0f) * M_PI / 180.0f;
			if (angle >= 360.0) {
				angle -= 360.0;
			}

			// On converti l'angle en index (sur 3600 ou 360 points)
			uint16_t idx = (uint16_t)((angle + correction_angle) * FACTEUR) ;
			if (idx >= N_ANGLES) {
				continue; // Angle is out of range, skip
			}

            // Store the distance if it's valid
            if (distance_mm > 0 && distance_mm < MAX_RANGE_LIDAR) { // A valid distance is greater than 0
            	if (vue[idx].distance == 0){
            	                	compteur = (compteur + 1);
            	                	remplissage =(float) compteur / 360.0f;
            	}

            	float X1 = -cos(-rad) * distance_mm;
            	float Y1 = sin(-rad) * distance_mm;

            	if (X1 > 0 && X1 < X_MAX && Y1 > 0 && Y1<Y_MAX){
                    vue[idx].distance = distance_mm;
                    vue[idx].X = X1;
                    vue[idx].Y = Y1;
                    vue[idx].Angle = angle;
            	}

            	else{

            		vue[idx].distance = 0;
					vue[idx].X = 0;
					vue[idx].Y = 0;
                    vue[idx].Angle = angle ;

            	}

            }
        	else{
        		vue[idx].distance = 0;
				vue[idx].X = 0;
				vue[idx].Y = 0;
                vue[idx].Angle = angle ;
        	}
        }

        // On avance l'idx à la fin
        dma_read_idx = (dma_read_idx + frame_len) % DMA_BYTE_BUF_SIZE;
    }
}

/**
 * compute_cluster :
 * - calcule barycentre en ignorant points < MIN_VALID_DISTANCE
 * - ignore si aucun point valide
 * - ignore si barycentre trop proche de l'origine
 * - fusionne avec cluster existant si proche (< MERGE_THRESHOLD)
 * - sinon écrit le nouveau cluster dans clusters[idx]
 *
 * retourne 1 si un nouveau cluster a été ajouté, 0 sinon (fusion ou invalide)
 */


int compute_cluster(Pos *points, uint16_t start, uint16_t end, uint8_t idx) {
    float sum_x = 0.0f, sum_y = 0.0f;
    uint16_t count = 0;

    for (uint16_t i = start; i <= end; i++) {
        uint16_t d = points[i].distance;
        if (d < MIN_VALID_DISTANCE) continue; // filtrer points invalides

        float theta = (points[i].Angle - 90.0f) * (float)M_PI / 180.0f; // conversion rad
        float x = -(float)d * cosf(-theta);
        float y =  (float)d * sinf(-theta);

        sum_x += x;
        sum_y += y;
        count++;
    }

    if (count == 0) return 0; // pas de point valide -> pas de cluster

    float cx = sum_x / (float)count;
    float cy = sum_y / (float)count;

    // ignorer cluster centré trop près de l'origine (bruit)
    if (hypotf(cx, cy) < MIN_CLUSTER_DIST) return 0;

    // On vérifie si le cluster existe déja
    for (uint8_t k = 0; k < cluster_count; k++) {
        if (!clusters[k].active) continue;
        float dx = clusters[k].x - cx;
        float dy = clusters[k].y - cy;
        float dist2 = dx*dx + dy*dy;
        if (dist2 < (MERGE_THRESHOLD * MERGE_THRESHOLD) && dx != 0 && dy != 0) { // on regarde si l'écart entre les 2 clusters est grande
        	// calcul du barycentre pondéré pour fusionner deux clusters
        	uint32_t new_size = (uint32_t)clusters[k].size + (uint32_t)count;
        	clusters[k].x = (clusters[k].x * clusters[k].size + cx * count) / (float)new_size;
        	clusters[k].y = (clusters[k].y * clusters[k].size + cy * count) / (float)new_size;

			#define SMOOTH_FACTOR 0.01f  // ajustable 0.1 à 0.5
			clusters[k].x = clusters[k].x * (1.0f - SMOOTH_FACTOR) + cx * SMOOTH_FACTOR;
			clusters[k].y = clusters[k].y * (1.0f - SMOOTH_FACTOR) + cy * SMOOTH_FACTOR;

        	clusters[k].size = (uint16_t)new_size;
            // étendre les indices pour recouvrir toute la plage
            if (start < clusters[k].start_idx) clusters[k].start_idx = start;
            if (end   > clusters[k].end_idx)   clusters[k].end_idx   = end;
            return 0; // fusion => pas de nouveau cluster ajouté
        }
    }

    // on crée un nouveau cluster
    if (idx >= MAX_CLUSTERS) return 0; // sécurité
    if (clusters[idx].x != cx && clusters[idx].y != cy)
    {
        clusters[idx].start_idx = start;
        clusters[idx].end_idx   = end;
        clusters[idx].size      = count;
        clusters[idx].x         = cx;
        clusters[idx].y         = cy;
        clusters[idx].active    = 1;
        return 1;
    }

}

/**
 * segment_points :
 * - réinitialise clusters[]
 * - segmente en se basant sur DIST_THRESHOLD
 * - traite les points invalides en tant que séparateurs
 * - n'incrémente cluster_count que si compute_cluster ajoute réellement un cluster
 */

void segment_points(Pos *points, uint16_t n_points) {
    // réinitialiser le tableau de clusters pour éviter les restes d'anciennes frames
    memset(clusters, 0, sizeof(clusters));
    cluster_count = 0;

    uint16_t start = 0;
    for (uint16_t i = 0; i < n_points - 1; i++) {
        uint16_t d1 = points[i].distance;
        uint16_t d2 = points[i+1].distance;

        // si un point est invalide, on force une séparation (pas de cluster qui traverse un point invalide)
        if (d1 < MIN_VALID_DISTANCE || d2 < MIN_VALID_DISTANCE) {
            if (start <= i) {
                if (cluster_count < MAX_CLUSTERS) {
                    if (compute_cluster(points, start, i, cluster_count)) cluster_count++;
                }
            }
            start = i + 1;
            continue;
        }
        // détection de point valide
        uint16_t diff = (d1 > d2) ? (d1 - d2) : (d2 - d1);
        if (diff > DIST_THRESHOLD) {
            if (start <= i) {
                if (cluster_count < MAX_CLUSTERS) {
                    if (compute_cluster(points, start, i, cluster_count)) cluster_count++;
                }
            }
            start = i + 1;
        }
    }

    // dernier cluster (s'il reste des points)
    if (start < n_points) {
        if (cluster_count < MAX_CLUSTERS) {
            if (compute_cluster(points, start, n_points - 1, cluster_count)) cluster_count++;
        }
    }
}
