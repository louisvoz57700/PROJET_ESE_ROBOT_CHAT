/* LIDAR.c - Version Hybride (Parsing Ami + Target Logic) */

#include "capteurs/lidar.h" // Vérifie ton chemin d'include
#include "main.h"
#include <stdlib.h>

// Variables Globales
Pos vue[N_ANGLES];
Cluster clusters[MAX_CLUSTERS];
uint8_t cluster_count = 0;

// Buffer DMA
uint8_t dma_byte_buf[DMA_BYTE_BUF_SIZE];
static volatile uint32_t dma_read_idx = 0;
extern UART_HandleTypeDef hlpuart1;

// Constante Math
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// --- DMA ---
void YD_Start_UART_DMA(void) {
	HAL_StatusTypeDef ret= HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)dma_byte_buf, DMA_BYTE_BUF_SIZE);
}

static uint32_t get_dma_write_index(void) {
    return (DMA_BYTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(hlpuart1.hdmarx)) % DMA_BYTE_BUF_SIZE;
}


void YD_Parser_MainLoop(Pos *vue_ptr) {
    static const uint32_t MIN_FRAME_SIZE = 10;
    uint32_t write_idx = get_dma_write_index();

    while (dma_read_idx != write_idx) {
        uint32_t available = (write_idx + DMA_BYTE_BUF_SIZE - dma_read_idx) % DMA_BYTE_BUF_SIZE;
        if (available < MIN_FRAME_SIZE) return;

        // Sync Check 0xAA 0x55
        if (dma_byte_buf[dma_read_idx] != 0xAA ||
            dma_byte_buf[(dma_read_idx + 1) % DMA_BYTE_BUF_SIZE] != 0x55) {
            dma_read_idx = (dma_read_idx + 1) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        uint8_t lsn = dma_byte_buf[(dma_read_idx + 3) % DMA_BYTE_BUF_SIZE];
        if (lsn == 0) { // Erreur, skip header
            dma_read_idx = (dma_read_idx + 2) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        uint32_t frame_len = 10 + lsn * 2;
        if (available < frame_len) return;

        // Extraction Frame
        uint8_t frame[256]; // Buffer temporaire suffisant
        for (uint32_t i = 0; i < frame_len; i++) {
            frame[i] = dma_byte_buf[(dma_read_idx + i) % DMA_BYTE_BUF_SIZE];
        }

        // Calcul Angles
        double start_deg = (((uint16_t)frame[5] << 8 | frame[4]) >> 1) / 64.0;
        double end_deg   = (((uint16_t)frame[7] << 8 | frame[6]) >> 1) / 64.0;
        double angle_diff = end_deg - start_deg;
        if (angle_diff < 0) angle_diff += 360.0;
        double step = (lsn > 1) ? (angle_diff / (lsn - 1.0)) : 0;

        // Remplissage Points
        for (uint16_t i = 0; i < lsn; i++) {
            uint16_t raw_dist = (uint16_t)frame[10 + i * 2] | ((uint16_t)frame[11 + i * 2] << 8);
            uint16_t dist_mm = raw_dist / 4;

            // Angle final
            double angle = start_deg + step * i;
            if (angle >= 360.0) angle -= 360.0;

            // Conversion Index
            uint16_t idx = (uint16_t)(angle * FACTEUR);
            if (idx >= N_ANGLES) continue;

            if (dist_mm > MIN_VALID_DISTANCE && dist_mm < MAX_RANGE_LIDAR) {
                vue_ptr[idx].distance = dist_mm;
                vue_ptr[idx].Angle = (float)angle;

                // --- MAGIE DU COPAIN : Calcul X/Y immédiat ---
                // Note : Vérifie si tu as besoin de -90° ou pas selon le montage du lidar
                float rad = (angle - 90.0f) * M_PI / 180.0f;
                vue_ptr[idx].X = -cosf(-rad) * dist_mm;
                vue_ptr[idx].Y =  sinf(-rad) * dist_mm;
            } else {
                vue_ptr[idx].distance = 0;
                vue_ptr[idx].X = 0;
                vue_ptr[idx].Y = 0;
            }
        }
        dma_read_idx = (dma_read_idx + frame_len) % DMA_BYTE_BUF_SIZE;
    }
}

// --- CLUSTERING DU COPAIN (Simplifié et intégré) ---

// Fonction interne pour calculer ou fusionner un cluster
int compute_cluster(Pos *points, uint16_t start, uint16_t end, uint8_t idx_limit) {
    float sum_x = 0, sum_y = 0;
    uint16_t count = 0;
    uint16_t min_d = 0xFFFF;

    for (uint16_t i = start; i <= end; i++) {
        if (points[i].distance < MIN_VALID_DISTANCE) continue;
        sum_x += points[i].X;
        sum_y += points[i].Y;
        if (points[i].distance < min_d) min_d = points[i].distance;
        count++;
    }

    if (count == 0) return 0;

    float cx = sum_x / count;
    float cy = sum_y / count;

    // Filtre bruit centre
    if (hypotf(cx, cy) < MIN_CLUSTER_DIST) return 0;

    // Tentative de fusion avec clusters existants
    for (uint8_t k = 0; k < cluster_count; k++) {
        float dx = clusters[k].x - cx;
        float dy = clusters[k].y - cy;

        // Si proche d'un cluster existant -> FUSION
        if ((dx*dx + dy*dy) < (MERGE_THRESHOLD * MERGE_THRESHOLD)) {
            // Moyenne pondérée simple pour mettre à jour la position
            uint16_t total = clusters[k].size + count;
            clusters[k].x = (clusters[k].x * clusters[k].size + cx * count) / total;
            clusters[k].y = (clusters[k].y * clusters[k].size + cy * count) / total;
            clusters[k].size = total;
            if (min_d < clusters[k].dist_min) clusters[k].dist_min = min_d;
            return 0; // Pas de nouveau cluster créé
        }
    }

    // Nouveau Cluster
    if (cluster_count >= MAX_CLUSTERS) return 0;

    clusters[cluster_count].x = cx;
    clusters[cluster_count].y = cy;
    clusters[cluster_count].dist_min = min_d;
    clusters[cluster_count].size = count;
    clusters[cluster_count].angle = atan2f(cy, cx) * 180.0f / M_PI;
    if (clusters[cluster_count].angle < 0) clusters[cluster_count].angle += 360.0f;
    clusters[cluster_count].active = 1;

    return 1; // Nouveau cluster créé
}

void segment_points(Pos *points, uint16_t n_points) {
    memset(clusters, 0, sizeof(clusters));
    cluster_count = 0;

    uint16_t start = 0;

    for (uint16_t i = 0; i < n_points - 1; i++) {
        uint16_t d1 = points[i].distance;
        uint16_t d2 = points[i+1].distance;

        // Rupture si saut de distance ou point invalide
        bool gap = (d1 < MIN_VALID_DISTANCE || d2 < MIN_VALID_DISTANCE);
        if (!gap) {
             int diff = (int)d1 - (int)d2;
             if (abs(diff) > DIST_THRESHOLD) gap = true;
        }

        if (gap) {
            if (start <= i) {
                if (compute_cluster(points, start, i, MAX_CLUSTERS)) cluster_count++;
            }
            start = i + 1;
        }
    }

    // Dernier segment
    if (start < n_points) {
         if (compute_cluster(points, start, n_points - 1, MAX_CLUSTERS)) cluster_count++;
    }
}


void LIDAR_Get_Closest_Cluster(Target_t *target) {
    // Timeout si rien détecté
    static uint8_t lost_counter = 0;

    if (cluster_count == 0) {
        lost_counter++;
        if (lost_counter > 5) {
            target->detected = 0;
            target->dist = 0;
        }
        return;
    }
    lost_counter = 0;

    // Recherche du plus proche
    int best_idx = -1;
    float min_dist_sq = 100000000.0f; // Grand nombre

    for (int i = 0; i < cluster_count; i++) {
        if (!clusters[i].active) continue;

        // Distance au carré (plus rapide que sqrt)
        float d2 = clusters[i].x*clusters[i].x + clusters[i].y*clusters[i].y;

        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            best_idx = i;
        }
    }

    if (best_idx != -1) {
        target->detected = 1;
        target->dist = sqrtf(min_dist_sq);
        target->angle = clusters[best_idx].angle;
    }
}
