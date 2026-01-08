/*
 * lidar.c
 *
 * Created on: Nov 25, 2025
 * Author: Antle
 * Updated: Correction types Target_t et calculs clusters
 */

#include "capteurs/lidar.h"
#include <math.h>
#include <stdlib.h> // Pour abs()

/* --- VARIABLES GLOBALES --- */
volatile uint32_t lidar_frame_count = 0; /* Debug counter */

uint16_t compteur = 0;
float remplissage = 0.0;

/* Buffer circulaire pour le DMA */
volatile uint8_t dma_byte_buf[DMA_BYTE_BUF_SIZE];

/* Index de lecture (géré par le soft) */
static volatile uint32_t dma_read_idx = 0;

/* Tableaux de données (Allocation mémoire ici) */
Cluster clusters[MAX_CLUSTERS];
uint8_t cluster_count = 0;

/* Handle externe pour le LPUART1 (Généré par CubeMX) */
extern UART_HandleTypeDef hlpuart1;


/* --- FONCTIONS PRIVÉES --- */

/* Récupère la position d'écriture du DMA (Hardware) */
static uint32_t get_dma_write_index(void)
{
    return (DMA_BYTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(hlpuart1.hdmarx)) % DMA_BYTE_BUF_SIZE;
}


/* --- FONCTIONS PUBLIQUES --- */

/* Démarrage de la réception en mode Circulaire */
void YD_Start_UART_DMA(void)
{
    HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)dma_byte_buf, DMA_BYTE_BUF_SIZE);
}

/* * Parser Principal */
void YD_Parser_MainLoop(Pos *vue_ptr)
{
    static const uint32_t MIN_FRAME_SIZE = 10;
    uint32_t write_idx = get_dma_write_index();

    while (dma_read_idx != write_idx)
    {
        uint32_t available = (write_idx + DMA_BYTE_BUF_SIZE - dma_read_idx) % DMA_BYTE_BUF_SIZE;
        if (available < MIN_FRAME_SIZE) return;

        uint8_t b0 = dma_byte_buf[dma_read_idx];
        uint8_t b1 = dma_byte_buf[(dma_read_idx + 1) % DMA_BYTE_BUF_SIZE];

        if (b0 != 0xAA || b1 != 0x55) {
            dma_read_idx = (dma_read_idx + 1) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        uint8_t lsn = dma_byte_buf[(dma_read_idx + 3) % DMA_BYTE_BUF_SIZE];
        if (lsn == 0 || lsn > MAX_SAMPLES_PER_PKT) {
            dma_read_idx = (dma_read_idx + 2) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        uint32_t frame_len = 10 + lsn * 2;
        if (available < frame_len) return;

        lidar_frame_count++;

        uint8_t frame_buf[10 + MAX_SAMPLES_PER_PKT * 2];
        for (uint32_t i = 0; i < frame_len; i++) {
            frame_buf[i] = dma_byte_buf[(dma_read_idx + i) % DMA_BYTE_BUF_SIZE];
        }

        double start_deg = (((uint16_t)frame_buf[5] << 8 | ((uint16_t)frame_buf[4])) >> 1 ) / 64.0;
        double end_deg = (((uint16_t)frame_buf[7] << 8 | ((uint16_t)frame_buf[6])) >> 1 ) / 64.0;

        double angle_diff = end_deg - start_deg;
        if (angle_diff < 0) angle_diff += 360.0;
        double step = angle_diff / (lsn > 1 ? (lsn - 1.0) : 1.0);

        for (uint16_t i = 0; i < lsn; i++) {
            uint16_t raw_dist = (uint16_t)frame_buf[10 + i * 2] | ((uint16_t)frame_buf[11 + i * 2] << 8);
            uint16_t distance_mm = raw_dist / 4;

            double correction_angle = 0;
            if(distance_mm > 0) {
                correction_angle = atanf(21.8f*((155.3f - distance_mm)/(155.3f + distance_mm)));
            }

            double angle = start_deg + step * i;
            double angle_corr_deg = angle + (correction_angle * 180.0 / M_PI);

            if (angle_corr_deg >= 360.0) angle_corr_deg -= 360.0;
            if (angle_corr_deg < 0) angle_corr_deg += 360.0;

            double rad = (angle - 90.0f) * M_PI / 180.0f;
            uint16_t idx = (uint16_t)(angle_corr_deg * FACTEUR);
            if (idx >= N_ANGLES) idx = 0;

            if (distance_mm > 0 && distance_mm < MAX_RANGE_LIDAR) {
                if (vue_ptr[idx].distance == 0){
                    compteur++;
                    remplissage = (float)compteur / 360.0f;
                }

                float X1 = -cos(-rad) * distance_mm;
                float Y1 = sin(-rad) * distance_mm;

                if (X1 > 0 && X1 < X_MAX && Y1 > 0 && Y1 < Y_MAX){
                    vue_ptr[idx].distance = distance_mm;
                    vue_ptr[idx].X = X1;
                    vue_ptr[idx].Y = Y1;
                    vue_ptr[idx].Angle = angle_corr_deg;
                } else {
                    vue_ptr[idx].distance = 0;
                    vue_ptr[idx].X = 0;
                    vue_ptr[idx].Y = 0;
                    vue_ptr[idx].Angle = angle_corr_deg;
                }
            } else {
                vue_ptr[idx].distance = 0;
                vue_ptr[idx].X = 0;
                vue_ptr[idx].Y = 0;
                vue_ptr[idx].Angle = angle_corr_deg;
            }
        }
        dma_read_idx = (dma_read_idx + frame_len) % DMA_BYTE_BUF_SIZE;
    }
}

/*
 * Clustering
 */
int compute_cluster(Pos *pts, uint16_t start, uint16_t end, uint8_t idx) {
    float sum_x = 0.0f, sum_y = 0.0f;
    uint16_t count = 0;

    for (uint16_t i = start; i <= end; i++) {
        uint16_t d = (uint16_t)pts[i].distance;
        if (d < MIN_VALID_DISTANCE) continue;

        // Conversion Polaire -> Cartésien locale pour barycentre
        float theta = (pts[i].Angle - 90.0f) * (float)M_PI / 180.0f;
        float x = -(float)d * cosf(-theta);
        float y =  (float)d * sinf(-theta);

        sum_x += x;
        sum_y += y;
        count++;
    }

    if (count == 0) return 0;

    float cx = sum_x / (float)count;
    float cy = sum_y / (float)count;
    float dist_center = hypotf(cx, cy);

    if (dist_center < MIN_CLUSTER_DIST) return 0;

    // Calcul de l'angle du barycentre (important pour le tracking)
    float angle_center_rad = atan2f(cy, cx); // En repère mathématique
    // Conversion repère robot (0° devant, sens horaire ou anti-horaire selon tes formules)
    // Ici on inverse ta formule : theta = angle - 90 => angle = theta + 90
    // Attention au sens trigonométrique vs robot
    // Pour simplifier :
    float angle_center_deg = (atan2f(cy, -cx) * 180.0f / (float)M_PI) + 90.0f;
    if(angle_center_deg < 0) angle_center_deg += 360.0f;

    // Fusion avec clusters existants
    for (uint8_t k = 0; k < cluster_count; k++) {
        if (!clusters[k].active) continue;
        float dx = clusters[k].x - cx;
        float dy = clusters[k].y - cy;
        float dist2 = dx*dx + dy*dy;
        if (dist2 < (MERGE_THRESHOLD * MERGE_THRESHOLD)) {
            uint32_t new_size = (uint32_t)clusters[k].size + (uint32_t)count;

            float target_x = (clusters[k].x * clusters[k].size + cx * count) / (float)new_size;
            float target_y = (clusters[k].y * clusters[k].size + cy * count) / (float)new_size;

            #define SMOOTH_FACTOR 0.1f
            clusters[k].x = clusters[k].x * (1.0f - SMOOTH_FACTOR) + target_x * SMOOTH_FACTOR;
            clusters[k].y = clusters[k].y * (1.0f - SMOOTH_FACTOR) + target_y * SMOOTH_FACTOR;

            // Mise à jour distance/angle après lissage
            clusters[k].dist_min = (uint16_t)hypotf(clusters[k].x, clusters[k].y);
            // On approxime l'angle par celui calculé précédemment pour ce chunk
            clusters[k].angle_center = angle_center_deg;

            clusters[k].size = (uint16_t)new_size;
            if (start < clusters[k].start_idx) clusters[k].start_idx = start;
            if (end   > clusters[k].end_idx)   clusters[k].end_idx   = end;
            return 0; // Fusion faite
        }
    }

    // Nouveau cluster
    if (idx >= MAX_CLUSTERS) return 0;
    clusters[idx].start_idx = start;
    clusters[idx].end_idx   = end;
    clusters[idx].size      = count;
    clusters[idx].x         = cx;
    clusters[idx].y         = cy;
    // AJOUT CRUCIAL : Calcul distance et angle pour le nouveau cluster
    clusters[idx].dist_min = (uint16_t)dist_center;
    clusters[idx].angle_center = angle_center_deg;

    clusters[idx].active    = 1;
    return 1;
}

void segment_points(Pos *pts, uint16_t n_points) {
    memset(clusters, 0, sizeof(clusters));
    cluster_count = 0;

    uint16_t start = 0;
    for (uint16_t i = 0; i < n_points - 1; i++) {
        uint16_t d1 = (uint16_t)pts[i].distance;
        uint16_t d2 = (uint16_t)pts[i+1].distance;

        if (d1 < MIN_VALID_DISTANCE || d2 < MIN_VALID_DISTANCE) {
            if (start <= i) {
                if (cluster_count < MAX_CLUSTERS) {
                    if (compute_cluster(pts, start, i, cluster_count)) cluster_count++;
                }
            }
            start = i + 1;
            continue;
        }

        uint16_t diff = (d1 > d2) ? (d1 - d2) : (d2 - d1);
        if (diff > DIST_THRESHOLD) {
            if (start <= i) {
                if (cluster_count < MAX_CLUSTERS) {
                    if (compute_cluster(pts, start, i, cluster_count)) cluster_count++;
                }
            }
            start = i + 1;
        }
    }

    if (start < n_points) {
        if (cluster_count < MAX_CLUSTERS) {
            if (compute_cluster(pts, start, n_points - 1, cluster_count)) cluster_count++;
        }
    }
}


/////////////////// LOGIQUE DE CIBLE (TRACKING + BLIND SPOT) ///////////////

/* MODIFICATION ICI : Utilisation de Target_t* au lieu de Pos* */
void LIDAR_Get_Closest_Cluster(Target_t *target) {

    // Mémoire statique adaptée à Target_t
    static Target_t prev_target = {0};
    static uint8_t lost_counter = 0;
    const uint8_t MAX_LOST = 10;

    // --- CONFIGURATION ---
    const float BLIND_SPOT_DIST = 200.0f;
    const float BLIND_SPOT_FAKE_DIST = 50.0f;

    int8_t best_idx = -1;

    // --- ÉTAPE 1 : TRACKING ---
    int8_t tracked_idx = -1;
    float min_diff_angle = 1000.0f;

    if (prev_target.detected) {
        for (int i = 0; i < cluster_count; i++) {
            if (!clusters[i].active) continue;

            float c_ang = clusters[i].angle_center;
            if (c_ang > 180.0f) c_ang -= 360.0f;

            // Correction ici : prev_target.angle (minuscule)
            float diff = fabsf(c_ang - prev_target.angle);
            if (diff > 180.0f) diff = 360.0f - diff;

            // Correction ici : prev_target.dist (minuscule)
            if (diff < 30.0f && abs((int)clusters[i].dist_min - (int)prev_target.dist) < 200) {
                if (diff < min_diff_angle) {
                    min_diff_angle = diff;
                    tracked_idx = i;
                }
            }
        }
    }

    // --- ÉTAPE 2 : SÉLECTION ---
    if (tracked_idx != -1) {
        best_idx = tracked_idx;
        lost_counter = 0;
    }
    else {
        // Cible perdue ! Vérification ZONE AVEUGLE
        // Correction ici : prev_target.dist (minuscule)
        if (prev_target.detected && prev_target.dist < BLIND_SPOT_DIST) {
            target->detected = true;
            target->dist = BLIND_SPOT_FAKE_DIST;
            target->angle = prev_target.angle;

            prev_target = *target;
            lost_counter = 0;
            return;
        }

        lost_counter++;

        if (lost_counter > MAX_LOST) {
            // Scan global
            float min_d = 100000.0f;
            for (int i = 0; i < cluster_count; i++) {
                if (!clusters[i].active) continue;
                if (clusters[i].dist_min < min_d) {
                    min_d = clusters[i].dist_min;
                    best_idx = i;
                }
            }
        } else {
            if (prev_target.detected) {
                *target = prev_target;
                return;
            }
        }
    }

    // --- ÉTAPE 3 : MISE À JOUR FINALE ---
    if (best_idx != -1) {
        float raw_ang = clusters[best_idx].angle_center;
        if (raw_ang > 180.0f) raw_ang -= 360.0f;

        target->detected = true;
        target->dist = (float)clusters[best_idx].dist_min;
        target->angle = raw_ang;

        prev_target = *target;
        lost_counter = 0;
    } else {
        target->detected = false;
        prev_target.detected = false;
    }
}
