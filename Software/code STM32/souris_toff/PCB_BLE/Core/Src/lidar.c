/*
 * LIDAR.c - Version FINALE SUMO
 * Inclus : Parsing Polaire, Offset 90°, Segmentation Wrap-Around,
 * Tracking Spatial et Protection Zone Aveugle.
 */

#include "LIDAR.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/////////////////// VARIABLES ///////////////

Pos vue[N_ANGLES];

// Buffer DMA Circulaire
#define RX_BUF_SIZE DMA_BYTE_BUF_SIZE
uint8_t dma_byte_buf[RX_BUF_SIZE];
static volatile uint32_t dma_read_idx = 0;

// Hardware Handles
extern UART_HandleTypeDef hlpuart1;

// Clusters (Objets détectés)
Cluster clusters[MAX_CLUSTERS];
uint8_t cluster_count = 0;

// Debug
volatile uint32_t debug_dma_counter = 0;

/////////////////// FONCTIONS BAS NIVEAU ///////////////

void YD_Start_UART_DMA(void) {
    HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)dma_byte_buf, DMA_BYTE_BUF_SIZE);
}

static uint32_t get_dma_write_index(void) {
    return (DMA_BYTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(hlpuart1.hdmarx)) % DMA_BYTE_BUF_SIZE;
}

/* FILTRE : Bouche les trous (points à 0 isolés) */
static void filter_noise_and_fill_gaps(Pos *points) {
    for (int i = 1; i < N_ANGLES - 1; i++) {
        if (points[i].distance == 0) {
            uint16_t d_prev = points[i-1].distance;
            uint16_t d_next = points[i+1].distance;

            // Si entouré de points valides et proches (< 10cm d'écart)
            if (d_prev > MIN_VALID_DISTANCE && d_next > MIN_VALID_DISTANCE) {
                if (abs((int)d_prev - (int)d_next) < 100) {
                    points[i].distance = (d_prev + d_next) / 2;
                    points[i].Angle = (points[i-1].Angle + points[i+1].Angle) / 2.0f;
                }
            }
        }
    }
}

/* PARSER : Lit le flux UART et remplit le tableau 'vue' */
void YD_Parser_MainLoop(Pos *vue_ptr) {
    static const uint32_t MIN_FRAME_SIZE = 10;
    uint32_t write_idx = get_dma_write_index();

    if (write_idx != dma_read_idx) debug_dma_counter++;

    while (dma_read_idx != write_idx) {
        uint32_t available = (write_idx + DMA_BYTE_BUF_SIZE - dma_read_idx) % DMA_BYTE_BUF_SIZE;
        if (available < MIN_FRAME_SIZE) return;

        uint8_t b0 = dma_byte_buf[dma_read_idx];
        uint8_t b1 = dma_byte_buf[(dma_read_idx + 1) % DMA_BYTE_BUF_SIZE];

        // Sync Header (0xAA 0x55)
        if (b0 != 0xAA || b1 != 0x55) {
            dma_read_idx = (dma_read_idx + 1) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        // LSN (Sample Count)
        uint8_t lsn = dma_byte_buf[(dma_read_idx + 3) % DMA_BYTE_BUF_SIZE];
        if (lsn == 0 || lsn > MAX_SAMPLES_PER_PKT) {
            dma_read_idx = (dma_read_idx + 2) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        uint32_t frame_len = 10 + lsn * 2;
        if (available < frame_len) return;

        // Copie trame dans buffer temp
        uint8_t frame_buf[10 + MAX_SAMPLES_PER_PKT * 2];
        for (uint32_t i = 0; i < frame_len; i++) {
            frame_buf[i] = dma_byte_buf[(dma_read_idx + i) % DMA_BYTE_BUF_SIZE];
        }

        // Parsing Angles
        double start_deg = (((uint16_t)frame_buf[5] << 8 | ((uint16_t)frame_buf[4])) >> 1 ) / 64.0;
        double end_deg   = (((uint16_t)frame_buf[7] << 8 | ((uint16_t)frame_buf[6])) >> 1 ) / 64.0;

        double angle_diff = end_deg - start_deg;
        if (angle_diff < 0) angle_diff += 360.0;
        double step = (lsn > 1) ? (angle_diff / (lsn - 1.0)) : 0;

        for (uint16_t i = 0; i < lsn; i++) {
            uint16_t raw_dist = (uint16_t)frame_buf[10 + i * 2] | ((uint16_t)frame_buf[11 + i * 2] << 8);
            uint16_t distance_mm = raw_dist / 4;

            // Correction interne du Lidar (Trigo)
            double correction_angle = 0;
            if (distance_mm > 0) {
                 correction_angle = atan(21.8 * ((155.3 - distance_mm)/(155.3 + distance_mm))) * (180.0 / 3.14159);
            }

            double angle = start_deg + step * i + correction_angle;

            // --- OFFSET ---
            angle += LIDAR_ANGLE_OFFSET;
            // ------------------

            if (angle >= 360.0) angle -= 360.0;
            if (angle < 0.0)    angle += 360.0;

            uint16_t idx = (uint16_t)(angle * FACTEUR);
            if (idx >= N_ANGLES) continue;

            if (distance_mm > MIN_VALID_DISTANCE && distance_mm < MAX_RANGE_LIDAR) {
                vue_ptr[idx].distance = distance_mm;
                vue_ptr[idx].Angle = (float)angle;
            } else {
                vue_ptr[idx].distance = 0;
                vue_ptr[idx].Angle = (float)angle;
            }
        }
        dma_read_idx = (dma_read_idx + frame_len) % DMA_BYTE_BUF_SIZE;
    }

    // Filtre final
    filter_noise_and_fill_gaps(vue_ptr);
}

/////////////////// SEGMENTATION (CLUSTERING) ///////////////

/* Helper pour ajouter un cluster */
void add_cluster(float sum_angle, uint16_t min_dist, uint16_t count, uint16_t start, uint16_t end) {
    if (cluster_count >= MAX_CLUSTERS) return;

    clusters[cluster_count].angle_center = sum_angle / count;
    clusters[cluster_count].dist_min = min_dist;
    clusters[cluster_count].size = count;
    clusters[cluster_count].start_idx = start;
    clusters[cluster_count].end_idx = end;
    clusters[cluster_count].active = 1;
    cluster_count++;
}

void segment_points(Pos *points, uint16_t n_points) {
    cluster_count = 0;
    memset(clusters, 0, sizeof(clusters));

    uint16_t start = 0;
    float sum_angle = 0;
    uint16_t min_dist = 60000;
    uint16_t count = 0;

    // 1. Segmentation Linéaire
    for (uint16_t i = 0; i < n_points; i++) {
        uint16_t d = points[i].distance;

        // Rupture si distance nulle
        if (d == 0) {
            if (count > 3) add_cluster(sum_angle, min_dist, count, start, i-1);
            count = 0;
            continue;
        }

        if (count == 0) {
            start = i;
            sum_angle = points[i].Angle;
            min_dist = d;
            count = 1;
        }
        else {
            uint16_t d_prev = points[i-1].distance;
            // Rupture si saut de distance > Seuil
            if (abs((int)d - (int)d_prev) > DIST_THRESHOLD) {
                if (count > 3) add_cluster(sum_angle, min_dist, count, start, i-1);

                start = i;
                sum_angle = points[i].Angle;
                min_dist = d;
                count = 1;
            } else {
                sum_angle += points[i].Angle;
                if (d < min_dist) min_dist = d;
                count++;
            }
        }
    }
    // Fin du tableau
    if (count > 3) add_cluster(sum_angle, min_dist, count, start, n_points-1);

    // 2. CORRECTION WRAP-AROUND (Recollement 360° -> 0°)
    if (cluster_count >= 2) {
        Cluster *first = &clusters[0];
        Cluster *last = &clusters[cluster_count - 1];

        // Si le dernier cluster touche la fin et le premier touche le début
        if (last->end_idx >= n_points - 5 && first->start_idx <= 5) {
            // Et s'ils sont proches en distance
            if (abs((int)first->dist_min - (int)last->dist_min) < DIST_THRESHOLD) {
                // Fusion !
                if (last->dist_min < first->dist_min) first->dist_min = last->dist_min;
                first->size += last->size;
                last->active = 0;
                cluster_count--;
            }
        }
    }
}

/////////////////// LOGIQUE DE CIBLE (TRACKING + BLIND SPOT) ///////////////

void LIDAR_Get_Closest_Cluster(Target_t *target) {

    // Mémoire statique
    static Target_t prev_target = {0};
    static uint8_t lost_counter = 0;
    const uint8_t MAX_LOST = 10;

    // --- CONFIGURATION ---
    const float BLIND_SPOT_DIST = 200.0f; // Si < 20cm, on active la protection
    const float BLIND_SPOT_FAKE_DIST = 50.0f; // Distance simulée pour forcer l'impact

    int8_t best_idx = -1;

    // --- ÉTAPE 1 : TRACKING ---
    // On essaie de retrouver le cluster qu'on suivait au tour précédent
    int8_t tracked_idx = -1;
    float min_diff_angle = 1000.0f;

    if (prev_target.detected) {
        for (int i = 0; i < cluster_count; i++) {
            if (!clusters[i].active) continue;

            float c_ang = clusters[i].angle_center;
            if (c_ang > 180.0f) c_ang -= 360.0f;

            float diff = fabsf(c_ang - prev_target.angle);
            if (diff > 180.0f) diff = 360.0f - diff;

            // Critères : Angle proche (+/- 30°) et Distance cohérente (+/- 20cm)
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
        // Cible retrouvée -> On reste dessus
        best_idx = tracked_idx;
        lost_counter = 0;
    }
    else {
        // Cible perdue ! Vérification ZONE AVEUGLE
        if (prev_target.detected && prev_target.dist < BLIND_SPOT_DIST) {
            // La cible était proche et elle a disparu -> Elle est collée à nous !
            // On force les valeurs pour l'attaque
            target->detected = 1;
            target->dist = BLIND_SPOT_FAKE_DIST;
            target->angle = prev_target.angle; // On garde le cap

            // Mise à jour mémoire pour le cycle suivant
            prev_target = *target;
            lost_counter = 0;
            return; // Fin immédiate
        }

        // Sinon, c'est une vraie perte (objet parti ou mouvement brusque)
        lost_counter++;

        if (lost_counter > MAX_LOST) {
            // Après X cycles perdus, on scanne tout pour trouver le nouveau plus proche
            float min_d = 100000.0f;
            for (int i = 0; i < cluster_count; i++) {
                if (!clusters[i].active) continue;
                if (clusters[i].dist_min < min_d) {
                    min_d = clusters[i].dist_min;
                    best_idx = i;
                }
            }
        } else {
            // Anti-clignotement : on ne fait rien, on garde l'état précédent implicitement
            // (Note: ici target n'est pas mis à jour, donc le main utilisera l'ancienne valeur si on ne reset pas detected)
            // Pour être propre, on peut renvoyer l'ancienne cible si < MAX_LOST
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

        target->detected = 1;
        target->dist = (float)clusters[best_idx].dist_min;
        target->angle = raw_ang;

        prev_target = *target;
        lost_counter = 0;
    } else {
        // Rien trouvé, et timeout écoulé
        target->detected = 0;
        prev_target.detected = 0;
    }
}
