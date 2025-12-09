/*
 * lidar.c
 *
 * Created on: Nov 25, 2025
 * Author: Antle
 */

#include "capteurs/lidar.h"

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
    /* On utilise hlpuart1.hdmarx pour trouver le bon handle DMA */
    /* Attention : Assurez-vous que le DMA est bien activé en mode Circular dans CubeMX */
    return (DMA_BYTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(hlpuart1.hdmarx)) % DMA_BYTE_BUF_SIZE;
}


/* --- FONCTIONS PUBLIQUES --- */

/* Démarrage de la réception en mode Circulaire */
void YD_Start_UART_DMA(void)
{
    HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)dma_byte_buf, DMA_BYTE_BUF_SIZE);
}

/* * Parser Principal
 * Lit le buffer circulaire, cherche l'en-tête, extrait les données et remplit 'vue'
 */
void YD_Parser_MainLoop(Pos *vue_ptr, uint16_t dummy)
{
    UNUSED(dummy);
    static const uint32_t MIN_FRAME_SIZE = 10;

    // Où le DMA a-t-il fini d'écrire ?
    uint32_t write_idx = get_dma_write_index();

    // Tant qu'on n'a pas rattrapé le DMA
    while (dma_read_idx != write_idx)
    {
        // Calcul des octets disponibles
        uint32_t available = (write_idx + DMA_BYTE_BUF_SIZE - dma_read_idx) % DMA_BYTE_BUF_SIZE;

        // Pas assez de données pour un en-tête ? On sort.
        if (available < MIN_FRAME_SIZE) {
            return;
        }

        // Lecture des octets courants pour chercher AA 55
        uint8_t b0 = dma_byte_buf[dma_read_idx];
        uint8_t b1 = dma_byte_buf[(dma_read_idx + 1) % DMA_BYTE_BUF_SIZE];

        // Vérification Sync Header (0xAA 0x55)
        if (b0 != 0xAA || b1 != 0x55) {
            // Pas bon, on avance d'un octet et on réessaie
            dma_read_idx = (dma_read_idx + 1) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        // Récupération du nombre de samples (LSN)
        // Offset +3 par rapport au début (CT)
        uint8_t lsn = dma_byte_buf[(dma_read_idx + 3) % DMA_BYTE_BUF_SIZE];

        // Vérification cohérence LSN
        if (lsn == 0 || lsn > MAX_SAMPLES_PER_PKT) {
            // Header corrompu, on saute le header et on continue
            dma_read_idx = (dma_read_idx + 2) % DMA_BYTE_BUF_SIZE;
            continue;
        }

        // Calcul taille totale de la trame
        uint32_t frame_len = 10 + lsn * 2;

        // A-t-on reçu toute la trame ?
        if (available < frame_len) {
            return; // On attend la suite au prochain tour
        }

        /* --- TRAME VALIDE REÇUE --- */
        lidar_frame_count++; // Incrément pour le debug

        // On copie la trame dans un buffer linéaire temporaire pour simplifier le calcul
        uint8_t frame_buf[10 + MAX_SAMPLES_PER_PKT * 2];
        for (uint32_t i = 0; i < frame_len; i++) {
            frame_buf[i] = dma_byte_buf[(dma_read_idx + i) % DMA_BYTE_BUF_SIZE];
        }

        // Extraction Angle Début (FSA) et Fin (LSA)
        double start_deg = (((uint16_t)frame_buf[5] << 8 | ((uint16_t)frame_buf[4])) >> 1 ) / 64.0;
        double end_deg = (((uint16_t)frame_buf[7] << 8 | ((uint16_t)frame_buf[6])) >> 1 ) / 64.0;

        // Calcul pas angulaire
        double angle_diff = end_deg - start_deg;
        if (angle_diff < 0) {
            angle_diff += 360.0;
        }
        double step = angle_diff / (lsn > 1 ? (lsn - 1.0) : 1.0);

        // Boucle sur chaque échantillon
        for (uint16_t i = 0; i < lsn; i++) {
            uint16_t raw_dist = (uint16_t)frame_buf[10 + i * 2] | ((uint16_t)frame_buf[11 + i * 2] << 8);
            uint16_t distance_mm = raw_dist / 4;

            // Correction angulaire spécifique YDLIDAR X4
            double correction_angle = 0;
            if(distance_mm > 0) {
                correction_angle = atanf(21.8f*((155.3f - distance_mm)/(155.3f + distance_mm)));
            }

            double angle = start_deg + step * i;
            double angle_corr_deg = angle + (correction_angle * 180.0 / M_PI);

            if (angle_corr_deg >= 360.0) angle_corr_deg -= 360.0;
            if (angle_corr_deg < 0) angle_corr_deg += 360.0;

            // Conversion en radians pour X,Y
            double rad = (angle - 90.0f) * M_PI / 180.0f; // Convention robotique standard

            // Calcul Index tableau
            uint16_t idx = (uint16_t)(angle_corr_deg * FACTEUR);
            if (idx >= N_ANGLES) idx = 0; // Sécurité

            // Filtrage et stockage
            if (distance_mm > 0 && distance_mm < MAX_RANGE_LIDAR) {
                if (vue_ptr[idx].distance == 0){
                    compteur++;
                    remplissage = (float)compteur / 360.0f;
                }

                float X1 = -cos(-rad) * distance_mm;
                float Y1 = sin(-rad) * distance_mm;

                // Filtre zone rectangle
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

        // On a fini de traiter cette trame, on avance l'index de lecture
        dma_read_idx = (dma_read_idx + frame_len) % DMA_BYTE_BUF_SIZE;
    }
}

/*
 * Clustering
 * Calcule des objets (clusters) à partir des points bruts
 */
int compute_cluster(Pos *pts, uint16_t start, uint16_t end, uint8_t idx) {
    float sum_x = 0.0f, sum_y = 0.0f;
    uint16_t count = 0;

    for (uint16_t i = start; i <= end; i++) {
        uint16_t d = (uint16_t)pts[i].distance;
        if (d < MIN_VALID_DISTANCE) continue;

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

    if (hypotf(cx, cy) < MIN_CLUSTER_DIST) return 0;

    // Fusion avec clusters existants
    for (uint8_t k = 0; k < cluster_count; k++) {
        if (!clusters[k].active) continue;
        float dx = clusters[k].x - cx;
        float dy = clusters[k].y - cy;
        float dist2 = dx*dx + dy*dy;
        if (dist2 < (MERGE_THRESHOLD * MERGE_THRESHOLD)) {
            uint32_t new_size = (uint32_t)clusters[k].size + (uint32_t)count;

            // Barycentre pondéré
            float target_x = (clusters[k].x * clusters[k].size + cx * count) / (float)new_size;
            float target_y = (clusters[k].y * clusters[k].size + cy * count) / (float)new_size;

            // Lissage
            #define SMOOTH_FACTOR 0.1f
            clusters[k].x = clusters[k].x * (1.0f - SMOOTH_FACTOR) + target_x * SMOOTH_FACTOR;
            clusters[k].y = clusters[k].y * (1.0f - SMOOTH_FACTOR) + target_y * SMOOTH_FACTOR;

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
    clusters[idx].active    = 1;
    return 1;
}

void segment_points(Pos *pts, uint16_t n_points) {
    // Reset partiel ou total des clusters
    memset(clusters, 0, sizeof(clusters));
    cluster_count = 0;

    uint16_t start = 0;
    for (uint16_t i = 0; i < n_points - 1; i++) {
        uint16_t d1 = (uint16_t)pts[i].distance;
        uint16_t d2 = (uint16_t)pts[i+1].distance;

        // Rupture si point invalide
        if (d1 < MIN_VALID_DISTANCE || d2 < MIN_VALID_DISTANCE) {
            if (start <= i) {
                if (cluster_count < MAX_CLUSTERS) {
                    if (compute_cluster(pts, start, i, cluster_count)) cluster_count++;
                }
            }
            start = i + 1;
            continue;
        }

        // Rupture si distance trop grande entre deux points (saut de profondeur)
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

    // Dernier segment
    if (start < n_points) {
        if (cluster_count < MAX_CLUSTERS) {
            if (compute_cluster(pts, start, n_points - 1, cluster_count)) cluster_count++;
        }
    }
}
