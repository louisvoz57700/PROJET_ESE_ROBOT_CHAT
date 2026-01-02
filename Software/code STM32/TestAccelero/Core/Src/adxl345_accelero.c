/*
 * adxl345_accelero.c
 *
 *  Created on: Oct 1, 2025
 *      Author: kennystflr
 */


// Fichier contenant les fonctions et variables concernant l'accelerometre adxl345
// Module autonome pour STM32 avec FreeRTOS, DMA et calibration

#include "adxl345.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>

// Définir M_PI si non défini
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Taille maximale du buffer pour la lecture FIFO (32 échantillons max * 6 bytes)
#define ADXL345_FIFO_MAX_SAMPLES 32
#define ADXL345_FIFO_BUFFER_SIZE (ADXL345_FIFO_MAX_SAMPLES * 6)

// Buffer global pour la réception DMA
static uint8_t adxl345_rx_buffer[ADXL345_FIFO_BUFFER_SIZE + 1];  // +1 pour le byte de commande

// Flag pour indiquer la fin de transfert DMA
static volatile uint8_t adxl345_dma_complete = 0;

// Semaphores pour les tâches
static SemaphoreHandle_t tapSemaphore;
static SemaphoreHandle_t fifoSemaphore;

// Structure pour stocker les données calculées (roll et pitch)
typedef struct {
    float roll;  // En degrés
    float pitch; // En degrés
} ADXL345_AxesData;

static ADXL345_AxesData adxl345_axes_data = {0.0f, 0.0f};

// Structure pour stocker les événements de tap
typedef struct {
    uint8_t single_tap_detected;
    uint8_t double_tap_detected;
} ADXL345_TapData;

static ADXL345_TapData adxl345_tap_data = {0, 0};

// Offsets de calibration (en g)
static float adxl345_ax_offset = 0.0f;
static float adxl345_ay_offset = 0.0f;
static float adxl345_az_offset = 0.0f;

// Callback pour la fin de transfert SPI DMA
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi1) {
        ADXL345_CS_HIGH();
        adxl345_dma_complete = 1;
    }
}

// Callback EXTI pour interruptions ADXL345
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == ADXL345_INT1_Pin) {  // INT1 pour taps
        xSemaphoreGiveFromISR(tapSemaphore, &xHigherPriorityTaskWoken);
    } else if (GPIO_Pin == ADXL345_INT2_Pin) {  // INT2 pour watermark
        xSemaphoreGiveFromISR(fifoSemaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Écriture d'un registre
static void ADXL345_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = {reg & 0x3F, value};  // MSB=0 pour écriture
    ADXL345_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);
    ADXL345_CS_HIGH();
}

// Lecture d'un registre (ou plusieurs)
static void ADXL345_ReadReg(uint8_t reg, uint8_t* buffer, uint8_t len) {
    uint8_t tx_data[ADXL345_FIFO_BUFFER_SIZE + 1] = {0};
    tx_data[0] = (reg & 0x3F) | 0x80;  // MSB=1 pour lecture
    if (len > 1) tx_data[0] |= 0x40;   // Multi-byte
    ADXL345_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_data, buffer, len + 1, HAL_MAX_DELAY);
    ADXL345_CS_HIGH();
    // Décalage pour ignorer byte de commande
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = buffer[i + 1];
    }
}

// Lecture asynchrone via DMA pour les données FIFO
static void ADXL345_ReadFIFO_DMA(uint8_t* buffer, uint16_t len) {
    uint8_t cmd = (0x32 & 0x3F) | 0x80 | 0x40;  // Lecture multi-byte depuis DATAX0
    uint8_t tx_dummy[ADXL345_FIFO_BUFFER_SIZE + 1] = {0};
    tx_dummy[0] = cmd;
    adxl345_dma_complete = 0;
    ADXL345_CS_LOW();
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_dummy, buffer, len + 1);
}

// Attendre la fin du transfert DMA et décaler les données
static void ADXL345_WaitDMA(uint16_t len) {
    while (!adxl345_dma_complete) {
        vTaskDelay(1);  // Attendre 1 tick pour éviter busy-wait
    }
    // Décalage pour ignorer byte de commande
    for (uint16_t i = 0; i < len; i++) {
        adxl345_rx_buffer[i] = adxl345_rx_buffer[i + 1];
    }
}

// Obtenir le nombre d'échantillons dans la FIFO
static uint8_t ADXL345_GetFIFOSamples(void) {
    uint8_t status;
    ADXL345_ReadReg(0x39, &status, 1);  // FIFO_STATUS (0x39)
    return status & 0x3F;
}

// Nouvelle fonction : Calibration des axes
// Prend num_samples lectures, calcule les offsets en g
void ADXL345_Calibrate(uint8_t num_samples) {
    if (num_samples == 0) num_samples = 100;  // Valeur par défaut pour une bonne moyenne

    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    uint8_t raw_data[6];  // 6 bytes pour X,Y,Z (2 bytes chacun)

    // Collecter num_samples échantillons
    for (uint8_t i = 0; i < num_samples; i++) {
        // Lecture polling des 6 bytes (DATAX0 à DATAZ1)
        ADXL345_ReadReg(0x32, raw_data, 6);

        // Convertir en int16_t (little-endian)
        int16_t ax_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
        int16_t ay_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
        int16_t az_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

        // Convertir en g et sommer
        sum_ax += (float)ax_raw * ADXL345_SENSITIVITY_G_PER_LSB;
        sum_ay += (float)ay_raw * ADXL345_SENSITIVITY_G_PER_LSB;
        sum_az += (float)az_raw * ADXL345_SENSITIVITY_G_PER_LSB;

        // Petit délai pour éviter de surcharger le SPI (optionnel)
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms entre échantillons
    }

    // Calculer les moyennes et offsets
    float avg_ax = sum_ax / (float)num_samples;
    float avg_ay = sum_ay / (float)num_samples;
    float avg_az = sum_az / (float)num_samples;

    adxl345_ax_offset = avg_ax - 0.0f;  // Attendu : 0g sur X
    adxl345_ay_offset = avg_ay - 0.0f;  // Attendu : 0g sur Y
    adxl345_az_offset = avg_az - 1.0f;  // Attendu : 1g sur Z (gravité)

    // Optionnel : Debug (remplacez par votre logger)
    // printf("Calibration: ax_offset=%.3fg, ay_offset=%.3fg, az_offset=%.3fg\n",
    //        adxl345_ax_offset, adxl345_ay_offset, adxl345_az_offset);
}

// Tâche pour gérer les axes, roll et pitch (mise à jour avec calibration)
static void AxesTask(void *pvParameters) {
    while (1) {
        xSemaphoreTake(fifoSemaphore, portMAX_DELAY);

        uint8_t int_source;
        ADXL345_ReadReg(0x30, &int_source, 1);  // INT_SOURCE

        if (int_source & 0x02) {  // Watermark interrupt
            uint8_t num_samples = ADXL345_GetFIFOSamples();
            if (num_samples > 0) {
                uint16_t len = num_samples * 6;
                ADXL345_ReadFIFO_DMA(adxl345_rx_buffer, len);
                ADXL345_WaitDMA(len);

                // Traiter le dernier échantillon
                uint8_t *last = &adxl345_rx_buffer[len - 6];
                int16_t ax_raw = (int16_t)((last[1] << 8) | last[0]);
                int16_t ay_raw = (int16_t)((last[3] << 8) | last[2]);
                int16_t az_raw = (int16_t)((last[5] << 8) | last[4]);

                // Convertir en g et appliquer les offsets de calibration
                float ax_g = ((float)ax_raw * ADXL345_SENSITIVITY_G_PER_LSB) - adxl345_ax_offset;
                float ay_g = ((float)ay_raw * ADXL345_SENSITIVITY_G_PER_LSB) - adxl345_ay_offset;
                float az_g = ((float)az_raw * ADXL345_SENSITIVITY_G_PER_LSB) - adxl345_az_offset;

                // Calculer roll et pitch avec les valeurs corrigées
                adxl345_axes_data.roll = atan2(ay_g, az_g) * 180.0f / (float)M_PI;
                adxl345_axes_data.pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / (float)M_PI;
            }
        }
    }
}

// Tâche pour gérer les interruptions de tap (inchangée)
static void TapTask(void *pvParameters) {
    while (1) {
        xSemaphoreTake(tapSemaphore, portMAX_DELAY);

        uint8_t int_source;
        ADXL345_ReadReg(0x30, &int_source, 1);  // INT_SOURCE

        adxl345_tap_data.single_tap_detected = (int_source & 0x40) ? 1 : 0;  // Single tap
        adxl345_tap_data.double_tap_detected = (int_source & 0x20) ? 1 : 0;  // Double tap
    }
}

// Initialisation de l'ADXL345 (inchangée, mais vous pouvez ajouter calibration ici si désiré)
void ADXL345_Init(void) {
    // Mode mesure
    ADXL345_WriteReg(0x2D, 0x08);

    // Format données : full resolution, ±16g
    ADXL345_WriteReg(0x31, 0x0B);

    // Activer tap detection sur 3 axes
    ADXL345_WriteReg(0x2A, 0x07);

    // Seuil tap (~3g)
    ADXL345_WriteReg(0x1D, 0x30);

    // Durée tap (~10ms)
    ADXL345_WriteReg(0x21, 0x10);

    // Configurer FIFO en mode stream avec watermark à 16 échantillons
    ADXL345_WriteReg(0x38, 0x90);

    // Mapper interruptions : watermark à INT2, taps à INT1
    ADXL345_WriteReg(0x2F, 0x02);

    // Activer interruptions single/double tap et watermark
    ADXL345_WriteReg(0x2E, 0x62);

    // Créer semaphores
    tapSemaphore = xSemaphoreCreateBinary();
    fifoSemaphore = xSemaphoreCreateBinary();

    // Créer tâches
    xTaskCreate(AxesTask, "AxesTask", 256, NULL, 2, NULL);
    xTaskCreate(TapTask, "TapTask", 256, NULL, 2, NULL);

    // Optionnel : Calibrer automatiquement après init (décommentez si désiré)
    // ADXL345_Calibrate(100);
}

// Fonctions publiques pour accéder aux données (inchangées)
ADXL345_AxesData ADXL345_GetAxesData(void) {
    ADXL345_AxesData data;
    taskENTER_CRITICAL();
    data = adxl345_axes_data;
    taskEXIT_CRITICAL();
    return data;
}

ADXL345_TapData ADXL345_GetTapData(void) {
    ADXL345_TapData data;
    taskENTER_CRITICAL();
    data = adxl345_tap_data;
    // Réinitialiser les flags après lecture
    adxl345_tap_data.single_tap_detected = 0;
    adxl345_tap_data.double_tap_detected = 0;
    taskEXIT_CRITICAL();
    return data;
}
