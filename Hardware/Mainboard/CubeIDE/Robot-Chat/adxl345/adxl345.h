#ifndef ADXL345_H
#define ADXL345_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "cmsis_os.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"

// ---------------------------------------------------------------------------
// Type de bus
// ---------------------------------------------------------------------------
typedef enum {
    ADXL_BUS_I2C = 0,
    ADXL_BUS_SPI = 1
} ADXL_BusType_t;

// ---------------------------------------------------------------------------
// Callback utilisateur
// ---------------------------------------------------------------------------
typedef enum {
    ADXL_NO_TAP     = 0,
    ADXL_SINGLE_TAP = 1,
    ADXL_DOUBLE_TAP = 2
} ADXL_TapType_t;

// ---------------------------------------------------------------------------
// Callback utilisateur
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  type;        // 1=single tap, 2=double tap, 3=freefall, 4=activity
    uint8_t  axis;        // bit0=X, bit1=Y, bit2=Z
    bool     positive;    // direction du choc
    int16_t  ax, ay, az;  // acceleration brute au moment du choc
    float    pitch, roll; // inclinaison instantanée
    uint32_t timestamp;
} RichTapEvent_t;

typedef void (*ADXL_TapCallback_t)(ADXL_TapType_t tap_type, uint32_t timestamp);

// ---------------------------------------------------------------------------
// Handle opaque
// ---------------------------------------------------------------------------
typedef struct ADXL345_Device_t ADXL345_Handle_t;

// ---------------------------------------------------------------------------
// Création du périphérique (I2C ou SPI)
// ---------------------------------------------------------------------------
ADXL345_Handle_t* ADXL345_Create_I2C(I2C_HandleTypeDef *hi2c, uint16_t dev_addr);
ADXL345_Handle_t* ADXL345_Create_SPI(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     GPIO_TypeDef *int1_port, uint16_t int1_pin,
                                     GPIO_TypeDef *int2_port, uint16_t int2_pin);

// Destruction
void ADXL345_Delete(ADXL345_Handle_t *dev);

// Init + config tap
int ADXL345_Init(ADXL345_Handle_t *dev);
int ADXL345_ConfigTap(ADXL345_Handle_t *dev,
                      uint8_t thresh_tap, uint8_t dur,
                      uint8_t latent, uint8_t window,
                      bool enable_x, bool enable_y, bool enable_z,
                      bool enable_single, bool enable_double);

// Enregistrement callback (appelé depuis ISR !)
void ADXL345_RegisterCallback(ADXL345_Handle_t *dev, ADXL_TapCallback_t cb);

// Démarre la surveillance (à appeler une seule fois)
void ADXL345_Start(ADXL345_Handle_t *dev);

// Utilitaires
uint8_t ADXL345_GetDeviceID(ADXL345_Handle_t *dev);

#endif // ADXL345_H
