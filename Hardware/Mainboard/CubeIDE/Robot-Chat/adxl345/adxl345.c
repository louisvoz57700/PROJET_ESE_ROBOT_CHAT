#include "adxl345.h"
#include "stm32g4xx_hal.h"


// ---------------------------------------------------------------------------
// Registres ADXL345
// ---------------------------------------------------------------------------
#define REG_DEVID           0x00
#define REG_POWER_CTL       0x2D
#define REG_DATA_FORMAT     0x31
#define REG_INT_ENABLE      0x2E
#define REG_INT_MAP         0x2F
#define REG_INT_SOURCE      0x30
#define REG_THRESH_TAP      0x1D
#define REG_DUR             0x21
#define REG_LATENT          0x22
#define REG_WINDOW          0x23
#define REG_TAP_AXES        0x2A

#define INT_SINGLE_TAP      (1<<6)
#define INT_DOUBLE_TAP      (1<<5)

QueueHandle_t adxlQueue;

// ---------------------------------------------------------------------------
// Structure interne du driver
// ---------------------------------------------------------------------------
struct ADXL345_Device_t {
    ADXL_BusType_t      bus_type;

    // I2C
    I2C_HandleTypeDef   *hi2c;
    uint16_t            i2c_addr;

    // SPI
    SPI_HandleTypeDef   *hspi;
    GPIO_TypeDef        *cs_port;
    uint16_t            cs_pin;

    // Interruptions
    GPIO_TypeDef        *int1_port;
    uint16_t            int1_pin;
    GPIO_TypeDef        *int2_port;
    uint16_t            int2_pin;

    // Callback utilisateur
    ADXL_TapCallback_t  callback;
};

// ---------------------------------------------------------------------------
// Fonctions privées communes
// ---------------------------------------------------------------------------
static void cs_low(ADXL345_Handle_t *dev)  { if(dev->cs_port) HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static void cs_high(ADXL345_Handle_t *dev) { if(dev->cs_port) HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

static int write_reg(ADXL345_Handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    if (dev->bus_type == ADXL_BUS_I2C) {
        return (HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, buf, 2, 100) == HAL_OK) ? 0 : -1;
    } else {
        cs_low(dev);
        int ret = (HAL_SPI_Transmit(dev->hspi, buf, 2, 100) == HAL_OK) ? 0 : -1;
        cs_high(dev);
        return ret;
    }
}

static int read_reg(ADXL345_Handle_t *dev, uint8_t reg, uint8_t *val)
{
    if (dev->bus_type == ADXL_BUS_I2C) {
        if (HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr << 1, &reg, 1, 100) != HAL_OK) return -1;
        return (HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr << 1, val, 1, 100) == HAL_OK) ? 0 : -1;
    } else {
        uint8_t tx[2] = { reg | 0xC0, 0x00 };  // SPI : bit7=1 (read), bit6=1 (MB)
        uint8_t rx[2];
        cs_low(dev);
        int ret = (HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100) == HAL_OK) ? 0 : -1;
        cs_high(dev);
        if (ret == 0) *val = rx[1];
        return ret;
    }
}

// ---------------------------------------------------------------------------
// API publique
// ---------------------------------------------------------------------------
ADXL345_Handle_t* ADXL345_Create_I2C(I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
    ADXL345_Handle_t *dev = (ADXL345_Handle_t*)pvPortMalloc(sizeof(ADXL345_Handle_t));
    if (!dev) return NULL;

    dev->bus_type = ADXL_BUS_I2C;
    dev->hi2c = hi2c;
    dev->i2c_addr = dev_addr;
    dev->callback = NULL;

    return dev;
}

ADXL345_Handle_t* ADXL345_Create_SPI(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     GPIO_TypeDef *int1_port, uint16_t int1_pin,
                                     GPIO_TypeDef *int2_port, uint16_t int2_pin)
{
    ADXL345_Handle_t *dev = (ADXL345_Handle_t*)pvPortMalloc(sizeof(ADXL345_Handle_t));
    if (!dev) return NULL;

    dev->bus_type = ADXL_BUS_SPI;
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->int1_port = int1_port;
    dev->int1_pin = int1_pin;
    dev->int2_port = int2_port;
    dev->int2_pin = int2_pin;
    dev->callback = NULL;

    // CS haute au repos
    cs_high(dev);

    return dev;
}

void ADXL345_Delete(ADXL345_Handle_t *dev)
{
    if (dev) vPortFree(dev);
}

int ADXL345_Init(ADXL345_Handle_t *dev)
{
    uint8_t id;
    write_reg(REG_DATA_FORMAT, 0x08);     // ±4g, full res
	write_reg(REG_THRESH_TAP,   0x35);     // ~3.3g → ajuste 0x30-0x50 selon sensibilité voulue
	write_reg(REG_DUR,          0x30);     // 30 ms max
	write_reg(0x2B,             0x00);     // Window = 0 → désactive double tap
	write_reg(REG_TAP_AXES,     0x07);     // X+Y+Z
	write_reg(REG_INT_MAP,      0x00);     // Single Tap → INT1
	write_reg(REG_INT_ENABLE,   0x40);     // active uniquement Single Tap
	write_reg(REG_DATA_FORMAT,   0x40);     // active uniquement Single Tap
	write_reg(REG_POWER_CTL,    0x08);     // mode mesure

    if (read_reg(dev, REG_DEVID, &id) != 0 || id != 0xE5)
        return -1;


    return 0;
}

int ADXL345_ConfigTap(ADXL345_Handle_t *dev,
                      uint8_t thresh_tap, uint8_t dur,
                      uint8_t latent, uint8_t window,
                      bool enable_x, bool enable_y, bool enable_z,
                      bool enable_single, bool enable_double)
{
    write_reg(dev, REG_THRESH_TAP, thresh_tap);
    write_reg(dev, REG_DUR,        dur);
    write_reg(dev, REG_LATENT,     latent);
    write_reg(dev, REG_WINDOW,     window);

    uint8_t axes = (enable_z << 2) | (enable_y << 1) | (enable_x << 0);
    if (!enable_single) axes |= 0x08;  // SUPPRESS bit
    write_reg(dev, REG_TAP_AXES, axes);

    // On mappe Single Tap sur INT1 et Double Tap sur INT2 (ou les deux sur INT1 si tu veux)
    write_reg(dev, REG_INT_MAP, enable_double ? INT_DOUBLE_TAP : 0); // 0 = INT1, bit=1 → INT2

    uint8_t int_en = 0;
    if (enable_single) int_en |= INT_SINGLE_TAP;
    if (enable_double) int_en |= INT_DOUBLE_TAP;
    write_reg(dev, REG_INT_ENABLE, int_en);

    return 0;
}

void ADXL345_RegisterCallback(ADXL345_Handle_t *dev, ADXL_TapCallback_t cb)
{
    dev->callback = cb;
}

// ---------------------------------------------------------------------------
// ISR communes
// ---------------------------------------------------------------------------
void ADXL345_INT1_ISR(ADXL345_Handle_t *dev)
{
    if (dev->callback) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        dev->callback(ADXL_SINGLE_TAP, xTaskGetTickCountFromISR());
        // Optionnel : clear latch si tu utilises Activity/Inactivity
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void ADXL345_INT2_ISR(ADXL345_Handle_t *dev)
{
    if (dev->callback) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        dev->callback(ADXL_DOUBLE_TAP, xTaskGetTickCountFromISR());
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//// ---------------------------------------------------------------------------
//// Exemple d'ISR réelles (à adapter selon tes pins)
//// ---------------------------------------------------------------------------
///* Exemple si INT1 sur PC6 (EXTI6) et INT2 sur PC7 (EXTI7) */
//void EXTI9_5_IRQHandler(void)
//{
//    extern ADXL345_Handle_t *g_adxl;  // déclaré dans main.c
//
//    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != 0x00u) {
//        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
//        ADXL345_INT1_ISR(g_adxl);
//    }
//    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != 0x00u) {
//        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
//        ADXL345_INT2_ISR(g_adxl);
//    }
//}
