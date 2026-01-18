/* bm71_driver.h */
#ifndef BM71_DRIVER_H
#define BM71_DRIVER_H

#include "stm32g4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

#define BM71_RX_BUF_SIZE      2048
#define BM71_MAX_PACKET_LEN   512

typedef enum {
    BM71_EVT_CONNECTED,
    BM71_EVT_DISCONNECTED,
    BM71_EVT_RAW_DATA,           // Données shell
    BM71_EVT_HIDDEN_COMMAND,     // Trame cachée IHM
} bm71_event_type_t;

typedef struct {
    bm71_event_type_t type;
    uint16_t          len;
    uint8_t           data[BM71_MAX_PACKET_LEN];
} bm71_event_t;

typedef struct {
    UART_HandleTypeDef *huart;
    QueueHandle_t       event_queue;

    uint8_t             rx_dma_buf[BM71_RX_BUF_SIZE];
    volatile uint16_t   rx_head;
    volatile uint16_t   rx_tail;

    TaskHandle_t        parser_task_handle;

    bool                connected;
    bool                transparent_mode;
} bm71_t;

/* API publique */
bm71_t* bm71_create(UART_HandleTypeDef *huart, QueueHandle_t queue);
void 	bm71_start(bm71_t *bm);
int     bm71_write(bm71_t *bm, const uint8_t *data, uint16_t len);
int     bm71_send_hidden_cmd(bm71_t *bm, uint8_t opcode, const uint8_t *params, uint16_t len);
void BLEShellTask(void *argument);
void BLEParseTask(void *argument); //Tache FreeRTOS

#endif
