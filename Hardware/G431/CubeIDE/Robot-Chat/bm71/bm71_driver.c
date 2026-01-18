/* bm71_driver.c */
#include "bm71_driver.h"
#include "cmsis_os.h"
#include <string.h>
#include "stdio.h"

static bm71_t *g_bm = NULL;

static uint8_t calc_checksum(const uint8_t *data, uint16_t len)
{
	uint8_t sum = 0;
	for (uint16_t i = 0; i < len; i++) sum += data[i];
	return (uint8_t)(0x100 - sum);
}

static void start_rx_dma(bm71_t *bm)
{
	HAL_UART_Receive_DMA(bm->huart, bm->rx_dma_buf, BM71_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(bm->huart->hdmarx, DMA_IT_HT);
}

static void post_event(bm71_t *bm, bm71_event_type_t type, const uint8_t *data, uint16_t len)
{
	bm71_event_t evt = { .type = type, .len = len };
	if (len && data) {
		uint16_t cp = (len > BM71_MAX_PACKET_LEN) ? BM71_MAX_PACKET_LEN : len;
		memcpy(evt.data, data, cp);
	}
	xQueueSendFromISR(bm->event_queue, &evt, NULL);   // Safe from ISR
}

/* Tâche parser – priorité très basse */
void BLEParseTask(void *arg)
{
	bm71_t *bm = (bm71_t*)arg;
	uint8_t pkt[BM71_MAX_PACKET_LEN + 10];
	uint16_t pos = 0;

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // Attente notification depuis ISR

		uint16_t tail = BM71_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(bm->huart->hdmarx);

		while (bm->rx_head != tail) {
			uint8_t c = bm->rx_dma_buf[bm->rx_head];
			bm->rx_head = (bm->rx_head + 1) % BM71_RX_BUF_SIZE;

			if (bm->transparent_mode) {
				/* Détection trame cachée */
				if (c == 0xAA && pos == 0) {
					pkt[pos++] = c;
					continue;
				}
				if (pos > 0) {
					pkt[pos++] = c;
					if (pos >= 4) {
						uint16_t pkt_len = (pkt[1] << 8) | pkt[2];
						if (pos >= pkt_len + 1) {
							if (calc_checksum(&pkt[1], pkt_len) == pkt[pkt_len]) {
								post_event(bm, BM71_EVT_HIDDEN_COMMAND, pkt + 3, pkt_len - 2);
							}
							pos = 0;
						}
						if (pos >= sizeof(pkt)) pos = 0;
					}
				} else {
					post_event(bm, BM71_EVT_RAW_DATA, &c, 1);
				}
			} else {
				/* Mode commande au boot */
				if (c == 0xAA && pos == 0) pkt[pos++] = c;
				else if (pos > 0) {
					pkt[pos++] = c;
					if (pos >= 4) {
						uint16_t pkt_len = (pkt[1] << 8) | pkt[2];
						if (pos >= pkt_len + 1 && calc_checksum(&pkt[1], pkt_len) == pkt[pkt_len]) {
							uint8_t opcode = pkt[3];
							if (opcode == 0x71) { bm->connected = true;  post_event(bm, BM71_EVT_CONNECTED, NULL, 0); }
							if (opcode == 0x72) { bm->connected = false; post_event(bm, BM71_EVT_DISCONNECTED, NULL, 0); }
						}
						pos = 0;
					}
				}
			}
		}
	}
}

/* ISR DMA */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (g_bm && huart == g_bm->huart && g_bm->parser_task_handle) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(g_bm->parser_task_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* API */
bm71_t* bm71_create(UART_HandleTypeDef *huart, QueueHandle_t queue)
{
	static bm71_t bm = {0};
	bm.huart = huart;
	bm.event_queue = queue;
	g_bm = &bm;
	return &bm;
}

int bm71_write(bm71_t *bm, const uint8_t *data, uint16_t len)
{
	return (HAL_UART_Transmit_DMA(bm->huart, (uint8_t*)data, len) == HAL_OK) ? len : 0;
}

int bm71_send_hidden_cmd(bm71_t *bm, uint8_t opcode, const uint8_t *params, uint16_t len)
{
	uint8_t pkt[BM71_MAX_PACKET_LEN + 5];
	uint16_t total = 3 + len;
	pkt[0] = 0xAA;
	pkt[1] = total >> 8;
	pkt[2] = total & 0xFF;
	pkt[3] = opcode;
	if (len) memcpy(&pkt[4], params, len);
	pkt[4 + len] = calc_checksum(&pkt[1], total);
	return bm71_write(bm, pkt, total + 2);
}



/* ===================================================================
   Tâche shell intégrée au driver – plus jamais besoin de pointeur !
   =================================================================== */
void BLEShellTask(void *argument)
{
    bm71_t *bm = (bm71_t*)argument;

    char line[160];           // ← 160 au lieu de 128 = marge de sécurité
    uint16_t pos = 0;
    bm71_event_t evt;

    // Message de bienvenue au boot (facultatif)
    vTaskDelay(pdMS_TO_TICKS(1000));
    bm71_write(bm, (uint8_t*)"\r\n\r\n*** G431-BLE Shell v1.0 ***\r\n> ", 37);

    for(;;)  // ← BOUCLE INFINIE OBLIGATOIRE
    {
        if (xQueueReceive(bm->event_queue, &evt, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            if (evt.type == BM71_EVT_CONNECTED)
            {
                bm71_write(bm, (uint8_t*)"\r\n*** CONNECTED ***\r\n> ", 23);
                pos = 0;
            }
            else if (evt.type == BM71_EVT_RAW_DATA)
            {
                for (uint16_t i = 0; i < evt.len; i++)
                {
                    char c = evt.data[i];

                    if (c == '\r' || c == '\n')
                    {
                        if (pos == 0) continue;
                        line[pos] = '\0';

                        bm71_write(bm, (uint8_t*)"\r\n", 2);

                        if (strcmp(line, "reboot") == 0)
                        {
                            bm71_write(bm, (uint8_t*)"Bye !\r\n", 7);
                            vTaskDelay(200);
                            NVIC_SystemReset();
                        }
                        else if (strcmp(line, "help") == 0)
                        {
                            bm71_write(bm, (uint8_t*)"uptime  reboot  help\r\n> ", 23);
                        }
                        else
                        {
                            bm71_write(bm, (uint8_t*)"OK\r\n> ", 6);
                        }
                        pos = 0;
                    }
                    else if ((c == 8 || c == 127) && pos > 0)
                    {
                        pos--;
                        bm71_write(bm, (uint8_t*)"\b \b", 3);
                    }
                    else if (c >= 32 && c <= 126 && pos < sizeof(line)-1)
                    {
                        line[pos++] = c;
                        bm71_write(bm, (uint8_t*)&c, 1);
                    }
                }
            }
        }
        else
        {
            // timeout toutes les secondes → juste pour garder la tâche vivante
            // rien à faire
        }
    }
    // ← NE JAMAIS ARRIVER ICI
    // vTaskDelete(NULL);  ← À SUPPRIMER SI TU L’AVAIS MIS !
}

void bm71_start(bm71_t *bm)
{
	start_rx_dma(bm);

	/* Reset logiciel du BM71 */
	bm71_send_hidden_cmd(bm, 0x02, NULL, 0);

	/* Active transparent + adv (commandes envoyées tout de suite) */
	uint8_t trans[] = {0x0C, 0x01};
	bm71_send_hidden_cmd(bm, 0x40, trans, 2);
	uint8_t adv[] = {0x01};
	bm71_send_hidden_cmd(bm, 0x1C, adv, 1);

	bm->transparent_mode = true;
}

