// usb_vcp.c
#include "usb_vcp.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#define USB_RX_BUFFER_SIZE   256
#define USB_LINE_BUFFER_SIZE 128
#define HISTORY_SIZE   10
#define HISTORY_LEN    USB_LINE_BUFFER_SIZE
// Macro de fonction ECHO
#define ECHO_HEX(ch) do { \
    static const char hex[] = "0123456789ABCDEF"; \
    uint8_t c = (uint8_t)(ch); \
    uint8_t msg[5] = "0x?? "; \
    msg[2] = hex[(c >> 4) & 0xF]; \
    msg[3] = hex[c & 0xF]; \
    usb_vcp_send(msg, 5); \
} while(0)

#define ECHO_CHAR(ch)   usb_vcp_send((const uint8_t[]){ (uint8_t)(ch) }, 1)

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t ADXL345_Read(uint8_t reg);

static uint8_t  usb_rx_buffer[USB_RX_BUFFER_SIZE];
static char     line_buffer[USB_LINE_BUFFER_SIZE];
static uint16_t line_pos = 0;
static bool     line_ready = false;
static char history[HISTORY_SIZE][HISTORY_LEN];
static int  history_count = 0;
static int  history_pos   = 0;   // position actuelle dans l'historique

static QueueHandle_t line_queue = NULL;
static QueueHandle_t vcp_event_queue = NULL;

typedef enum { EVT_CHAR, EVT_ENTER, EVT_BACKSPACE, EVT_ARROW_UP, EVT_ARROW_DOWN } vcp_evt_t;
typedef struct { vcp_evt_t type; char c; } vcp_event_t;



static void history_add(const char *line)
{
	if (!line[0]) return;
	if (history_count > 0 && strcmp(history[(history_count-1)%HISTORY_SIZE], line) == 0) return;
	strncpy(history[history_count % HISTORY_SIZE], line, HISTORY_LEN-1);
	history[history_count % HISTORY_SIZE][HISTORY_LEN-1] = '\0';
	history_count++;
	history_pos = history_count;
}

static const char* history_get(int offset)
{
	int pos = history_pos + offset;
	if (pos < 0) pos = 0;
	if (pos >= history_count) { history_pos = history_count; return ""; }
	history_pos = pos;
	return history[pos % HISTORY_SIZE];
}

/* Fonction appelée par la stack USB CDC quand des données arrivent */
int8_t usb_vcp_data_received(uint8_t* Buf, uint32_t *Len)
{
	vcp_event_t evt;

	for (uint32_t i = 0; i < *Len; i++)
	{
		uint8_t c = Buf[i];

		if (c == '\r' || c == '\n')
		{
			evt.type = EVT_ENTER;
			xQueueSendFromISR(vcp_event_queue, &evt, NULL);
		}
		else if (c == 8 || c == 127)  // Backspace
		{
			evt.type = EVT_BACKSPACE;
			xQueueSendFromISR(vcp_event_queue, &evt, NULL);
		}
		else if (c == '\e' && i+2 < *Len && Buf[i+1] == '[')
		{
			i += 2;
			if (Buf[i] == 'A')      { evt.type = EVT_ARROW_UP;   xQueueSendFromISR(vcp_event_queue, &evt, NULL); }
			if (Buf[i] == 'B')      { evt.type = EVT_ARROW_DOWN; xQueueSendFromISR(vcp_event_queue, &evt, NULL); }
		}
		else if (c >= 32 && c <= 126)
		{
			evt.type = EVT_CHAR;
			evt.c = c;
			xQueueSendFromISR(vcp_event_queue, &evt, NULL);
		}
	}

	// Réarme la réception (très important !)
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_rx_buffer);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);

	return (USBD_OK);
}

/* À appeler une seule fois dans main() après MX_USB_DEVICE_Init() */
void usb_vcp_init(void)
{
	line_queue = xQueueCreate(8, USB_LINE_BUFFER_SIZE);
	vcp_event_queue = xQueueCreate(32, sizeof(vcp_event_t));
}
/* Tâche qui fait vivre le VCP  */
void usb_vcp_task(void *argument)
{
	vcp_event_t evt;
	usb_vcp_printf("\r\n=== STM32G431 USB Shell ===\r\n> ");

	for(;;)
	{
		if (xQueueReceive(vcp_event_queue, &evt, portMAX_DELAY) == pdPASS)
		{
			switch (evt.type)
			{
			case EVT_CHAR:
				if (line_pos < USB_LINE_BUFFER_SIZE-1)
				{
					line_buffer[line_pos++] = evt.c;
					ECHO_CHAR(evt.c);  // écho local
				}
				break;

			case EVT_BACKSPACE:
				if (line_pos > 0)
				{
					line_pos--;
					usb_vcp_send((uint8_t*)"\b \b", 3);
				}
				break;

			case EVT_ARROW_UP:
			case EVT_ARROW_DOWN:
			{
				const char *h = history_get(evt.type == EVT_ARROW_UP ? -1 : +1);
				while (line_pos > 0) { usb_vcp_send((uint8_t*)"\b \b", 3); line_pos--; }
				strcpy(line_buffer, h);
				line_pos = strlen(h);
				usb_vcp_send((uint8_t*)line_buffer, line_pos);
				break;
			}

			case EVT_ENTER:
				line_buffer[line_pos] = '\0';
				usb_vcp_printf("\r\n");
				if (line_pos > 0)
				{
					history_add(line_buffer);
					// === ICI LES COMMANDES ===
					if (strcmp(line_buffer, "reboot") == 0)
					{
						usb_vcp_printf("Rebooting...\r\n");
						vTaskDelay(100);
						NVIC_SystemReset();
					}
					else if (strcmp(line_buffer, "help") == 0)
					{
						usb_vcp_printf("help  reboot  status\r\n");
					}
					else if (strcmp(line_buffer, "BT_Toogle") == 0)
					{
						HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
						usb_vcp_printf("BlueTooth State : %d\r\n", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10));
					}
					else if (strcmp(line_buffer, "ACC_Status") == 0)
					{
						usb_vcp_printf("Read Device ID : 0x02%x \r\n", ADXL345_Read(0x00) );
						vTaskDelay(5);
						usb_vcp_printf("INT_ENABLE = 0x%02X  (doit être 0x40)\r\n", ADXL345_Read(0x2E));
						vTaskDelay(5);
						usb_vcp_printf("INT_MAP    = 0x%02X  (bit6 doit être 0 → INT1)\r\n", ADXL345_Read(0x2F));
						vTaskDelay(5);
						usb_vcp_printf("INT_SOURCE = 0x%02X  (0x40 = Single Tap détecté)\r\n", ADXL345_Read(0x30));
					}
					else
					{
						usb_vcp_printf("OK\r\n");
					}
				}
				usb_vcp_printf("%d > ",line_pos);
				line_pos = 0;
				break;
			}
		}
	}
}


/* Fonctions utilitaires */
bool usb_vcp_line_ready(void)
{
	return line_ready;
}

uint16_t usb_vcp_get_line(char *buf, uint16_t max_len)
{
	if (xQueueReceive(line_queue, buf, 0) == pdPASS)
	{
		line_ready = false;
		return strlen(buf);
	}
	return 0;
}

/* Fonction d’envoi normal (utilisée par usb_vcp_printf) → bypass l’écho hexa */
void usb_vcp_send_raw(const uint8_t *data, uint16_t len)
{
    CDC_Transmit_FS((uint8_t*)data, len);
}

/* Remplace toutes les usb_vcp_send() normales par usb_vcp_send_raw() dans les printf */
int usb_vcp_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) usb_vcp_send_raw((const uint8_t*)buf, len);
    return len;
}

void usb_vcp_send(const uint8_t *data, uint16_t len)
{
	CDC_Transmit_FS((uint8_t*)data, len);
}
