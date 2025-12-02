// usb_vcp.h
#ifndef USB_VCP_H
#define USB_VCP_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdarg.h>

void     usb_vcp_init(void);                                            // appelé une fois dans main()
void     usb_vcp_task(void *argument);                                  // tâche à créer après vTaskStartScheduler()
bool     usb_vcp_line_ready(void);                                      // true si une ligne complète est prête
uint16_t usb_vcp_get_line(char *buf, uint16_t max_len);                 // récupère la ligne (bloquant ou non)
int      usb_vcp_printf(const char *fmt, ...);                          // printf sur le port USB
void     usb_vcp_send(const uint8_t *data, uint16_t len);               // envoi brut

#endif
