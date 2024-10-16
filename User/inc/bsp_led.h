#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "main.h"

typedef enum
{
	yellowLed = 0,
}X_LED_t;


void bsp_InitLed(void);
void bsp_LedOn(uint8_t _no);
void bsp_LedOff(uint8_t _no);
void bsp_LedToggle(uint8_t _no);
	
#endif /* __LED_H */