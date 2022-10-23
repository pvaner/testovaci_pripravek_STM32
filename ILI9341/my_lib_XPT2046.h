/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <my_lib_ILI9341.h>
#include "main.h"
#include <stdbool.h>
/* Definitions ------------------------------------------------------------------*/
/* Variables ------------------------------------------------------------------*/
/* Functions ------------------------------------------------------------------*/
void XPT2046_Init(SPI_HandleTypeDef *touchSPI, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *irqPort, uint16_t irqPin);
bool XPT2046_TouchPressed();
bool XPT2046_TouchGetCoordinates(uint16_t* x, uint16_t* y);
uint8_t XPT2046_getOrientation(void);

