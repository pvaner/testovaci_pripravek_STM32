/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* Definitions ------------------------------------------------------------------*/
#define RESET_ACTIVE() HAL_GPIO_WritePin(tftRESET_GPIO,tftRESET_PIN,GPIO_PIN_RESET)
#define RESET_IDLE() HAL_GPIO_WritePin(tftRESET_GPIO,tftRESET_PIN,GPIO_PIN_SET)
#define CS_ACTIVE() HAL_GPIO_WritePin(tftCS_GPIO,tftCS_PIN,GPIO_PIN_RESET)
#define CS_IDLE() HAL_GPIO_WritePin(tftCS_GPIO,tftCS_PIN,GPIO_PIN_SET)
#define DC_COMMAND() HAL_GPIO_WritePin(tftDC_GPIO,tftDC_PIN,GPIO_PIN_RESET)
#define DC_DATA() HAL_GPIO_WritePin(tftDC_GPIO,tftDC_PIN,GPIO_PIN_SET)
//seznam barev
#define COLOR_BLACK           0x0000
#define COLOR_NAVY            0x000F
#define COLOR_DGREEN          0x03E0
#define COLOR_DCYAN           0x03EF
#define COLOR_MAROON          0x7800
#define COLOR_PURPLE          0x780F
#define COLOR_OLIVE           0x7BE0
#define COLOR_LGRAY           0xC618
#define COLOR_DGRAY           0x7BEF
#define COLOR_BLUE            0x001F
#define COLOR_BLUE2			  0x051D
#define COLOR_GREEN           0x07E0
#define COLOR_GREEN2		  0xB723
#define COLOR_GREEN3		  0x8000
#define COLOR_CYAN            0x07FF
#define COLOR_RED             0xF800
#define COLOR_MAGENTA         0xF81F
#define COLOR_YELLOW          0xFFE0
#define COLOR_WHITE           0xFFFF
#define COLOR_ORANGE          0xFD20
#define COLOR_GREENYELLOW     0xAFE5
#define COLOR_BROWN 		  0XBC40
#define COLOR_BRRED 		  0XFC07
//-------------------------------------------------------------------
#define swap(a,b) {int16_t t=a;a=b;b=t;}
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
/* Variables ------------------------------------------------------------------*/
/* Functions ------------------------------------------------------------------*/
void TFT9341_SendCommand(uint8_t cmd);
void TFT9341_SendData(uint8_t dt);
void TFT9341_Init(GPIO_TypeDef *csPORT, uint16_t csPIN, GPIO_TypeDef *dcPORT, uint16_t dcPIN, GPIO_TypeDef *resetPORT, uint16_t resetPIN);
void TFT9341_reset(void);
void TFT9341_FillScreen(uint16_t color);
void TFT9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void TFT9341_DrawPixel(int x, int y, uint16_t color);
void TFT9341_DrawCircle(uint16_t x0, uint16_t y0, int r, uint16_t color);
void TFT9341_SetRotation(uint8_t r);
void TFT9341_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void TFT9341_printText(char text[], int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t size);
void TFT9341_DrawLine(uint16_t color, uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2);
void TFT9341_DrawRect(uint16_t color, uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2);
uint8_t TFT9341_getRotation(void);
