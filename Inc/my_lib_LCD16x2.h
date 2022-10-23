/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include <stdlib.h>
/* Definitions ------------------------------------------------------------------*/
#define LCD_CLEARDISPLAY 		0x01
#define LCD_RETURNHOME 			0x02
#define LCD_ENTRYMODESET 		0x04
#define LCD_DISPLAYCONTROL 		0x08
#define LCD_CURSORSHIFT 		0x10
#define LCD_FUNCTIONSET 		0x20
#define LCD_SETCGRAMADDR 		0x40
#define LCD_SETDDRAMADDR 		0x80
#define LCD_ENTRY_SH		 	0x01
#define LCD_ENTRY_ID 			0x02
#define LCD_ENTRY_SH		 	0x01
#define LCD_ENTRY_ID 			0x02
#define LCD_DISPLAY_B			0x01
#define LCD_DISPLAY_C			0x02
#define LCD_DISPLAY_D			0x04
#define LCD_SHIFT_RL			0x04
#define LCD_SHIFT_SC			0x08
#define LCD_FUNCTION_F			0x04
#define LCD_FUNCTION_N			0x08
#define LCD_FUNCTION_DL			0x10
/* Variables ------------------------------------------------------------------*/
/* Functions ------------------------------------------------------------------*/
static void LCD1602_EnablePulse(void);
static void LCD1602_RS(bool state);
static void LCD1602_write(uint8_t byte);
static void LCD1602_TIM_Config(void);
static void LCD1602_TIM_MicorSecDelay(uint32_t uSecDelay);
static void LCD1602_writeCommand(uint8_t command);
static void LCD1602_writeData(uint8_t data);
static void LCD1602_write4bitCommand(uint8_t nibble);
void LCD1602_Begin4BIT(GPIO_TypeDef* PORT_RS, uint16_t RS,GPIO_TypeDef* PORT_E, uint16_t E, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7);
void LCD1602_print(char string[]);
void LCD1602_setCursor(uint8_t row, uint8_t col);
void LCD1602_1stLine(void);
void LCD1602_2ndLine(void);
void LCD1602_TwoLines(void);
void LCD1602_OneLine(void);
void LCD1602_noCursor(void);
void LCD1602_cursor(void);
void LCD1602_clear(void);
void LCD1602_noBlink(void);
void LCD1602_blink(void);
void LCD1602_noDisplay(void);
void LCD1602_display(void);
void LCD1602_shiftToRight(uint8_t num);
void LCD1602_shiftToLeft(uint8_t num);
void LCD1602_PrintInt(int number);
void LCD1602_PrintFloat(float number, int decimalPoints);
static void LCD1602_swap(char* a, char* b);
static void LCD1602_rev(char* array, size_t n);
void LCD1602_rotate_one(char* str);
