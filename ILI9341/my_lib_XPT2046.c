/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <my_lib_XPT2046.h>
#include <stdio.h>
#include <stdlib.h>
/* Definitions ------------------------------------------------------------------*/
#define READ_X 0xD0
#define READ_Y 0x90
#define READ_Z 0x30
/* Variables ------------------------------------------------------------------*/
//zmen v zavislosti na oriantaci displeje
uint16_t XPT2046_SCALE_X=240;
uint16_t XPT2046_SCALE_Y=320;

uint16_t XPT2046_MIN_RAW_X=3400;
uint16_t XPT2046_MAX_RAW_X=29000;
uint16_t XPT2046_MIN_RAW_Y=3300;
uint16_t XPT2046_MAX_RAW_Y=30000;

static SPI_HandleTypeDef tsSPIhandle;
static GPIO_TypeDef  *tsCS_GPIO;
static uint16_t tsCS_PIN;
static GPIO_TypeDef  *tsIRQ_GPIO;
static uint16_t tsIRQ_PIN;


/* Functions ------------------------------------------------------------------*/
//inicializace
void XPT2046_Init(SPI_HandleTypeDef *touchSPI, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *irqPort, uint16_t irqPin)
{
	memcpy(&tsSPIhandle, touchSPI, sizeof(*touchSPI));
	tsCS_GPIO = csPort;
	tsCS_PIN = csPin;
	tsIRQ_GPIO = irqPort;
	tsIRQ_PIN = irqPin;
}
//komunikace pro dotyk -vybrana
static void XPT2046_TouchSelect()
{
    HAL_GPIO_WritePin(tsCS_GPIO, tsCS_PIN, GPIO_PIN_RESET);
}
//komunikace pro dotyk -nevybrana
static void XPT2046_TouchUnselect()
{
    HAL_GPIO_WritePin(tsCS_GPIO, tsCS_PIN, GPIO_PIN_SET);
}
//preruseni pro stisk
bool XPT2046_TouchPressed()
{
    return HAL_GPIO_ReadPin(tsIRQ_GPIO, tsIRQ_PIN) == GPIO_PIN_RESET;
}
//funkce pro ziskani souradnic dotyku
bool XPT2046_TouchGetCoordinates(uint16_t* x, uint16_t* y)
{
    static uint8_t cmd_read_x[1];
    static uint8_t cmd_read_y[1];

    if(XPT2046_getOrientation()==1 || XPT2046_getOrientation()==3)
    {
        cmd_read_x[0] =READ_X;
        cmd_read_y[0] =READ_Y;
    	XPT2046_SCALE_X=240;
    	XPT2046_SCALE_Y=320;
    	XPT2046_MIN_RAW_X=3400;
		XPT2046_MAX_RAW_X=29000;
		XPT2046_MIN_RAW_Y=3300;
		XPT2046_MAX_RAW_Y=30000;
    }
    else
    {
        cmd_read_x[0] =READ_Y;
        cmd_read_y[0] =READ_X;
    	XPT2046_SCALE_X=320;
    	XPT2046_SCALE_Y=240;
    	XPT2046_MIN_RAW_X=3300;
		XPT2046_MAX_RAW_X=30000;
		XPT2046_MIN_RAW_Y=3400;
		XPT2046_MAX_RAW_Y=29000;
    }

    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };

    XPT2046_TouchSelect();

    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;

    for(uint8_t i = 0; i < 16; i++)
    {
        if(!XPT2046_TouchPressed())
            break;

        nsamples++;

        HAL_SPI_Transmit(&tsSPIhandle, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), HAL_MAX_DELAY);
        uint8_t y_raw[2];
        HAL_SPI_TransmitReceive(&tsSPIhandle, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), HAL_MAX_DELAY);

        HAL_SPI_Transmit(&tsSPIhandle, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), HAL_MAX_DELAY);
        uint8_t x_raw[2];
        HAL_SPI_TransmitReceive(&tsSPIhandle, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), HAL_MAX_DELAY);

        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
    }

    XPT2046_TouchUnselect();

    if(nsamples < 16)
        return false;

    uint32_t raw_x = (avg_x / 16);
    if(raw_x < XPT2046_MIN_RAW_X) raw_x = XPT2046_MIN_RAW_X;
    if(raw_x > XPT2046_MAX_RAW_X) raw_x = XPT2046_MAX_RAW_X;

    uint32_t raw_y = (avg_y / 16);
    if(raw_y < XPT2046_MIN_RAW_Y) raw_y = XPT2046_MIN_RAW_Y;
    if(raw_y > XPT2046_MAX_RAW_Y) raw_y = XPT2046_MAX_RAW_Y;

    switch(XPT2046_getOrientation())
  	{
  		case 1:
  		    *x = XPT2046_SCALE_X-((raw_x - XPT2046_MIN_RAW_X) * XPT2046_SCALE_X / (XPT2046_MAX_RAW_X - XPT2046_MIN_RAW_X));
  		    *y = XPT2046_SCALE_Y-((raw_y - XPT2046_MIN_RAW_Y) * XPT2046_SCALE_Y / (XPT2046_MAX_RAW_Y - XPT2046_MIN_RAW_Y));
  			break;
  		case 2:
  		    *x = XPT2046_SCALE_X-((raw_x - XPT2046_MIN_RAW_X) * XPT2046_SCALE_X / (XPT2046_MAX_RAW_X - XPT2046_MIN_RAW_X));
  		    *y = ((raw_y - XPT2046_MIN_RAW_Y) * XPT2046_SCALE_Y / (XPT2046_MAX_RAW_Y - XPT2046_MIN_RAW_Y));
  			break;
  		case 3:
  		    *x = (raw_x - XPT2046_MIN_RAW_X) * XPT2046_SCALE_X / (XPT2046_MAX_RAW_X - XPT2046_MIN_RAW_X);
  		    *y = (raw_y - XPT2046_MIN_RAW_Y) * XPT2046_SCALE_Y / (XPT2046_MAX_RAW_Y - XPT2046_MIN_RAW_Y);
  			break;
  		case 4:
  		    *x = ((raw_x - XPT2046_MIN_RAW_X) * XPT2046_SCALE_X / (XPT2046_MAX_RAW_X - XPT2046_MIN_RAW_X));
  		    *y = XPT2046_SCALE_Y-((raw_y - XPT2046_MIN_RAW_Y) * XPT2046_SCALE_Y / (XPT2046_MAX_RAW_Y - XPT2046_MIN_RAW_Y));
  			break;
  	}




    return true;
}
//vrati orientaci obrazovky
uint8_t XPT2046_getOrientation(void)
{
	return TFT9341_getRotation();
}
