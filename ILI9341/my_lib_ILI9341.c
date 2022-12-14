/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <my_lib_ILI9341.h>
/* Definitions ------------------------------------------------------------------*/
/* Variables ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi3;
extern RNG_HandleTypeDef hrng;
static uint8_t rotationNum=1;
static bool _cp437 = false;
static GPIO_TypeDef  *tftCS_GPIO;
static uint16_t tftCS_PIN; //chip select pin
static GPIO_TypeDef  *tftDC_GPIO;
static uint16_t tftDC_PIN; //data command pin
static GPIO_TypeDef  *tftRESET_GPIO;
static uint16_t tftRESET_PIN; //reset pin
uint16_t TFT9341_WIDTH=240;
uint16_t TFT9341_HEIGHT=320;
extern uint8_t dma_spi_fl;
extern uint32_t dma_spi_cnt;
uint8_t frm_buf[65536] = {0};

static //pole jednoducheho textoveho fontu
const unsigned char font1[] = {
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x3E, 0x5B, 0x4F, 0x5B, 0x3E,
	0x3E, 0x6B, 0x4F, 0x6B, 0x3E,
	0x1C, 0x3E, 0x7C, 0x3E, 0x1C,
	0x18, 0x3C, 0x7E, 0x3C, 0x18,
	0x1C, 0x57, 0x7D, 0x57, 0x1C,
	0x1C, 0x5E, 0x7F, 0x5E, 0x1C,
	0x00, 0x18, 0x3C, 0x18, 0x00,
	0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
	0x00, 0x18, 0x24, 0x18, 0x00,
	0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
	0x30, 0x48, 0x3A, 0x06, 0x0E,
	0x26, 0x29, 0x79, 0x29, 0x26,
	0x40, 0x7F, 0x05, 0x05, 0x07,
	0x40, 0x7F, 0x05, 0x25, 0x3F,
	0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
	0x7F, 0x3E, 0x1C, 0x1C, 0x08,
	0x08, 0x1C, 0x1C, 0x3E, 0x7F,
	0x14, 0x22, 0x7F, 0x22, 0x14,
	0x5F, 0x5F, 0x00, 0x5F, 0x5F,
	0x06, 0x09, 0x7F, 0x01, 0x7F,
	0x00, 0x66, 0x89, 0x95, 0x6A,
	0x60, 0x60, 0x60, 0x60, 0x60,
	0x94, 0xA2, 0xFF, 0xA2, 0x94,
	0x08, 0x04, 0x7E, 0x04, 0x08,
	0x10, 0x20, 0x7E, 0x20, 0x10,
	0x08, 0x08, 0x2A, 0x1C, 0x08,
	0x08, 0x1C, 0x2A, 0x08, 0x08,
	0x1E, 0x10, 0x10, 0x10, 0x10,
	0x0C, 0x1E, 0x0C, 0x1E, 0x0C,
	0x30, 0x38, 0x3E, 0x38, 0x30,
	0x06, 0x0E, 0x3E, 0x0E, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x5F, 0x00, 0x00,
	0x00, 0x07, 0x00, 0x07, 0x00,
	0x14, 0x7F, 0x14, 0x7F, 0x14,
	0x24, 0x2A, 0x7F, 0x2A, 0x12,
	0x23, 0x13, 0x08, 0x64, 0x62,
	0x36, 0x49, 0x56, 0x20, 0x50,
	0x00, 0x08, 0x07, 0x03, 0x00,
	0x00, 0x1C, 0x22, 0x41, 0x00,
	0x00, 0x41, 0x22, 0x1C, 0x00,
	0x2A, 0x1C, 0x7F, 0x1C, 0x2A,
	0x08, 0x08, 0x3E, 0x08, 0x08,
	0x00, 0x80, 0x70, 0x30, 0x00,
	0x08, 0x08, 0x08, 0x08, 0x08,
	0x00, 0x00, 0x60, 0x60, 0x00,
	0x20, 0x10, 0x08, 0x04, 0x02,
	0x3E, 0x51, 0x49, 0x45, 0x3E,
	0x00, 0x42, 0x7F, 0x40, 0x00,
	0x72, 0x49, 0x49, 0x49, 0x46,
	0x21, 0x41, 0x49, 0x4D, 0x33,
	0x18, 0x14, 0x12, 0x7F, 0x10,
	0x27, 0x45, 0x45, 0x45, 0x39,
	0x3C, 0x4A, 0x49, 0x49, 0x31,
	0x41, 0x21, 0x11, 0x09, 0x07,
	0x36, 0x49, 0x49, 0x49, 0x36,
	0x46, 0x49, 0x49, 0x29, 0x1E,
	0x00, 0x00, 0x14, 0x00, 0x00,
	0x00, 0x40, 0x34, 0x00, 0x00,
	0x00, 0x08, 0x14, 0x22, 0x41,
	0x14, 0x14, 0x14, 0x14, 0x14,
	0x00, 0x41, 0x22, 0x14, 0x08,
	0x02, 0x01, 0x59, 0x09, 0x06,
	0x3E, 0x41, 0x5D, 0x59, 0x4E,
	0x7C, 0x12, 0x11, 0x12, 0x7C,
	0x7F, 0x49, 0x49, 0x49, 0x36,
	0x3E, 0x41, 0x41, 0x41, 0x22,
	0x7F, 0x41, 0x41, 0x41, 0x3E,
	0x7F, 0x49, 0x49, 0x49, 0x41,
	0x7F, 0x09, 0x09, 0x09, 0x01,
	0x3E, 0x41, 0x41, 0x51, 0x73,
	0x7F, 0x08, 0x08, 0x08, 0x7F,
	0x00, 0x41, 0x7F, 0x41, 0x00,
	0x20, 0x40, 0x41, 0x3F, 0x01,
	0x7F, 0x08, 0x14, 0x22, 0x41,
	0x7F, 0x40, 0x40, 0x40, 0x40,
	0x7F, 0x02, 0x1C, 0x02, 0x7F,
	0x7F, 0x04, 0x08, 0x10, 0x7F,
	0x3E, 0x41, 0x41, 0x41, 0x3E,
	0x7F, 0x09, 0x09, 0x09, 0x06,
	0x3E, 0x41, 0x51, 0x21, 0x5E,
	0x7F, 0x09, 0x19, 0x29, 0x46,
	0x26, 0x49, 0x49, 0x49, 0x32,
	0x03, 0x01, 0x7F, 0x01, 0x03,
	0x3F, 0x40, 0x40, 0x40, 0x3F,
	0x1F, 0x20, 0x40, 0x20, 0x1F,
	0x3F, 0x40, 0x38, 0x40, 0x3F,
	0x63, 0x14, 0x08, 0x14, 0x63,
	0x03, 0x04, 0x78, 0x04, 0x03,
	0x61, 0x59, 0x49, 0x4D, 0x43,
	0x00, 0x7F, 0x41, 0x41, 0x41,
	0x02, 0x04, 0x08, 0x10, 0x20,
	0x00, 0x41, 0x41, 0x41, 0x7F,
	0x04, 0x02, 0x01, 0x02, 0x04,
	0x40, 0x40, 0x40, 0x40, 0x40,
	0x00, 0x03, 0x07, 0x08, 0x00,
	0x20, 0x54, 0x54, 0x78, 0x40,
	0x7F, 0x28, 0x44, 0x44, 0x38,
	0x38, 0x44, 0x44, 0x44, 0x28,
	0x38, 0x44, 0x44, 0x28, 0x7F,
	0x38, 0x54, 0x54, 0x54, 0x18,
	0x00, 0x08, 0x7E, 0x09, 0x02,
	0x18, 0xA4, 0xA4, 0x9C, 0x78,
	0x7F, 0x08, 0x04, 0x04, 0x78,
	0x00, 0x44, 0x7D, 0x40, 0x00,
	0x20, 0x40, 0x40, 0x3D, 0x00,
	0x7F, 0x10, 0x28, 0x44, 0x00,
	0x00, 0x41, 0x7F, 0x40, 0x00,
	0x7C, 0x04, 0x78, 0x04, 0x78,
	0x7C, 0x08, 0x04, 0x04, 0x78,
	0x38, 0x44, 0x44, 0x44, 0x38,
	0xFC, 0x18, 0x24, 0x24, 0x18,
	0x18, 0x24, 0x24, 0x18, 0xFC,
	0x7C, 0x08, 0x04, 0x04, 0x08,
	0x48, 0x54, 0x54, 0x54, 0x24,
	0x04, 0x04, 0x3F, 0x44, 0x24,
	0x3C, 0x40, 0x40, 0x20, 0x7C,
	0x1C, 0x20, 0x40, 0x20, 0x1C,
	0x3C, 0x40, 0x30, 0x40, 0x3C,
	0x44, 0x28, 0x10, 0x28, 0x44,
	0x4C, 0x90, 0x90, 0x90, 0x7C,
	0x44, 0x64, 0x54, 0x4C, 0x44,
	0x00, 0x08, 0x36, 0x41, 0x00,
	0x00, 0x00, 0x77, 0x00, 0x00,
	0x00, 0x41, 0x36, 0x08, 0x00,
	0x02, 0x01, 0x02, 0x04, 0x02,
	0x3C, 0x26, 0x23, 0x26, 0x3C,
	0x1E, 0xA1, 0xA1, 0x61, 0x12,
	0x3A, 0x40, 0x40, 0x20, 0x7A,
	0x38, 0x54, 0x54, 0x55, 0x59,
	0x21, 0x55, 0x55, 0x79, 0x41,
	0x22, 0x54, 0x54, 0x78, 0x42,
	0x21, 0x55, 0x54, 0x78, 0x40,
	0x20, 0x54, 0x55, 0x79, 0x40,
	0x0C, 0x1E, 0x52, 0x72, 0x12,
	0x39, 0x55, 0x55, 0x55, 0x59,
	0x39, 0x54, 0x54, 0x54, 0x59,
	0x39, 0x55, 0x54, 0x54, 0x58,
	0x00, 0x00, 0x45, 0x7C, 0x41,
	0x00, 0x02, 0x45, 0x7D, 0x42,
	0x00, 0x01, 0x45, 0x7C, 0x40,
	0x7D, 0x12, 0x11, 0x12, 0x7D,
	0xF0, 0x28, 0x25, 0x28, 0xF0,
	0x7C, 0x54, 0x55, 0x45, 0x00,
	0x20, 0x54, 0x54, 0x7C, 0x54,
	0x7C, 0x0A, 0x09, 0x7F, 0x49,
	0x32, 0x49, 0x49, 0x49, 0x32,
	0x3A, 0x44, 0x44, 0x44, 0x3A,
	0x32, 0x4A, 0x48, 0x48, 0x30,
	0x3A, 0x41, 0x41, 0x21, 0x7A,
	0x3A, 0x42, 0x40, 0x20, 0x78,
	0x00, 0x9D, 0xA0, 0xA0, 0x7D,
	0x3D, 0x42, 0x42, 0x42, 0x3D,
	0x3D, 0x40, 0x40, 0x40, 0x3D,
	0x3C, 0x24, 0xFF, 0x24, 0x24,
	0x48, 0x7E, 0x49, 0x43, 0x66,
	0x2B, 0x2F, 0xFC, 0x2F, 0x2B,
	0xFF, 0x09, 0x29, 0xF6, 0x20,
	0xC0, 0x88, 0x7E, 0x09, 0x03,
	0x20, 0x54, 0x54, 0x79, 0x41,
	0x00, 0x00, 0x44, 0x7D, 0x41,
	0x30, 0x48, 0x48, 0x4A, 0x32,
	0x38, 0x40, 0x40, 0x22, 0x7A,
	0x00, 0x7A, 0x0A, 0x0A, 0x72,
	0x7D, 0x0D, 0x19, 0x31, 0x7D,
	0x26, 0x29, 0x29, 0x2F, 0x28,
	0x26, 0x29, 0x29, 0x29, 0x26,
	0x30, 0x48, 0x4D, 0x40, 0x20,
	0x38, 0x08, 0x08, 0x08, 0x08,
	0x08, 0x08, 0x08, 0x08, 0x38,
	0x2F, 0x10, 0xC8, 0xAC, 0xBA,
	0x2F, 0x10, 0x28, 0x34, 0xFA,
	0x00, 0x00, 0x7B, 0x00, 0x00,
	0x08, 0x14, 0x2A, 0x14, 0x22,
	0x22, 0x14, 0x2A, 0x14, 0x08,
	0x55, 0x00, 0x55, 0x00, 0x55,
	0xAA, 0x55, 0xAA, 0x55, 0xAA,
	0xFF, 0x55, 0xFF, 0x55, 0xFF,
	0x00, 0x00, 0x00, 0xFF, 0x00,
	0x10, 0x10, 0x10, 0xFF, 0x00,
	0x14, 0x14, 0x14, 0xFF, 0x00,
	0x10, 0x10, 0xFF, 0x00, 0xFF,
	0x10, 0x10, 0xF0, 0x10, 0xF0,
	0x14, 0x14, 0x14, 0xFC, 0x00,
	0x14, 0x14, 0xF7, 0x00, 0xFF,
	0x00, 0x00, 0xFF, 0x00, 0xFF,
	0x14, 0x14, 0xF4, 0x04, 0xFC,
	0x14, 0x14, 0x17, 0x10, 0x1F,
	0x10, 0x10, 0x1F, 0x10, 0x1F,
	0x14, 0x14, 0x14, 0x1F, 0x00,
	0x10, 0x10, 0x10, 0xF0, 0x00,
	0x00, 0x00, 0x00, 0x1F, 0x10,
	0x10, 0x10, 0x10, 0x1F, 0x10,
	0x10, 0x10, 0x10, 0xF0, 0x10,
	0x00, 0x00, 0x00, 0xFF, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0xFF, 0x10,
	0x00, 0x00, 0x00, 0xFF, 0x14,
	0x00, 0x00, 0xFF, 0x00, 0xFF,
	0x00, 0x00, 0x1F, 0x10, 0x17,
	0x00, 0x00, 0xFC, 0x04, 0xF4,
	0x14, 0x14, 0x17, 0x10, 0x17,
	0x14, 0x14, 0xF4, 0x04, 0xF4,
	0x00, 0x00, 0xFF, 0x00, 0xF7,
	0x14, 0x14, 0x14, 0x14, 0x14,
	0x14, 0x14, 0xF7, 0x00, 0xF7,
	0x14, 0x14, 0x14, 0x17, 0x14,
	0x10, 0x10, 0x1F, 0x10, 0x1F,
	0x14, 0x14, 0x14, 0xF4, 0x14,
	0x10, 0x10, 0xF0, 0x10, 0xF0,
	0x00, 0x00, 0x1F, 0x10, 0x1F,
	0x00, 0x00, 0x00, 0x1F, 0x14,
	0x00, 0x00, 0x00, 0xFC, 0x14,
	0x00, 0x00, 0xF0, 0x10, 0xF0,
	0x10, 0x10, 0xFF, 0x10, 0xFF,
	0x14, 0x14, 0x14, 0xFF, 0x14,
	0x10, 0x10, 0x10, 0x1F, 0x00,
	0x00, 0x00, 0x00, 0xF0, 0x10,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0xFF,
	0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
	0x38, 0x44, 0x44, 0x38, 0x44,
	0xFC, 0x4A, 0x4A, 0x4A, 0x34,
	0x7E, 0x02, 0x02, 0x06, 0x06,
	0x02, 0x7E, 0x02, 0x7E, 0x02,
	0x63, 0x55, 0x49, 0x41, 0x63,
	0x38, 0x44, 0x44, 0x3C, 0x04,
	0x40, 0x7E, 0x20, 0x1E, 0x20,
	0x06, 0x02, 0x7E, 0x02, 0x02,
	0x99, 0xA5, 0xE7, 0xA5, 0x99,
	0x1C, 0x2A, 0x49, 0x2A, 0x1C,
	0x4C, 0x72, 0x01, 0x72, 0x4C,
	0x30, 0x4A, 0x4D, 0x4D, 0x30,
	0x30, 0x48, 0x78, 0x48, 0x30,
	0xBC, 0x62, 0x5A, 0x46, 0x3D,
	0x3E, 0x49, 0x49, 0x49, 0x00,
	0x7E, 0x01, 0x01, 0x01, 0x7E,
	0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
	0x44, 0x44, 0x5F, 0x44, 0x44,
	0x40, 0x51, 0x4A, 0x44, 0x40,
	0x40, 0x44, 0x4A, 0x51, 0x40,
	0x00, 0x00, 0xFF, 0x01, 0x03,
	0xE0, 0x80, 0xFF, 0x00, 0x00,
	0x08, 0x08, 0x6B, 0x6B, 0x08,
	0x36, 0x12, 0x36, 0x24, 0x36,
	0x06, 0x0F, 0x09, 0x0F, 0x06,
	0x00, 0x00, 0x18, 0x18, 0x00,
	0x00, 0x00, 0x10, 0x10, 0x00,
	0x30, 0x40, 0xFF, 0x01, 0x01,
	0x00, 0x1F, 0x01, 0x01, 0x1E,
	0x00, 0x19, 0x1D, 0x17, 0x12,
	0x00, 0x3C, 0x3C, 0x3C, 0x3C,
	0x00, 0x00, 0x00, 0x00, 0x00
};

/* Functions ------------------------------------------------------------------*/
//posli command
void TFT9341_SendCommand(uint8_t cmd)
{
  DC_COMMAND();
  HAL_SPI_Transmit (&hspi3, &cmd, 1, 5000);
}
//posli data
void TFT9341_SendData(uint8_t dt)
{
	DC_DATA();
	HAL_SPI_Transmit (&hspi3, &dt, 1, 5000);
}
//zapis data
static void TFT9341_WriteData(uint8_t* buff, size_t buff_size)
{
	DC_DATA();
	while(buff_size > 0) {
		uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
		HAL_SPI_Transmit(&hspi3, buff, chunk_size, HAL_MAX_DELAY);
		buff += chunk_size;
		buff_size -= chunk_size;
	}
}
//inicializace tft displeje
void TFT9341_Init(GPIO_TypeDef *csPORT, uint16_t csPIN, GPIO_TypeDef *dcPORT, uint16_t dcPIN, GPIO_TypeDef *resetPORT, uint16_t resetPIN)
{
	 //CS pin
	 tftCS_GPIO = csPORT;
	 tftCS_PIN = csPIN;
	 //DC pin
	 tftDC_GPIO = dcPORT;
	 tftDC_PIN = dcPIN;
	 //RESET pin
	 tftRESET_GPIO = resetPORT;
	 tftRESET_PIN = resetPIN;
	  uint8_t data[15];
	  CS_ACTIVE();
	  TFT9341_reset();
	  //Software Reset
	  TFT9341_SendCommand(0x01);
	  HAL_Delay(1000);
	  //Power Control A
	  data[0] = 0x39;
	  data[1] = 0x2C;
	  data[2] = 0x00;
	  data[3] = 0x34;
	  data[4] = 0x02;
	  TFT9341_SendCommand(0xCB);
	  TFT9341_WriteData(data, 5);
	  //Power Control B
	  data[0] = 0x00;
	  data[1] = 0xC1;
	  data[2] = 0x30;
	  TFT9341_SendCommand(0xCF);
	  TFT9341_WriteData(data, 3);
	  //Driver timing control A
	  data[0] = 0x85;
	  data[1] = 0x00;
	  data[2] = 0x78;
	  TFT9341_SendCommand(0xE8);
	  TFT9341_WriteData(data, 3);
	  //Driver timing control B
	  data[0] = 0x00;
	  data[1] = 0x00;
	  TFT9341_SendCommand(0xEA);
	  TFT9341_WriteData(data, 2);
	  //Power on Sequence control
	  data[0] = 0x64;
	  data[1] = 0x03;
	  data[2] = 0x12;
	  data[3] = 0x81;
	  TFT9341_SendCommand(0xED);
	  TFT9341_WriteData(data, 4);
	  //Pump ratio control
	  data[0] = 0x20;
	  TFT9341_SendCommand(0xF7);
	  TFT9341_WriteData(data, 1);
	  //Power Control,VRH[5:0]
	  data[0] = 0x10;
	  TFT9341_SendCommand(0xC0);
	  TFT9341_WriteData(data, 1);
	  //Power Control,SAP[2:0];BT[3:0]
	  data[0] = 0x10;
	  TFT9341_SendCommand(0xC1);
	  TFT9341_WriteData(data, 1);
	  //VCOM Control 1
	  data[0] = 0x3E;
	  data[1] = 0x28;
	  TFT9341_SendCommand(0xC5);
	  TFT9341_WriteData(data, 2);
	  //VCOM Control 2
	  data[0] = 0x86;
	  TFT9341_SendCommand(0xC7);
	  TFT9341_WriteData(data, 1);
	  //Memory Acsess Control
	  data[0] = 0x48;
	  TFT9341_SendCommand(0x36);
	  TFT9341_WriteData(data, 1);
	  //Pixel Format Set
	  data[0] = 0x55;//16bit
	  TFT9341_SendCommand(0x3A);
	  TFT9341_WriteData(data, 1);
	  //Frame Rratio Control, Standard RGB Color
	  data[0] = 0x00;
	  data[1] = 0x18;
	  TFT9341_SendCommand(0xB1);
	  TFT9341_WriteData(data, 2);
	  //Display Function Control
	  data[0] = 0x08;
	  data[1] = 0x82;
	  data[2] = 0x27;//320
	  TFT9341_SendCommand(0xB6);
	  TFT9341_WriteData(data, 3);
	  //Enable 3G
	  data[0] = 0x00;
	  TFT9341_SendCommand(0xF2);
	  TFT9341_WriteData(data, 1);
	  //Gamma set
	  data[0] = 0x01;//Gamma Curve (G2.2)
	  TFT9341_SendCommand(0x26);
	  TFT9341_WriteData(data, 1);
	  //Positive Gamma  Correction
	  data[0] = 0x0F;
	  data[1] = 0x31;
	  data[2] = 0x2B;
	  data[3] = 0x0C;
	  data[4] = 0x0E;
	  data[5] = 0x08;
	  data[6] = 0x4E;
	  data[7] = 0xF1;
	  data[8] = 0x37;
	  data[9] = 0x07;
	  data[10] = 0x10;
	  data[11] = 0x03;
	  data[12] = 0x0E;
	  data[13] = 0x09;
	  data[14] = 0x00;
	  TFT9341_SendCommand(0xE0);
	  TFT9341_WriteData(data, 15);
	  //Negative Gamma  Correction
	  data[0] = 0x00;
	  data[1] = 0x0E;
	  data[2] = 0x14;
	  data[3] = 0x03;
	  data[4] = 0x11;
	  data[5] = 0x07;
	  data[6] = 0x31;
	  data[7] = 0xC1;
	  data[8] = 0x48;
	  data[9] = 0x08;
	  data[10] = 0x0F;
	  data[11] = 0x0C;
	  data[12] = 0x31;
	  data[13] = 0x36;
	  data[14] = 0x0F;
	  TFT9341_SendCommand(0xE1);
	  TFT9341_WriteData(data, 15);
	  TFT9341_SendCommand(0x11);
	  HAL_Delay(120);
	  //Display ON
	  data[0] = 0x48;
	  TFT9341_SendCommand(0x29);
	  TFT9341_WriteData(data, 1);
}
//reset TFT displeje
void TFT9341_reset(void)
{
	RESET_ACTIVE();
	HAL_Delay(5);
	RESET_IDLE();
}
//nastaveni adresy okna kam se bude vykreslovat
static void TFT9341_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	  // column address set
	  TFT9341_SendCommand(0x2A); // CASET
	  {
	    uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
	    TFT9341_WriteData(data, sizeof(data));
	  }
	  // row address set
	  TFT9341_SendCommand(0x2B); // RASET
	  {
	    uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
	    TFT9341_WriteData(data, sizeof(data));
	  }
	  // write to RAM
	  TFT9341_SendCommand(0x2C); // RAMWR
}
//vybarveni cele obrazovky
void TFT9341_FillScreen(uint16_t color)
{
	  uint32_t i, n;
	  TFT9341_SetAddrWindow(0, 0, TFT9341_WIDTH-1, TFT9341_HEIGHT-1);
	  for(i=0;i<25600;i++)
	  {
	    frm_buf[i*2] = color >> 8;
	    frm_buf[i*2+1] = color & 0xFF;
	  }
	  n = 51200;
	  DC_DATA();
	  dma_spi_cnt = 3;
	  HAL_SPI_Transmit_DMA(&hspi3, frm_buf, n);
	  while(!dma_spi_fl) {}
	  dma_spi_fl=0;
}
//vykresleni obdelniku
void TFT9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	 uint32_t i, n, cnt, buf_size;
		if(x1>x2) swap(x1,x2);
		if(y1>y2) swap(y1,y2);
	  TFT9341_SetAddrWindow(x1, y1, x2, y2);
	  DC_DATA();
	  n = (x2-x1+1)*(y2-y1+1)*2;
	  if(n<=65535)
	  {
	    cnt = 1;
	    buf_size = n;
	  }
	  else
	  {
	    cnt = n/2;
	    buf_size = 2;
	    for(i = 3; i < n/3; i++)
	    {
	      if(n%i == 0)
	      {
	        cnt = i;
	        buf_size = n/i;
	        break;
	      }
	    }
	  }
	  for(i = 0; i < buf_size/2; i++)
	  {
	    frm_buf[i*2] = color >> 8;
	    frm_buf[i*2+1] = color & 0xFF;
	  }
	  dma_spi_cnt = cnt;
	  HAL_SPI_Transmit_DMA(&hspi3, frm_buf, buf_size);
	  while(!dma_spi_fl) {}
	  dma_spi_fl=0;
}
//vykresleni pixelu
void TFT9341_DrawPixel(int x, int y, uint16_t color)
{
	if((x<0)||(y<0)||(x>=TFT9341_WIDTH)||(y>=TFT9341_HEIGHT)) return;
	TFT9341_SetAddrWindow(x,y,x,y);
	TFT9341_SendCommand(0x2C);
	TFT9341_SendData(color>>8);
	TFT9341_SendData(color & 0xFF);
}
//vykresleni kruhu
void TFT9341_DrawCircle(uint16_t x0, uint16_t y0, int r, uint16_t color)
{
	int f = 1-r;
	int ddF_x=1;
	int ddF_y=-2*r;
	int x = 0;
	int y = r;
	TFT9341_DrawPixel(x0,y0+r,color);
	TFT9341_DrawPixel(x0,y0-r,color);
	TFT9341_DrawPixel(x0+r,y0,color);
	TFT9341_DrawPixel(x0-r,y0,color);
	while (x<y)
	{
		if (f>=0)
		{
			y--;
			ddF_y+=2;
			f+=ddF_y;
		}
		x++;
		ddF_x+=2;
		f+=ddF_x;
		TFT9341_DrawPixel(x0+x,y0+y,color);
		TFT9341_DrawPixel(x0-x,y0+y,color);
		TFT9341_DrawPixel(x0+x,y0-y,color);
		TFT9341_DrawPixel(x0-x,y0-y,color);
		TFT9341_DrawPixel(x0+y,y0+x,color);
		TFT9341_DrawPixel(x0-y,y0+x,color);
		TFT9341_DrawPixel(x0+y,y0-x,color);
		TFT9341_DrawPixel(x0-y,y0-x,color);
	}
}
//nastaveni rotace displeje
void TFT9341_SetRotation(uint8_t r)
{
  TFT9341_SendCommand(0x36);
  switch(r)
	{
		case 1:
			  rotationNum = 1;
		      TFT9341_SendData(0x48);
		      TFT9341_WIDTH = 240;
		      TFT9341_HEIGHT = 320;
			break;
		case 2:
			  rotationNum = 2;
		      TFT9341_SendData(0x28);
		      TFT9341_WIDTH = 320;
		      TFT9341_HEIGHT = 240;
			break;
		case 3:
			  rotationNum = 3;
		      TFT9341_SendData(0x88);
		      TFT9341_WIDTH = 240;
		      TFT9341_HEIGHT = 320;
			break;
		case 4:
			  rotationNum = 4;
		      TFT9341_SendData(0xE8);
		      TFT9341_WIDTH = 320;
		      TFT9341_HEIGHT = 240;
			break;
		default:
			  rotationNum = 1;
		      TFT9341_SendData(0x48);
		      TFT9341_WIDTH = 240;
		      TFT9341_HEIGHT = 320;
			break;
	}
}
//vykresleni znaku
void TFT9341_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
	if(rotationNum == 2 || rotationNum ==4)
	{
		if((x >= TFT9341_WIDTH) || (y >= TFT9341_HEIGHT) || ((x + 6 * size - 1) < 0) || ((y + 8 * size - 1) < 0))
		{
			return;
		}
	}
	else
	{
		if((y >= TFT9341_WIDTH) || (x >= TFT9341_HEIGHT) || ((y + 6 * size - 1) < 0) || ((x + 8 * size - 1) < 0))
		{
		    return;
		}
	}
  if(!_cp437 && (c >= 176)) c++;

  for (int8_t i=0; i<6; i++ ) {
    uint8_t line;
    if (i == 5)
      line = 0x0;
    else
      line = pgm_read_byte(font1+(c*5)+i);
    for (int8_t j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
        	TFT9341_DrawPixel(x+i, y+j, color);
        else {  // big size
        	TFT9341_FillRect(x+(i*size), y+(j*size), size + x+(i*size), size+1 + y+(j*size), color);
        }
      } else if (bg != color) {
        if (size == 1) // default size
        	TFT9341_DrawPixel(x+i, y+j, bg);
        else {  // big size
        	TFT9341_FillRect(x+i*size, y+j*size, size + x+i*size, size+1 + y+j*size, bg);
        }
      }
      line >>= 1;
    }
  }
}
//vykresleni textu
void TFT9341_printText(char text[], int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t size)
{
	int16_t offset;
	offset = size*6;
	for(uint16_t i=0; i<40 && text[i]!=NULL; i++)
	{
		TFT9341_drawChar(x+(offset*i), y, text[i],color,bg,size);
	}
}
//vykreslen?? ????ry
void TFT9341_DrawLine(uint16_t color, uint16_t x1, uint16_t y1,	uint16_t x2, uint16_t y2)
{
  int steep = abs(y2-y1)>abs(x2-x1);
  if(steep)
  {
    swap(x1,y1);
    swap(x2,y2);
  }
  if(x1>x2)
  {
    swap(x1,x2);
    swap(y1,y2);
  }
  int dx,dy;
  dx=x2-x1;
  dy=abs(y2-y1);
  int err=dx/2;
  int ystep;
  if(y1<y2) ystep=1;
  else ystep=-1;
  for(;x1<=x2;x1++)
  {
    if(steep) TFT9341_DrawPixel(y1,x1,color);
    else TFT9341_DrawPixel(x1,y1,color);
    err-=dy;
    if(err<0)
    {
      y1 += ystep;
      err=dx;
    }
  }
}
//vykreslen?? obdeln??ku
void TFT9341_DrawRect(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	TFT9341_DrawLine(color,x1,y1,x2,y1);
	TFT9341_DrawLine(color,x2,y1,x2,y2);
	TFT9341_DrawLine(color,x1,y1,x1,y2);
	TFT9341_DrawLine(color,x1,y2,x2,y2);
}
//vrati orientaci obrazovky
uint8_t TFT9341_getRotation(void)
{
	return rotationNum;
}
