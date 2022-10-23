/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <my_lib_STH20.h>
#include "stm32l4xx_hal.h"
/* Definitions ------------------------------------------------------------------*/
/* Variables ------------------------------------------------------------------*/
const int16_t POLYNOMIAL = 0x131;
static I2C_HandleTypeDef shtI2Chandle;
SHT20_INFO sht20_info;

/* Functions ------------------------------------------------------------------*/
//inicializace
void SHT20_Init(I2C_HandleTypeDef *I2CshtHandle)
{
	 memcpy(&shtI2Chandle, I2CshtHandle, sizeof(*I2CshtHandle));
}
//reset
void SHT20_reset(void)
{
    HAL_I2C_Mem_Write(&shtI2Chandle, SHT20_Write_Add, SHT20_SOFT_RESET, I2C_MEMADD_SIZE_8BIT,(void*)0, 1, 1000);
    HAL_Delay(15);
}
//cteni user registru
unsigned char  SHT20_read_user_reg(void)
{
    unsigned char val = 0;
    HAL_I2C_Mem_Read(&shtI2Chandle, SHT20_Read_Add, SHT20_READ_REG, I2C_MEMADD_SIZE_8BIT,&val, 1, 1000);
    return val;
}
//vypocet crc
char SHT2x_CheckCrc(char data[], char nbrOfBytes, char checksum)
{
    char crc = 0;
    char bit = 0;
    char byteCtr = 0;
    //calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for ( bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else crc = (crc << 1);
        }
    }
    if(crc != checksum)
		return 1;
    else
		return 0;
}
//vypocet teploty
float SHT2x_CalcTemperatureC(unsigned short u16sT)
{
    float temperatureC = 0;
    u16sT &= ~0x0003;
    temperatureC = -46.85 + 175.72 / 65536 * (float)u16sT; //T= -46.85 + 175.72 * ST/2^16
    return temperatureC;
}
//vypocet vlhkosti
float SHT2x_CalcRH(unsigned short u16sRH)
{
    float humidityRH = 0;
    u16sRH &= ~0x0003;
    //humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    humidityRH = ((float)u16sRH * 0.00190735) - 6;
    return humidityRH;
}
//mereni
float SHT2x_MeasureHM(unsigned char cmd, unsigned short *pMeasurand)
{
    char  checksum = 0;
    char  data[2];
	unsigned char addr = 0;
    unsigned short tmp = 0;
    float t = 0;
	HAL_I2C_Mem_Read(&shtI2Chandle, SHT20_Read_Add, cmd, I2C_MEMADD_SIZE_8BIT,data,2, 0xFFFF);
	
	SHT2x_CheckCrc(data, 2, checksum);
    tmp = (data[0] << 8) + data[1];
    if(cmd == SHT20_Measurement_T_HM)
    {
        t = SHT2x_CalcTemperatureC(tmp);
    }
    else
    {
        t = SHT2x_CalcRH(tmp);
    }
    if(pMeasurand)
    {
        *pMeasurand = (unsigned short)t;
    }
    return t;
}
//získání hodnoty
void SHT20_GetValue(void)
{
	sht20_info.tempreture = SHT2x_MeasureHM(SHT20_Measurement_T_HM, (void *)0);
	HAL_Delay(70);
	sht20_info.humidity = SHT2x_MeasureHM(SHT20_Measurement_RH_HM, (void *)0);
	HAL_Delay(25);
	SHT20_reset();
}
