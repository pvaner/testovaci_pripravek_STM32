/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <my_lib_MCP795.h>
#include "main.h"
/* Definitions ------------------------------------------------------------------*/
#define RTC_CS_low() HAL_GPIO_WritePin(RTC_CS_GPIO_Port,RTC_CS_Pin,GPIO_PIN_RESET)
#define RTC_CS_high() HAL_GPIO_WritePin(RTC_CS_GPIO_Port,RTC_CS_Pin,GPIO_PIN_SET)
/* Variables ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
rtc_time_t g_rtcTime = { 	.wday_name[0] = "Mon",
							.wday_name[1] = "Tue",
							.wday_name[2] = "Wed",
							.wday_name[3] = "Thu",
							.wday_name[4] = "Fri",
							.wday_name[5] = "Sat",
							.wday_name[6] = "Sun"
						};

/* Functions ------------------------------------------------------------------*/
//pošli byte přes SPI
static uint8_t spi_acc_send_byte(uint8_t u8_data)
{
	uint8_t data=0;
	HAL_SPI_TransmitReceive(&hspi2, &u8_data, &data, 1, 1);
	return data;
}
//pošli byte přes SPI
static uint8_t rtc_send_byte(uint8_t val)
{
	return (uint8_t) ( spi_acc_send_byte( val ));
}
//přečti status registr
static uint8_t rtc_read_status_reg(void)
{
	uint8_t status;
	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_SPREAD_cmd );
	status = rtc_send_byte( 0x00u );
	RTC_CS_high();
	return status;
}
//zapis byte do SRAM
static void rtc_write_byte_sram(uint8_t addr, uint8_t val)
{
	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_WRITE_cmd );
	rtc_send_byte( addr & 0xffu );
	rtc_send_byte( val & 0xffu );
	RTC_CS_high();
}
//zapis stranku do SRAM
static void rtc_write_blobk_sram(uint8_t addr, uint8_t *buf, uint8_t size)
{
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );
	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_WRITE_cmd );
	rtc_send_byte( addr & 0xffu );
	while(size--){
		rtc_send_byte(( *buf++ ) & 0xffu );
	}
	RTC_CS_high();
}
//precti byte z SRAM
static uint8_t rtc_read_byte_sram(uint8_t addr)
{
	uint8_t data;
	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_READ_cmd );
	rtc_send_byte( addr & 0xffu );
	data = rtc_send_byte( 0x00u );
	RTC_CS_high();
	return ( uint8_t ) ( data );
}
//precti blok z SRAM
static uint8_t* rtc_read_block_sram(uint8_t addr, uint8_t size)
{
	static uint8_t buf[10];
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );
	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_READ_cmd );
	rtc_send_byte( addr & 0xffu );
	for ( uint8_t i = 0; i < size; i++ ){
		buf[i] = ( uint8_t ) ( rtc_send_byte( 0x00u ) );
	}
	RTC_CS_high();
	return (uint8_t*) ( &buf );
}
// Start/Stop oscilator
static void rtc_enable_on_board_oscillator(bool state)
{
	if ( state )
	{
		uint8_t ctrl_reg = rtc_read_byte_sram(MCP795_CONTROL_REG_addr);
		ctrl_reg |= ( MCP795_CONTROL_REG_SQWE_msk );
		rtc_write_byte_sram(MCP795_CONTROL_REG_addr, ctrl_reg );
		uint8_t sec_reg = rtc_read_byte_sram(MCP795_SECONDS_addr);
		sec_reg |= ( MCP795_SECONDS_START_OSC_msk );
		rtc_write_byte_sram(MCP795_SECONDS_addr, sec_reg);
		HAL_Delay(10);
	}
	else
	{
		rtc_write_byte_sram(MCP795_SECONDS_addr, 0x00u);
	}
}
//BCD enkodovani
static uint8_t rtc_bcd_encoding(uint8_t val)
{
	if ( val >= 10 ){
		val = ((( val / 10 ) << 4u ) & 0xf0u ) | (( val % 10 ) & 0x0fu );
	}
	return val;
}
//BCD dekodovani
static uint8_t rtc_bcd_decoding(uint8_t val)
{
	return ( uint8_t ) (((( val >> 4u ) & 0x0fu ) * 10u ) + ( val & 0x0fu ));
}
//inicializace
void rtc_init()
{
	rtc_enable_on_board_oscillator(true);

////////////////////////////////////////////////
// 	manualne nastaveny cas
////////////////////////////////////////////////
	g_rtcTime.year 		= 2020;
	g_rtcTime.month 	= 2;
	g_rtcTime.mday 		= 25;
	g_rtcTime.hour 		= 20;
	g_rtcTime.min 		= 5;
	g_rtcTime.sec 		= 1;
	g_rtcTime.wday 		= 1;
	rtc_set_time();
}
//nastav cas
void rtc_set_time(void)
{
	rtc_enable_on_board_oscillator( false );
	uint8_t *time_reg = rtc_read_block_sram(MCP795_SECONDS_addr, 7u);
	uint8_t sec_reg 	= time_reg[0];
	uint8_t hour_reg 	= time_reg[2];
	uint8_t wday_reg 	= time_reg[3];
	uint8_t month_reg	= time_reg[5];
	hour_reg &= ~( MCP795_HOURS_12_24_msk | ( 0x01u << 5u ));
	uint8_t buf[10] = 	{ 	rtc_bcd_encoding( g_rtcTime.sec ) | ( sec_reg & MCP795_SECONDS_START_OSC_msk ),
							rtc_bcd_encoding( g_rtcTime.min ),
							(( rtc_bcd_encoding( g_rtcTime.hour ) & MCP795_HOURS_VAL_msk ) | ( hour_reg & MCP795_HOURS_CTRL_msk )),
							(( g_rtcTime.wday & MCP795_DAY_VAL_msk ) | ( wday_reg & MCP795_DAY_CTRL_msk )),
							rtc_bcd_encoding( g_rtcTime.mday ),
							(( rtc_bcd_encoding( g_rtcTime.month ) & MCP795_MONTH_VAL_msk ) | ( month_reg & MCP795_MONTH_LP_msk )),
							rtc_bcd_encoding( g_rtcTime.year - RTC_PRESET_YEAR )
						};
	rtc_write_blobk_sram(MCP795_SECONDS_addr, (uint8_t*) &buf, 5u);
	if ( 		g_rtcTime.year == 2020u 
			|| 	g_rtcTime.year == 2024u 
			|| 	g_rtcTime.year == 2028u 
		)
	{
		rtc_write_byte_sram(MCP795_MONTH_addr, ( buf[5] | MCP795_MONTH_LP_msk ));
	}
	else{
		rtc_write_byte_sram(MCP795_MONTH_addr, buf[5]);
	}
	rtc_write_byte_sram(MCP795_YEAR_addr, buf[6]);
	rtc_enable_on_board_oscillator( true );
	HAL_Delay(10);
}
//update time struct
void rtc_update_time()
{
	uint8_t *buf;
	buf = rtc_read_block_sram(MCP795_SECONDS_addr, 7u);
	g_rtcTime.sec 	= rtc_bcd_decoding( *buf++ & MCP795_SECONDS_VAL_msk);
	g_rtcTime.min 	= rtc_bcd_decoding( *buf++ );
	g_rtcTime.hour 	= rtc_bcd_decoding( *buf++ & MCP795_HOURS_VAL_msk );	
	g_rtcTime.wday	= (( *buf++ ) & MCP795_DAY_VAL_msk );
	g_rtcTime.mday	= rtc_bcd_decoding( *buf++ );
	g_rtcTime.month = rtc_bcd_decoding( *buf++ & MCP795_MONTH_VAL_msk );
	g_rtcTime.year 	= rtc_bcd_decoding( *buf++ ) + RTC_PRESET_YEAR;
}
//vrat RTC cas
rtc_time_t *rtc_get_time()
{
	return (rtc_time_t*) ( &g_rtcTime );
}
