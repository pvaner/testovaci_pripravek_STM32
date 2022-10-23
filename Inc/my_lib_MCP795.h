/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"
#include "stdbool.h"
#include "stdint.h"
/* Definitions ------------------------------------------------------------------*/
#define RTC_PRESET_YEAR 						( uint16_t ) ( 2019u )
#define MCP795_HUNDREDTH_OF_SEC_addr		( uint8_t ) ( 0x00u )
#define MCP795_SECONDS_addr					( uint8_t )	( 0x01u )
#define MCP795_SECONDS_VAL_msk				( uint8_t ) ( 0x7Fu )
#define MCP795_SECONDS_CTRL_msk				( uint8_t ) ( 0x80u )
#define MCP795_SECONDS_START_OSC_bp			( uint8_t )	( 7u )											// Start/Stop oscillator options. (1 - start, 0 - stop)
#define MCP795_SECONDS_START_OSC_msk		( uint8_t ) ( 0x01u << MCP795_SECONDS_START_OSC_bp )
#define MCP795_MINUTES_addr					( uint8_t ) ( 0x02u )
#define MCP795_HOURS_addr					( uint8_t )	( 0x03u )
#define MCP795_HOURS_VAL_msk				( uint8_t ) ( 0x3Fu )
#define MCP795_HOURS_CTRL_msk				( uint8_t ) ( 0xE0u )
#define MCP795_HOURS_CALCSGN_bp				( uint8_t ) ( 7u )											// Calibration Sign bit
#define MCP795_HOURS_CALCSGN_msk			( uint8_t ) ( 0x01u << MCP795_HOURS_CALCSGN_bp )
#define MCP795_HOURS_12_24_bp				( uint8_t ) ( 6u )											// 24h or 12h format (0 - 24-hour format, 1 - 12-hour format)
#define MCP795_HOURS_12_24_msk				( uint8_t ) ( 0x01u << MCP795_HOURS_12_24_bp )
#define MCP795_DAY_addr						( uint8_t ) ( 0x04u )
#define MCP795_DAY_VAL_msk					( uint8_t ) ( 0x07u )
#define MCP795_DAY_CTRL_msk					( uint8_t ) ( 0x38u )
#define MCP795_DAY_OSCON_bp					( uint8_t )	( 5u )											// Oscillator ON bit (set/clear by HW)
#define MCP795_DAY_OSCON_msk				( uint8_t ) ( 0x01u << MCP795_DAY_OSCON_bp )
#define MCP795_DAY_VBAT_bp					( uint8_t ) ( 4u )											// External battery switch flag (1 - supplied via VBAT, 0 - supplied via VCC - set/clear by HW )
#define MCP795_DAY_VBAT_msk					( uint8_t ) ( 0x01u << MCP795_DAY_OSCON_bp )
#define MCP795_DAY_VBATEN_bp				( uint8_t ) ( 3u )											// Switch to external battery (1 - supply via VBAT, 0 - supply via VCC)
#define MCP795_DAY_VBATEN_msk				( uint8_t ) ( 0x01u << MCP795_DAY_VBATEN_bp )
#define MCP795_DATE_addr					( uint8_t ) ( 0x05u )
#define MCP795_MONTH_addr					( uint8_t ) ( 0x06u )
#define MCP795_MONTH_VAL_msk				( uint8_t )	( 0x1Fu )
#define MCP795_MONTH_LP_bp					( uint8_t ) ( 5u )											// Leap year ( set during leap year )
#define MCP795_MONTH_LP_msk					( uint8_t ) ( 0x01u << MCP795_MONTH_LP_bp )
#define MCP795_YEAR_addr					( uint8_t )	( 0x07u )
#define MCP795_CONTROL_REG_addr				( uint8_t )	( 0x08u )
#define MCP795_CONTROL_REG_OUT_bp			( uint8_t ) ( 7u )											// Output polarity of CLKOUT pin
#define MCP795_CONTROL_REG_OUT_msk			( uint8_t ) ( MCP795_CONTROL_REG_OUT_bp )
#define MCP795_CONTROL_REG_SQWE_bp			( uint8_t ) ( 6u )											// Squarewave enable bit
#define MCP795_CONTROL_REG_SQWE_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_SQWE_bp )
#define MCP795_CONTROL_REG_ALM_0_bp			( uint8_t ) ( 4u )											// Alarm Configuration bits
#define MCP795_CONTROL_REG_ALM_0_msk		( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_ALM_0_bp )
#define MCP795_CONTROL_REG_ALM_1_bp			( uint8_t ) ( 5u )
#define MCP795_CONTROL_REG_ALM_1_msk		( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_ALM_1_bp )
#define MCP795_CONTROL_REG_ALM_msk			( uint8_t ) ( 0x03u << MCP795_CONTROL_REG_ALM_0_bp )
#define MCP795_CONTROL_REG_EXTOSC_bp		( uint8_t ) ( 3u )											// External Oscillator Input bit
#define MCP795_CONTROL_REG_EXTOSC_msk		( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_EXTOSC_bp )
#define MCP795_CONTROL_REG_RS_0_bp			( uint8_t ) ( 0u )											// Calibration Mode bits
#define MCP795_CONTROL_REG_RS_0_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_RS_0_bp )
#define MCP795_CONTROL_REG_RS_1_bp			( uint8_t ) ( 1u )
#define MCP795_CONTROL_REG_RS_1_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_RS_1_bp )
#define MCP795_CONTROL_REG_RS_2_bp			( uint8_t ) ( 2u )
#define MCP795_CONTROL_REG_RS_2_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_RS_2_bp )
#define MCP795_CONTROL_REG_RS_msk			( uint8_t ) ( 0x03u << MCP795_CONTROL_REG_RS_0_bp )
#define MCP795_PWRDOWN_MINUTES_addr			( uint8_t ) ( 0x18u )
#define MCP795_PWRDOWN_HOURS_addr			( uint8_t ) ( 0x19u )
#define MCP795_PWRDOWN_DATE_addr			( uint8_t ) ( 0x1Au )
#define MCP795_PWRDOWN_MONTH_addr			( uint8_t ) ( 0x1Bu )
#define MCP795_PWRUP_MINUTES_addr			( uint8_t ) ( 0x1Cu )
#define MCP795_PWRUP_HOURS_addr				( uint8_t ) ( 0x1Du )
#define MCP795_PWRUP_DATE_addr				( uint8_t ) ( 0x1Eu )
#define MCP795_PWRUP_MONTH_addr				( uint8_t ) ( 0x1Fu )
#define MCP795_STATUS_REG_WIP_bp			( uint8_t ) ( 0u )
#define MCP795_STATUS_REG_WIP_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_WIP_bp )
#define MCP795_STATUS_REG_WEL_bp			( uint8_t ) ( 1u )
#define MCP795_STATUS_REG_WEL_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_WEL_bp )
#define MCP795_STATUS_REG_BP0_bp			( uint8_t ) ( 2u )
#define MCP795_STATUS_REG_BP0_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_BP0_bp )
#define MCP795_STATUS_REG_BP1_bp			( uint8_t ) ( 3u )
#define MCP795_STATUS_REG_BP1_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_BP1_bp )
#define MCP795_ISA_EEREAD_cmd				( uint8_t ) ( 0x03u )
#define MCP795_ISA_EEWRITE_cmd				( uint8_t ) ( 0x02u )
#define MCP795_ISA_EEWRDI_cmd				( uint8_t ) ( 0x04u )
#define MCP795_ISA_EEWREN_cmd				( uint8_t ) ( 0x06u )
#define MCP795_ISA_SPREAD_cmd				( uint8_t ) ( 0x05u )
#define MCP795_ISA_SPWRITE_cmd				( uint8_t ) ( 0x01u )
#define MCP795_ISA_READ_cmd					( uint8_t ) ( 0x13u )
#define MCP795_ISA_WRITE_cmd				( uint8_t ) ( 0x12u )
#define MCP795_ISA_UNLOCK_cmd				( uint8_t ) ( 0x14u )
#define MCP795_ISA_IDWRITE_cmd				( uint8_t ) ( 0x34u )
#define MCP795_ISA_IDREAD_cmd				( uint8_t ) ( 0x33u )
#define MCP795_ISA_CLRWDT_cmd				( uint8_t ) ( 0x44u )
#define MCP795_ISA_CLRRAM					( uint8_t ) ( 0x54u )
/* Variables ------------------------------------------------------------------*/
typedef struct
{
	const char* wday_name[7];		// day name
	uint16_t	year;				// 1970..2106
	uint8_t		month;				// 1..12
	uint8_t		mday;				// 1..31
	uint8_t		hour;				// 0..23
	uint8_t		min;				// 0..59
	uint8_t		sec;				// 0..59
	uint8_t		wday;				// 0..6 (Sun..Sat)
}rtc_time_t;
/* Functions ------------------------------------------------------------------*/
void rtc_init(void);
void rtc_set_time(void);
void rtc_update_time(void);
rtc_time_t *rtc_get_time(void);



