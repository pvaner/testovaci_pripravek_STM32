/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <my_lib_LCD16x2.h>
/* Definitions ------------------------------------------------------------------*/
/* Variables ------------------------------------------------------------------*/
static const uint32_t writeTimeConstant = 10;
static uint8_t mode_8_4_I2C = 1;
static GPIO_TypeDef* PORT_RS;					// RS PORT
static GPIO_TypeDef* PORT_E;					//E PORT
static uint16_t PIN_RS, PIN_E;					// RS and E pins
static GPIO_TypeDef* PORT_LSB;					// LSBs D0, D1, D2 and D3 PORT
static uint16_t D0_PIN, D1_PIN, D2_PIN, D3_PIN;	// LSBs D0, D1, D2 and D3 pins
static GPIO_TypeDef* PORT_MSB;					// MSBs D5, D6, D7 and D8 PORT
static uint16_t D4_PIN, D5_PIN, D6_PIN, D7_PIN;	// MSBs D5, D6, D7 and D8 pins
static uint8_t DisplayControl = 0x0F;
static uint8_t FunctionSet = 0x38;
//promenne casovace
volatile unsigned int *DWT_CYCCNT = (volatile unsigned int *)0xE0001004;   //address of the register
volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000;  //address of the register
volatile unsigned int *SCB_DEMCR = (volatile unsigned int *)0xE000EDFC;    //address of the register
/* Functions ------------------------------------------------------------------*/
//enable pulz
static void LCD1602_EnablePulse(void)
{
	HAL_GPIO_WritePin(PORT_E, PIN_E, GPIO_PIN_SET);
	TimingDelay(10);
	HAL_GPIO_WritePin(PORT_E, PIN_E, GPIO_PIN_RESET);
	TimingDelay(600);
}
//RS ovladani
static void LCD1602_RS(bool state)
{
	if(state) HAL_GPIO_WritePin(PORT_RS, PIN_RS, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(PORT_RS, PIN_RS, GPIO_PIN_RESET);
}

//paralelni rozhrani
static void LCD1602_write(uint8_t byte)
{
	uint8_t LSB_nibble = byte&0xF, MSB_nibble = (byte>>4)&0xF;
	 
	if(mode_8_4_I2C == 1)		//8bits mode
	{
		//LSB data
		HAL_GPIO_WritePin(PORT_LSB, D0_PIN, (GPIO_PinState)(LSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_LSB, D1_PIN, (GPIO_PinState)(LSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_LSB, D2_PIN, (GPIO_PinState)(LSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_LSB, D3_PIN, (GPIO_PinState)(LSB_nibble&0x8));
		//MSB data
		HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(MSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(MSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(MSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(MSB_nibble&0x8));
		LCD1602_EnablePulse();
		HAL_Delay(5);
	}
	else if(mode_8_4_I2C == 2)	//4 bits mode
	{
		//MSB data
		HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(MSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(MSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(MSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(MSB_nibble&0x8));
		LCD1602_EnablePulse();
		HAL_Delay(5);
		
		//LSB data
		HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(LSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(LSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(LSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(LSB_nibble&0x8));
		LCD1602_EnablePulse();
		HAL_Delay(5);
	}
}
//povoleni casovani
void EnableTiming(void)
{
*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
*DWT_CYCCNT = 0;                      // reset the counter
*DWT_CONTROL = *DWT_CONTROL | 1 ;     // enable the counter
}
//zpozdeni
void TimingDelay(unsigned int tick)
{
	EnableTiming();
	unsigned int start, current;
	start = *DWT_CYCCNT;
	do
	{
	current = *DWT_CYCCNT;
	} while((current - start) < tick);
}
//zapis prikaz
static void LCD1602_writeCommand(uint8_t command)
{
	LCD1602_RS(false);
	LCD1602_write(command);
}
//zapis 8bit data
static void LCD1602_writeData(uint8_t data)
{
	LCD1602_RS(true);
	LCD1602_write(data);
}
//zapis 4bit prikaz
static void LCD1602_write4bitCommand(uint8_t nibble)
{
	uint8_t LSB_nibble = nibble&0xF;
	LCD1602_RS(false);
	//LSB data
	HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(LSB_nibble&0x1));
	HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(LSB_nibble&0x2));
	HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(LSB_nibble&0x4));
	HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(LSB_nibble&0x8));
	LCD1602_EnablePulse();
	HAL_Delay(5);
}
//inicializace pro 4bit rozhrani
void LCD1602_Begin4BIT(GPIO_TypeDef* PORTp_RS, uint16_t RS,GPIO_TypeDef* PORTp_E, uint16_t E, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7)
{
	//Set GPIO Ports and Pins data
	PORT_RS = PORTp_RS;
	PIN_RS = RS;
	PORT_E=PORTp_E;
	PIN_E = E;
	PORT_MSB = PORT_MSBs4to7;
	D4_PIN = D4;
	D5_PIN = D5;
	D6_PIN = D6;
	D7_PIN = D7;
	mode_8_4_I2C = 2;
	FunctionSet = 0x28;
	//Initialise LCD
	//1. Wait at least 15ms
	HAL_Delay(20);
	//2. Attentions sequence
	LCD1602_write4bitCommand(0x3);
	HAL_Delay(5);
	LCD1602_write4bitCommand(0x3);
	HAL_Delay(1);
	LCD1602_write4bitCommand(0x3);
	HAL_Delay(1);
	LCD1602_write4bitCommand(0x2);  //4 bit mode
	HAL_Delay(1);
	//3. Display control (Display ON, Cursor ON, blink cursor)
	LCD1602_writeCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
	//4. Clear LCD and return home
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	HAL_Delay(3);
	//4. Function set; Enable 2 lines, Data length to 8 bits
	LCD1602_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N);
	HAL_Delay(3);
	LCD1602_clear();
}
//vypis string
void LCD1602_print(char string[])
{
	for(uint8_t i=0;  i< 16 && string[i]!=NULL; i++)
	{
		LCD1602_writeData((uint8_t)string[i]);
	}
}
//nastaveni pozice kurzoru
void LCD1602_setCursor(uint8_t row, uint8_t col)
{
	uint8_t maskData;
	maskData = (col-1)&0x0F;
	if(row==1)
	{
		maskData |= (0x80);
		LCD1602_writeCommand(maskData);
	}
	else
	{
		maskData |= (0xc0);
		LCD1602_writeCommand(maskData);
	}
}	
//prvni radek
void LCD1602_1stLine(void)
{
	LCD1602_setCursor(1,1);
}
//druhy radek
void LCD1602_2ndLine(void)
{
	LCD1602_setCursor(2,1);
}
//povoleni obou radku
void LCD1602_TwoLines(void)
{
	FunctionSet |= (0x08);
	LCD1602_writeCommand(FunctionSet);
}
//povoleni jednoho radku
void LCD1602_OneLine(void)
{
	FunctionSet &= ~(0x08);
	LCD1602_writeCommand(FunctionSet);
}
//skrytí kurzoru
void LCD1602_noCursor(void)
{
	DisplayControl &= ~(0x02);
	LCD1602_writeCommand(DisplayControl);
}
//kurzor je viditelny
void LCD1602_cursor(void)
{
	DisplayControl |= (0x02);
	LCD1602_writeCommand(DisplayControl);
}
//vymaž obsah displeje
void LCD1602_clear(void)
{
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	HAL_Delay(3);
}
void LCD1602_noBlink(void)
{
	DisplayControl &= ~(0x01);
	LCD1602_writeCommand(DisplayControl);
}
void LCD1602_blink(void)
{
	DisplayControl |= 0x01;
	LCD1602_writeCommand(DisplayControl);
}
//vypni displej
void LCD1602_noDisplay(void)
{
	DisplayControl &= ~(0x04);
	LCD1602_writeCommand(DisplayControl);
}
//zapni displej
void LCD1602_display(void)
{
	DisplayControl |= (0x04);
	LCD1602_writeCommand(DisplayControl);
}
//posun textu do prava
void LCD1602_shiftToRight(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(0x1c);
	}
}
//posun textu do levy
void LCD1602_shiftToLeft(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(0x18);
	}
}
//zobraz int
void LCD1602_PrintInt(int number)
{
	char numStr[16];
	sprintf(numStr,"%d", number);
	LCD1602_print(numStr);
}
//zobraz float
void LCD1602_PrintFloat(float number, int decimalPoints)
{
	char numStr[16];
	sprintf(numStr,"%.*f",decimalPoints, number);
	LCD1602_print(numStr);
}
//dodatecne funkce pro posun textu po displeji
static void LCD1602_swap(char* a, char* b)
{
    char tmp = *a;
    *a = *b;
    *b = tmp;
}
static void LCD1602_rev(char* array, size_t n)
{
    for (size_t i = 0; i < n / 2; i++)
    	LCD1602_swap(array + i, array +(n - i - 1));
}
void LCD1602_rotate_right(char* str)
{
	LCD1602_rev(str + 1, strlen(str) - 1);
	LCD1602_rev(str, strlen(str));
}
