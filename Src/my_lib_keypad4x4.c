/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <my_lib_keypad4x4.h>
/* Definitions ------------------------------------------------------------------*/
/* Variables ------------------------------------------------------------------*/
static Keypad_WiresTypeDef KeypadStruct;
static char Keypad_keys[16] =
{
	'1',
	'2',
	'3',
	'A',
	'4',
	'5',
	'6',
	'B',
	'7',
	'8',
	'9',
	'C',
	'*',
	'0',
	'#',
	'D'
};
/* Functions ------------------------------------------------------------------*/
//inicializace
void Keypad4x4_Init(Keypad_WiresTypeDef  *KeypadWiringStruct)
{
	//Step(1): Copy the Keypad wirings to the library
	KeypadStruct = *KeypadWiringStruct;
	
	HAL_GPIO_WritePin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, SET);
	HAL_GPIO_WritePin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, SET);
	HAL_GPIO_WritePin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, SET);
	HAL_GPIO_WritePin(KeypadStruct.OUT3_Port, KeypadStruct.OUT3pin, SET);
}
//zmena sloupce
void Keypad4x4_ChangeColomn(uint8_t colNum_0_to_3)
{
	if(colNum_0_to_3==0)
	{
		HAL_GPIO_WritePin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, RESET);
		HAL_GPIO_WritePin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT3_Port, KeypadStruct.OUT3pin, SET);
	}
	else if(colNum_0_to_3==1)
	{
		HAL_GPIO_WritePin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, RESET);
		HAL_GPIO_WritePin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT3_Port, KeypadStruct.OUT3pin, SET);
	}
	else if(colNum_0_to_3==2)
	{
		HAL_GPIO_WritePin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, RESET);
		HAL_GPIO_WritePin(KeypadStruct.OUT3_Port, KeypadStruct.OUT3pin, SET);
	}
	else if(colNum_0_to_3==3)
	{
		HAL_GPIO_WritePin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, SET);
		HAL_GPIO_WritePin(KeypadStruct.OUT3_Port, KeypadStruct.OUT3pin, RESET);
	}
}
//funkce pro cteni aktivni klavesy
void Keypad4x4_ReadKeypad(bool keys[16])
{
	//Step(1): Make Col0 High and check the rows
	Keypad4x4_ChangeColomn(0);
	keys[0] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[4] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[8] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[12] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
	
	//Step(2): Make Col1 High and check the rows
	Keypad4x4_ChangeColomn(1);
	keys[1] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[5] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[9] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[13] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
	
	//Step(3): Make Col2 High and check the rows
	Keypad4x4_ChangeColomn(2);
	keys[2] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[6] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[10] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[14] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
	
	//Step(4): Make Col3 High and check the rows
	Keypad4x4_ChangeColomn(3);
	keys[3] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[7] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[11] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[15] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
}	
//funkce pro ziskani znaku
char Keypad4x4_GetChar(uint8_t keypadSw)
{
	return Keypad_keys[keypadSw];
}

