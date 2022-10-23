/**
  ******************************************************************************
  *
  * Author: Pavel Vaner
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <my_lib_gui.h>
#include <my_lib_ILI9341.h>
#include <my_lib_LCD16x2.h>
/* Definitions ------------------------------------------------------------------*/
/* Variables ------------------------------------------------------------------*/
extern uint8_t TFTstav;
uint16_t paint_color=COLOR_RED;
extern char str[16];

/* Functions ------------------------------------------------------------------*/
//vykresleni hlavniho menu
void GUI_page_main(void)
{
	//pozadí
	TFT9341_FillScreen(COLOR_WHITE);
	TFT9341_FillRect(160, 0, 161 ,240, COLOR_BLUE);
	TFT9341_FillRect(0, 60, 320 ,61, COLOR_BLUE);
	TFT9341_FillRect(0, 120, 320 ,121, COLOR_BLUE);
	TFT9341_FillRect(0, 180, 320 ,181, COLOR_BLUE);
	//levy sloupec
	TFT9341_printText("MALOVANI",32, 24, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("TEPLOTA",38, 74, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("VLHKOST",38, 94, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("NTC",62, 134, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("HEATER",44, 154, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("LED DIODY",26, 204, COLOR_BLACK, COLOR_WHITE, 2);
	//pravy sloupec
	TFT9341_printText("ENCODER",199, 13, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("POTENCIOMETR",169, 33, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("KLAVESNICE",181, 84, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("REPRODUKTOR",175, 144, COLOR_BLACK, COLOR_WHITE, 2);
	TFT9341_printText("TLACITKA",193, 204, COLOR_BLACK, COLOR_WHITE, 2);
	TFTstav=0;
}
//vyber polozky pri dotyku obrazovky
void GUI_touch_main(uint16_t x, uint16_t y)
{
	   if(x>=0 && x<=160 && y>=0 && y<=60) //malovani
	   {
		   GUI_page_malovani();
		   TFTstav=1;
	   }
	   if(x>=161 && x<=320 && y>=0 && y<=60) //encoder/potenciometr
	   {
		   GUI_zmena(2);
		   TFT9341_FillRect(162, 0, 319, 5, COLOR_BLUE2);//horni
		   TFT9341_FillRect(162, 0, 167, 59, COLOR_BLUE2);//levy
		   TFT9341_FillRect(314, 0, 319, 59, COLOR_BLUE2);//pravy
		   TFT9341_FillRect(162, 54, 319, 59, COLOR_BLUE2);//spodni
		   TFTstav=2;
	   }
	   if(x>=0 && x<=160 && y>=61 && y<=120) //teplota/vlhkost
	   {
		   GUI_zmena(3);
		   TFT9341_FillRect(0, 62, 159, 67, COLOR_BLUE2);//horni
		   TFT9341_FillRect(0, 62, 5, 119, COLOR_BLUE2);//levy
		   TFT9341_FillRect(154, 62, 159, 119, COLOR_BLUE2);//pravy
		   TFT9341_FillRect(0, 114, 159, 119, COLOR_BLUE2);//spodni
		   TFTstav=3;
	   }
	   if(x>=161 && x<=320 && y>=61 && y<=120) //klavesnice
	   {
		   GUI_zmena(4);
		   TFT9341_FillRect(162, 62, 319, 67, COLOR_BLUE2);//horni
		   TFT9341_FillRect(162, 62, 167, 119, COLOR_BLUE2);//levy
		   TFT9341_FillRect(314, 62, 319, 119, COLOR_BLUE2);//pravy
		   TFT9341_FillRect(162, 114, 319, 119, COLOR_BLUE2);//spodni
		   TFTstav=4;
	   }
	   if(x>=0 && x<=160 && y>=121 && y<=180) //NTC/heater
	   {
		   GUI_zmena(5);
		   TFT9341_FillRect(0, 122, 159, 127, COLOR_BLUE2);//horni
		   TFT9341_FillRect(0, 122, 5, 179, COLOR_BLUE2);//levy
		   TFT9341_FillRect(154, 122, 159, 179, COLOR_BLUE2);//pravy
		   TFT9341_FillRect(0, 174, 159, 179, COLOR_BLUE2);//spodni
		   TFTstav=5;
	   }
	   if(x>=161 && x<=320 && y>=121 && y<=180) //reproduktor
	   {
		   GUI_zmena(6);
		   TFT9341_FillRect(162, 122, 319, 127, COLOR_BLUE2);//horni
		   TFT9341_FillRect(162, 122, 167, 179, COLOR_BLUE2);//levy
		   TFT9341_FillRect(314, 122, 319, 179, COLOR_BLUE2);//pravy
		   TFT9341_FillRect(162, 174, 319, 179, COLOR_BLUE2);//spodni
		   TFTstav=6;
	   }
	   if(x>=0 && x<=160 && y>=181 && y<=240) //led diody
	   {
		   GUI_zmena(7);
		   TFT9341_FillRect(0, 182, 159, 187, COLOR_BLUE2);//horni
		   TFT9341_FillRect(0, 182, 5, 239, COLOR_BLUE2);//levy
		   TFT9341_FillRect(154, 182, 159, 239, COLOR_BLUE2);//pravy
		   TFT9341_FillRect(0, 234, 159, 239, COLOR_BLUE2);//spodni
		   TFTstav=7;
	   }
	   if(x>=161 && x<=320 && y>=181 && y<=240) //tlacitka
	   {
		   GUI_zmena(8);
		   TFT9341_FillRect(162, 182, 319, 187, COLOR_BLUE2);//horni
		   TFT9341_FillRect(162, 182, 167, 239, COLOR_BLUE2);//levy
		   TFT9341_FillRect(314, 182, 319, 239, COLOR_BLUE2);//pravy
		   TFT9341_FillRect(162, 234, 319, 239, COLOR_BLUE2);//spodni
		   TFTstav=8;
	   }
}
//vykresleni polozky MALOVANI
void GUI_page_malovani(void)
{
	TFT9341_FillScreen(COLOR_WHITE);
    TFT9341_FillRect(0, 0, 40 ,39, COLOR_RED);
	TFT9341_FillRect(0, 40, 40, 79, COLOR_GREEN);
	TFT9341_FillRect(0, 80, 40, 119, COLOR_BLUE);
	TFT9341_FillRect(0, 120, 40, 159, COLOR_YELLOW);
	TFT9341_FillRect(0, 160, 40, 199, COLOR_BLACK);
	TFT9341_FillRect(260, 0, 320, 20, COLOR_LGRAY);
	TFT9341_printText("Zpet", 265, 3, COLOR_BLACK, COLOR_LGRAY, 2);
	TFTstav=1;
}
//vykreslovani malych kruznic v MALOVANI a zmena barvy
void GUI_touch_malovani(uint16_t x, uint16_t y)
{
	   if(x>=0 && x<=40 && y>=0 && y<=39)
	   {
		   paint_color=COLOR_RED;
	   }
	   if(x>=0 && x<=40 && y>=40 && y<=79)
	   {
		   paint_color=COLOR_GREEN;
	   }
	   if(x>=0 && x<=40 && y>=80 && y<=119)
	   {
		   paint_color=COLOR_BLUE;
	   }
	   if(x>=0 && x<=40 && y>=120 && y<=159)
	   {
		   paint_color=COLOR_YELLOW;
	   }
	   if(x>=0 && x<=40 && y>=160 && y<=199)
	   {
		   paint_color=COLOR_BLACK;
	   }
	   if(x>=0 && x<=40 && y>=200 && y<=240)
	   {
		   GUI_page_malovani();
		   return;
	   }
	   if(x>=260 && x<=320 && y>=0 && y<=20)
	   {
		   //zpet
		   GUI_page_main();
		   return;
	   }
	   if(x>40)
	   {
		   TFT9341_DrawCircle(x, y,2, paint_color);
	   }
}
//vyber funkce pri dotyku
void GUI_touch(uint8_t stav, uint16_t x, uint16_t y)
{
	if(stav==1)
	{
		GUI_touch_malovani(x,y);
	}
	else if(stav==0 || stav==2 || stav==3 || stav==4 || stav==5 || stav==6 || stav==7 || stav==8)
	{
		GUI_touch_main(x,y);
	}
}
//prekresleni GUI pri vyberu jine polozky
void GUI_zmena(uint8_t stav)
{
	switch(stav)
	{
	case 2:
		//teplota/vlhkost
		TFT9341_FillRect(0, 62, 159, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 62, 5, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 62, 159, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 114, 159, 119, COLOR_WHITE);//spodni
		//klavesnice
		TFT9341_FillRect(162, 62, 319, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 62, 167, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 62, 319, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 114, 319, 119, COLOR_WHITE);//spodni
		//NTC/heater
		TFT9341_FillRect(0, 122, 159, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 122, 5, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 122, 159, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 174, 159, 179, COLOR_WHITE);//spodni
		//reproduktor
		TFT9341_FillRect(162, 122, 319, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 122, 167, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 122, 319, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 174, 319, 179, COLOR_WHITE);//spodni
		//led diody
		TFT9341_FillRect(0, 182, 159, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 182, 5, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 182, 159, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 234, 159, 239, COLOR_WHITE);//spodni
		//tlacitka
		TFT9341_FillRect(162, 182, 319, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 182, 167, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 182, 319, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 234, 319, 239, COLOR_WHITE);//spodni
		break;
	case 3:
		//encoder/potenciometr
		TFT9341_FillRect(162, 0, 319, 5, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 0, 167, 59, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 0, 319, 59, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 54, 319, 59, COLOR_WHITE);//spodni
		//klavesnice
		TFT9341_FillRect(162, 62, 319, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 62, 167, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 62, 319, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 114, 319, 119, COLOR_WHITE);//spodni
		//NTC/heater
		TFT9341_FillRect(0, 122, 159, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 122, 5, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 122, 159, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 174, 159, 179, COLOR_WHITE);//spodni
		//reproduktor
		TFT9341_FillRect(162, 122, 319, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 122, 167, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 122, 319, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 174, 319, 179, COLOR_WHITE);//spodni
		//led diody
		TFT9341_FillRect(0, 182, 159, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 182, 5, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 182, 159, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 234, 159, 239, COLOR_WHITE);//spodni
		//tlacitka
		TFT9341_FillRect(162, 182, 319, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 182, 167, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 182, 319, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 234, 319, 239, COLOR_WHITE);//spodni
		break;
	case 4:
		//encoder/potenciometr
		TFT9341_FillRect(162, 0, 319, 5, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 0, 167, 59, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 0, 319, 59, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 54, 319, 59, COLOR_WHITE);//spodni
		//teplota/vlhkost
		TFT9341_FillRect(0, 62, 159, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 62, 5, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 62, 159, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 114, 159, 119, COLOR_WHITE);//spodni
		//NTC/heater
		TFT9341_FillRect(0, 122, 159, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 122, 5, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 122, 159, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 174, 159, 179, COLOR_WHITE);//spodni
		//reproduktor
		TFT9341_FillRect(162, 122, 319, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 122, 167, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 122, 319, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 174, 319, 179, COLOR_WHITE);//spodni
		//led diody
		TFT9341_FillRect(0, 182, 159, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 182, 5, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 182, 159, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 234, 159, 239, COLOR_WHITE);//spodni
		//tlacitka
		TFT9341_FillRect(162, 182, 319, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 182, 167, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 182, 319, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 234, 319, 239, COLOR_WHITE);//spodni
		break;
	case 5:
		//encoder/potenciometr
		TFT9341_FillRect(162, 0, 319, 5, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 0, 167, 59, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 0, 319, 59, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 54, 319, 59, COLOR_WHITE);//spodni
		//teplota/vlhkost
		TFT9341_FillRect(0, 62, 159, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 62, 5, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 62, 159, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 114, 159, 119, COLOR_WHITE);//spodni
		//klavesnice
		TFT9341_FillRect(162, 62, 319, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 62, 167, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 62, 319, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 114, 319, 119, COLOR_WHITE);//spodni
		//reproduktor
		TFT9341_FillRect(162, 122, 319, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 122, 167, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 122, 319, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 174, 319, 179, COLOR_WHITE);//spodni
		//led diody
		TFT9341_FillRect(0, 182, 159, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 182, 5, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 182, 159, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 234, 159, 239, COLOR_WHITE);//spodni
		//tlacitka
		TFT9341_FillRect(162, 182, 319, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 182, 167, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 182, 319, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 234, 319, 239, COLOR_WHITE);//spodni
		break;
	case 6:
		//encoder/potenciometr
		TFT9341_FillRect(162, 0, 319, 5, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 0, 167, 59, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 0, 319, 59, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 54, 319, 59, COLOR_WHITE);//spodni
		//teplota/vlhkost
		TFT9341_FillRect(0, 62, 159, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 62, 5, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 62, 159, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 114, 159, 119, COLOR_WHITE);//spodni
		//klavesnice
		TFT9341_FillRect(162, 62, 319, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 62, 167, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 62, 319, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 114, 319, 119, COLOR_WHITE);//spodni
		//NTC/heater
		TFT9341_FillRect(0, 122, 159, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 122, 5, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 122, 159, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 174, 159, 179, COLOR_WHITE);//spodni
		//led diody
		TFT9341_FillRect(0, 182, 159, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 182, 5, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 182, 159, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 234, 159, 239, COLOR_WHITE);//spodni
		//tlacitka
		TFT9341_FillRect(162, 182, 319, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 182, 167, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 182, 319, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 234, 319, 239, COLOR_WHITE);//spodni
		break;
	case 7:
		//encoder/potenciometr
		TFT9341_FillRect(162, 0, 319, 5, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 0, 167, 59, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 0, 319, 59, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 54, 319, 59, COLOR_WHITE);//spodni
		//teplota/vlhkost
		TFT9341_FillRect(0, 62, 159, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 62, 5, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 62, 159, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 114, 159, 119, COLOR_WHITE);//spodni
		//klavesnice
		TFT9341_FillRect(162, 62, 319, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 62, 167, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 62, 319, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 114, 319, 119, COLOR_WHITE);//spodni
		//NTC/heater
		TFT9341_FillRect(0, 122, 159, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 122, 5, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 122, 159, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 174, 159, 179, COLOR_WHITE);//spodni
		//reproduktor
		TFT9341_FillRect(162, 122, 319, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 122, 167, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 122, 319, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 174, 319, 179, COLOR_WHITE);//spodni
		//tlacitka
		TFT9341_FillRect(162, 182, 319, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 182, 167, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 182, 319, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 234, 319, 239, COLOR_WHITE);//spodni
		break;
	case 8:
		//encoder/potenciometr
		TFT9341_FillRect(162, 0, 319, 5, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 0, 167, 59, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 0, 319, 59, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 54, 319, 59, COLOR_WHITE);//spodni
		//teplota/vlhkost
		TFT9341_FillRect(0, 62, 159, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 62, 5, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 62, 159, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 114, 159, 119, COLOR_WHITE);//spodni
		//klavesnice
		TFT9341_FillRect(162, 62, 319, 67, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 62, 167, 119, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 62, 319, 119, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 114, 319, 119, COLOR_WHITE);//spodni
		//NTC/heater
		TFT9341_FillRect(0, 122, 159, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 122, 5, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 122, 159, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 174, 159, 179, COLOR_WHITE);//spodni
		//reproduktor
		TFT9341_FillRect(162, 122, 319, 127, COLOR_WHITE);//horni
		TFT9341_FillRect(162, 122, 167, 179, COLOR_WHITE);//levy
		TFT9341_FillRect(314, 122, 319, 179, COLOR_WHITE);//pravy
		TFT9341_FillRect(162, 174, 319, 179, COLOR_WHITE);//spodni
		//led diody
		TFT9341_FillRect(0, 182, 159, 187, COLOR_WHITE);//horni
		TFT9341_FillRect(0, 182, 5, 239, COLOR_WHITE);//levy
		TFT9341_FillRect(154, 182, 159, 239, COLOR_WHITE);//pravy
		TFT9341_FillRect(0, 234, 159, 239, COLOR_WHITE);//spodni
		break;
	}
}
