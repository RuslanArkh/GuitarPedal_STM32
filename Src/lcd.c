
#include "lcd.h"

void delay(void)
{
		uint32_t i;

		for(i=0;i<12000;i++) {}
}

void LCD_WriteData(uint8_t dt)
{
		if(((dt >> 3)&0x01)==1) {d7_set();} else {d7_reset();}
		if(((dt >> 2)&0x01)==1) {d6_set();} else {d6_reset();}
		if(((dt >> 1)&0x01)==1) {d5_set();} else {d5_reset();}
		if(((dt >> 0)&0x01)==1) {d4_set();} else {d4_reset();}
}


void LCD_Command(uint8_t dt)
{
	rs0;
	LCD_WriteData(dt>>4);
	e1;
	delay();
	//HAL_Delay(1);
	e0;
	LCD_WriteData(dt);
	e1;
	delay();
	//HAL_Delay(1);
	e0;
}

void LCD_Command4(uint8_t dt) {
	rs0;
	LCD_WriteData(dt);
	e1;
	delay();
	//HAL_Delay(1);
	e0;
}

void LCD_ini(void)
{
			
			HAL_Delay(45);
			
			LCD_Command(0x80);
			HAL_Delay(1);
			
			LCD_Command(0x28);
			HAL_Delay(1);
			
			LCD_Command(0x28);
			HAL_Delay(1);
			
			LCD_Command(0x0C);
			HAL_Delay(1);
			
			LCD_Command(0x01);
			HAL_Delay(1);
			
			LCD_Command(0x06);
			HAL_Delay(1);
			
			LCD_Command(0x02);
			HAL_Delay(2);
}

void LCD_Data(uint8_t dt)
{
        rs1;
        LCD_WriteData(dt>>4);
        e1;
        delay();
				//HAL_Delay(1);
        e0;
        LCD_WriteData(dt);
        e1;
        delay();
				//HAL_Delay(1);
        e0;
}

void LCD_Clear(void)
{
		LCD_Command(0x1);
		HAL_Delay(3);
}

void LCD_SetPos(uint8_t x, uint8_t y)
{
		switch(y)
		{

		case 0:
						LCD_Command(x|0x80);
						HAL_Delay(1);
						break;
		case 1:
						LCD_Command((0x40+x)|0x80);
						HAL_Delay(1);
						break;
		case 2:
						LCD_Command((0x14+x)|0x80);
						HAL_Delay(1);
						break;
		case 3:
						LCD_Command((0x54+x)|0x80);
						HAL_Delay(1);
						break;
		}

}

void LCD_String(char* st)
{
		uint8_t i=0;
		while(st[i]!=0)
		{
			if (i == 16)
				LCD_SetPos(0,1);
			LCD_Data((uint8_t)st[i]);
			delay();
			//HAL_Delay(2);
			i++;
		}
}
