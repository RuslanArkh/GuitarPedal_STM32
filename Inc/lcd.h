#ifndef LCD_H_

#define LCD_H_ 

#include "stm32f4xx_hal.h"


#define d4_set() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)

#define d5_set() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET)

#define d6_set() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET)

#define d7_set() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET)

#define d4_reset() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)

#define d5_reset() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET)

#define d6_reset() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET)

#define d7_reset() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET)

 

#define e1 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET) 

#define e0 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET) 

#define rs1 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET) 

#define rs0 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET) 


void LCD_WriteData(uint8_t dt);

void delay(void);

void LCD_Command(uint8_t dt);

void LCD_Command4(uint8_t dt);

void LCD_ini(void);

void LCD_Data(uint8_t dt);

void LCD_Clear(void);

void LCD_SetPos(uint8_t x, uint8_t y);

void LCD_String(char* st);

#endif
