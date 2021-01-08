#ifndef		__LCDTXT_H
#define		__LCDTXT_H

#include "stm32f4xx.h"


/*------------- Define LCD Use -----------------*/

/*Note: Comment which not use */

#define LCD16xN //For lcd16x2 or lcd16x4
//#define LCD20xN //For lcd20x4

/*------------- Define For Connection -----------------*/

#define RS_PORT		GPIOE
#define RS_PIN		GPIO_PIN_0

#define EN_PORT		GPIOE
#define EN_PIN		GPIO_PIN_1

#define D7_PORT		GPIOC
#define D7_PIN		GPIO_PIN_9

#define D6_PORT		GPIOC
#define D6_PIN		GPIO_PIN_8

#define D5_PORT		GPIOC
#define D5_PIN		GPIO_PIN_7

#define D4_PORT		GPIOC
#define D4_PIN		GPIO_PIN_6


/*------------ Declaring Private Macro -----------------*/

#define PIN_LOW(PORT,PIN)	HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_RESET);
#define PIN_HIGH(PORT,PIN)	HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_SET);

/*------------ Declaring Function Prototype -------------*/
void lcd_init(void);
void lcd_write(uint8_t type,uint8_t data);
void lcd_puts(uint8_t x, uint8_t y, int8_t *string);
void lcd_clear(void);
#endif

