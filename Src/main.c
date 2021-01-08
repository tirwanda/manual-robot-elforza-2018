/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
		LCD 2X16
		*PORTE0 ->	RS
		*PORTE1 ->	EN
		*PORTC6 ->	D4
		*PORTC7 ->	D5
		*PORTC8 ->	D6
		*PORTC9 ->	D7

		JOYSTICK
		*PORTA2 ->	Attn
		*PORTA5 ->	SCK
		*PORTA6 ->	MISO
		*PORTA7 ->	MOSI
		*PWM1		->	PORTE9
		*PWM2		->	PORTE11
		*PWM3		->	PORTE13
		*PWM4		->	PORTE14

		PENEUMATIC
		*PORTA1	-> Peneumatic Tz2 Maju
		*PORTA4	-> kosong
		*PORTC1	->
		*PORTB0	-> Peneumatic Tz2 Capit
		*PORTA0	-> Peneumatic Tz1 Maju
		*PORTD3	->
		*PORTC0	-> Peneumatic Tz1 Capit
		
		SENSOR
		*PORTA8		-> Limit Switch Bagian Depan Untuk ngerol
		*PORTA9		-> Proximity untuk Stop Tz2
		*PORTA10	-> Limit Switch Bagian Depan Untuk Lengan
		*PORTA11	-> Proximity Untuk Stop Tz1
		*PORTA12	-> Limit Switch Bagian Depan Untuk Lengan
		*PORTA13	-> Proximity Untuk Stop Tz1 ke 2
		*PORTA14	-> Limit Switch Bagian Belakang Untuk ngerol
		*PORTA15	-> Proximity ambil bola
		
		DIRECTION MOTOR
		*MOTOR1	
			dirA	-> 	PORTG4
			dirB	->	PORTG6
			PWM		->	PORTE9
			
		*MOTOR2	
			dirA	-> 	PORTG7
			dirB	->	PORTG5
			PWM		->	PORTE11
			
		*MOTOR3	
			dirA	-> 	PORTD10
			dirB	->	PORTG8
			PWM		->	PORTE13
			
		*MOTOR_Lengan	
			dirA	-> 	PORTG14
			dirB	->	PORTF11
			PWM		->	PORTE14

*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lcd_txt.h"

/* USER CODE BEGIN Includes */
#define M1FWD		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
#define M1RVS		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
#define M1STOP	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
#define M1REM		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
//#define	PWM1		htim1.Instance->CCR1

#define M2FWD		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
#define M2RVS		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
#define M2STOP	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
#define M2REM		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
//#define	PWM2		htim1.Instance->CCR2

#define M3FWD		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
#define M3RVS		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
#define M3STOP	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
#define M3REM		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
//#define	PWM3		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 30)

#define M4FWD		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);
#define M4RVS		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);
#define M4STOP	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET),HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);
#define M4REM		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET),HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);
//#define	PWM4		htim1.Instance->CCR4

#define Limit_Rol_Depan							(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
#define Limit_Rol_Belakang 					(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_RESET)

#define Limit_Lengan_Depan					(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET)
#define Limit_Lengan_Belakang 			(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET)

#define Proximity_Stop_Tz1 					(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET)
#define Proximity_Stop_Tz2 					(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET)
#define Proximity_Stop_Tz1_2 				(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) == GPIO_PIN_RESET)
#define PB1											 		(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)

#define Not_Limit_Rol_Depan					HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET
#define Not_Limit_Rol_Belakang 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_SET
#define Not_Proximity_Stop_Tz1_2 				(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) == GPIO_PIN_SET)
#define Not_Proximity_Stop_Tz1 					(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET)

#define Not_Limit_Lengan_Depan			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET
#define Not_Limit_Lengan_Belakang 	HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET

#define up				(PS2buf[3]==238)
#define	down			(PS2buf[3]==190)
#define	left			(PS2buf[3]==126)
#define	right			(PS2buf[3]==222)
#define	rightdown			(PS2buf[3]==158)

#define	segitiga	(PS2buf[4]==239)
#define	x					(PS2buf[4]==191)
#define	kotak			(PS2buf[4]==127)
#define	o					(PS2buf[4]==223)

#define	AnLUp			((PS2buf[1]==115)&&(PS2buf[9]==0)&&(PS2buf[8]==1))
#define	AnLDown		((PS2buf[1]==115)&&(PS2buf[8]==255))
#define	AnLLeft		((PS2buf[1]==115)&&(PS2buf[8]==122))
#define	AnLRight	((PS2buf[1]==115)&&(PS2buf[7]==255))

//#define	AnRUp			((PS2buf[6]==0) && (PS2buf[7]==122))//lama
#define	AnRUp			((PS2buf[6]==1) && (PS2buf[7]==122))//baru
//#define AnRDown		(PS2buf[6]==254)//lama
#define AnRDown		((PS2buf[1]==115)&&(PS2buf[6]==255))//baru
#define AnRRight	((PS2buf[5]==255) && (PS2buf[7]==123))
#define	AnRLeft		((PS2buf[1]==115)&&(PS2buf[5]==1))

#define	L1				(PS2buf[4]==251)
//#define	L2				(PS2buf[5]==132)//lama
#define	L2				(PS2buf[5]==122)//baru
#define L3				(PS2buf[3]==252)
#define	R1				(PS2buf[4]==247)
#define	R2				(PS2buf[4]==253)
#define	R3				(PS2buf[3]==250)

#define	L1L2			((PS2buf[4]==251)&&(PS2buf[5]==122))

#define	Start			(PS2buf[3]==246)
#define	Select		((PS2buf[3]==254) && (PS2buf[4]==254))

#define test_e	1


int p1=20;
int p2=15;
int	p3=30;

uint8_t test=0;
uint16_t eeprom_data[500],i;

#define	EEPROM_START_ADDRESS	((uint32_t)0x0801F800)
uint8_t PS2buf[10]={0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	
char buffer[30]={0};

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

uint16_t Read_Flash(uint32_t adr){
	uint16_t * Pntr = (uint16_t *)adr;
	return(*Pntr);
}

void Unlock_Flash(void){
	FLASH->KEYR=0x45670123;
	FLASH->KEYR=0xCDEF89AB;
}

void Lock_Flash(void){
	FLASH->CR=0x80000000;
}

void Erase_Flash(uint32_t adr){
	FLASH->CR|=0x00000002;
	FLASH->CR=adr;
	FLASH->CR|=0x00000040;
	while((FLASH->SR&0x00010000));
	FLASH->CR &= ~0x00000042;	
}

void Write_Flash(uint32_t adr, uint16_t data){
	FLASH->CR|=0x00000001;
	*(__IO uint16_t *)adr = data;
	while((FLASH->CR|=0x00000001));
}

void eeprom_all(){
	Unlock_Flash();
	Erase_Flash(EEPROM_START_ADDRESS);
	for(i=0; i<=499; i++){
		Write_Flash(EEPROM_START_ADDRESS+(uint32_t)(i*2), eeprom_data[i]);
	}
	Lock_Flash();
}

void stop(){
  M1STOP;
  M2STOP;
  M3STOP;
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
}

void lengan_rem(){
	M4REM;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
}

void rem(){

	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);

		M1REM;
		M2REM;
		M3REM;
}

void maju(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3){
		M1FWD;
		M2FWD;
		M3FWD;
	
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm1);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm2);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm3);
		
}

void mundur(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3){
		M1RVS;
		M2FWD;
		M3RVS;
	
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm1);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm2);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm3);
}

void mundur_kanan(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3){
		M1RVS;
		M2RVS;
		M3RVS;
	
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm1);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm2);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm3);
}

void p_kanan(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3){
  M1FWD;
  M2FWD;
  M3RVS;
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm1);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm2);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm3);
}

void p_kiri(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3){
  M1RVS;
  M2RVS;
  M3FWD;
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm1);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm2);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm3);
}

void kiri(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3){
	M1RVS;
  M2FWD;
  M3FWD;
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm1);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm2);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm3);
		
}

void kanan(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3){
	M1FWD;
  M2RVS;
  M3RVS;
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm1);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm2);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm3);
		
}

void lengan_p_kanan(uint8_t pwm1){
	M4FWD;
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm1);
}

void lengan_p_kiri(uint8_t pwm1){
	M4RVS;
	
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm1);
}

void pn_tz2_maju_on(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void pn_tz2_maju_off(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void pn_tz1_maju_on(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

void pn_tz1_maju_off(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void pn_tz2_capit_on(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void pn_tz2_capit_off(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void pn_tz1_capit_on(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
}

void pn_tz1_capit_off(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}

void test_pn(){
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	
	HAL_Delay(2000);
	
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	
}

uint8_t PS2_RWByte(uint8_t komenda, uint8_t tablica, int wielkosc){
   HAL_SPI_TransmitReceive(&hspi1, &komenda, &tablica, sizeof(uint8_t), 1);
   //HAL_SPI_Receive(&hspi1, &tablica, sizeof(tablica), 1);
   return(tablica);
}


uint8_t Get_PS2Dat(uint8_t buf[]){
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
   HAL_Delay(15/1000);
   buf[0]=PS2_RWByte(0x01, buf[0], 8);HAL_Delay(15/1000);
   buf[1]=PS2_RWByte(0x42, buf[1], 8);HAL_Delay(15/1000);
   buf[2]=PS2_RWByte(0x00, buf[2], 8);HAL_Delay(15/1000);
   buf[3]=PS2_RWByte(0x00, buf[3], 8);HAL_Delay(15/1000);
   buf[4]=PS2_RWByte(0x00, buf[4], 8);HAL_Delay(15/1000);
   buf[5]=PS2_RWByte(0x00, buf[5], 8);HAL_Delay(15/1000);
   buf[6]=PS2_RWByte(0x00, buf[6], 8);HAL_Delay(15/1000);
   buf[7]=PS2_RWByte(0x00, buf[7], 8);HAL_Delay(15/1000);
   buf[8]=PS2_RWByte(0x00, buf[8], 8);HAL_Delay(15/1000);
	 buf[9]=PS2_RWByte(0x00, buf[9], 8);HAL_Delay(15/1000);
	
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	
	 if((buf[0]==0xff)&&(buf[1]==0x41)&&(buf[2]==0x5a))
   return 1;
   if((buf[0]==0xff)&&(buf[1]==0x73)&&(buf[2]==0x5a))
   return 2;
   return 0;
}

void save(){
	Unlock_Flash();
	Write_Flash(EEPROM_START_ADDRESS+(uint32_t)(test_e * 2), test);
}

void load(){
	Unlock_Flash();
	test=Read_Flash(EEPROM_START_ADDRESS+(uint32_t)(test_e * 2));
}



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

osThreadId defaultTaskHandle;
osThreadId thread2Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
void Thread1(void const * argument);
void Thread2(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	HAL_Init();  
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
	
	HAL_SPI_MspInit(&hspi1);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	lcd_init();
	
  /* USER CODE BEGIN 1 */
/*===============================Start Menu=====================================*/	
/*
	panel1:
	while(1){
		lcd_puts(0,0,(int8_t*)"   KRAI_2018   ");
		lcd_puts(0,0,(int8_t*)"Setting    Start");
		Get_PS2Dat(PS2buf);
	
		while(1){
			Get_PS2Dat(PS2buf);
		
			if(Select){while(Select){};Get_PS2Dat(PS2buf); test++; save();}
			if(Start){while(Start){}; Get_PS2Dat(PS2buf); test--; save();}
			
			
			
			sprintf(buffer,"%3d",test);
			lcd_puts(1,0, (int8_t*)buffer);
		
		}
	
		panel2:
		lcd_clear();
		while(1){
		}
	}
*/	
/*===============================End Menu=====================================*/

	/*
	step1:
	while(Limit_Lengan_Depan){
		lengan_rem();
		Get_PS2Dat(PS2buf);
		stop();
		lcd_puts(0,0,(int8_t*)"Step1");
		
		if(up){	
			lcd_clear();
		
			while(Limit_Lengan_Depan){
				lengan_rem();
				Get_PS2Dat(PS2buf);
			
				lcd_puts(1,0,(int8_t*)"Up");
				HAL_Delay(10);
				maju(40, 25, 60);
				lcd_clear();
			
				if(!up){
					rem();
					goto step1;
				}
			}
		}
		
		else if(down){	
			lcd_clear();
			
			while(Limit_Lengan_Depan){
				lengan_rem();
				Get_PS2Dat(PS2buf);
			
				lcd_puts(1,0,(int8_t*)"Down");
				HAL_Delay(10);
				mundur(100, 25, 85);
				lcd_clear();
			
				if(!down){
					rem();
					goto start;
				}
			}
		}
		
		else if(left){
			lcd_clear();
			
			while(Limit_Lengan_Depan){
				lengan_rem();
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"Left");
				HAL_Delay(10);
				p_kiri(15, 15, 15);
				
				if(!left)goto step1;
			}
		}
		
	}
	
	while(Not_Limit_Lengan_Depan){
		Get_PS2Dat(PS2buf);
		stop();
		lengan_p_kiri(20);
		lcd_puts(0,0,(int8_t*)"Step1.2");
		
		if(up){	
			lcd_clear();
		
			while(Not_Limit_Lengan_Depan){
				lengan_p_kiri(20);
				Get_PS2Dat(PS2buf);
			
				lcd_puts(1,0,(int8_t*)"Up");
				HAL_Delay(10);
				lengan_p_kiri(20);
				maju(40, 25, 60);
				lcd_clear();
			
				if(!up){
					rem();
					goto step1;
				}
			}
		}
	}
	*/
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, Thread1, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of thread2 */
  osThreadDef(thread2, Thread2, osPriorityIdle, 0, 128);
  thread2Handle = osThreadCreate(osThread(thread2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  start:
	while (1){
		Get_PS2Dat(PS2buf);
		stop();
		lengan_rem();
		
		/*	
		sprintf(buffer,"%3d",p1);
		lcd_puts(1,0, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",p2);
		lcd_puts(1,7, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",p3);
		lcd_puts(1,12, (int8_t*)buffer);
	
		if(L2){
			p1=p1+5;
			HAL_Delay(50);
		}
	
		if(R2){
			p3=p3+5;
			HAL_Delay(50);
		}
	
		if(R1){
			p2=p2+5;
			HAL_Delay(50);
		}
	
		if(kotak){
			p1=p1-5;
			HAL_Delay(50);
		}
	
		if(x){
			p2=p2-5;
			HAL_Delay(50);
		}
	
		if(o){
			p3=p3-5;
			HAL_Delay(50);
		}
		*/	
		
		if(Start){
			while(1){
				lcd_clear();
				Get_PS2Dat(PS2buf);
			
				if(Limit_Rol_Depan){
					lcd_puts(1,0,(int8_t*)"OK_1");
					HAL_Delay(100);
					lcd_clear();
				}
	
				if(Limit_Rol_Belakang){
					lcd_puts(1,4,(int8_t*)"OK_2");
					HAL_Delay(100);
					lcd_clear();
				}

				if(Limit_Lengan_Depan){
					lcd_puts(1,9,(int8_t*)"OK_3");
					HAL_Delay(100);
					lcd_clear();
				}
	
				if(Limit_Lengan_Belakang){
					lcd_puts(1,13,(int8_t*)"OK_4");
					HAL_Delay(100);
					lcd_clear();
				}
		
				if(Proximity_Stop_Tz1){
					lcd_puts(0,0,(int8_t*)"Proximity1");
					HAL_Delay(100);
					lcd_clear();
				}
				
				if(Proximity_Stop_Tz2){
					lcd_puts(0,10,(int8_t*)"Proximity2");
					HAL_Delay(100);
					lcd_clear();
				}
				
				if(Not_Limit_Rol_Depan){
					lcd_puts(1,0,(int8_t*)"Off1");
					HAL_Delay(100);
					lcd_clear();
				}
	
				if(Not_Limit_Rol_Belakang){
					lcd_puts(1,4,(int8_t*)"Off2");
					HAL_Delay(100);
					lcd_clear();
				}

				if(Not_Limit_Lengan_Depan){
					lcd_puts(1,9,(int8_t*)"Off3");
					HAL_Delay(100);
					lcd_clear();
				}
	
				if(Not_Limit_Lengan_Belakang){
					lcd_puts(1,13,(int8_t*)"Off4");
					HAL_Delay(100);
					lcd_clear();
				}
				if(Proximity_Stop_Tz1_2){
					lcd_puts(0,5,(int8_t*)"Proximity3");
					HAL_Delay(100);
					lcd_clear();
				}
				pn_tz1_maju_on();
															
				if(!Start)goto start;
			}
		}
		
		/*
		if(Not_Limit_Rol_Depan){
			lcd_puts(1,0,(int8_t*)"Off1");
			HAL_Delay(100);
			lcd_clear();
		}
	
		if(Not_Limit_Rol_Belakang){
			lcd_puts(1,4,(int8_t*)"Off2");
			HAL_Delay(100);
			lcd_clear();
		}

		if(Not_Limit_Lengan_Depan){
			lcd_puts(1,9,(int8_t*)"Off3");
			HAL_Delay(100);
			lcd_clear();
		}
	
		if(Not_Limit_Lengan_Belakang){
			lcd_puts(1,13,(int8_t*)"Off4");
			HAL_Delay(100);
			lcd_clear();
		}	
		*/

		


		if(up){	
			lcd_clear();
			//pn_tz2_maju_on();
		
			while(1){
				Get_PS2Dat(PS2buf);
			
				lcd_puts(1,0,(int8_t*)"Up");
				HAL_Delay(10);
				maju(70, 27, 95);
				lengan_p_kiri(25);
				lcd_clear();
				
				if(Limit_Lengan_Depan){
					pn_tz1_maju_off();
					//pn_tz2_maju_off();
					//maju(70, 25, 90);
					lengan_rem();
					//break;
				}
			
				if(!up){
					rem();
					HAL_Delay(400);
					goto start;
				}
			}
		}
	
		else if(down){	
			lcd_clear();
			pn_tz1_maju_off();
			pn_tz2_maju_off();
			
			while(1){
				Get_PS2Dat(PS2buf);
			
				lcd_puts(1,0,(int8_t*)"Down");
				HAL_Delay(10);
				mundur(100, 25, 85);
				lcd_clear();
				
				if(rightdown){
					lcd_clear();
			
					while(1){
						Get_PS2Dat(PS2buf);
				
						lcd_puts(1,0,(int8_t*)"Serong");
						HAL_Delay(10);
						mundur_kanan(80, 25, 100); //50,23,84
						lcd_clear();
						
						if(Select){
							rem();
							HAL_Delay(700);
							goto start;
						}
						
						if(Limit_Rol_Belakang){
							
							while(1){
								mundur_kanan(35, 17, 70);
								lengan_p_kanan(30);
								
								if((Not_Limit_Rol_Belakang) && (Proximity_Stop_Tz2)){
									rem();
									
									M2FWD;
									__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 18);
									HAL_Delay(300);
									
									while(1){
										mundur(25, 0, 26);
										
										if(Not_Proximity_Stop_Tz1_2){
											rem();
											HAL_Delay(400);
											maju(25, 0, 25);
											
											M2RVS;
											__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
											
											HAL_Delay(200);
											rem();
											M2FWD;
											__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
											HAL_Delay(200);
											rem();
											
											
											while(Proximity_Stop_Tz1_2){
												mundur(18, 0, 18);
												
											}
											rem();
											
											//pn_tz1_maju_on();
											HAL_Delay(400);
											
											if(Not_Proximity_Stop_Tz1){
												while(1){
													kanan(15, 40, 20);
													
													if(Proximity_Stop_Tz1){
														rem();
														HAL_Delay(400);
														goto tz1;
													}
													
												}
											}
											
											while(1){
												tz1:
												stop();
												Get_PS2Dat(PS2buf);
												
											 if(AnRUp){
													lcd_clear();
													
													while(1){
														Get_PS2Dat(PS2buf);
														
														lcd_puts(1,0,(int8_t*)"An.R.Up");
														HAL_Delay(20);
														maju(20, 0, 20);
														
														if(!AnRUp)goto tz1;
													}
												}
														
												else if(AnRDown){
													lcd_clear();
													
													while(1){
														Get_PS2Dat(PS2buf);
														
														lcd_puts(1,0,(int8_t*)"An.R.Down");
														HAL_Delay(20);
														mundur(20, 0, 20);
														
														if(!AnRDown)goto tz1;
													}
												}
												
												else if(AnRRight){
													lcd_clear();
													
													while(1){
														Get_PS2Dat(PS2buf);
														
														lcd_puts(1,0,(int8_t*)"An.R.Right");
														HAL_Delay(20);
														kanan(15, 40, 20);
														
														if(!AnRRight)goto tz1;
													}
												}
													
												else if(AnRLeft){
													lcd_clear();
													
													while(1){
														Get_PS2Dat(PS2buf);
														
														lcd_puts(1,0,(int8_t*)"An.R.Left");
														HAL_Delay(20);
														kiri(20, 35, 15);
														
														if(!AnRLeft)goto tz1;
													}
												}
												
												else if(left){
													lcd_clear();
													
													while(1){
														Get_PS2Dat(PS2buf);
														
														lcd_puts(1,0,(int8_t*)"Left");
														HAL_Delay(10);
														p_kiri(20, 20, 20);
														
														if(!left)goto tz1;
													}
												}
												

												else if(right){
													lcd_clear();
													
													while(1){
														Get_PS2Dat(PS2buf);
														
														lcd_puts(1,0,(int8_t*)"Right");
														HAL_Delay(10);
														p_kanan(20, 20, 20);

														if(!right)goto tz1;
													}
												}
												
												else if(kotak){
													lcd_clear();
													
													//while(1){
														//Get_PS2Dat(PS2buf);
														
														pn_tz1_maju_on();
														HAL_Delay(350);
														pn_tz1_capit_off();
														HAL_Delay(100);
														pn_tz1_maju_off();
														goto start;
														
														//if(!kotak)goto start;
													//}
													
													/*
													Get_PS2Dat(PS2buf);
													lcd_puts(1,0,(int8_t*)"Kotak");
													pn_tz1_capit_off();
													HAL_Delay(400);
													pn_tz1_maju_off();
													HAL_Delay(300);
													maju(25, 0, 25);
													HAL_Delay(130);
													rem();
													HAL_Delay(50);
													goto start;
													*/
												}
											}
										}
									}	
								}						
							}
						}
					}
				}
				
				if(!down){
					rem();
					HAL_Delay(500);
					goto start;
				}
			}
		}
	
	
		else if(left){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"Left");
				HAL_Delay(10);
				p_kiri(20, 20, 20);
				
				if(!left)goto start;
			}
		}
		

		else if(right){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"Right");
				HAL_Delay(10);
				p_kanan(20, 20, 20);

				if(!right)goto start;
			}
		}

/*
		if(rightdown){	
			lcd_clear();
	
			while(1){
				Get_PS2Dat(PS2buf);
		
				lcd_puts(1,0,(int8_t*)"Serong");
				HAL_Delay(10);
				mundur_kanan(55, 25, 82); //70,30,95
				lcd_clear();
		
				if(Select){
					rem();
					HAL_Delay(700);
					goto start;
				}
		
				if(Proximity_Stop_Tz1){
					rem();
					HAL_Delay(200);
						
					M2RVS;
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
					
					HAL_Delay(300);
					goto start;
				}
	
				if((Limit_Rol_Depan) && (Limit_Rol_Belakang)){

					while(1){
						mundur_kanan(60, 25, 75);
						lengan_p_kanan(30);
				
			
						if(Limit_Lengan_Belakang){
							lengan_rem();
							HAL_Delay(500);
							break;
						}
				
						if(Select){
							rem();
							HAL_Delay(700);
							goto start;
						}
			
						if((Limit_Rol_Depan) && (Not_Limit_Rol_Belakang)){
							while(1){
								mundur(20, 25, 30);
						
								if(Select){
									rem();
									HAL_Delay(700);
									goto start;
								}
						
								if(Proximity_Stop_Tz1){
									rem();
									HAL_Delay(200);
							
									M2RVS;
									__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
							
									HAL_Delay(300);
									goto start;
								}
						
								while((Not_Limit_Rol_Depan) && (Not_Limit_Rol_Belakang)){
									mundur(20, 15, 20);
							
									if(Proximity_Stop_Tz1){
										while(1){
											rem();
											HAL_Delay(300);
								
											M2RVS;
											__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
								
											HAL_Delay(300);
											
											while(1){
												mundur_kanan(25, 0, 25);
												
												if(Not_Proximity_Stop_Tz1_2){
													rem();
													HAL_Delay(500);
													pn_tz1_maju_on();
													HAL_Delay(600);
													goto start;															
												}
												
												if(Proximity_Stop_Tz1_2){
													mundur_kanan(25, 0, 25);												
												}
												
												if(Select){
													rem();
													HAL_Delay(700);
													goto start;
												}
											}
										}
									}
							
									if(Select){
										rem();
										HAL_Delay(700);
										goto start;
									}
								}
							}
						}
					}		
				}
			}
		}
*/
		else if(x){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				lcd_puts(1,0,(int8_t*)"X");
				HAL_Delay(20);	
				pn_tz1_capit_on();
				pn_tz2_capit_on();

				if(!x)goto start;
			}
		}
		
		else if(segitiga){
			lcd_clear();
			
			while(1){
				
				Get_PS2Dat(PS2buf);
				//maju(70, 27, 95);
				//lengan_p_kiri(25);
				
				//if(Limit_Lengan_Depan){
					//pn_tz1_maju_off();
					////pn_tz2_maju_off();
					////maju(70, 25, 90);
					//lengan_rem();
					////break;
				//}
				
				//if(Proximity_Stop_Tz1_Bola){
					//HAL_Delay(10);
					//rem();
					//HAL_Delay(400);
					//pn_tz1_capit_on();
					//pn_tz2_capit_on();
					//HAL_Delay(100);
					//goto start;
				//}
				
				//if(Select){
					//rem();
					//HAL_Delay(400);
					//goto start;
				//}
				
				Get_PS2Dat(PS2buf);
				lcd_puts(1,0,(int8_t*)"Segitiga");
				HAL_Delay(20);
				pn_tz2_maju_off();
				pn_tz1_maju_off();

				if(!segitiga)goto start;
				
			}
		}
		
		else if(kotak){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				lcd_puts(1,0,(int8_t*)"Kotak");
				HAL_Delay(20);
				pn_tz1_capit_off();
				HAL_Delay(400);
				pn_tz1_maju_off();

				if(!kotak)goto start;
			}
		}
		
		else if(o){
			lcd_clear();
			Get_PS2Dat(PS2buf);
			lcd_puts(1,0,(int8_t*)"O");
			pn_tz2_maju_on();
			HAL_Delay(450);
			pn_tz2_capit_off();
		}
		
		else if(AnLUp){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.L.Up");
				maju(50, 0, 50);
				HAL_Delay(10);
				
				if(!AnLUp)goto start;
			}
		}


		else if(AnLDown){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.L.Down");
				HAL_Delay(10);
				mundur(50, 0, 50);
				
				if(!AnLDown)goto start;
			}
		}
	
		else if(AnLLeft){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.L.Left");
				HAL_Delay(20);
				kiri(33, 60, 35);
				
				if(!AnLLeft)goto start;
			}
		}
		
		else if(AnLRight){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.R.Right");
				HAL_Delay(20);
				kanan(35, 60, 35);
				
				if(!AnLRight)goto start;
			}
		}

		else if(AnRUp){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.R.Up");
				HAL_Delay(20);
				maju(20, 0, 20);
				
				if(!AnRUp)goto start;
			}
		}
				
		else if(AnRDown){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.R.Down");
				HAL_Delay(20);
				mundur(20, 0, 20);
				
				if(!AnRDown)goto start;
			}
		}
		
		else if(AnRRight){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.R.Right");
				HAL_Delay(20);
				kanan(15, 40, 20);
				
				if(!AnRRight)goto start;
			}
		}
			
		else if(AnRLeft){
			lcd_clear();
			
			while(1){
				Get_PS2Dat(PS2buf);
				
				lcd_puts(1,0,(int8_t*)"An.R.Left");
				HAL_Delay(20);
				kiri(20, 35, 15);
				
				if(!AnRLeft)goto start;
			}
		}
	
		if(R2){
			lcd_clear();
		
			while(1){
				Get_PS2Dat(PS2buf);
			
				lengan_p_kiri(25);
			
				if(Limit_Lengan_Depan){
					lengan_rem();
					goto start;
				}
			
				if(!R2)goto start;
			}
		}
	
		if(R1){
			lcd_clear();
		
			while(1){
				Get_PS2Dat(PS2buf);
			
				lengan_p_kanan(25);
			
				if(Limit_Lengan_Belakang){
					lengan_rem();
					goto start;
				}
				if(!R1)goto start;
			}
		}
	
		if(L2){
			lcd_clear();
			HAL_Delay(100);
			pn_tz2_capit_on();
		
			while(1){
				Get_PS2Dat(PS2buf);
				mundur(100, 25, 85);
			
				if(L1L2){
					lcd_clear();
					mundur(100, 25, 100);
					HAL_Delay(3200);
				
					while(1){
						Get_PS2Dat(PS2buf);
					
						lcd_puts(1,0,(int8_t*)"L1 L2");
						mundur(76, 25, 40);
						lengan_p_kanan(35);
					
						if(Limit_Lengan_Belakang){
							lengan_rem();
							pn_tz2_maju_on();
							//break;
						}
						
						/*if(x){
							lcd_clear();
							
							while(1){
								Get_PS2Dat(PS2buf);
								mundur(76, 25, 40); //25
									
								if(Proximity_Stop_Tz2){
									HAL_Delay(120);
									rem();
									HAL_Delay(250);
									pn_tz2_capit_off();
									M2RVS;
									__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
									HAL_Delay(200);
									pn_tz2_maju_off();
									
									goto start;
								}
								
								if(Select){
									rem();
									HAL_Delay(700);
									goto start;
								}
							}
						}*/
						
						
						if(Proximity_Stop_Tz2){
							rem();
							HAL_Delay(350);
							pn_tz2_capit_off();
							M2RVS;
							__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
							HAL_Delay(400);
							pn_tz2_maju_off();
							
							goto start;
						}
						
						if(Select){
							rem();
							HAL_Delay(700);
							goto start;
						}
					}
				}

			if(!L2){
				rem();
				HAL_Delay(700);
				goto start;
			}
		}
	}
		
	if(L1){
		lcd_clear();
		
		while(1){
			Get_PS2Dat(PS2buf);
			lcd_puts(1,0,(int8_t*)"L1");
			maju(50, 50, 90);
			pn_tz2_maju_off();
			pn_tz1_maju_off();
			
			if(!L1)goto start;
		}
	}
	
	if(L1L2){
		lcd_clear();
			
		while(1){
			Get_PS2Dat(PS2buf);
			
			lcd_puts(1,0,(int8_t*)"L1 L2");		
			mundur(100, 25, 100);
			lengan_p_kanan(30);
				
			if(Limit_Lengan_Belakang){
				lengan_rem();
				break;
			}
					
			if(Proximity_Stop_Tz2){
				rem();
				pn_tz2_capit_off();
				HAL_Delay(400);
				M2RVS;
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
				HAL_Delay(300);
				pn_tz2_maju_off();
				
				goto start;
			}
						
			if(Select){
				rem();
				HAL_Delay(700);
				goto start;
			}
		}
	}
	
/*	if(R3){
		lcd_clear();
		
		while(1){
			Get_PS2Dat(PS2buf);
			maju(70, 27, 95);
			
			if(Proximity_Stop_Tz1_Bola){
				rem();
				HAL_Delay(300);
				pn_tz1_capit_on();
				pn_tz2_capit_on();
				HAL_Delay(100);
				goto start;
			}
			
			if(Select){
				rem();
				HAL_Delay(400);
				goto start;
			}
		
		}
	}*/

	if(L3){
		lcd_clear();
		
		while(1){
			
			lcd_clear();
					mundur(100, 25, 100);
					HAL_Delay(3200);
				
					while(1){
						Get_PS2Dat(PS2buf);
					
						lcd_puts(1,0,(int8_t*)"L3");
						mundur(76, 25, 40);
												
						if(Proximity_Stop_Tz2){
							
							
							rem();
							HAL_Delay(350);
							pn_tz2_maju_on();
							
							while(1){
								lengan_p_kanan(35);
					
						if(Limit_Lengan_Belakang){
							HAL_Delay(100);
							lengan_rem();
							pn_tz2_capit_off();
							M2RVS;
							__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 20);
							HAL_Delay(400);
							pn_tz2_maju_off();
							
							goto start;
							
	}
													
							//break;
						}
							
							
							
						}
						
						if(Select){
							rem();
							HAL_Delay(700);
							goto start;
						}
					}
				}
			
			/*Get_PS2Dat(PS2buf);
			maju(30, 20, 37);
			if(!L3){
				rem();
				HAL_Delay(300);
				goto start;
			}*/
		}
	
	
/*
		sprintf(buffer,"%3d",PS2buf[8]);
		lcd_puts(0,0, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",PS2buf[1]);
		lcd_puts(0,4, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",PS2buf[9]);
		lcd_puts(0,8, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",PS2buf[3]);
		lcd_puts(0,12, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",PS2buf[4]);
		lcd_puts(1,0, (int8_t*)buffer);

		sprintf(buffer,"%3d",PS2buf[5]);
		lcd_puts(1,4, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",PS2buf[6]);
		lcd_puts(1,8, (int8_t*)buffer);
		
		sprintf(buffer,"%3d",PS2buf[7]);
		lcd_puts(1,12, (int8_t*)buffer);
*/
  }


}
  /* USER CODE END 3 */


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC6 PC7 
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG4 PG5 PG6 PG7 
                           PG8 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 
                           PA12 PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* Thread1 function */
void Thread1(void const * argument)
{

  for(;;)
  {
if(PB1){
			lcd_clear();
		
			while(1){
				lengan_p_kanan(25);
			
				if(Limit_Lengan_Belakang){
					lengan_rem();				
				}			
		}
	}			

		osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* Thread2 function */
void Thread2(void const * argument)
{
  /* USER CODE BEGIN Thread2 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Thread2 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
