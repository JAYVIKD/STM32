#ifndef _lcd4bit_h
#define _lcd4bit_h

		#include "stm32f10x.h"                  // Device header
		//#include "RTE_Components.h"             // Component selection
		//#include "RTE_Device.h" 								// Keil::Device:Startup
		#include <math.h>

		// pin assignement for lcd 4_bit mode  
		#define db4 GPIO_ODR_ODR6
		#define db5 GPIO_ODR_ODR7
		#define db6 GPIO_ODR_ODR8
		#define db7 GPIO_ODR_ODR9
		#define E 	GPIO_ODR_ODR13
		#define Rs  GPIO_ODR_ODR14
		#define lcd_delay 3
		// functions prototypes	
		void cmd_config(char command);
		void cmd(char command);
		void config_lcd();
		void clc();
		void lcd(char n);
		void lcd(char s[]);
		void lcd(char s[],int line_no);
		void lcd(int a);
		void lcd(float b);
		void lcd(float b,int d);
		void lcd(double b,int d);
		void delay_ms(int dl);
		void delay_us(int dl);
		void delay_init();

#endif
