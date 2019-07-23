#include "lcd_4bit.h"

void config_lcd(){										
		//delay_ms(1000);
		//gpio clock enable 
		delay_init();
		RCC->APB2ENR|=0x00000018;
		//GPIO INIT
		GPIOC->CRL=(GPIO_CRL_MODE6|GPIO_CRL_MODE7);
		GPIOC->CRH=(GPIO_CRH_MODE8|GPIO_CRH_MODE9);	
		GPIOB->CRH=(GPIO_CRH_MODE13|GPIO_CRH_MODE14);
		//lcd config cmd
		cmd_config(0x02);
		cmd_config(0x28);
		cmd_config(0x0E);
		cmd_config(0x06);
		cmd_config(0x01);
		cmd_config(0x80);
		

}
void delay(){
	for(int i=0;i<500000;i++);
}

void cmd_config(char command){
		GPIOC->ODR=(command<<2);
		GPIOB->ODR&=~Rs;
		GPIOB->ODR|=E;
		delay_ms(lcd_delay);
		GPIOB->ODR&=~E;
	
		GPIOC->ODR=(command<<6);
		GPIOB->ODR&=~Rs;
		GPIOB->ODR|=E;
		delay_ms(lcd_delay);
		GPIOB->ODR&=~E;
	
	
}

void cmd(char command){
		GPIOC->ODR=(command<<2);
		GPIOB->ODR|=Rs;
		GPIOB->ODR|=E;
		delay_ms(lcd_delay);
		GPIOB->ODR&=~E;
	
		GPIOC->ODR=(command<<6);
		GPIOB->ODR|=Rs;
		GPIOB->ODR|=E;
		delay_ms(lcd_delay);
		GPIOB->ODR&=~E;
}

void clc(){
	cmd_config(0x01);
}

void lcd(int n){
	if(n<0){
		n=-n;
		cmd('-');
	}
	char str[16];
	int rem,len=0,num;
	num=n;
	while(num!=0){
		len++;
		num/=10;
	}
	for(int i=0;i<len;i++){
		rem=n%10;
		n=n/10;
		str[len-(i+1)]=rem+'0';
	}
	str[len]='\0';
	lcd(str);

}

void lcd(long int n){
	long int remd,rev=0;
	int count=0;
	
	
	char s[16];
	while(n!=0){
		remd=n%10;
		n=n/10;
		rev=10*rev+remd;
		remd+=48;
		s[count++]=remd;
	}
	
	char srev[16];
	for(int i=0;i<count;i++){
		srev[i]=s[count-i-1];
	}
	lcd(srev,1);

}

void lcd(char s[]){
	for(int i=0;s[i]!='\0';i++){
		cmd(s[i]);
	}
}


void lcd(char s[],int line_no=1){
	if(line_no==2)
	cmd_config(0xC0);
	for(int i=0;s[i]!='\0';i++){
		cmd(s[i]);
	}
}

void lcd(float n,int d=3){
	
	lcd((int)n);
	n-=(int)n;
	n=n*pow((float)10,d);
	cmd(46);
	lcd((int)n);
	
}

void lcd(double n,int d=3){
	
	lcd((int)n);
	n-=(int)n;
	n=n*pow((float)10,d);
	cmd(46);
	lcd((int)n);
}

void lcd(float n){
	
	lcd((int)n);
	n-=(int)n;
	n=n*pow((float)10,3);
	cmd(46);
	lcd((int)n);
	
}

void delay_init(){
	RCC->APB1ENR|=RCC_APB1ENR_TIM6EN;
}
void delay_ms(int dl){
TIM6->PSC=23999;
TIM6->ARR=dl;//23413
TIM6->CR1=0x000D;
while((TIM6->SR & 0x0001)!=1);
TIM6->SR=0;
}
void delay_us(int dl){
TIM6->PSC=23999/1000;
TIM6->ARR=dl;//23413
TIM6->CR1=0x000D;
while((TIM6->SR & 0x0001)!=1);
TIM6->SR=0;
}

