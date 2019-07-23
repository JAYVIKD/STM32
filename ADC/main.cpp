// DELAY mS VALUE CHANGED CHECK IF NOT WORKING 
// NO INTERRUPT MODE 
//CONTINOUS MODE ONLY 
//LAST CHECKED : 08-03-19







#include "stm32f10x.h"                  // Device header
//#include "RT7E_Components.h"             // Component selection
#include "RTE_Device.h"                 // Keil::Device:Startup
#include "lcd_4bit.h"
extern "C" {
 void ADC1_IRQHandler(void){
	
	int stat_adc;
	stat_adc=ADC1->DR;
	//clc();
	//lcd(stat_adc);
	//delay_ms(1000);
	
}
}
void adc_init();
float adc_cont();
float adc_data;

int main(){
		float adc_data;
		//NVIC_SetPriority(ADC1_IRQn, ((0x01<<3)|0x01));
		config_lcd();
		clc();
		lcd("adc");
		delay_ms(1000);
		lcd("jayvik1");
		delay_ms(1000);
//		_enable_irq();
		//NVIC_EnableIRQ(ADC1_IRQn);	
		adc_init();
			adc_cont();
	while(1){
	
	adc_data=adc_cont();
	clc();
	lcd(adc_data);
	delay_ms(100);
	}
}


	
void adc_init(){
	int stat_adc;
	
	RCC->APB2ENR|=0x00000200;
	RCC->CFGR|=RCC_CFGR_ADCPRE_0;
	
	
	ADC1->SMPR1=0;
	ADC1->SMPR2=0;
	ADC1->SQR1=0;
	ADC1->SQR3=10;
	//ADC1->CR1|=ADC_CR1_EOCIE;
	ADC1->CR2|=(ADC_CR2_ADON)|(ADC_CR2_CONT);//BIT 22
	delay_ms(20);
	ADC1->CR2|=ADC_CR2_ADON;
	stat_adc=ADC1->SR;
	

	
}
float adc_cont(){
	int stat_adc,data_adc;
	float results;
	float x=-(44/399);
	float y=(10100/19);
	ADC1->CR2=(ADC_CR2_ADON);
	delay_ms(1);
	ADC1->CR2|=ADC_CR2_ADON;
	stat_adc=ADC1->SR;
	while((stat_adc & 0x0002)==1);
	stat_adc=0;
	data_adc=ADC1->DR;
	results = (data_adc*3.3)/4095;
	results = (110/(results-1))-2;
	//results=data_adc;
	return (float)results;
}

