
//LAST CHECKED 25-02-19
#include "stm32f10x.h"                  // Device header
#include "RTE_Device.h"                 // Keil::Device:Startup
#include "RTE_Components.h"             // Component selection
#include "lcd_4bit.h"
//defines
#define x 0
#define y 1
#define z 2


//global variables
signed int digi_t[3];
signed int digi_p[9];
uint32_t pressure;
float mpu_acc[3];
float mpu_gyro[3];
float acc_div,gyro_div;
float mpu_acc_sen[3],mpu_gyro_sen[3];
//function declarations
void spi_init();
int send_spidata(uint8_t send_data);
int mpu_dataread(uint8_t add);
int bmp280_dataread(uint8_t address);
void mpu_writedata(uint8_t address,uint8_t write_data);
void bmp280_writedata(uint8_t address,uint8_t write_data);
void bmp280_pre();
void bmp280_digiread();
float bmp280_pressure();
float bmp280_temperature();	
void mpu_setscale();
void mpu_readgyro();
void mpu_readacc();

//void mpu_calibrate();
void mpu_calibrate2();
void mpu_reset();
int main(){
	int data;
	config_lcd();
	clc();
	lcd("SPI12");
	delay_ms(1000);
	spi_init();
	delay_ms(100);
	mpu_setscale();
	clc();
	lcd("mpu calibrate");
	
	//mpu_calibrate();
	delay_ms(1000);
	while(1){
	bmp280_digiread();
	clc();
	lcd(digi_p[0]);
	delay_ms(1000);
	clc();
	lcd(digi_p[1]);
	delay_ms(1000);
	clc();
	lcd(digi_p[2]);
	delay_ms(1000);
	
		/*	mpu_readgyro();
		clc();
		lcd(mpu_gyro[0]);
		lcd("o ");
		lcd(mpu_gyro[1]);
		lcd("o ");
		lcd("",2);
		lcd(mpu_gyro[2]);
		lcd("o");
		delay_ms(100);
	*/
	}
}

void spi_init(){
	RCC->APB2ENR|=(RCC_APB2ENR_SPI1EN)|(RCC_APB2ENR_IOPAEN);
	SPI1->CR1|=(1<<5)|(1<<4)|(1<<3);//|(1<<SPI_CR1_CPHA);//toogle through all the cpha and cpol config
	SPI1->CR1|=(1<<2);//MASTER ENABLE 
	SPI1->CR1|=(1<<9);//SSM
	SPI1->CR1|=(1<<8);//SSI
	//SPI1->CR1|=(1<<7);//LSB FIRST
	
	SPI1->CR2|=SPI_CR2_SSOE;
	GPIOA->CRL=0;
	GPIOA->CRL=(1<<21)|(1<<23)|(1<<20); //1001//A5 SCL-ALT PUSH-PULL//23
	GPIOA->CRL|=(1<<31)|(1<<28)|(1<<29);//1001//A7 MOSI-ALT PUSH-PULL
	GPIOA->CRL|=(1<<27)|(1<<25)|(1<<24);//A6 MISO-INPUT FLOATING
	GPIOA->CRL|=(1<<17)|(1<<16);//A4 NSS-INPUT FLOATING//changed
	GPIOA->CRL|=(1<<13)|(1<<12);//A3 CS
	GPIOA->CRL|=(1<<9)|(1<<8);
	GPIOA->ODR|=(1<<3)|(1<<2);
	SPI1->CR1|=(1<<6);//spi enable 
}

int send_spidata(uint8_t send_data ){
		
		int data;
		SPI1->CR1|=(1<<6);
	
		SPI1->DR=send_data;
		while(SPI1->SR & SPI_SR_BSY);
		data=SPI1->DR;
		
		SPI1->CR1 &=~(1<<6);
		return data;
}
int  mpu_dataread(uint8_t address){
		int rec_data;
		address+=0x80;
		GPIOA->ODR&=~(1<<3);
		send_spidata(address);
		rec_data=send_spidata(0x00);
		GPIOA->ODR|=(1<<3);
	return rec_data;
}

void mpu_writedata(uint8_t address,uint8_t write_data){
		GPIOA->ODR&=~(1<<3);
		send_spidata(address);
		send_spidata(write_data);
		GPIOA->ODR|=(1<<3);
}

void bmp280_writedata(uint8_t address,uint8_t write_data){
		address-=0x80;
		GPIOA->ODR&=~(1<<2);
		send_spidata(address);
		send_spidata(write_data);
		GPIOA->ODR|=(1<<2);
}
int bmp280_dataread(uint8_t address){
		int rec_data;
	//	address+=0x80;
		GPIOA->ODR&=~(1<<2);
		send_spidata(address);
		rec_data=send_spidata(0x00);
		GPIOA->ODR|=(1<<2);
		return rec_data;
}

void bmp280_pre(){
		uint16_t result;
		bmp280_writedata(0xF4,0x55);
		pressure=bmp280_dataread(0xf8);
		pressure+=(bmp280_dataread(0xf7)<<8);
		
}

void mpu_readacc(){
		uint8_t data_l,data_h;
		uint16_t data;
		for(int i=0;i<3;i++){
			data_h=mpu_dataread(59+2*i);
			data_l=mpu_dataread(60+2*i);
			data=((uint16_t)data_h<<8)|(data_l);
			mpu_acc[i]=(((float)data/acc_div)-mpu_acc_sen[i]);
			
		}
		
}

void mpu_readgyro(){
		uint8_t data_l,data_h;
		uint16_t data;
		for(int i=0;i<3;i++){
			data_h=mpu_dataread(67+2*i);
			data_l=mpu_dataread(68+2*i);
			data=((uint16_t)data_h<<8)|(data_l);
			mpu_gyro[i]=(((float)data/gyro_div)-mpu_gyro_sen[i]);
}
}
		
void mpu_setscale(){
		mpu_writedata(0x1b,24);			// selecting scale gyro
		mpu_writedata(0x1c,24);			// selecting scale acc
		acc_div=2048;								// acc divider according to scale 
		gyro_div =16.4;							// gyro divider according to scale
}

void mpu_reset(){

{
  // reset device
  mpu_writedata(0x6b, 0x80);
  delay_ms(100); 
}
}
	
void mpu_calibrate2(){
uint32_t gyro_bias[3]={0,0,0},accel_bias[3]={0,0,0};
		uint8_t data[6][3]={{0,0,0},{0,0,0},{0,0,0}};
	//	uint8_t packet_count, fifo_count;
		int data_h, data_l;
	  
		mpu_writedata(0x6b,0x80);	// reset in mpu_power managment
		delay_ms(100);						// give it some time to reset 
		mpu_writedata(0x6b,0x01);
		delay_ms(100);
		mpu_writedata(0x1a,0x03);	// config register 
		mpu_writedata(0x19,0x04); //smp_lrt register write
		// gyro settings
		uint8_t c =  mpu_dataread(0x1b);
    mpu_writedata(0x1b, c & ~0xE0); // Clear self-test bits [7:5] 
    mpu_writedata(	0x1b, c & ~0x18); // Clear AFS bits [4:3]
    mpu_writedata(0x1b, c | 0 << 3);
	// itnitalize all the required data;	
	mpu_writedata(0x38,0x00);// int disable 
	mpu_writedata(0x6b,0x00); // internal clock enable 
	mpu_writedata(0x6a,0x00);
	mpu_writedata(0x6a,0x00); // disable fifo and reset it 
	mpu_writedata(0x1a,0x01); // set low pass to 188hz config registe
	mpu_writedata(0x19,0x00); // setting sampling rate to 1khz smp_lrt register
	uint16_t gyrosensitivity=131, accelsensitivity=16384;
	for(int j=0;j<6;j++){
				
			for(int i=0;i<3;i++){
				data_h=mpu_dataread(67+2*i);
				data_l=mpu_dataread(68+2*i);
				data[j][i]=((uint16_t)data_h<<8)|(data_l);
			}
			
	}
	for(int i=0;i<6;i++){
		accel_bias[0]+=data[i][0];
		accel_bias[1]+=data[i][1];
		accel_bias[2]+=data[i][2];
	}
	
	accel_bias[0]/=6;
	accel_bias[1]/=6;
	accel_bias[2]/=6;
	mpu_acc_sen[0]/=(float)accel_bias[2];
	mpu_acc_sen[1]/=(float)accel_bias[2];
	mpu_acc_sen[2]/=(float)accel_bias[2];


for(int j=0;j<6;j++){
				
			for(int i=0;i<3;i++){
				data_h=mpu_dataread(59+2*i);
				data_l=mpu_dataread(60+2*i);
				data[j][i]=((uint16_t)data_h<<8)|(data_l);
			}
			
	}
	for(int i=0;i<6;i++){
		gyro_bias[0]+=data[i][0];
		gyro_bias[1]+=data[i][1];
		gyro_bias[2]+=data[i][2];
	}
	
	gyro_bias[0]/=6;
	gyro_bias[1]/=6;
	gyro_bias[2]/=6;
	mpu_gyro_sen[0]/=(float)gyro_bias[2];
	mpu_gyro_sen[1]/=(float)gyro_bias[2];
	mpu_gyro_sen[2]/=(float)gyro_bias[2];

}

void bmp280_digiread(){
		int data_h,data_l;
		
	for(int i=0;i<3;i++){
		data_h=bmp280_dataread(0x89+i);
		data_l=bmp280_dataread(0x88+i);
		digi_t[i]=(((uint16_t)(data_h<<8))+data_l);
	}
	for(int i=0;i<9;i++){
		data_h=bmp280_dataread(0x8f +i);
		data_l=bmp280_dataread(0x8e +i);
		digi_p[i]=(((uint16_t)(data_h<<8))+data_l);
	}
}

float bmp280_temperature(signed int digi_t[3]){
	signed int var1,t;
	signed int var2;

	float temp;
	signed int  adc_T_h=bmp280_dataread(0xfa);
	signed int  adc_T_l=bmp280_dataread(0xfb);
	signed int  adc_T_x= bmp280_dataread(0xfc);
	signed int adc_T=((adc_T_x<<12)+(adc_T_h<<8)+adc_T_l);
	
	var1  = ((adc_T>>3) - (digi_t[0]<<1));
	var1=var1 * (digi_t[1]);
	var1 = var1>>11 ;
  var2 = (adc_T>>4) – (digi_t[0]);
	var2 = (var2 - (adc_T>>4) – (digi_t[1]));
	var2 = (var2 >> 12);
	var2= var2 * (digi_t[2]);
	var2= (var2 >>14);
	t=var1+var2;
	temp = ((float)((t * 5 + 128) >> 8)/100);
	return temp;
	
}

float bmp280_pressure(){
}
/*
void mpu_calibrate(){
		uint32_t gyro_bias[3]={0,0,0},accel_bias[3]={0,0,0};
		uint8_t data[12];
		uint8_t packet_count, fifo_count;
	
		mpu_writedata(0x6b,0x80);	// reset in mpu_power managment
		delay_ms(100);						// give it some time to reset 
		mpu_writedata(0x6b,0x01);
		delay_ms(100);
		mpu_writedata(0x1a,0x03);	// config register 
		mpu_writedata(0x19,0x04); //smp_lrt register write
	// gyro config	 
	uint8_t c =  mpu_dataread(0x1b);
  mpu_writedata(0x1b, c & ~0xE0); // Clear self-test bits [7:5] 
  mpu_writedata(	0x1b, c & ~0x18); // Clear AFS bits [4:3]
  mpu_writedata(0x1b, c | 0 << 3);
	// acc config 
	c =  mpu_dataread(0x1c);
  mpu_writedata(0x1c, c & ~0xE0); // Clear self-test bits [7:5] 
  mpu_writedata(0x1c, c & ~0x18); // Clear AFS bits [4:3]
  mpu_writedata(0x1c, c | 0 << 3);
	// 	i2c master disable 
	mpu_writedata(0x24,0);
	// for bias value calculations
	mpu_writedata(0x38,0x00);// int disable 
	mpu_writedata(0x6b,0x00); // internal clock enable 
	mpu_writedata(0x6a,0x00);
	mpu_writedata(0x6a,0x00); // disable fifo and reset it 
	mpu_writedata(0x1a,0x01); // set low pass to 188hz config registe
	mpu_writedata(0x19,0x00); // setting sampling rate to 1khz smp_lrt register
	uint16_t gyrosensitivity=131, accelsensitivity=16384;
	mpu_writedata(0x1a,0x40);// fifo enable 
	mpu_writedata(0x23,0x78);//fifo enable (gyro and acc)
	delay_ms(100);
	mpu_writedata(0x23,0x00);//fifo disable 
	uint8_t data_h=mpu_dataread(0x72);		//fifo count read
	uint8_t data_l=mpu_dataread(0x73);
	
	fifo_count=(((uint16_t)data_h<<8)|data_l);
	packet_count=fifo_count/12;
	
	for(int i=0;i<packet_count;i++){
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    for(int ii=0;ii<12;ii++){data[ii]=mpu_dataread(0x74+ii);} // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t) accel_temp[0]; 
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t)  accelsensitivity;}
	
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
	
	// pushing data to gyro_offset
	  mpu_writedata(0x13, data[0]);
    mpu_writedata(0x14, data[1]);
    mpu_writedata(0x15, data[2]);
    mpu_writedata(0x16, data[3]);
    mpu_writedata(0x17, data[4]);
    mpu_writedata(0x18, data[5]);
	 int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  // Read factory accelerometer trim values

	 data[0]=mpu_dataread(0x77);
	 data[1]=mpu_dataread(0x78);
	 
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  data[0]=mpu_dataread(0x7a);
	 data[1]=mpu_dataread(0x7b);
	 
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  data[0]=mpu_dataread(0x7d);
	 data[1]=mpu_dataread(0x7e);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	 mpu_gyro_sen[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  mpu_gyro_sen[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  mpu_gyro_sen[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
	 
	 accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
	
	 mpu_acc_sen[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   mpu_acc_sen[1] = (float)accel_bias[1]/(float)accelsensitivity;
   mpu_acc_sen[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

*/