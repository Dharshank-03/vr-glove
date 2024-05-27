/*
 * i2c.c
 *
 *  Created on: 23-Dec-2023
 *      Author: Admin
 */


#include "i2c.h"

void gpio(){
	uint32_t *rcc = (uint32_t*) 0x40023800;
	uint32_t *rcc1 = (uint32_t*) 0x40023830;
	uint32_t *rcc2 = (uint32_t*) 0x40023840;

	*rcc1 |= 1<<1;
	*rcc2 |= 1<<21;

	uint32_t *p= (uint32_t*) 0x40020400;
	uint32_t *p1= (uint32_t*) 0x40020404;
	uint32_t *p2= (uint32_t*) 0x4002040c;
	uint32_t *p22= (uint32_t*) 0x40020408;
	uint32_t *p3= (uint32_t*) 0x40020424;
	*p |=  (2<<16) | (2<<18);
	*p1 |= (1<<8) | (1<<9);
	*p2 |= (1<<16) | (1<<18);
	*p22 |= (3<<16) | (3<<18);

	*p3 |= (4<<0) | (4<<4);

}

void init3(uint8_t addr){
     gpio();
	i2c_base->I2C_CR1 |= 1<<15;
		i2c_base->I2C_CR1 &= ~(1<<15);


	i2c_base->I2C_CR2 = 0x08;

	uint16_t ccrvalue= 0xA0;

	i2c_base->I2C_OAR1 = addr<<1;
	i2c_base->I2C_OAR1 &= ~(1);
	i2c_base->I2C_OAR1 = 1<<14;
	i2c_base->I2C_CCR = 1<<15;
	i2c_base->I2C_CCR = 0x1b;
	i2c_base->I2C_TRISE = 8+1;

}
void ack(){

	i2c_base->I2C_CR1 |= 1<<10;
}
void pe_enable(){

	i2c_base->I2C_CR1 |= 1<<0;

}
void start(){

	i2c_base->I2C_CR1 |= 1<<8;
   //  i2c_base->I2C_SR1 = 1<<0;
	while (!(i2c_base->I2C_SR1 & (1<<0)));
}
void addr(uint8_t slave){
	i2c_base->I2C_DR = slave<<1;
	while (!(i2c_base->I2C_SR1 & 1<<1 ));

		int dummyread = i2c_base->I2C_SR1;
		 dummyread = i2c_base->I2C_SR2;
}
void stop(){


	while (!(i2c_base->I2C_SR1 &= 132));
			i2c_base->I2C_CR1 = 1<<9;
	}
void nack(){
	i2c_base->I2C_CR1 &= ~(1<<10);
}

void read(uint8_t addr,uint8_t *a,uint8_t len){
	pe_enable();
	start();
	ack();

	i2c_base->I2C_DR = addr<<1|1;

	//i2c_base->I2C_CR1 &= ~(1<<10);

	while (!(i2c_base->I2C_SR1 & 1<<1 ));
	int dummyread = i2c_base->I2C_SR1;
	 dummyread = i2c_base->I2C_SR2;
	 if(len==1){
	 nack();
	 stop();
	 while (!(i2c_base->I2C_SR1 & 1<<6 ));

	 *a=i2c_base->I2C_DR;

	 }
	 else{
		 ack();
		 int temp=0;
		 while(len>3){
			 while (!(i2c_base->I2C_SR1 & 1<<6 ));
				 *(a+temp)=i2c_base->I2C_DR;
			//	 printf("%d\n",*a);
				 len--;
				 temp++;
		 }

		 while(!(i2c_base->I2C_SR1 & 1<<2 ));
		 *(a+temp)=i2c_base->I2C_DR;
		 temp++;
		 nack();

		 *(a+temp)=i2c_base->I2C_DR;
		 temp++;
		 while (!(i2c_base->I2C_SR1 & 1<<6 ));
		 stop();
		 *(a+temp)=i2c_base->I2C_DR;
	 }
}

void write(uint8_t addr,uint8_t *data,int len){
	pe_enable();
	//ack();
	start();
	i2c_base->I2C_DR = addr<<1;
		while (!(i2c_base->I2C_SR1 & 1<<1 ));

			int dummyread = i2c_base->I2C_SR1;
			 dummyread = i2c_base->I2C_SR2;

	while(len>0){
		i2c_base->I2C_DR = *data;
		while (!(i2c_base->I2C_SR1 &= 1<<7));
     data++;
		len--;
	}
	stop();
	}






