/*
 * i2c.h
 *
 *  Created on: 23-Dec-2023
 *      Author: Admin
 */
#include<stdint.h>
#ifndef I2C_H_I2C_H_
#define I2C_H_I2C_H_

	typedef struct {
			uint32_t I2C_CR1;
			uint32_t I2C_CR2;
			uint32_t I2C_OAR1;
			uint32_t I2C_OAR2;
			uint32_t I2C_DR;
			uint32_t I2C_SR1;
			uint32_t I2C_SR2;
			uint32_t I2C_CCR;
			uint32_t I2C_TRISE;
		}i2c_baseaddr;
#define i2c_base  ((i2c_baseaddr*)0x40005400)

void init3(uint8_t addr);
void read(uint8_t addr,uint8_t *a,uint8_t len);
void write(uint8_t addr,uint8_t *data,int len);
void gpio();
void start();
void stop();
void ack();
void pe_enable();
void addr3(uint8_t slave);
void nack();
#endif /* I2C_H_I2C_H_ */
