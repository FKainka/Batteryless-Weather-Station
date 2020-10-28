/*
 * I2C.h
 *
 * Created: 16/04/2019 3:42:38 AM
 *  Author: goo17e
 */ 


#ifndef I2C_H_
#define I2C_H_

void TWI_Init(char);
void TWI_Enable();
char TWI_Start(char);
char TWI_Read(char*,char);
char TWI_Write(char);
void TWI_Stop();

#endif /* I2C_H_ */