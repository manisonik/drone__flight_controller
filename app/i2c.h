#ifndef __I2C_H
#define __I2C_H

#include <stdlib.h>
#include <stdio.h>

void i2c_init();
void i2c_start();
void i2c_reset();
void i2c_stop();
int i2c_busy();
void i2c_deinit();
void i2c_address_direction(uint8_t address, uint8_t direction);
void i2c_transmit(uint8_t byte);
uint8_t i2c_receive_ack();
uint8_t i2c_receive_nack();
void i2c_write(uint8_t address, uint8_t data);
void i2c_read(uint8_t address, uint8_t* data);
void i2c_write_register(uint8_t address, uint8_t reg, uint8_t data);
void i2c_read_register(uint8_t address, uint8_t reg, uint8_t* data);
void i2c_readmulti_register(uint8_t address, uint8_t reg, uint8_t* data, uint32_t count);

#endif // __I2C_H_
