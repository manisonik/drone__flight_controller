#include "i2c.h"
#include "delay.h"
#include "stm32f10x.h"

#define I2Cx I2C1
#define I2Cx_RCC RCC_APB1Periph_I2C1
#define I2C_GPIO_RCC RCC_APB2Periph_GPIOB
#define I2C_GPIO GPIOB

void i2c_init() {
	// Initialization struct
	I2C_InitTypeDef I2C_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Initialize I2C
	RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
	I2C_InitStruct.I2C_ClockSpeed = 400000; // 400 kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStruct);
	I2C_Cmd(I2Cx, ENABLE);

	/*RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

	// Check if  I2Cx is busy; if so it may be locked up
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) {
		printf("I2C bus locked...\n\r");
		GPIO_SetBits(I2C_GPIO, GPIO_Pin_7);
		DelayUs(10000);
		GPIO_ResetBits(I2C_GPIO, GPIO_Pin_7);
	}*/

	// Initialize GPIO as open drain alternate function
	RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);
}

void i2c_start() {
    // Wait until I2Cx is not busy anymore
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    // Generate start condition
    I2C_GenerateSTART(I2Cx, ENABLE);

    // Wait for I2C EV5.
    // It means that the start condition has been correctly released
    // on the I2C bus (the bus is free, no other devices is communicating))
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop() {
    // Generate I2C stop condition
    I2C_GenerateSTOP(I2Cx, ENABLE);

    // Wait until I2C stop condition is finished
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));

    //Clear the stop flag for next transfers
    I2C_ClearFlag(I2Cx, I2C_FLAG_STOPF);
}

void i2c_address_direction(uint8_t address, uint8_t direction) {

    // Send slave address
    I2C_Send7bitAddress(I2Cx, address, direction);

    // Wait for I2C EV6
    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter) {
    	while (!I2C_CheckEvent(I2Cx,
    			I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    } else if (direction == I2C_Direction_Receiver) {
        while (!I2C_CheckEvent(I2Cx,
            I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

void i2c_transmit(uint8_t byte){
    // Send data byte
    I2C_SendData(I2Cx, byte);

    // Wait for I2C EV8_2.
    // It means that the data has been physically shifted out and
    // output on the bus)
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t i2c_receive_ack() {
    // Enable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}

uint8_t i2c_receive_nack() {
    // Disable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, DISABLE);

    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}

void i2c_write(uint8_t address, uint8_t data) {

    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Transmitter);
    i2c_transmit(data);
    i2c_stop();
}

void i2c_read(uint8_t address, uint8_t* data) {
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Receiver);
    *data = i2c_receive_nack();
    i2c_stop();
}

void i2c_readmulti_register(uint8_t address, uint8_t reg, uint8_t* data, uint32_t count) {
	i2c_start();
	i2c_address_direction(address << 1, I2C_Direction_Transmitter);
	i2c_transmit(reg);

	// Repeated start
	I2C_GenerateSTART(I2Cx, ENABLE);
	i2c_address_direction(address << 1, I2C_Direction_Receiver);

	while(count > 1) {
		*data = i2c_receive_ack();
		data++;
		count--;
	}

	// Receive last byte
	*data = i2c_receive_nack();

	i2c_stop();
}

void i2c_write_register(uint8_t address, uint8_t reg, uint8_t data) {
	i2c_start();
	i2c_address_direction(address << 1, I2C_Direction_Transmitter);
	i2c_transmit(reg);
	i2c_transmit(data);
	i2c_stop();
}

void i2c_read_register(uint8_t address, uint8_t reg, uint8_t* data) {
	i2c_start();
	i2c_address_direction(address << 1, I2C_Direction_Transmitter);
	i2c_transmit(reg);
	i2c_stop();
	// stop ??

	i2c_start();
	i2c_address_direction(address << 1, I2C_Direction_Receiver);
	*data = i2c_receive_nack();
	i2c_stop();
}

