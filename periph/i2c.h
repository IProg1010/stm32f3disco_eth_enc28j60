#ifndef I2C_H
#define I2C_H

#include "stm32f303xc.h"

#define ON 1
#define OFF 0

typedef struct
{ 
    uint32_t baud_rate;
    uint8_t data_bit_count;
    uint8_t stop_bit;
} i2c_config;

void i2c_init(I2C_TypeDef* i2c_interf, i2c_config* config);

void i2c_write(I2C_TypeDef* spi_interf, uint8_t* data, uint16_t size);
void i2c_read(I2C_TypeDef* spi_interf, uint8_t* data, uint16_t size);

void i2c_enable(I2C_TypeDef* spi_interf, uint8_t enable);

void i2c_gpio_enable(I2C_TypeDef* addr);

//interrup function redefenition for i2c
void I2C1_EV_IRQHandler();
void I2C1_ER_IRQHandler();
void I2C2_EV_IRQHandler();
void I2C2_ER_IRQHandler();

#endif
