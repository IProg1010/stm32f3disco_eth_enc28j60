#ifndef GPIO_H
#define GPIO_H

#include "stm32f303xc.h"

#define ON 1
#define OFF 0

/*
typedef struct
{ 
    uint32_t baud_rate;
    uint8_t data_bit_count;
    uint8_t stop_bit;
} spi_config;

void spi_init(SPI_TypeDef* spi_interf, spi_config* config);

void spi_write(SPI_TypeDef* spi_interf, uint8_t* data, uint16_t size);
void spi_write_data(uint8_t* data, uint16_t size);
void spi_read(SPI_TypeDef* spi_interf, uint8_t* data, uint16_t size);

void spi_enable(SPI_TypeDef* spi_interf, uint8_t enable);

void spi_gpio_enable(SPI_TypeDef* addr);

//interrup function redefenition for uart
void SPI1_IRQHandler();                            
void SPI2_IRQHandler();*/

#endif