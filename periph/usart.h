#ifndef USART_H
#define USART_H

#include "stm32f103xb.h"

#define ON 1
#define OFF 0

typedef struct
{ 
    uint32_t baud_rate;
    uint8_t data_bit_count;
    uint8_t stop_bit;
} usart_config;


void usart_init(USART_TypeDef* usart_inter, usart_config* config);
void usart_enable(USART_TypeDef* usart_inter,uint8_t enable);
void usart_gpio_enable(USART_TypeDef* usart_inter);

void usart_write(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);
void usart_read(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);

//interrup function redefenition for uart
void USART1_IRQHandler();                            
void USART2_IRQHandler();
void USART3_IRQHandler();

#endif
