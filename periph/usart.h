#ifndef USART_H
#define USART_H

#include "stm32f303xc.h"

#define ON 1
#define OFF 0

#define BUFF_SIZE_READ 256
#define BUFF_SIZE_WRITE 256

typedef struct
{ 
    USART_TypeDef* usart_base;
    uint32_t baud_rate;
    uint8_t data_bit;
    uint8_t stop_bit;
    uint8_t parity;
    uint8_t parity_type;
    uint8_t port_num;
} usart_config;

typedef struct
{ 
    uint32_t received[BUFF_SIZE_READ];
    uint32_t transmitt[BUFF_SIZE_WRITE];
} usart_buff;

void usart_init(USART_TypeDef* usart_inter, usart_config* config);
void usart_enable(USART_TypeDef* usart_inter,uint8_t enable);

void usart_write(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);
void usart_read(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);
void usart_write_dma(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);
void usart_read_dma(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);

//interrup function redefenition for uart
//void USART1_IRQHandler();                            
//void USART2_IRQHandler();
//void USART3_IRQHandler();

#endif
