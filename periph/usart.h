#ifndef USART_H
#define USART_H

#include "stm32f303xc.h"

#define ON 1
#define OFF 0

#define BUFF_SIZE_READ 256
#define BUFF_SIZE_WRITE 256

typedef struct
{ 
    //USART_TypeDef* usart_base;
    uint32_t baud_rate;
    uint8_t data_bit;
    uint8_t stop_bit;
    uint8_t parity;
    uint8_t parity_type;
    uint8_t port_num;
} usart_config;

typedef struct
{ 
    uint16_t rv_data_count;
    uint16_t rv_data_len;
    uint8_t rv_flag;

    uint8_t* received;

    void (*read_end_call) (uint16_t len);
    
    uint16_t tr_data_count;
    uint16_t tr_data_len;
    uint8_t tr_flag;

    uint8_t* transmitt;

    void (*write_end_call) (uint16_t len);
} usart_buff;

void usart_init(USART_TypeDef* usart_inter, usart_config* config);
void usart_config_from_ethernet();
void usart_enable(USART_TypeDef* usart_inter,uint8_t enable);

void usart_write(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);

void usart_write_it(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size, void (*write_end_callback) (uint16_t len));

void usart_read(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);
void usart_read_it(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size, void (*read_end_callback) (uint16_t len));

void usart_write_dma(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);
void usart_read_dma(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size);

#endif
