#ifndef DMA_H
#define DMA_H

#include "stm32f103xb.h"

#define ON 1
#define OFF 0

typedef struct
{ 
    uint32_t baud_rate;
    uint8_t data_bit_count;
    uint8_t stop_bit;
} dma_config;


void dma_init(DMA_TypeDef* dma_interface, DMA_Channel_TypeDef* dma_channel, dma_config* config);
void dma_enable(DMA_TypeDef* dma_interface, DMA_Channel_TypeDef* dma_channel, uint8_t enable);

//interrup function redefenition for uart
void DMA1_Channel1_IRQHandler();                            
void DMA1_Channel2_IRQHandler();
void DMA1_Channel3_IRQHandler();
void DMA1_Channel4_IRQHandler();
void DMA1_Channel5_IRQHandler();
void DMA1_Channel6_IRQHandler();
void DMA1_Channel7_IRQHandler();

#endif
