#ifndef TIMER_H
#define TIMER_H

#include "stm32f103xb.h"

#define ON 1
#define OFF 0

#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)


typedef struct
{ 
    uint32_t baud_rate;
    uint8_t data_bit_count;
    uint8_t stop_bit;
} timer_config;


void timer_init(TIM_TypeDef* timer_inter, timer_config* config);
void timer_enable(TIM_TypeDef* timer_inter, uint8_t enable);

//interrup function redefenition for timer
//void USART1_IRQHandler();                            
//void USART2_IRQHandler();
//void USART3_IRQHandler();

#endif
