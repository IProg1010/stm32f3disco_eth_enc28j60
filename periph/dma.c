#include "dma.h"

//uart init
void dma_init(DMA_TypeDef* dma_inter, DMA_Channel_TypeDef* dma_channel, dma_config* config)
{
    //enable clock for dma peripherial
    RCC->AHBENR |= 1 << RCC_AHBENR_DMA1EN;
    SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);

    dma_inter->ISR = 1;
    dma_inter->IFCR = 1;

    /*! DMA channel config register
        --[14]  MEM2MEM - Memory to memory mode
        --[13:12]  PL - Channel priority level
        --[11:10]  MSIZE - Memory size
        --[9:8]  PSIZE - Peripheral size
        --[7]   MINC - Memory increment mode
        --[6]   PINC - Peripheral increment mode
        --[5]   CIRC - Circular mode
        --[4]   DIR - Data transfer direction
        --[3]   TEIE - Transfer error interrupt enable
        --[2]   HTIE - Half transfer interrupt enable
        --[1]   TCIE - Transfer complete interrupt enable
        --[0]   EN - Channel enable
    */
    dma_channel->CCR = 1;
    
    //Disable cyclic mode 
    CLEAR_BIT(dma_channel->CCR, DMA_CCR_CIRC);
    //Peripherial address not increment 
    CLEAR_BIT(dma_channel->CCR, DMA_CCR_PINC);
    //Memory address increment
    SET_BIT(dma_channel->CCR, DMA_CCR_MINC);

    //Set peripherial data width
    MODIFY_REG(dma_channel->CCR, DMA_CCR_PSIZE_1, DMA_CCR_PSIZE_0);
    //Set memory data width
    MODIFY_REG(dma_channel->CCR, DMA_CCR_MSIZE_1, DMA_CCR_MSIZE_0);
    
    //Enable Transfer complete interrupt 
    SET_BIT(dma_channel->CCR, DMA_CCR_TCIE);
    //Enable Transfer error interrupt 
    SET_BIT(dma_channel->CCR, DMA_CCR_TEIE);
    
    /*! DMA channel number of data register
        --[15:0]  data number
    */
    //dma_channel->CNDTR = 1;
    
    /*! DMA channel peripherial address register
        --[31:0]  address
    */
    //dma_channel->CPAR = 1; 
    
    /*! DMA channel memory address register
        --[31:0]  address
    */
    //dma_channel->CMAR = 1;
}
//enable uart
void dma_enable(DMA_Channel_TypeDef* dma_channel, uint8_t enable)
{
    if(enable == ON)
        dma_channel->CCR |= 1 << DMA_CCR_EN;
    else
        dma_channel->CCR &= ~(1 << DMA_CCR_EN);
}



//interrupt function
void DMA1_Channel1_IRQHandler()
{

}

void DMA1_Channel2_IRQHandler()
{

}

void DMA1_Channel3_IRQHandler()
{

}

void DMA1_Channel4_IRQHandler()
{

}

void DMA1_Channel5_IRQHandler()
{

}

void DMA1_Channel6_IRQHandler()
{

}

void DMA1_Channel7_IRQHandler()
{

}