#include "usart.h"
#include "stm32f103xb.h"
#include "stm32f1xx.h"

//uart init
void usart_init(USART_TypeDef* usart_inter, usart_config* config)
{
    /*! Init USART GPIO*/
    //SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    usart_gpio_enable(usart_inter);
    /*! Init USART GPIO*/
    
    /*! Enable USART peripherial clock 3 uart*/
    //SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
    //SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);

    /*! USART Status register*/
    //uart_num->SR = 1;         
    
    /*! USART Data register*/
    //uart_num->DR = 1;         
    
    /*! USART baud rate set register
        --[11:0]    DIV_Mantissa - mantissa of USARTDIV
                    These 12 bits define the mantissa of the USART Divider (USARTDIV)
        --[3:0]     DIV_Fraction - fraction of USARTDIV
                    These 4 bits define the fraction of the USART Divider (USARTDIV)
    */
    usart_inter->BRR = 1;        
    
    /*! USART Control register1
        --[15]  OVER8 - USART enable
        --[13]  UE - USART enable (0 - disable, 1 - enable)
        --[12]  M - Word length (0 - 8 bit, 1 - 9 bit)
        --[11]  WAKE - Wakeup method
        --[10]  PCE - Parity control enable ()
        --[9]   PS - Parity selection
        --[8]   PEIE - PE interrupt enable (0 - disable, 1 - interrupt PE == 1)
        --[7]   TXEIE - TXE interrupt enable (0 - disable, 1 - interrupt TXE == 1)
        --[6]   TCIE - Transmission complete interrupt enable
        --[5]   RXNEIE - RXNE interrupt enable
        --[4]   IDLEIE - IDLE interrupt enable
        --[3]   TE - USART transmitter enable
        --[2]   RE - USART receiver enable
        --[1]   RWU - Receiver wakeup
        --[0]   SBK - Send break
    */
    usart_inter->CR1 |= 1 << USART_CR1_UE;        
   
    /*! USART Control register2
        --[14]  LINEN - LIN mode enable
        --[13:12]  STOP - STOP bits(00-1;01-0.5;10-2;11-1.5)
        --[11]  CLKEN - CLOCK enable (Not use uart4 and uart5)
        --[10]  CPOL - Clock POLARITY (Not use uart4 and uart5)
        --[9]   CPHA - Clock phase (Not use uart4 and uart5)
        --[8]   LBCL - Last bit clock pulse (Not use uart4 and uart5)
        --[6]   LBDIE - LIN break detection interrupt enable
        --[5]   LBDL - LIN break detection length
        --[3:0] ADD - Address of the USART node
    */   
    usart_inter->CR2 = 1;        
    
    /*! USART Control register3
        --[10]  CTSIE - CTS interrupt enable
        --[9]   CTSE - CTS enable
        --[8]   RTSE - RTS enable
        --[7]   DMAT - DMA enable transmitter
        --[6]   DMAR - DMA enable receiver
        --[5]   SCEN - Smartcard mode enable
        --[4]   NACK - Smartcard NACK enable
        --[3]   HDSEL - Half-duplex selection
        --[2]   IRLP - IrDA low-power
        --[1]   IREN - IrDA mode enable
        --[0]   EIE - Error interrupt enable
    */   
    usart_inter->CR3 = 1;        
    
    /*! USART Guard time and prescaler register (register not aviable UART4 and UART5)
        --
    */
    usart_inter->GTPR = 1;       
}

//enable uart
void usart_enable(USART_TypeDef* uart_name,uint8_t enable)
{
    if(enable == ON)
        uart_name->CR1 |= 1 << USART_CR1_UE | 1 << USART_CR1_TE | 1 << USART_CR1_TE;    
    else
        uart_name->CR1 &= ~(1 << USART_CR1_UE | 1 << USART_CR1_TE | 1 << USART_CR1_TE);
}

void usart_write(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size)
{

}

void usart_read(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size)
{

}

void usart_gpio_enable(USART_TypeDef* addr)
{
    /*
USART pinout
        Configuration                        GPIO configuration
    Full duplex                         Alternate function push-pull
    Half duplex synchronous mode        Alternate function push-pull

USARTx_TX(1)
    Full duplex                         Input floating / Input pull-up
    Half duplex synchronous mode        Not used. Can be used as a general IO

USARTx_CK
    Synchronous mode                    Alternate function push-pull

USARTx_RTS
    Hardware flow control               Alternate function push-pull

USARTx_CTS
    Hardware flow control               Input floating/ Input pull-up
    */
    if(addr == USART1_BASE)
    {
        /*USART1 pin
            RX - GPIOA10( PA10 )
            TX - GPIOA9 ( PA9 )
        */

        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

        /*PA10 Output Alternate function push-pull*/
        WRITE_REG(GPIOA->CRH, GPIO_CRH_MODE10);
        WRITE_REG(GPIOA->CRH, GPIO_CRH_CNF10); 
        /*PA9 Output Alternate function push-pull*/
        WRITE_REG(GPIOA->CRH, GPIO_CRH_MODE9);
        WRITE_REG(GPIOA->CRH, GPIO_CRH_CNF9); 
    }
    if(addr == USART2_BASE)
    {
        /*USART2 pin
            RX - GPIOA3 ( PA3 )
            TX - GPIOA2 ( PA2 )
        */
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

        /*PA3 Output Alternate function push-pull*/
        WRITE_REG(GPIOA->CRL, GPIO_CRL_MODE3);
        WRITE_REG(GPIOA->CRL, GPIO_CRL_CNF3); 
        /*PA2 Output Alternate function push-pull*/
        WRITE_REG(GPIOA->CRL, GPIO_CRL_MODE2);
        WRITE_REG(GPIOA->CRL, GPIO_CRL_CNF2); 
    }
    if(addr == USART3_BASE)
    {
        /*USART3 pin
            RX - GPIOB11 ( PB11 )
            TX - GPIOB10 ( PB10 )
        */
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);

        /*PB11 Output Alternate function push-pull*/
        WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE11);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF11); 
        /*PB10 Output Alternate function push-pull*/
        WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE10);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF10); 
    }
}   

//usart1 interrupt function
void USART1_IRQHandler()
{

}

//usart2 interrupt function                            
void USART2_IRQHandler()
{

}       

//usart3 interrupt function
void USART3_IRQHandler()
{

}