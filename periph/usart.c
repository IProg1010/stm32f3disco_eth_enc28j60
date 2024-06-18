#include "usart.h"
#include "stm32f303xc.h"
#include "F3_MACROS.h"

void usart_gpio_enable(USART_TypeDef* usart_inter, int port_num);
void usart_dma_init(USART_TypeDef* usart_inter);
extern uint32_t SystemCoreClock;  

usart_buff usart1_buff;
usart_buff usart2_buff;
usart_buff usart3_buff;
usart_buff uart4_buff;
usart_buff uart5_buff;
//uart init
void usart_init(USART_TypeDef* usart_inter, usart_config* config)
{
    /*! Init USART GPIO*/
    //SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    usart_gpio_enable(usart_inter, config->port_num);
    /*! Init USART GPIO*/
    
    /*! Enable USART peripherial clock uart*/
    if(usart_inter == USART1_BASE)
    {
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
    }
    else if(usart_inter == USART1_BASE)
    {
        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
    }
    else if(usart_inter == USART1_BASE)
    {   
        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
    }  
    else if(usart_inter == USART1_BASE)
    { 
        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);
    }
    else if(usart_inter == USART1_BASE)
    {    
        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);
    }
    else
    {

    }

    if(usart_inter->CR1 & USART_CR1_UE)
    {
        usart_inter->CR1 &= ~USART_CR1_UE;
    }
    if(usart_inter->CR1 & USART_CR1_TE)
    {
        usart_inter->CR1 &= ~USART_CR1_TE;
    }
    if(usart_inter->CR1 & USART_CR1_RE)
    {
        usart_inter->CR1 &= ~USART_CR1_RE;
    }
    
    uint32_t temp = 0x0000; 
    /*! USART Control register1
        --[31:29] - Reserved, must be kept at reset value.
        --[28] - M1 - Word length
                            This bit, with bit 12 (M0), determines the word length. It is set or cleared by software.
                            M[1:0] = 00: 1 Start bit, 8 data bits, n stop bits
                            M[1:0] = 01: 1 Start bit, 9 data bits, n stop bits
                            M[1:0] = 10: 1 Start bit, 7 data bits, n stop bits
                            This bit can only be written when the USART is disabled (UE=0).
                            Note: Not all modes are supported In 7-bit data length mode. Refer to Section 29.4: USART
                            implementation for details.
        --[27] - EOBIE- End of Block interrupt enable
                            This bit is set and cleared by software.
                            0: Interrupt is inhibited
                            1: A USART interrupt is generated when the EOBF flag is set in the USART_ISR register.
                            Note: If the USART does not support smartcard mode, this bit is reserved and must be kept
                            at reset value. Please refer to Section 29.4: USART implementation on page 896.
        --[26] - RTOIE - Receiver timeout interrupt enable
                            This bit is set and cleared by software.
                            0: Interrupt is inhibited
                            1: An USART interrupt is generated when the RTOF bit is set in the USART_ISR register.
        --[25:21] - DEAT - Driver Enable assertion time
                            This 5-bit value defines the time between the activation of the DE (Driver Enable) signal and
                            the beginning of the start bit. It is expressed in sample time units (1/8 or 1/16 bit duration,
                            depending on the oversampling rate).
                            This bit field can only be written when the USART is disabled (UE=0).
        --[20:16]  DEDT - Driver Enable de-assertion time
                            This 5-bit value defines the time between the end of the last stop bit, in a transmitted
                            message, and the de-activation of the DE (Driver Enable) signal. It is expressed in sample
                            time units (1/8 or 1/16 bit duration, depending on the oversampling rate).
                            If the USART_TDR register is written during the DEDT time, the new data is transmitted only
                            when the DEDT and DEAT times have both elapsed.
                            This bit field can only be written when the USART is disabled (UE=0).
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
    if(config->data_bit == 8)
    {
        temp &= ~(USART_CR1_M0 | 1 << 28);
    }
    else if(config->data_bit == 9)
    {
        temp |= USART_CR1_M0;
    }
    else if(config->data_bit == 7)
    {
        temp |= 1 << 28;
    }
    
    //temp |= USART_CR1_OVER8;
    
    if(config->parity != 0)
    {
        temp |= USART_CR1_PCE;
        if(config->parity_type == 0x01)
        {
            temp |= USART_CR1_PS;
        }
        else if(config->parity_type == 0x02)
        {
            temp &= ~USART_CR1_PS;
        }
        else
        {

        }
    }

    usart_inter->CR1 = temp;        
   
    temp = 0x0000;
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

    if(config->stop_bit == 1)
    {
        temp |= USART_CR2_STOP_0;
        temp &= ~USART_CR2_STOP_1;
    }
    if(config->stop_bit == 2)
    {
        temp &= ~(USART_CR2_STOP_0 | USART_CR2_STOP_1);
    }
    if(config->stop_bit == 3)
    {
        temp |= USART_CR2_STOP_0 | USART_CR2_STOP_1;
    }
    if(config->stop_bit == 4)
    {
        temp |= USART_CR2_STOP_1;
        temp &= ~USART_CR2_STOP_0;
    }
    else
    {}
    
    usart_inter->CR2 = temp;        
    
    temp = 0x0000;
    
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
    
    usart_inter->CR3 = temp;        
    
    temp = 0x0000;
    
    
    /*! USART baud rate set register
        --[11:0]    DIV_Mantissa - mantissa of USARTDIV
                    These 12 bits define the mantissa of the USART Divider (USARTDIV)
        --[3:0]     DIV_Fraction - fraction of USARTDIV
                    These 4 bits define the fraction of the USART Divider (USARTDIV)
    */
    uint32_t usartdiv        
    if(config->baud_rate == 1)
    {
        usartdiv = SystemCoreClock/9600;
    }
    else if (config->baud_rate == 2)
    {
        usartdiv = SystemCoreClock/19200;
    }
    else if (config->baud_rate == 3)
    {
        usartdiv = SystemCoreClock/38400;
    }
    else if (config->baud_rate == 4)
    {
        usartdiv = SystemCoreClock/57600;
    }
    else
    {
        usartdiv = SystemCoreClock/115200;
    }

    usart_inter->BRR = 0x0000;
    usart_inter->BRR = usartdiv;

    /*! USART Guard time and prescaler register (register not aviable UART4 and UART5)
        --Bits 31:16 Reserved, must be kept at reset value.
            Bits 15:8 GT[7:0]: Guard time value
            This bit-field is used to program the Guard time value in terms of number of baud clock
            periods.
            This is used in smartcard mode. The Transmission Complete flag is set after this guard time
            value.
            This bit field can only be written when the USART is disabled (UE=0).
            Note: If smartcard mode is not supported, this bit is reserved and must be kept at reset value.
            Please refer to Section 29.4: USART implementation on page 896.
            Bits 7:0 PSC[7:0]: Prescaler value
            In IrDA Low-power and normal IrDA mode:
            PSC[7:0] = IrDA normal and low-power baud rate
            Used for programming the prescaler for dividing the USART source clock to achieve the low-
            power frequency:
            The source clock is divided by the value given in the register (8 significant bits):
            00000000: Reserved - do not program this value
            00000001: divides the source clock by 1
            00000010: divides the source clock by 2
            ...
            In smartcard mode:
            PSC[4:0]: Prescaler value
            Used for programming the prescaler for dividing the USART source clock to provide the
            smartcard clock.
            The value given in the register (5 significant bits) is multiplied by 2 to give the division factor
            of the source clock frequency:
            00000: Reserved - do not program this value
            00001: divides the source clock by 2
            00010: divides the source clock by 4
            00011: divides the source clock by 6
            ...
            This bit field can only be written when the USART is disabled (UE=0).
    */
    usart_inter->GTPR = 1;      

    /*! USART     */
    /*
    usart_inter->RTOR = 1;
    usart_inter->RQR = 1;  
    usart_inter->ISR = 1; 
    usart_inter->ICR = 1;

    
    usart_inter->RDR = 1;
    usart_inter->TDR = 1;   
    */
}

//init usart from usart_config
void usart_init(usart_config* config)
{
    usart(config->usart_base, config);
}

//enable uart
void usart_enable(USART_TypeDef* usart_inter,uint8_t enable)
{
    if(enable == ON)
        usart_inter->CR1 |= 1 << USART_CR1_UE | 1 << USART_CR1_RE | 1 << USART_CR1_TE;    
    else
        usart_inter->CR1 &= ~(1 << USART_CR1_UE | 1 << USART_CR1_RE | 1 << USART_CR1_TE);
}

void usart_write(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size)
{
    uint16_t i = 0;
    while(i < size)
    {
        while(!(usart_inter->ISR & USART_ISR_TXE)) { } 
        usart_inter->TDR = data;
        i++;
    }
}

void usart_read(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size)
{
    uint16_t i = 0;
    while(i < size)
    {
        while(!(usart_inter->ISR & USART_ISR_RXNE)) { } 
        data[i] = usart_inter->RDR;
        i++;
    }
}

void usart_write_dma(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size)
{
    uint16_t i = 0;
    while(i < size)
    {
        while(!(usart_inter->ISR & USART_ISR_TXE)) { } 
        usart_inter->TDR = data;
        i++;
    }
}

void usart_read_dma(USART_TypeDef* usart_inter, uint8_t* data, uint16_t size)
{
    uint16_t i = 0;
    while(i < size)
    {
        while(!(usart_inter->ISR & USART_ISR_TXE)) { } 
        usart_inter->TDR = data;
        i++;
    }
}

void usart_dma_init(USART_TypeDef* usart_inter)
{
    DMA_TypeDef* dma;
    DMA_Channel_TypeDef* dma_channel_tx;
    DMA_Channel_TypeDef* dma_channel_rx;

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
    //dma_channel->CCR = 1;
    
    //Disable cyclic mode 
    //CLEAR_BIT(dma_channel->CCR, DMA_CCR_CIRC);
    //Peripherial address not increment 
    //CLEAR_BIT(dma_channel->CCR, DMA_CCR_PINC);
    //Memory address increment
    //SET_BIT(dma_channel->CCR, DMA_CCR_MINC);

    //Set peripherial data width
    ///MODIFY_REG(dma_channel->CCR, DMA_CCR_PSIZE_1, DMA_CCR_PSIZE_0);
    //Set memory data width
    //MODIFY_REG(dma_channel->CCR, DMA_CCR_MSIZE_1, DMA_CCR_MSIZE_0);
    
    //Enable Transfer complete interrupt 
    //SET_BIT(dma_channel->CCR, DMA_CCR_TCIE);
    //Enable Transfer error interrupt 
    //SET_BIT(dma_channel->CCR, DMA_CCR_TEIE);
    
    /*! DMA channel number of data to transfer register
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
    if(usart_inter == USART1_BASE)
    {
        SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
        dma_channel_tx = DMA1_Channel4_BASE;   //TX
        dma_channel_rx = DMA1_Channel5_BASE;   //RX
    }
    else if(usart_inter == USART2_BASE)
    {
        SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
        dma_channel_tx = DMA1_Channel7_BASE;   //TX
        dma_channel_rx = DMA1_Channel6_BASE;   //RX
    }
    else if(usart_inter == USART3_BASE)
    {   
        SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
        dma_channel_tx = DMA1_Channel2_BASE;   //TX
        dma_channel_rx = DMA1_Channel3_BASE;   //RX
    }  
    else if(usart_inter == UART4_BASE)
    { 
        SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN);
        dma_channel_tx = DMA2_Channel5_BASE;   //TX
        dma_channel_rx = DMA2_Channel3_BASE;   //RX
    }
    else if(usart_inter == UART5_BASE)
    {    
        //error not dma
    }
    else
    {
        //error not dma
    }

    dma_channel_tx->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
    dma_channel_rx->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_MINC | DMA_CCR_TCIE;

}

void usart_gpio_enable(USART_TypeDef* addr, int port_num)
{
    /*
USART pinout
        Configuration                        GPIO configuration
USARTx_RX(1) (UART)    
    Full duplex                         Alternate function push-pull
    Half duplex synchronous mode        Alternate function push-pull

USARTx_TX(1) (UART)
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
            TX - PA9 - USART1_TX  (AF7)
            RX - PA10 - USART1_RX  (AF7)

            TX - PB6 - USART1_TX  (AF7)
            RX - PB7 - USART1_RX  (AF7)

            TX - PC4 - USART1_TX  (AF7)
            RX - PC5 - USART1_RX  (AF7)

            TX - PE0 - USART1_TX  (AF7)
            RX - PE1 - USART1_RX  (AF7)
        */
        if(port_num == 1)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

            //PA9 Alternate function push-pull
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER9_0);
            SET_BIT(GPIOA->MODER, GPIO_MODER_MODER9_1);

            CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_9);

            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9_1);
            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9_0);

            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR9_0);
            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR9_1);

            //PA10 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER10_0);
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER10_1);

            CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_10);

            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_1);
            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_0);

            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR10_0);
            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR10_1);

            GPIOA->AFR[1] |= (0x7 << GPIO_AFRH_AFRH1_Pos) | (0x7 << GPIO_AFRH_AFRH2_Pos);
        }
        else if(port_num == 2)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

            //PB6 Alternate function push-pull
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER6_0);
            SET_BIT(GPIOB->MODER, GPIO_MODER_MODER6_1);

            CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_6);

            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6_1);
            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6_0);

            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR6_0);
            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR6_1);

            //PB7 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER7_0);
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER7_1);

            CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_7);

            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR7_1);
            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR7_0);

            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR7_0);
            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR7_1);

            GPIOB->AFR[0] |= (0x7 << GPIO_AFRH_AFRH6_Pos) | (0x7 << GPIO_AFRH_AFRH7_Pos);
        }
        else if(port_num == 3)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

            //PC4 Alternate function push-pull
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER4_0);
            SET_BIT(GPIOC->MODER, GPIO_MODER_MODER4_1);

            CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_4);

            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR4_1);
            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR4_0);

            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR4_0);
            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR4_1);

            //PC5 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER5_0);
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER5_1);

            CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_5);

            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5_1);
            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5_0);

            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR5_0);
            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR5_1);

            GPIOC->AFR[0] |= (0x7 << GPIO_AFRH_AFRH4_Pos) | (0x7 << GPIO_AFRH_AFRH5_Pos);
        }
        else if(port_num == 4)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOEEN);

            //PE0 Alternate function push-pull
            CLEAR_BIT(GPIOE->MODER, GPIO_MODER_MODER0_0);
            SET_BIT(GPIOE->MODER, GPIO_MODER_MODER0_1);

            CLEAR_BIT(GPIOE->OTYPER, GPIO_OTYPER_OT_0);

            SET_BIT(GPIOE->OSPEEDR, GPIO_OSPEEDER_OSPEEDR0_1);
            SET_BIT(GPIOE->OSPEEDR, GPIO_OSPEEDER_OSPEEDR0_0);

            CLEAR_BIT(GPIOE->PUPDR, GPIO_PUPDR_PUPDR0_0);
            CLEAR_BIT(GPIOE->PUPDR, GPIO_PUPDR_PUPDR0_1);

            //PE1 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOE->MODER, GPIO_MODER_MODER1_0);
            CLEAR_BIT(GPIOE->MODER, GPIO_MODER_MODER1_1);

            CLEAR_BIT(GPIOE->OTYPER, GPIO_OTYPER_OT_1);

            SET_BIT(GPIOE->OSPEEDR, GPIO_OSPEEDER_OSPEEDR1_1);
            SET_BIT(GPIOE->OSPEEDR, GPIO_OSPEEDER_OSPEEDR1_0);

            CLEAR_BIT(GPIOE->PUPDR, GPIO_PUPDR_PUPDR1_0);
            CLEAR_BIT(GPIOE->PUPDR, GPIO_PUPDR_PUPDR1_1);

            GPIOE->AFR[0] |= (0x7 << GPIO_AFRH_AFRH0_Pos) | (0x7 << GPIO_AFRH_AFRH1_Pos);
        }
        else
        {

        }
    }
    if(addr == USART2_BASE)
    {
        /*USART2 pin
            TX - PA2 - USART2_TX  (AF7)
            RX - PA3 - USART2_RX  (AF7)

            //not use jtag pin
            TX - PA14 - USART2_TX  (AF7)
            RX - PA15 - USART2_RX  (AF7)

            //not use jtag pin
            TX - PB3 - USART2_TX  (AF7)
            RX - PB4 - USART2_RX  (AF7)

            TX - PD5 - USART2_TX  (AF7)
            RX - PD6 - USART2_RX  (AF7)
        */
        
        if(port_num == 1)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

            //PA2 Alternate function push-pull
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER2_0);
            SET_BIT(GPIOA->MODER, GPIO_MODER_MODER2_1);

            CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_2);

            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR2_1);
            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR2_0);

            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR2_0);
            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR2_1);

            //PA3 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER3_0);
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER3_1);

            CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_3);

            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR3_1);
            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR3_0);

            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR3_0);
            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR3_1);

            GPIOA->AFR[0] |= (0x7 << GPIO_AFRH_AFRH2_Pos) | (0x7 << GPIO_AFRH_AFRH3_Pos);
        }

        /*else if(port_num == 2)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

            //PA14 Alternate function push-pull
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER14_0);
            SET_BIT(GPIOA->MODER, GPIO_MODER_MODER14_1);

            CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_14);

            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR14_1);
            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR14_0);

            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR14_0);
            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR14_1);

            //PA15 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER15_0);
            CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER15_1);

            CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_15);

            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR15_1);
            SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR15_0);

            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR15_0);
            CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR15_1);

            GPIOA->AFR[1] |= (0x7 << GPIO_AFRH_AFRH6_Pos) | (0x7 << GPIO_AFRH_AFRH7_Pos);
        }*/
        /*else if(port_num == 3)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

            //PB3 Alternate function push-pull
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER3_0);
            SET_BIT(GPIOB->MODER, GPIO_MODER_MODER3_1);

            CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_3);

            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR3_1);
            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR3_0);

            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR3_0);
            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR3_1);

            //PB4 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER4_0);
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER4_1);

            CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_4);

            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR4_1);
            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR4_0);

            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR4_0);
            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR4_1);

            GPIOB->AFR[0] |= (0x7 << GPIO_AFRH_AFRH3_Pos) | (0x7 << GPIO_AFRH_AFRH4_Pos);
        }*/
        else if(port_num == 4)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN);

            //PD5 Alternate function push-pull
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER5_0);
            SET_BIT(GPIOD->MODER, GPIO_MODER_MODER5_1);

            CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT_5);

            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5_1);
            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5_0);

            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR5_0);
            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR5_1);

            //PD6 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER6_0);
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER6_1);

            CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT_6);

            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6_1);
            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6_0);

            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR6_0);
            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR6_1);

            GPIOD->AFR[0] |= (0x7 << GPIO_AFRH_AFRH5_Pos) | (0x7 << GPIO_AFRH_AFRH6_Pos);
        }
        else
        {

        }
    }
    if(addr == USART3_BASE)
    {
        /*USART3 pin
            TX - PB10 - USART3_TX  (AF7)
            RX - PB11 - USART3_RX  (AF7)

            TX - PC10 - USART3_TX  (AF7)
            RX - PC11 - USART3_RX  (AF7)

            TX - PD8 - USART3_TX  (AF7)
            RX - PD9 - USART3_RX  (AF7)
        */
        
        if(port_num == 1)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

            //PB10 Alternate function push-pull
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER10_0);
            SET_BIT(GPIOB->MODER, GPIO_MODER_MODER10_1);

            CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_10);

            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_1);
            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_0);

            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR10_0);
            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR10_1);

            //PB11 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER11_0);
            CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER11_1);

            CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_11);

            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR11_1);
            SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR11_0);

            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR11_0);
            CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR11_1);

            GPIOB->AFR[1] |= (0x7 << GPIO_AFRH_AFRH2_Pos) | (0x7 << GPIO_AFRH_AFRH3_Pos);
        }
        else if(port_num == 2)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

            //PC10 Alternate function push-pull
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER10_0);
            SET_BIT(GPIOC->MODER, GPIO_MODER_MODER10_1);

            CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_10);

            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_1);
            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_0);

            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR10_0);
            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR10_1);

            //PC11 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER11_0);
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER11_1);

            CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_11);

            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR11_1);
            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR11_0);

            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR11_0);
            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR11_1);

            GPIOC->AFR[1] |= (0x7 << GPIO_AFRH_AFRH2_Pos) | (0x7 << GPIO_AFRH_AFRH3_Pos);
        }
        else if(port_num == 3)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN);

            //PD8 Alternate function push-pull
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER8_0);
            SET_BIT(GPIOD->MODER, GPIO_MODER_MODER8_1);

            CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT_8);

            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR8_1);
            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR8_0);

            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR8_0);
            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR8_1);

            //PD9 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER9_0);
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER9_1);

            CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT_9);

            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9_1);
            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9_0);

            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR9_0);
            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR9_1);

            GPIOD->AFR[1] |= (0x7 << GPIO_AFRH_AFRH0_Pos) | (0x7 << GPIO_AFRH_AFRH1_Pos);
        }
        else
        {

        }
    }
    if(addr == UART4_BASE)
    {
        /*UART4 pin
            TX - PC10 - UART4_TX  (AF5)
            RX - PC11 - UART4_RX  (AF5)
        */
        
        if(port_num == 1)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

            //PC10 Alternate function push-pull
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER10_0);
            SET_BIT(GPIOC->MODER, GPIO_MODER_MODER10_1);

            CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_10);

            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_1);
            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_0);

            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR10_0);
            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR10_1);

            //PC11 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER11_0);
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER11_1);

            CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_11);

            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR11_1);
            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR11_0);

            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR11_0);
            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR11_1);

            GPIOC->AFR[1] |= (0x5 << GPIO_AFRH_AFRH2_Pos) | (0x5 << GPIO_AFRH_AFRH3_Pos);
        }
        else
        {
            
        }
    }
    if(addr == UART5_BASE)
    {
        /*UART5 pin
            TX - PC12 - UART5_TX  (AF5)
            RX - PD2 - UART5_RX  (AF5)
        */
 
        if(port_num == 1)
        {
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);
            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN);

            //PA9 Alternate function push-pull
            CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER12_0);
            SET_BIT(GPIOC->MODER, GPIO_MODER_MODER12_1);

            CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_12);

            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9_1);
            SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9_0);

            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR9_0);
            CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR9_1);

            //PD2 Input floating / Input pull-up
            
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER10_0);
            CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODER10_1);

            CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT_10);

            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_1);
            SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_0);

            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR10_0);
            CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR10_1);

            GPIOC->AFR[1] |= (0x5 << GPIO_AFRH_AFRH4_Pos)
            GPIOD->AFR[0] |= (0x5 << GPIO_AFRH_AFRH2_Pos);
        }
        else
        {

        }
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

//uart4 interrupt function
void UART4_IRQHandler()
{

}

//uart5 interrupt function
void UART5_IRQHandler()
{

}

void DMA1_Channel2_IRQn()
{
    //USART3_TX
    if(DMA1->ISR & DMA_ISR_TCIF2 == DMA_ISR_TCIF2)
    {

        DMA1->IFCR |= DMA_IFCR_CTCIF2;
    }
}

void DMA1_Channel3_IRQn()
{
    //USART3_RX
    if(DMA1->ISR & DMA_ISR_TCIF3 == DMA_ISR_TCIF3)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF3;
    }
}

void DMA1_Channel4_IRQn()
{
    //USART1_TX
    if(DMA1->ISR & DMA_ISR_TCIF4 == DMA_ISR_TCIF4)
    {

        DMA1->IFCR |= DMA_IFCR_CTCIF4;
    }
}

void DMA1_Channel5_IRQn()
{
    //USART1_RX
    if(DMA1->ISR & DMA_ISR_TCIF5 == DMA_ISR_TCIF5)
    {

        DMA1->IFCR |= DMA_IFCR_CTCIF5;
    }
}

void DMA1_Channel6_IRQn()
{
    //USART2_RX
    if(DMA1->ISR & DMA_ISR_TCIF6 == DMA_ISR_TCIF6)
    {

        DMA1->IFCR |= DMA_IFCR_CTCIF6;
    }
}
void DMA1_Channel7_IRQn()
{
    //USART2_TX
    if(DMA1->ISR & DMA_ISR_TCIF7 == DMA_ISR_TCIF7)
    {

        DMA1->IFCR |= DMA_IFCR_CTCIF7;
    }
}

void DMA2_Channel3_IRQn()
{
    //UART4_RX
    if(DMA2->ISR & DMA_ISR_TCIF3 == DMA_ISR_TCIF3)
    {

        DMA2->IFCR |= DMA_IFCR_CTCIF3;
    }
}

void DMA2_Channel5_IRQn()
{
    //UART4_TX
    if(DMA2->ISR & DMA_ISR_TCIF5 == DMA_ISR_TCIF5)
    {

        DMA2->IFCR |= DMA_IFCR_CTCIF5;
    }
}