#include "spi.h"
#include "stm32f303xc.h"
#include "F3_MACROS.h"

//spi init
void spi_init(SPI_TypeDef* spi_interf, spi_config* config)
{
    /*! Init SPI GPIO*/
    spi_gpio_enable(spi_interf);
    /*! Init SPI GPIO*/
    
    /*! Enable SPI peripherial clock*/
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
    //SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
    /*! SPI Control register1
        --[15]  BIDIMODE - Bidirectional data mode enable
        --[14]  BIDIOE - Output enable in bidirectional mode
        --[13]  CRCEN - Hardware CRC calculation enable
        --[12]  CRCNEXT - CRC transfer next
        --[11]  DFF - Data frame format (0 - 8 bit, 1 - 16 bit)
        --[10]  RXONLY - Receive only (1 - full duplex, 0 - output disable)
        --[9]   SSM - Software slave management (0 - disable, 1- enable)
        --[8]   SSI - Internal slave select (effect only when the SSM is set)
        --[7]   LSBFIRST - Frame format (1 - MSB, 0 - LSB)
        --[6]   SPE - SPI enable (1 - enabled, 0 - disabled)
        --[5:3] BR[2:0] - Baud rate control
                000: fPCLK/2
                001: fPCLK/4
                010: fPCLK/8
                011: fPCLK/16
                100: fPCLK/32
                101: fPCLK/64
                110: fPCLK/128
                111: fPCLK/256
        --[2]   MSTR - Master selection(0 - slave mode, 1 - master mode)
        --[1]   CPOL - Clock polarity
        --[0]   CPHA - Clock phase
    */
    
    //Bidirectional data mode enable
    SET_BIT(spi_interf->CR1, SPI_CR1_BIDIMODE);     
    //Output enable in bidirectional mode
    CLEAR_BIT(spi_interf->CR1, SPI_CR1_BIDIOE); 
    //Full duplex mode
    CLEAR_BIT(spi_interf->CR1, SPI_CR1_CRCEN);     
    //CRC transfer next
    CLEAR_BIT(spi_interf->CR1, SPI_CR1_CRCNEXT);         
    //8 bit data frame
    //CLEAR_BIT(spi_interf->CR1, SPI_CR1_DFF); 
    //Full duplex mode
    CLEAR_BIT(spi_interf->CR1, SPI_CR1_RXONLY);     
    //Software slave management
    SET_BIT(spi_interf->CR1, SPI_CR1_SSM);     
    //Internal slave select
    SET_BIT(spi_interf->CR1, SPI_CR1_SSI);
    //MSB Frame format
    SET_BIT(spi_interf->CR1, SPI_CR1_LSBFIRST);
    //Baud rate control
    SET_BIT(spi_interf->CR1, SPI_CR1_BR_0);
    CLEAR_BIT(spi_interf->CR1, SPI_CR1_BR_1);
    CLEAR_BIT(spi_interf->CR1, SPI_CR1_BR_2);
    
    //Master selection
    SET_BIT(spi_interf->CR1, SPI_CR1_MSTR);
    //SPI clock polarity
    CLEAR_BIT(spi_interf->CR1, SPI_CR1_CPOL);
    //SPI clock phase
    SET_BIT(spi_interf->CR1, SPI_CR1_CPHA);
    
    /*! SPI Control register2
        --[7]   TXEIE - Tx buffer empty interrupt enable
        --[6]   RXNEIE - RX buffer not empty interrupt enable
        --[5]   ERRIE - Error interrupt enable
        --[2]   SSOE - SS output enable
        --[1]   TXDMAEN - Tx buffer DMA enable
        --[0]   RXDMAEN - Rx buffer DMA enable
    */
 
    //Disable SPI TX buff empty interrupt
    CLEAR_BIT(spi_interf->CR2, SPI_CR2_TXEIE); 
    //Disable SPI RX buff not empty interrupt
    CLEAR_BIT(spi_interf->CR2, SPI_CR2_RXNEIE); 
    //Enable spi error interrupt
    CLEAR_BIT(spi_interf->CR2, SPI_CR2_ERRIE); 
    //SS output disable
    CLEAR_BIT(spi_interf->CR2, SPI_CR2_SSOE);
    //Enable DMA Tx SPI
    CLEAR_BIT(spi_interf->CR2, SPI_CR2_TXDMAEN);
    //Enable DMA Rx SPI
    CLEAR_BIT(spi_interf->CR2, SPI_CR2_RXDMAEN);

    /*! SPI Status register
    */
    //spi_interf->SR = 1;
    
    /*! SPI data read or write register
    */
    //spi_interf->DR = 1;
    
    /*! SPI crc polynomial register
    */
    //spi_interf->CRCPR = 1;
    
    /*! SPI crc RX register
    */
    //spi_interf->RXCRCR = 1;

     /*! SPI crc TX register
    */
    //spi_interf->TXCRCR = 1;
    
    //регистры для I2S
    //spi_interf->I2SCFGR = 1;
    //spi_interf->I2SPR = 1;
}

void spi_write(SPI_TypeDef* spi_interf,  uint8_t* data, uint16_t size)
{
    uint16_t i = 0;
    while(i < size)
    {
        while(!(READ_BIT(spi_interf->SR, SPI_SR_TXE) == (SPI_SR_TXE))) {}
        spi_interf->DR = data[i];
        i++;
    }
}

void spi_read(SPI_TypeDef* spi_interf, uint8_t* data, uint16_t size)
{
    uint16_t i = 0;
    while(i < size)
    {
        //spi_interf->DR = 0xFF;
        while(!(READ_BIT(spi_interf->SR, SPI_SR_RXNE) == (SPI_SR_RXNE))) {}
        data[i] = spi_interf->DR;
        i++;
    }
}

void spi_enable(SPI_TypeDef* spi_interf, uint8_t enable)
{
    if(enable == ON)
        //--[6]   SPE - SPI enable (1 - enabled, 0 - disabled)
        SET_BIT(spi_interf->CR1, SPI_CR1_SPE);
    else
        //--[6]   SPE - SPI enable (1 - enabled, 0 - disabled)
        CLEAR_BIT(spi_interf->CR1, SPI_CR1_SPE);
}

void spi_gpio_enable(SPI_TypeDef* addr)
{
    /*
SPI pinout
    
            Configuration                                GPIO configuration
SPIx_SCK
    Master                                          Alternate function push-pull
    Slave                                           Input floating

SPIx_MOSI
    Full duplex / master                            Alternate function push-pull
    Full duplex / slave                             Input floating / Input pull-up
    Simplex bidirectional data wire / master        Alternate function push-pull
    Simplex bidirectional data wire/ slave          Not used. Can be used as a GPIO
    
SPIx_MISO 
    Full duplex / master                            Input floating / Input pull-up
    Full duplex / slave (point to point)            Alternate function push-pull
    Full duplex / slave (multi-slave)               Alternate function open drain
    Simplex bidirectional data wire / master        Not used. Can be used as a GPIO
    Simplex bidirectional data wire/ slave
    (point to point)                                Alternate function push-pull
    Simplex bidirectional data wire/ slave
    (multi-slave)                                   Alternate function open drain
    
SPIx_NSS 
    Hardware master /slave                          Input floating/ Input pull-up / Input pull-down
    Hardware master/ NSS output enabled             Alternate function push-pull
    Software Not used.                              Can be used as a GPIO
    */

    if(addr == SPI1_BASE)
    {
        /*SPI1 pin
            SS - GPIOA4 (PA4) 
            SCK - GPIOA5 (PA5) 
            MISO - GPIOA6 (PA6)
            MOSI - GPIOA7 (PA7)
        */
        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

        //PA5 Alternate function push-pull
        CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5_0);
        SET_BIT(GPIOA->MODER, GPIO_MODER_MODER5_1);

        CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_5);

        SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5_1);
        SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5_0);

        CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR5_0);
        CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR5_1);
        
        GPIOA->AFR[0] |= (0x5 << GPIO_AFRH_AFRH5_Pos) | (0x5 << GPIO_AFRH_AFRH6_Pos) | (0x5 << GPIO_AFRH_AFRH7_Pos);

        //SET_BIT(GPIOA->AFR[0], GPIO_AFRH_AFRH5);
        //SET_BIT(GPIOA->AFR[0], GPIO_AFRH_AFRH5);
        
        //PA6 Input floating / Input pull-up
        
        CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER6_0);
        CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER6_1);

        CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_6);

        SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6_1);
        SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6_0);

        CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR6_0);
        CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR6_1);

        //PA7 Alternate function push-pull
        CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER7_0);
        SET_BIT(GPIOA->MODER, GPIO_MODER_MODER7_1);

        CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_7);

        SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR7_1);
        SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR7_0);

        CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR7_0);
        CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR7_1);
    }
    else if(addr == SPI2_BASE)
    {
        /*SPI2 pin
            SS - GPIOB12 (PB12)
            SCK - GPIOB13 (PB13) 
            MISO - GPIOB14 (PB14)
            MOSI - GPIOB15 (PB15)
        */
        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

        /*PB13 Output Alternate function push-pull*/
        /*WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE13);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF13); 
        /*PB14 Output Alternate function push-pull*/
        /*WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE14);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF14); 
        /*PB15 Output Alternate function push-pull*/
        /*WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE15);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF15); */
    }
    else if(addr == SPI3_BASE)
    {
        /*SPI3 pin
            SS - GPIOB12 (PA15)
            SCK - GPIOB13 (PC10) 
            MISO - GPIOB14 (PC11)
            MOSI - GPIOB15 (PC12)
        */
        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

        /*PB13 Output Alternate function push-pull*/
        /*WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE13);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF13); 
        /*PB14 Output Alternate function push-pull*/
        /*WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE14);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF14); 
        /*PB15 Output Alternate function push-pull*/
        /*WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE15);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF15); */
    }
}   

//function for spi1 interrupt
void SPI1_IRQHandler()
{
    if(SPI1->SR && (1 << SPI_SR_UDR))
    {

    }
}   

//function for spi2 interrupt
void SPI2_IRQHandler()
{
    if(SPI2->SR && (1 << SPI_SR_UDR))
    {

    }
}