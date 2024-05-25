#include "i2c.h"

//spi init
void i2c_init(I2C_TypeDef* i2c_interf, i2c_config* config)
{
    /*! Init I2C GPIO*/
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    
    /*! Init I2C GPIO*/
    
    /*! Enable I2C peripherial clock*/
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
    /*! I2C Control register1
        --[15]  BIDIMODE - USART enable
        --[14]  BIDIOE - USART enable
        --[13]  CRCEN - USART enable
        --[12]  CRCNEXT - USART enable
        --[11]  DFF - Data frame format (0 - 8 bit, 1 - 16 bit)
        --[10]  RXONLY - Receive only (1 - full duplex, 0 - output disable)
        --[9]   SSM - Software slave management (0 - disable, 1- enable)
        --[8]   SSI - Internal slave select (effect only when the SSM is set)
        --[7]   LSBFIRST - Frame format (1 - MSB, 2 - LSB)
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
    i2c_interf->CR1 &= ~(1 << SPI_CR1_CRCEN || 0 << SPI_CR1_DFF || 0 << SPI_CR1_RXONLY
                        || 1 << SPI_CR1_DFF || 1 << SPI_CR1_DFF) ;

    /*! I2C Control register2
        --[7]   TXEIE - Tx buffer empty interrupt enable
        --[6]   RXNEIE - RX buffer not empty interrupt enable
        --[5]   ERRIE - Error interrupt enable
        --[2]   SSOE - SS output enable
        --[1]   TXDMAEN - Tx buffer DMA enable
        --[0]   RXDMAEN - Rx buffer DMA enable
    */
    i2c_interf->CR2 = 1;
    /*! I2C Status register
    */
    //i2c_interf->SR = 1;
    
    /*! I2C data read or write register
    */
    //i2c_interf->DR = 1;
    
    /*! I2C crc polynomial register
    */
    //i2c_interf->CRCPR = 1;
    
    /*! I2C crc RX register
    */
    //i2c_interf->RXCRCR = 1;

     /*! I2C crc TX register
    */
    //i2c_interf->TXCRCR = 1;
    
    //регистры для I2C
    //i2c_interf->I2SCFGR = 1;
    //i2c_interf->I2SPR = 1;
}

void i2c_write(I2C_TypeDef* spi_interf,  uint8_t* data, uint16_t size)
{

}

void i2c_read(I2C_TypeDef* spi_interf, uint8_t* data, uint16_t size)
{

}

void i2c_enable(I2C_TypeDef* spi_interf, uint8_t enable)
{

}

void i2c_gpio_enable(I2C_TypeDef* addr)
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

    if(addr == I2C1_BASE)
    {
        /*SPI1 pin
            SS - GPIOA4 (PA4) 
            SCK - GPIOA5 (PA5) 
            MISO - GPIOA6 (PA6)
            MOSI - GPIOA7 (PA7)
        */
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
        
        /*PA5 Output Alternate function push-pull*/
        WRITE_REG(GPIOA->CRL, GPIO_CRL_MODE5);
        WRITE_REG(GPIOA->CRL, GPIO_CRL_CNF5); 
        /*PA6 Output Alternate function push-pull*/
        WRITE_REG(GPIOA->CRL, GPIO_CRL_MODE6);
        WRITE_REG(GPIOA->CRL, GPIO_CRL_CNF6); 
        /*PA7 Output Alternate function push-pull*/
        WRITE_REG(GPIOA->CRL, GPIO_CRL_MODE7);
        WRITE_REG(GPIOA->CRL, GPIO_CRL_CNF7); 
    }
    if(addr == I2C2_BASE)
    {
        /*SPI2 pin
            SS - GPIOB12 (PB12)
            SCK - GPIOB13 (PB13) 
            MISO - GPIOB14 (PB14)
            MOSI - GPIOB15 (PB15)
        */
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);

        /*PB13 Output Alternate function push-pull*/
        WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE13);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF13); 
        /*PB14 Output Alternate function push-pull*/
        WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE14);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF14); 
        /*PB15 Output Alternate function push-pull*/
        WRITE_REG(GPIOB->CRH, GPIO_CRH_MODE15);
        WRITE_REG(GPIOB->CRH, GPIO_CRH_CNF15); 
    }
}   

//interrup function redefenition for i2c
void I2C1_EV_IRQHandler()
{

}

void I2C1_ER_IRQHandler()
{

}

void I2C2_EV_IRQHandler()
{

}

void I2C2_ER_IRQHandler()
{

}