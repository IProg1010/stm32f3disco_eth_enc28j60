#include "gpio.h"
#include "stm32f303xc.h"
#include "F3_MACROS.h"

//spi init
void gpio_init()
{
    /*! Init GPIO*/
    
    /*! Enable GPIO clock*/
    //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
    //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);
    //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN);
    //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOEEN);
    //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOFEN);

    /*! GPIO port mode register (GPIOx_MODER)
        --[31:0]  MODER[15:0][1:0]: Port x configuration I/O pin y (y = 15 to 0)
                                    These bits are written by software to configure the I/O mode.
                                    00: Input mode (reset state)
                                    01: General purpose output mode
                                    10: Alternate function mode
                                    11: Analog mode
    */
    GPIOA->MODER, GPIO_MODER_MODER11;

    CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER0_0);
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER0_1);

    /*! GPIO port output type register (GPIOx_OTYPER)
        --[31:16]  Reserved - Reserved 
        --[15:0]   OTYPER - Port x configuration I/O pin y (y = 15 to 0)
                                These bits are written by software to configure the I/O output type.
                                0: Output push-pull (reset state)
                                1: Output open-drain
    */
    GPIOA->OTYPER, GPIO_OTYPER_OT_1;


    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_0);
    SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_0);
    
    /*! GPIO port output speed register (GPIOx_OSPEEDR)
        --[31:0]  OSPEEDR[15:0][1:0]: Port x configuration I/O pin y (y = 15 to 0)
                                These bits are written by software to configure the I/O output speed.
                                x0: Low speed
                                01: Medium speed
                                11: High speed
    */
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR0_1);
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR0_0);

    /*! GPIO port pull-up/pull-down register (GPIOx_PUPDR)
        --[31:0]  PUPDR[15:0][1:0]: Port x configuration I/O pin y (y = 15 to 0)
                                These bits are written by software to configure the I/O pull-up or pull-down
                                00: No pull-up, pull-down
                                01: Pull-up
                                10: Pull-down
                                11: Reserved
    */
    SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR0_0);
    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR0_1);


    /*! GPIO port input data register (GPIOx_IDR)
        --[31:16]  Reserved - Reserved 
        --[15:0]   IDR[15:0]: Port x input data I/O pin y (y = 15 to 0)
                                    These bits are read-only. They contain the input value of the corresponding I/O port.
    */
    //GPIOA->IDR, GPIO_MODER_MODER11;

    /*! GPIO port output data register (GPIOx_ODR)
        --[31:16]  Reserved - Reserved 
        --[15:0]   ODR[15:0]: Port output data I/O pin y (y = 15 to 0)
                                    These bits can be read and written by software.
                                    Note: For atomic bit set/reset, the ODR bits can be individually set and/or reset by writing to
                                    the GPIOx_BSRR register (x = A..FA to H).
    */
    //GPIOA->ODR, GPIO_MODER_MODER11;

    /*! GPIO port bit set/reset register (GPIOx_BSRR)
        --[31:16]  BR - Port x reset I/O pin y (y = 15 to 0)
                                    These bits are write-only. A read to these bits returns the value 0x0000.
                                    0: No action on the corresponding ODRx bit
                                    1: Resets the corresponding ODRx bit 
        --[15:0]   BS - Port x set I/O pin y (y = 15 to 0)
                                    These bits are write-only. A read to these bits returns the value 0x0000.
                                    0: No action on the corresponding ODRx bit
                                    1: Sets the corresponding ODRx bit
    */
    GPIOA->BSRR, GPIO_MODER_MODER11;

    /*! GPIO port configuration lock register (GPIOx_LCKR)
        --[31:17]  Reserved - Reserved
        --[16]    LCKK - Lock key
                                    This bit can be read any time. It can only be modified using the lock key write sequence.
                                    0: Port configuration lock key not active
                                    1: Port configuration lock key active. The GPIOx_LCKR register is locked until the next MCU
                                    reset or peripheral reset.
                                    LOCK key write sequence:
                                    WR LCKR[16] = 1 + LCKR[15:0]
                                    WR LCKR[16] = 0 + LCKR[15:0]
                                    WR LCKR[16] = 1 + LCKR[15:0]
                                    RD LCKR
                                    RD LCKR[16] = 1 (this read operation is optional but it confirms that the lock is active)
        --[15:0]   LCK - Port x lock I/O pin y (y = 15 to 0)
                                    These bits are read/write but can only be written when the LCKK bit is 0.
                                    0: Port configuration not locked
                                    1: Port configuration locked
    */
    GPIOA->LCKR, GPIO_MODER_MODER11;


    /*! GPIO alternate function low register (GPIOx_AFRL)
        --[31:0]  AFRy[3:0]: Alternate function selection for port x pin y (y = 0..7)
                                    These bits are written by software to configure alternate function I/Os
                                    AFRy selection:
                                    0000: AF0
                                    0001: AF1
                                    0010: AF2
                                    0011: AF3
                                    0100: AF4
                                    0101: AF5
                                    0110: AF6
                                    0111: AF7
                                    1000: AF8
                                    1001: AF9
                                    1010: AF10
                                    1011: AF11
                                    1100: AF12
                                    1101: AF13
                                    1110: AF14
                                    1111: AF15
    */
    GPIOA->AFR[0], GPIO_MODER_MODER11;

    /*! GPIO alternate function low register (GPIOx_AFRH)
        --[31:0]  AFRy[3:0]: Alternate function selection for port x pin y (y = 8..15)
                                    These bits are written by software to configure alternate function I/Os
                                    AFRy selection:
                                    0000: AF0
                                    0001: AF1
                                    0010: AF2
                                    0011: AF3
                                    0100: AF4
                                    0101: AF5
                                    0110: AF6
                                    0111: AF7
                                    1000: AF8
                                    1001: AF9
                                    1010: AF10
                                    1011: AF11
                                    1100: AF12
                                    1101: AF13
                                    1110: AF14
                                    1111: AF15
    */
    GPIOA->AFR[1], GPIO_MODER_MODER11;
    

    /*! GPIO port bit reset register (GPIOx_BRR)
        --[31:16]  Reserved - Reserved
        --[15:0]   BR[15:0]: Port x reset IO pin y (y = 15 to 0)
                                    These bits are write-only. A read to these bits returns the value 0x0000.
                                    0: No action on the corresponding ODx bit
                                    1: Reset the corresponding ODx bit
    */
    GPIOA->BRR, GPIO_MODER_MODER11;
}
