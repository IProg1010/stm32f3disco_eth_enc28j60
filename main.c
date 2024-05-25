// ---------------
#define STM32F103xB
#include "stm32f1xx.h"

//---------------
#include "usart.h"
#include "spi.h"
#include "i2c.h"
#include "dma.h"
#include "timer.h"
//--------include ethernet chip control header------
#include "enc28j60.h"
//---------------------------
//--------include lwip headers------
//#include "lwiplwip/opt.h"
//#include "lwip/debug.h"
//#include "lwip/stats.h"
//#include "lwip/tcp.h"
#include "lwip_port/tcp_server_raw/tcp_server.h"
#include "lwip_interface/lwip.h"
#include "APP/lwip_user.h"
//--------include my protocol for peripherial control------
//#include "proto_handle.h"

//---------------------------


void delay();
void enable_irq();
void disable_irq();
void init_GPIO();
void init_uart_service();
void led_on();
void led_off();
void debug_print(int data);

void SystemInit(void)
{
    // Because the debugger switches PLL on, we may
    // need to switch back to the HSI oscillator without PLL

    // Switch to HSI oscillator
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    // Wait until the switch is done
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
    {
    }

    // Disable the PLL, then we can configure it
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

    // Flash latency 2 wait states
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_1);

    // Enable HSE oscillator
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    // Wait until HSE oscillator is ready
    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY))
    {
    }

    // 72 MHz using the 8 MHz HSE oscillator with 9x PLL, lowspeed I/O runs at 36 MHz
    WRITE_REG(RCC->CFGR, RCC_CFGR_PLLSRC + RCC_CFGR_PLLMULL9 + RCC_CFGR_PPRE1_DIV2);

    // Enable PLL
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    // Wait until PLL is ready
    while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY))
    {
    }

    // Select PLL as clock source
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    // Update variable
   // SystemCoreClock = 72000000;
}

//spi_config spi_1;	
usart_config uart_param;
timer_config time_param;	
static struct tcp_pcb *server_pcb;
uint32_t lwip_localtime;

uint8_t macaddr[6];

int main()
{
	/*
    The JTAG pins are in input PU/PD after reset:
        PA15: JTDI in PU
        PA14: JTCK in PD
        PA13: JTMS in PU
        PB4: NJTRST in PU
        */
    //mem_init();
	//memp_init();
	init_GPIO();
    timer_init(TIM2_BASE, &time_param);	
	//init_uart_service();

    macaddr[0] = 0x01;
    macaddr[1] = 0x00;
    macaddr[2] = 0x12;
    macaddr[3] = 0x20;
    macaddr[4] = 0xEF;
    macaddr[5] = 0xFF;


    //uint8_t temp = lwip_config_init();
    #if LWIP_DHCP   //使用DHCP
        while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//等待DHCP获取成功/超时溢出
        {
            lwip_periodic_handle();	//LWIP内核需要定时处理的函数
        }
    #endif

    //lwip_init();
    lwip_init();
    //ENC28_SetPhyInterface(&spi_1);
    //ENC28J60_Init(macaddr);
    LwIP_Init();
    //server_init();
    //server_accept();

    TIM_EnableIT_UPDATE(TIM2);
    TIM_EnableCounter(TIM2);
	enable_irq();
#ifdef __DEBUG_MODE__
	
#endif
	
#ifdef __DEBUG_MODE__
	//printf("\r");
#endif
	
	while(1)
	{
        //lwip_periodic_handle();
        LwIP_Periodic_Handle(lwip_localtime);
        //LwIP_Pkt_Handle();
        	//led_on();
		//delay();
        sys_check_timeouts();
        //server_poll();
		//led_off();
		//delay();
        //sys_check_timeouts();
	}
}

void delay(){
	volatile int i = 0;
	for(i = 0; i <= 3000000; i++){
	}
}

void enable_irq()
{
    return 0;

}

void disable_irq()
{ 
    return 0;

}

void init_GPIO() 
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIOC->CRH |= GPIO_CRH_MODE13_0;

	GPIOC->CRH &= ~GPIO_CRH_CNF13;


    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE13_0);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE13_1);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF13_0); 
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF13_1);
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS13); 
}

void debug_print(int data)
{
    usart_write(USART2_BASE, &data, 1);
}

void init_uart_service()
{
    usart_init(USART2_BASE, &uart_param);
    debug_print(4);
    return 0;
}

void led_on() 
{
	//GPIOC->ODR |= 0x00002000;
    SET_BIT(GPIOC->BRR, GPIO_BRR_BR13);
    return 0;
	//GPIOC->BSRR = GPIO_BSRR_BS13;
}

void led_off()
{
	//GPIOC->ODR &= ~0x00002000;
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
    return 0;
	//GPIOC->BSRR = GPIO_BSRR_BS13;
}
// *********************************** E N D **********************************

uint32_t sys_now()
{
    return lwip_localtime;
}