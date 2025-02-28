#include "timer.h"
#include "stm32f303xc.h"
#include "F3_MACROS.h"

extern uint32_t lwip_localtime;

//uart init
void timer_init(TIM_TypeDef* timer_inter, timer_config* config)
{
    /*! Enable Timer peripherial clock enable*/
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    //SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    //SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    //SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
    NVIC_EnableIRQ(TIM2_IRQn);
    //SET_BIT(RCC->APB1ENR, RCC_AHB1ENR_TIM5EN);
    //SET_BIT(RCC->APB1ENR, RCC_APB2ENR_TIM6EN);
    //SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);
    
    /*! Timer Status register*/
    //timer_inter->SR = 1;         
    
    /*! Timer Control register1
        --[15:10]   reserved
        --[9:8]     CKD - Clock division (  00: tDTS = tCK_INT, 01: tDTS = 2 × tCK_INT
                                            10: tDTS = 4 × tCK_INT, 11: Reserved)
        --[7]   ARPE -Auto-reload preload enable    0: TIMx_ARR register is not buffered 
                                                        1: TIMx_ARR register is buffered
        --[6:5] CMS - Center-aligned mode selection
                            00: Edge-aligned mode. The counter counts up or down depending on the direction bit
                            (DIR).
                            01: Center-aligned mode 1. The counter counts up and down alternatively. Output compare
                            interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                            only when the counter is counting down.
                            10: Center-aligned mode 2. The counter counts up and down alternatively. Output compare
                            interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                            only when the counter is counting up.
                            11: Center-aligned mode 3. The counter counts up and down alternatively. Output compare
                            interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                            both when the counter is counting up or down.
        --[4]   DIR - Direction 0: Counter used as upcounter
                                    1: Counter used as downcounter
        --[3]   OPM - One-pulse mode    0: Counter is not stopped at update event
                                        1: Counter stops counting at the next update event (clearing the bit CEN)
        --[2]   URS - Update request source
                                        This bit is set and cleared by software to select the UEV event sources.
                                        0: Any of the following events generate an update interrupt or DMA request if enabled.
                                        These events can be:
                                        –
                                        Counter overflow/underflow
                                        –
                                        Setting the UG bit
                                        –
                                        Update generation through the slave mode controller
                                        1: Only cou
        --[1]   UDIS - Update disable
                                        This bit is set and cleared by software to enable/disable UEV event generation.
                                        0: UEV enabled. The Update (UEV) event is generated by one of the following events:
                                        – Counter overflow/underflow
                                        – Setting the UG bit
                                        – Update generation through the slave mode controller
                                            Buffered registers are then loaded with their preload values.
                                        1: UEV disabled. The Update event is not generated, shadow registers keep their value
                                        (ARR, PSC, CCRx). However the counter and the prescaler are reinitialized if the UG bit is
                                        set or if a hardware reset is received from the slave mode controller.
        --[0]   CEN - Counter enable    0: Counter disabled
                                        1: Counter enabled
        */
    //timer_inter->CR1 |= 1 << USART_CR1_UE;        
   
    /*! Timer Control register2
        --[15:8]  LINEN - LIN mode enable
        --[7]  TI1S: TI1 selection
                                    0: The TIMx_CH1 pin is connected to TI1 input
                                    1: The TIMx_CH1, CH2 and CH3 pins are connected to the TI1 input (XOR combination)
        --[6:4]   MMS - Master mode selection
        --[3]  CCDS - Capture/compare DMA selection
                                    0: CCx DMA request sent when CCx event occurs
                                    1: CCx DMA requests sent when update event occurs
        --[2:0] Reserved - Reserved,
    */   
    //timer_inter->CR2 = 1;

    /*! Timer slave mode control register
        --[15]  ETP - External trigger polarity
        --[14]  ECE - External clock enable
        --[13:12]  ETPS - External trigger prescaler
                                    00: Prescaler OFF
                                    01: ETRP frequency divided by 2
                                    10: ETRP frequency divided by 4
                                    11: ETRP frequency divided by 8
        --[11:8]  ETF - External trigger filter
                                    0000: No filter, sampling is done at fDTS
                                    0001: fSAMPLING=fCK_INT, N=2
                                    0010: fSAMPLING=fCK_INT, N=4
                                    0011: fSAMPLING=fCK_INT, N=8²
                                    0100: fSAMPLING=fDTS/2, N=6
                                    0101: fSAMPLING=fDTS/2, N=8
                                    0110: fSAMPLING=fDTS/4, N=6²
                                    0111: fSAMPLING=fDTS/4, N=8
                                    1000: fSAMPLING=fDTS/8, N=6
                                    1001: fSAMPLING=fDTS/8, N=8
                                    1010: fSAMPLING=fDTS/16, N=5
                                    1011: fSAMPLING=fDTS/16, N=6
                                    1100: fSAMPLING=fDTS/16, N=8
                                    1101: fSAMPLING=fDTS/32, N=5
                                    1110: fSAMPLING=fDTS/32, N=6
                                    1111: fSAMPLING=fDTS/32, N=8
        --[7]  MSM - Master/Slave mode
                                    0: No action
                                    1: The effect of an event on the trigger input (TRGI) is delayed to allow a perfect
                                    synchronization between the current timer and its slaves (through TRGO). It is useful if we
                                    want to synchronize several timers on a single external event.
        --[6:4]   TS - Trigger selection This bit-field selects the trigger input to be used to synchronize the counter.
                                    000: Internal Trigger 0 (ITR0).
                                    001: Internal Trigger 1 (ITR1).
                                    010: Internal Trigger 2 (ITR2).
                                    011: Internal Trigger 3 (ITR3).
                                    100: TI1 Edge Detector (TI1F_ED)
                                    101: Filtered Timer Input 1 (TI1FP1)
                                    110: Filtered Timer Input 2 (TI2FP2)
                                    111: External Trigger input (ETRF)
        --[3]   Reserved - Reserved,
        --[2:0]   SMS - Slave mode selection
    */   
    //timer_inter->SMCR = 1;        

    /*! Timer DMA/Interrupt enable register (TIMx_DIER)
        --[15]  Reserved - Reserved,
        --[14]  TDE - Trigger DMA request enable
        --[13]  Reserved - Reserved,
        --[12]  CC4DE - Capture/Compare 4 DMA request enable
        --[11]  CC3DE - Capture/Compare 3 DMA request enable
        --[10]  CC2DE - Capture/Compare 2 DMA request enable
        --[9]   CC1DE - Capture/Compare 1 DMA request enable
        --[8]   UDE - Update DMA request enable
        --[7]   Reserved - Reserved
        --[6]   TIE - Trigger interrupt enable
        --[5]   Reserved - Reserved
        --[4]   CC4IE - Capture/Compare 4 interrupt enable
        --[3]   CC3IE - Capture/Compare 3 interrupt enable
        --[2]   CC2IE - Capture/Compare 2 interrupt enable
        --[1]   CC1IE - Capture/Compare 1 interrupt enable
        --[0]   UIE - Update interrupt enable
    */   
    //timer_inter->DIER = 1;     
    
    /*! Timer event generation register (TIMx_EGR)
        --[15:7]  Reserved - Reserved,
        --[6]   TG - Trigger generation  This bit is set by software in order to generate an event, it is automatically cleared by hardware.
                                        0: No action
                                        1: The TIF flag is set in TIMx_SR register. Related interrupt or DMA transfer can occur if
                                        enabled.
        --[5]   Reserved - Reserved
        --[4]   CC4G - Capture/compare 4 generation
        --[3]   CC3G - Capture/compare 3 generation
        --[2]   CC2G - Capture/compare 2 generation
        --[1]   CC1G - Capture/compare 1 generation
        --[0]   UG - Update interrupt enable
    */   
    //timer_inter->EGR = 1;  

    /*! Timer capture/compare mode register 1 (TIMx_CCMR1)
        --[15]  OC2CE - Output compare 2 clear enable
        --[14:12]  OC2M - Output compare 2 mode
        --[11]  OC2PE - Output compare 2 preload enable
        --[10]  OC2FE - Output compare 2 fast enable
        --[9:8]   CC2S - Capture/Compare 2 selection This bit-field defines the direction of the channel (input/output) as well as the used input.
                                    00: CC2 channel is configured as output
                                    01: CC2 channel is configured as input, IC2 is mapped on TI2
                                    10: CC2 channel is configured as input, IC2 is mapped on TI1
                                    11: CC2 channel is configured as input, IC2 is mapped on TRC. This mode is working only if
                                    an internal trigger input is selected through the TS bit (TIMx_SMCR register)
        --[7]   OC1CE - Output compare 1 clear enable 
                                    0: OC1Ref is not affected by the ETRF input
                                    1: OC1Ref is cleared as soon as a High level is detected on ETRF input
        --[6:4]   OC1M - Output compare 1 mode
        --[3]   OC1PE - Output compare 1 preload enable
        --[2]   OC1FE - Output compare 1 fast enable
        --[1:0]   CC1S: Capture/Compare 1 selection This bit-field defines the direction of the channel (input/output) as well as the used input.
                                    00: CC1 channel is configured as output.
                                    01: CC1 channel is configured as input, IC1 is mapped on TI1.
                                    10: CC1 channel is configured as input, IC1 is mapped on TI2.
                                    11: CC1 channel is configured as input, IC1 is mapped on TRC. This mode is working only if
                                    an internal trig
    */   
    //timer_inter->CCMR1 = 1;  

    /*! Timer capture/compare mode register 2 (TIMx_CCMR2)
        --[15]  OC4CE - Output compare 4 clear enable
        --[14:12]  OC4M - Output compare 4 mode
        --[11]  OC4PE - Output compare 4 preload enable
        --[10]  OC4FE - Output compare 4 fast enable
        --[9:8]   CC4S - Capture/Compare 4 selection This bit-field defines the direction of the channel (input/output) as well as the used input.
                                    00: CC2 channel is configured as output
                                    01: CC2 channel is configured as input, IC2 is mapped on TI2
                                    10: CC2 channel is configured as input, IC2 is mapped on TI1
                                    11: CC2 channel is configured as input, IC2 is mapped on TRC. This mode is working only if
                                    an internal trigger input is selected through the TS bit (TIMx_SMCR register)
        --[7]   OC3CE - Output compare 3 clear enable 
                                    0: OC1Ref is not affected by the ETRF input
                                    1: OC1Ref is cleared as soon as a High level is detected on ETRF input
        --[6:4]   OC3M - Output compare 3 mode
        --[3]   OC3PE - Output compare 3 preload enable
        --[2]   OC3FE - Output compare 3 fast enable
        --[1:0]   CC3S: Capture/Compare 3 selection This bit-field defines the direction of the channel (input/output) as well as the used input.
                                    00: CC1 channel is configured as output.
                                    01: CC1 channel is configured as input, IC1 is mapped on TI1.
                                    10: CC1 channel is configured as input, IC1 is mapped on TI2.
                                    11: CC1 channel is configured as input, IC1 is mapped on TRC. This mode is working only if
                                    an internal trigger input is selected through TS bit (TIMx_SMCR register)
    */   
    //timer_inter->CCMR2 = 1;  

    /*! Timer capture/compare enable register (TIMx_CCER)
        --[15:14]  Reserved - Reserved,
        --[13]  CC4P: Capture/Compare 4 output polarity
        --[12]  CC4E: Capture/Compare 4 output enable
        --[11:10]  Reserved - Reserved,
        --[9]  CC3P: Capture/Compare 3 output polarity
        --[8]  CC3E: Capture/Compare 3 output enable
        --[7:6]   Reserved - Reserved
        --[5]  CC2P: gjckt Capture/Compare 2 output polarity
        --[4]  CC2E: Capture/Compare 2 output enable
        --[3:2]  Reserved - Reserved
        --[1]  CC1P: Capture/Compare 1 output polarity
        --[0]  CC1E: Capture/Compare 1 output enable
    */   
    //timer_inter->CCER = 1;  

    /*! Timer counter register
        --[15:0] CNT - Counter value
                
        timer_inter->CNT;
    */

    /*! Timer prescaler register
        --[15:0] PSC - Prescaler value
                The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
                PSC contains the value to be loaded in the active prescaler register at each update event
                (including when the counter is cleared through UG bit of TIMx_EGR register or through
                trigger controller when configured in “reset mode”).
        timer_inter->PSC;
    */
    WRITE_REG(TIM2->PSC, 5550);
    
    /*! Timer prescaler register
        --[15:0]  ARR - Prescaler value
                        ARR is the value to be loaded in the actual auto-reload register.
        timer_inter->ARR;
    */  

    WRITE_REG(TIM2->ARR, 1000);
    /*! Timer capture/compare register 1
        --[15:0]  CCR1 - Capture/Compare 1 value
                        If channel CC1 is configured as output:
                        CCR1 is the value to be loaded in the actual capture/compare 1 register (preload value).
                        It is loaded permanently if the preload feature is not selected in the TIMx_CCMR1 register
                        (bit OC1PE). Else the preload value is copied in the active capture/compare 1 register when
                        an update event occurs.
                        The active capture/compare register contains the value to be compared to the counter
                        TIMx_CNT and signaled on OC1 output.
                        If channel CC1is configured as input:
                        CCR1 is the counter value transferred by the last input capture 1 event (IC1). The
                        TIMx_CCR1 register is read-only and cannot be programmed.
        timer_inter->CCR1;
    */  

    /*! Timer capture/compare register 2
        --[15:0]  CCR2 - Capture/Compare 2 value
        timer_inter->CCR2;
    */  

    /*! Timer capture/compare register 3
        --[15:0]  CCR3 - Capture/Compare 3 value
        timer_inter->CCR3;
    */  

    /*! Timer capture/compare register 4
        --[15:0]  CCR4 - Capture/Compare 4 value
        timer_inter->CCR4;
    */  

    /*! Timer DMA control register
        --[15:13]  Reserved - Reserved.
        --[12:8]  DBL[4:0] - DMA burst length
        --[7:5]  Reserved - Reserved
        --[4:0]  DBA - DMA base address
    */   
    //timer_inter->DCR = 1;
    
    /*! Timer DMA address for full transfer register
        --[15:0]  DMAB - DMA register for burst accesses
    */   
    //timer_inter->DMAR = 1;        
    
      
}

//enable timer

void timer_enable(TIM_TypeDef* timer_inter, uint8_t enable)
{

}

void TIM2_IRQHandler(void)
{
    static int8_t i = 0;
    if(READ_BIT(TIM2->SR, TIM_SR_UIF))
    {
        CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
        lwip_localtime++;
        
        if(i == 0)
        {
            SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS_10);
            i = 1;
        }
        else
        {
            SET_BIT(GPIOE->BRR, GPIO_BRR_BR_10);
            i = 0;
        }
        //if(tim2_count>9) tim2_count=0;
    }
}