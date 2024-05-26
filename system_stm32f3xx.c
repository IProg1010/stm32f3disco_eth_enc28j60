/**
  ******************************************************************************
  * @file    system_stm32f3xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  * 1. This file provides two functions and one global variable to be called from
  *    user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f3xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * 2. After each device reset the HSI (8 MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_stm32f3xx.s" file, to
  *    configure the system clock before to branch to main program.
  *
  * 3. This file configures the system clock as follows:
  *=============================================================================
  *                         Supported STM32F3xx device
  *-----------------------------------------------------------------------------
  *        System Clock source                    | HSI
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 8000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 8000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        USB Clock                              | DISABLE
  *-----------------------------------------------------------------------------
  *=============================================================================
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f3xx_system
  * @{
  */

/** @addtogroup STM32F3xx_System_Private_Includes
  * @{
  */

#include "stm32f3xx.h"

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Defines
  * @{
  */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)8000000) /*!< Default value of the Internal oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSI_VALUE */

/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */

/******************************************************************************/
/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock there is no need to
               call the 2 first functions listed above, since SystemCoreClock variable is 
               updated automatically.
  */
uint32_t SystemCoreClock = 8000000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Functions
  * @{
  */

static void SetSysClock(void);
/**
  * @brief  Setup the microcontroller system
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
/* FPU settings --------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif
/* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR &= 0xF87FC00C;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;

  /* Reset PREDIV1[3:0] bits */
  RCC->CFGR2 &= (uint32_t)0xFFFFFFF0;

  /* Reset USARTSW[1:0], I2CSW and TIMs bits */
  RCC->CFGR3 &= (uint32_t)0xFF00FCCC;
  
  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

  /* Configure the System clock source, PLL Multiplier and Divider factors, 
     AHB/APBx prescalers and Flash settings ----------------------------------*/
  SetSysClock();

  /* Configure the Vector Table location -------------------------------------*/
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif /* USER_VECT_TAB_ADDRESS */
}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32f3xx_hal.h file (default value
  *             8 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in stm32f3xx_hal.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, predivfactor = 0;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case RCC_CFGR_SWS_HSI:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case RCC_CFGR_SWS_HSE:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case RCC_CFGR_SWS_PLL:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMUL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      pllmull = ( pllmull >> 18) + 2;

#if defined (STM32F302xE) || defined (STM32F303xE) || defined (STM32F398xx)
        predivfactor = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
      if (pllsource == RCC_CFGR_PLLSRC_HSE_PREDIV)
      {
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / predivfactor) * pllmull;
      }
      else
      {
        /* HSI oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSI_VALUE / predivfactor) * pllmull;
      }
#else      
      if (pllsource == RCC_CFGR_PLLSRC_HSI_DIV2)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
        predivfactor = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / predivfactor) * pllmull;
      }
#endif /* STM32F302xE || STM32F303xE || STM32F398xx */
      break;
    default: /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK clock frequency ----------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup 
   Timeout value 
   */
#if !defined  (HSE_STARTUP_TIMEOUT) 
 #define HSE_STARTUP_TIMEOUT  ((uint16_t)0x0500)   /*!< Time out for HSE start up */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief  Configures the System clock source, PLL Multiplier and Divider factors,
  *               AHB/APBx prescalers and Flash settings
  * @note   This function should be called only once the RCC clock configuration  
  *         is reset to the default reset state (done in SystemInit() function).             
  * @param  None
  * @retval None
  */
static void SetSysClock(void)
{
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/

	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------*/
	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		HSEStatus = (uint32_t)0x01;
	}
	else
	{
		HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01)
	{
		/* Enable Prefetch Buffer and set Flash Latency */
		FLASH->ACR = FLASH_ACR_PRFTBE | (uint32_t)FLASH_ACR_LATENCY_1;

		/* HCLK = SYSCLK / 1 */
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
			
		/* PCLK2 = HCLK / 1 */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
		
		/* PCLK1 = HCLK / 2 */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

		/* PLL configuration */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMUL9);

		/* Enable PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till PLL is ready */
		while((RCC->CR & RCC_CR_PLLRDY) == 0)
		{
		}
		
		/* Select PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

		/* Wait till PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
		{
		}
	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock
			configuration. User can add here some code to deal with this error */
	}
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
