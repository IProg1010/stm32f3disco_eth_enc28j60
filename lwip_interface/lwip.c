/**
  ******************************************************************************
  * @file    netconf.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   Network connection configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/tcp_impl.h"
#include "lwip/udp.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "ethernetif.h"
//#include "main.h"
#include "lwip.h"
#include "stm32f103xb.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
#define MAX_DHCP_TRIES 4

#define USE_LCD 1

#define DEBUG(...)	
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------


------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct netif gnetif;
uint32_t TCPTimer = 0;
uint32_t ARPTimer = 0;
uint32_t IPaddress = 0;

#ifdef USE_DHCP
	uint32_t DHCPfineTimer = 0;
	uint32_t DHCPcoarseTimer = 0;
	__IO uint8_t DHCP_state;
#endif

#ifndef USE_DHCP
	#define 	IP_ADDR0				192
	#define 	IP_ADDR1				168
	#define 	IP_ADDR2				1
	#define 	IP_ADDR3				118
	
	#define 	NETMASK_ADDR0		255
	#define 	NETMASK_ADDR1		255
	#define 	NETMASK_ADDR2		255
	#define 	NETMASK_ADDR3		0
	
	#define		GW_ADDR0				192
	#define		GW_ADDR1				168
	#define		GW_ADDR2				1
	#define		GW_ADDR3				1
#endif

extern __IO uint32_t EthStatus;

/* Private functions ---------------------------------------------------------*/
void LwIP_DHCP_Process_Handle(void);

/**
* @brief  Initializes the lwIP stack
* @param  None
* @retval None
*/
void LwIP_Init(void)
{
    struct ip_addr ipaddr;
    struct ip_addr netmask;
    struct ip_addr gw;
#ifdef USE_DHCP
    uint8_t iptab[4] = {0};
    uint8_t iptxt[20];
#endif

    /* Initializes the dynamic memory heap defined by MEM_SIZE.*/
    mem_init();

    /* Initializes the memory pools defined by MEMP_NUM_x.*/
    memp_init();

#ifdef USE_DHCP
    ipaddr.addr = 0;
    netmask.addr = 0;
    gw.addr = 0;
#else
    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
    IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif

    /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
  struct ip_addr *netmask, struct ip_addr *gw,
  void *state, err_t (* init)(struct netif *netif),
  err_t (* input)(struct pbuf *p, struct netif *netif))

  Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

    /*  Registers the default network interface.*/
    netif_set_default(&gnetif);

    //if (EthStatus == (ETH_INIT_FLAG | ETH_LINK_FLAG))
    if (1)
    {
        /* Set Ethernet link flag */
        gnetif.flags |= NETIF_FLAG_LINK_UP;

        /* When the netif is fully configured this function must be called */
        netif_set_up(&gnetif);

#ifdef USE_DHCP
        DHCP_state = DHCP_START;
#endif /* USE_DHCP */
    }
    else
    {
        /* When the netif link is down this function must be called */
        netif_set_down(&gnetif);

#ifdef USE_DHCP
        DHCP_state = DHCP_LINK_DOWN;
#endif /* USE_DHCP */
    }

    /* Set the link callback function, this function is called on change of link status*/
    //netif_set_link_callback(&gnetif, ETH_link_callback);
}

/**
* @brief  Called when a frame is received
* @param  None
* @retval None
*/
void LwIP_Pkt_Handle(void)
{
    /* Read a received packet from the Ethernet buffers and send it to the lwIP for handling */
    ethernetif_input(&gnetif);
}

/**
* @brief  LwIP periodic tasks
* @param  localtime the current LocalTime value
* @retval None
*/
void LwIP_Periodic_Handle(uint32_t localtime)
{
#if LWIP_TCP
    /* TCP periodic process every 250 ms */
    if (localtime - TCPTimer >= TCP_TMR_INTERVAL)
    {
        TCPTimer = localtime;
        tcp_tmr();
    }
#endif

    /* ARP periodic process every 5s */
    if ((localtime - ARPTimer) >= ARP_TMR_INTERVAL)
    {
        ARPTimer = localtime;
        etharp_tmr();
    }

#ifdef USE_DHCP
    /* Fine DHCP periodic process every 500ms */
    if (localtime - DHCPfineTimer >= DHCP_FINE_TIMER_MSECS)
    {
        DHCPfineTimer = localtime;
        dhcp_fine_tmr();
        if ((DHCP_state != DHCP_ADDRESS_ASSIGNED) &&
            (DHCP_state != DHCP_TIMEOUT) &&
            (DHCP_state != DHCP_LINK_DOWN))
        {
            /* toggle LED1 to indicate DHCP on-going process */
            //STM_EVAL_LEDToggle(LED1);

            /* process DHCP state machine */
            LwIP_DHCP_Process_Handle();
        }
    }

    /* DHCP Coarse periodic process every 60s */
    if (localtime - DHCPcoarseTimer >= DHCP_COARSE_TIMER_MSECS)
    {
        DHCPcoarseTimer = localtime;
        dhcp_coarse_tmr();
    }

#endif
}

#ifdef USE_DHCP
/**
* @brief  LwIP_DHCP_Process_Handle
* @param  None
* @retval None
*/
void LwIP_DHCP_Process_Handle()
{
    struct ip_addr ipaddr;
    struct ip_addr netmask;
    struct ip_addr gw;
    uint8_t iptab[4] = {0};
    uint8_t iptxt[20];

    switch (DHCP_state)
    {
    case DHCP_START:
    {
        DHCP_state = DHCP_WAIT_ADDRESS;
        dhcp_start(&gnetif);
        /* IP address should be set to 0 
         every time we want to assign a new DHCP address */
        IPaddress = 0;
    }
    break;

    case DHCP_WAIT_ADDRESS:
    {
        /* Read the new IP address */
        IPaddress = gnetif.ip_addr.addr;

        if (IPaddress != 0)
        {
            DHCP_state = DHCP_ADDRESS_ASSIGNED;

            /* Stop DHCP */
            dhcp_stop(&gnetif);
        }
        else
        {
            /* DHCP timeout */
            if (gnetif.dhcp->tries > MAX_DHCP_TRIES)
            {
                DHCP_state = DHCP_TIMEOUT;

                /* Stop DHCP */
                dhcp_stop(&gnetif);

#if 0
          /* Static address used */
          IP4_ADDR(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
          IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
          IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
          netif_set_addr(&gnetif, &ipaddr , &netmask, &gw);
#endif
							
            }
        }
    }
    break;
    default:
        break;
    }
}
#endif

#define SYSTEMTICK_PERIOD_MS 1

__IO uint32_t LocalTime = 0;

void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
