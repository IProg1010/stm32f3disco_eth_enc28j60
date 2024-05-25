#ifndef _LWIP_USER
#define _LWIP_USER

#include "../stm32f1xx.h"
#include "../stm32f103xb.h"

typedef struct
{
	uint8_t Mac[6];
	uint8_t IP[4];
	uint8_t Mask[4];
	uint8_t GW[4];
	uint8_t dhcpstatus;
}IP_Config;

uint8_t lwip_config_init(void);
void lwip_periodic_handle(void);
void lwip_dhcp_process_handle(void);

#endif

