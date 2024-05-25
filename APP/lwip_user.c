#include "lwip_user.h" 
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/init.h"
#include "ethernetif.h" 
#include "lwip/timers.h"
#include "lwip/tcp_impl.h"
#include "ip_frag.h"
#include "lwip/tcpip.h" 
#include "enc28j60.h"
//#include "ethernetif.h"
#include <stdio.h>

#define LWIP_MAX_DHCP_TRIES 4
IP_Config lwipdev;						//lwip���ƽṹ�� 
struct netif lwip_netif;				//����һ��ȫ�ֵ�����ӿ�

extern uint32_t lwip_localtime;		//lwip����ʱ�������,��λ:ms

uint32_t TCPTimer=0;			//TCP��ѯ��ʱ��
uint32_t ARPTimer=0;			//ARP��ѯ��ʱ��
uint32_t InputTimer = 0;

uint8_t Mac[6] = {0x04,0x02,0x35,0x00,0x00,0x01};

#if LWIP_DHCP
uint32_t DHCPfineTimer=0;	//DHCP��ϸ������ʱ��
uint32_t DHCPcoarseTimer=0;	//DHCP�ֲڴ�����ʱ��
#endif

//lwip Ĭ��IP����
//lwipx:lwip���ƽṹ��ָ��
void lwip_ip_config(IP_Config *lwipx)
{
//	//Ĭ��Զ��IPΪ:192.168.1.100
//	lwipx->remoteip[0]=192;	
//	lwipx->remoteip[1]=168;
//	lwipx->remoteip[2]=1;
//	lwipx->remoteip[3]=100;
	//MAC��ַ����(�����ֽڹ̶�Ϊ:2.0.0,�����ֽ���STM32ΨһID)
	lwipx->Mac[0]=Mac[0];
	lwipx->Mac[1]=Mac[1];
	lwipx->Mac[2]=Mac[2];
	lwipx->Mac[3]=Mac[3];
	lwipx->Mac[4]=Mac[4];
	lwipx->Mac[5]=Mac[5]; 
	//Ĭ�ϱ���IPΪ:192.168.1.30
	lwipx->IP[0]=192;	
	lwipx->IP[1]=168;
	lwipx->IP[2]=1;
	lwipx->IP[3]=16;
	//Ĭ����������:255.255.255.0
	lwipx->Mask[0]=255;	
	lwipx->Mask[1]=255;
	lwipx->Mask[2]=255;
	lwipx->Mask[3]=0;
	//Ĭ������:192.168.1.1
	lwipx->GW[0]=192;	
	lwipx->GW[1]=168;
	lwipx->GW[2]=1;
	lwipx->GW[3]=1;	
//	lwipx->GW=0;//û��DHCP	
	lwipx->dhcpstatus = 0;
} 

//LWIP��ʼ��(LWIP������ʱ��ʹ��)
//����ֵ:0,�ɹ�
//      1,�ڴ����
//      2,DM9000��ʼ��ʧ��
//      3,��������ʧ��.
uint8_t lwip_config_init(void)
{
	struct netif *Netif_Init_Flag;		//����netif_add()����ʱ�ķ���ֵ,�����ж������ʼ���Ƿ�ɹ�
	struct ip_addr ipaddr;  			//ip��ַ
	struct ip_addr netmask; 			//��������
	struct ip_addr gw;      			//Ĭ������ 
	mem_init();
	memp_init();
	ENC28J60_Init((uint8_t*)Mac); //return 1;		//��ʼ��ENC28J60
	lwip_init();						//��ʼ��LWIP�ں�
	lwip_ip_config(&lwipdev);	//����Ĭ��IP����Ϣ

#if LWIP_DHCP		//ʹ�ö�̬IP
	ipaddr.addr = 0;
	netmask.addr = 0;
	gw.addr = 0;
#else				//ʹ�þ�̬IP
	IP4_ADDR(&ipaddr,lwipdev.IP[0],lwipdev.IP[1],lwipdev.IP[2],lwipdev.IP[3]);
	IP4_ADDR(&netmask,lwipdev.Mask[0],lwipdev.Mask[1] ,lwipdev.Mask[2],lwipdev.Mask[3]);
	IP4_ADDR(&gw,lwipdev.GW[0],lwipdev.GW[1],lwipdev.GW[2],lwipdev.GW[3]);
#endif
	Netif_Init_Flag=netif_add(&lwip_netif,&ipaddr,&netmask,&gw,NULL,&ethernetif_init,&ethernet_input);//�������б�������һ������
	
#if LWIP_DHCP			//���ʹ��DHCP�Ļ�
	lwipdev.dhcpstatus=0;	//DHCP���Ϊ0
	dhcp_start(&lwip_netif);	//����DHCP����
#endif
	if(Netif_Init_Flag==NULL)return 3;//��������ʧ�� 
	else//�������ӳɹ���,����netifΪĬ��ֵ,���Ҵ�netif����
	{
		netif_set_default(&lwip_netif); //����netifΪĬ������
		netif_set_up(&lwip_netif);		//��netif����
	}
	return 0;//����OK.
}   

//�����յ����ݺ���� 
void lwip_pkt_handle(void)
{
	//�����绺�����ж�ȡ���յ������ݰ������䷢�͸�LWIP���� 
	ethernetif_input(&lwip_netif);
}

//LWIP��ѯ����
void lwip_periodic_handle(void)
{
	if(lwip_localtime - InputTimer >2)
	{
		InputTimer =  lwip_localtime;
		ethernetif_input(&lwip_netif );
	}	
	//ARPÿ5s�����Ե���һ��
	if ((lwip_localtime - ARPTimer) >= ARP_TMR_INTERVAL)
	{
		ARPTimer =  lwip_localtime;
		etharp_tmr();
	}

#if LWIP_TCP
	//ÿ250ms����һ��tcp_tmr()����
	if (lwip_localtime - TCPTimer >= TCP_TMR_INTERVAL)
	{
		TCPTimer =  lwip_localtime;
		tcp_tmr();
	}
#endif

#if LWIP_DHCP //���ʹ��DHCP�Ļ�
	//ÿ500ms����һ��dhcp_fine_tmr()
	if (lwip_localtime - DHCPfineTimer >= DHCP_FINE_TIMER_MSECS)
	{
		DHCPfineTimer =  lwip_localtime;
		dhcp_fine_tmr();
		if ((lwipdev.dhcpstatus != 2)&&(lwipdev.dhcpstatus != 0XFF))
		{ 
			lwip_dhcp_process_handle();  //DHCP����
		}
	}

	//ÿ60sִ��һ��DHCP�ֲڴ���
	if (lwip_localtime - DHCPcoarseTimer >= DHCP_COARSE_TIMER_MSECS)
	{
		DHCPcoarseTimer =  lwip_localtime;
		dhcp_coarse_tmr();
	}  
#endif
}


//���ʹ����DHCP
#if LWIP_DHCP

//DHCP��������
void lwip_dhcp_process_handle(void)
{
	uint32_t ip=0,netmask=0,gw=0;
	switch(lwipdev.dhcpstatus)
	{
		case 0: 	//����DHCP
			dhcp_start(&lwip_netif);
			lwipdev.dhcpstatus = 1;		//�ȴ�ͨ��DHCP��ȡ���ĵ�ַ
			break;
		case 1:		//�ȴ���ȡ��IP��ַ
		{
			ip=lwip_netif.ip_addr.addr;		//��ȡ��IP��ַ
			netmask=lwip_netif.netmask.addr;//��ȡ��������
			gw=lwip_netif.gw.addr;			//��ȡĬ������ 
			
			if(ip!=0)			//��ȷ��ȡ��IP��ַ��ʱ��
			{
				lwipdev.dhcpstatus=2;	//DHCP�ɹ�
				//������ͨ��DHCP��ȡ����IP��ַ
				lwipdev.IP[3]=(uint8_t)(ip>>24); 
				lwipdev.IP[2]=(uint8_t)(ip>>16);
				lwipdev.IP[1]=(uint8_t)(ip>>8);
				lwipdev.IP[0]=(uint8_t)(ip);
				//����ͨ��DHCP��ȡ�������������ַ
				lwipdev.Mask[3]=(uint8_t)(netmask>>24);
				lwipdev.Mask[2]=(uint8_t)(netmask>>16);
				lwipdev.Mask[1]=(uint8_t)(netmask>>8);
				lwipdev.Mask[0]=(uint8_t)(netmask);
				//������ͨ��DHCP��ȡ����Ĭ������
				lwipdev.GW[3]=(uint8_t)(gw>>24);
				lwipdev.GW[2]=(uint8_t)(gw>>16);
				lwipdev.GW[1]=(uint8_t)(gw>>8);
				lwipdev.GW[0]=(uint8_t)(gw);
			}else if(lwip_netif.dhcp->tries>LWIP_MAX_DHCP_TRIES) //ͨ��DHCP�����ȡIP��ַʧ��,�ҳ�������Դ���
			{
				lwipdev.dhcpstatus=0XFF;//DHCP��ʱʧ��.
				//ʹ�þ�̬IP��ַ
				IP4_ADDR(&(lwip_netif.ip_addr),lwipdev.IP[0],lwipdev.IP[1],lwipdev.IP[2],lwipdev.IP[3]);
				IP4_ADDR(&(lwip_netif.netmask),lwipdev.Mask[0],lwipdev.Mask[1],lwipdev.Mask[2],lwipdev.Mask[3]);
				IP4_ADDR(&(lwip_netif.gw),lwipdev.GW[0],lwipdev.GW[1],lwipdev.GW[2],lwipdev.GW[3]);
			}
		}
		break;
		default : break;
	}
}
#endif 


uint32_t sys_now(void){
	return lwip_localtime;
}


