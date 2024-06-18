#include "enc28j60.h"
//#include "main.h"
#include "F3_MACROS.h"
//#include "stm32f103xb.h"
#include <stdint.h>
#include <stdbool.h>

//uint16_t bufferSize;
bool ENC28J60_broadcast_enabled = false;
bool ENC28J60_promiscuous_enabled = false;

static uint8_t waitgwmac; // Bitwise flags of gateway router status - see below for states
//Define gateway router ARP statuses
#define WGW_INITIAL_ARP 1      // First request, no answer yet
#define WGW_HAVE_GW_MAC 2      // Have gateway router MAC
#define WGW_REFRESHING 4       // Refreshing but already have gateway MAC
#define WGW_ACCEPT_ARP_REPLY 8 // Accept an ARP reply

uint32_t delaycnt = 0; //request gateway ARP lookup
extern uint32_t lwip_localtime;
/* 
	in the C++ land of arduino buffer[] resolves through defines and pointers
	to the following allocation gPB which is defined in dchp.cpp
*/
//uint8_t gPB[500]; // also called buffer through a define directive

//variables

#define hspi				hspi1
#define PORT_CS			GPIOA
#define PIN_CS			GPIO_PIN_4

static uint8_t Enc28_Bank;
uint16_t gNextPacketPtr;

void del()
{
	//HAL_GPIO_WritePin(PORT_CS,PIN_CS,GPIO_PIN_RESET);

	delaycnt = lwip_localtime;
   	while((lwip_localtime-delaycnt) < 1){}
}

void enableChip()
{
	//HAL_GPIO_WritePin(PORT_CS,PIN_CS,GPIO_PIN_RESET);
   SET_BIT(GPIOA->BRR, GPIO_BRR_BR_4);

}

void disableChip()
{
	//HAL_GPIO_WritePin(PORT_CS,PIN_CS,GPIO_PIN_SET);
   SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_4);
}

void ENC28_SetPhyInterface(const spi_config* spi)
{
   //spi_param = spi;
}

uint8_t ENC28_readOp(uint8_t oper, uint8_t addr)
{
   uint8_t spiData[2];
   enableChip();
   spiData[0] = (oper | (addr & ADDR_MASK));
   //HAL_SPI_Transmit(&hspi, spiData, 1, 100);
   spi_write(SPI2_BASE, spiData, 1);
   if (addr & 0x80)
   {
      //HAL_SPI_Transmit(&hspi, spiData, 1, 100);
      //spi_write(SPI2_BASE, spiData, 1);
      spi_read(SPI2_BASE, &spiData[1], 1);
      //HAL_SPI_Receive(&hspi, &spiData[1], 1, 100);
   }
   //HAL_SPI_Receive(&hspi, &spiData[1], 1, 100);
   spi_read(SPI2_BASE, &spiData[1], 1);

   disableChip();

   return spiData[1];
}
void ENC28_writeOp(uint8_t oper, uint8_t addr, uint8_t data)
{
   uint8_t spiData[2];
   enableChip();
   spiData[0] = (oper | (addr & ADDR_MASK)); //((oper<<5)&0xE0)|(addr & ADDR_MASK);
   spiData[1] = data;
   //HAL_SPI_Transmit(&hspi, spiData, 2, 100);
   spi_write(SPI2_BASE, spiData, 2);
   disableChip();
}
uint8_t ENC28_readReg8(uint8_t addr)
{
   ENC28_setBank(addr);
   return ENC28_readOp(ENC28J60_READ_CTRL_REG, addr);
}

void ENC28_writeReg8(uint8_t addr, uint8_t data)
{
   ENC28_setBank(addr);
   ENC28_writeOp(ENC28J60_WRITE_CTRL_REG, addr, data);
}

uint16_t ENC28_readReg16(uint8_t addr)
{
   return ENC28_readReg8(addr) + (ENC28_readReg8(addr + 1) << 8);
}

void ENC28_writeReg16(uint8_t addrL, uint16_t data)
{
   ENC28_writeReg8(addrL, data);
   ENC28_writeReg8(addrL + 1, data >> 8);
}

void ENC28_setBank(uint8_t addr)
{
   if ((addr & BANK_MASK) != Enc28_Bank)
   {
      ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
      Enc28_Bank = addr & BANK_MASK;
      ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON1, Enc28_Bank >> 5);
   }
}

void ENC28_writePhy(uint8_t addr, uint16_t data)
{
   ENC28_writeReg8(MIREGADR, addr);
   ENC28_writeReg16(MIWRL, data);
   while (ENC28_readReg8(MISTAT) & MISTAT_BUSY)
      ;
}

uint8_t ENC28_readPhyByte(uint8_t addr)
{
   ENC28_writeReg8(MIREGADR, addr);     // pass the PHY address to the MII
   ENC28_writeReg8(MICMD, MICMD_MIIRD); // Enable Read bit
   while (ENC28_readReg8(MISTAT) & MISTAT_BUSY)
      ;                          // Poll for end of reading
   ENC28_writeReg8(MICMD, 0x00); // Disable MII Read

   //	return ENC28_readReg8(MIRDL) | (ENC28_readReg8(MIRDH) << 8);
   return ENC28_readReg8(MIRDH);
}

uint8_t ENC28J60_Init(const uint8_t *macaddr)
{
	/* in Arduino lib this function is called with a buffer size, mac address and CS pin
		* this buffer does not seem to be used anywhere local
	uint8_t spiData[2];

	*/
	//bufferSize = sizeof(gPB); // sets up read buffer sise

	// (1): Disable the chip CS pin

	//init cs pin 
	/*PA4 Output function push-pull*/

	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER4_1);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER4_0);

	CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_4);

	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR4_1);
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR4_0);

	CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR4_0);
	SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR4_1);

	disableChip();

	spi_init(SPI2_BASE, 0);
	spi_enable(SPI2_BASE, 1);
	//HAL_Delay(1);
	delay(1);
	//del();

	// (2): Perform soft reset to the ENC28J60 module
	ENC28_writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	//HAL_Delay(2);
	delay(1);
	//del();

	// (3): Wait untill Clock is ready
	while (!ENC28_readOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY)
		;

	// (4): Initialise RX and TX buffer size
	ENC28_writeReg16(ERXSTL, RXSTART_INIT);
	ENC28_writeReg16(ERXRDPTL, RXSTART_INIT);
	ENC28_writeReg16(ERXNDL, RXSTOP_INIT);
	ENC28_writeReg16(ERDPTL, RXSTART_INIT);
	ENC28_writeReg16(ETXSTL, TXSTART_INIT);
	ENC28_writeReg16(ETXNDL, TXSTOP_INIT);

	gNextPacketPtr = RXSTART_INIT;
	//if(gNextPacketPtr == RXSTART_INIT)
	//	ENC28_writeReg16(ERXRDPTL, RXSTOP_INIT);
	//else
	//	ENC28_writeReg16(ERXRDPTL, gNextPacketPtr-1);

	//ENC28J60_enableBroadcast(1);
	// Arduino lib set this here
	// Stretch pulses for LED, LED_A=Link, LED_B=activity
	//ENC28_writePhy(PHLCON, 0x476);

	// (5): Receive buffer filters
	
	uint8_t erxfcon =  ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_BCEN;
	ENC28_writeReg8(ERXFCON, erxfcon);
	//ENC28_writeReg8(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);

	// additional Arduino setup
	//ENC28_writeReg16(EPMM0, 0x303f); // pattern match filter
	//ENC28_writeReg16(EPMCS, 0xf7f9); // pattern match checksum filter

	// (6): MAC Control Register 1
	//	ENC28_writeReg8(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS|MACON1_PASSALL);
	// changed to
	//ENC28_writeReg8(MACON2, MACON1_MARXEN);
	ENC28_writeReg8(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
	//ENC28_writeReg8(MACON2, 0x00);

	// (7): MAC Control Register 3
	ENC28_writeReg8(MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN|MACON3_FULDPX);
	//ENC28_writeReg8(MACON4, MACON4_DEFER);

	// (8): NON/Back to back gap
	ENC28_writeReg16(MAIPGL, 0x0C12); // NonBackToBack gap
	ENC28_writeReg8(MABBIPG, 0x15);  // BackToBack gap

	// (9): Set Maximum framelenght
	ENC28_writeReg16(MAMXFLL, MAX_FRAMELEN); // Set Maximum frame length (any packet bigger will be discarded)

	// (10): Set the MAC address of the device
	ENC28_writeReg8(MAADR5, macaddr[0]);
	ENC28_writeReg8(MAADR4, macaddr[1]);
	ENC28_writeReg8(MAADR3, macaddr[2]);
	ENC28_writeReg8(MAADR2, macaddr[3]);
	ENC28_writeReg8(MAADR1, macaddr[4]);
	ENC28_writeReg8(MAADR0, macaddr[5]);

	/* 
		could back check the MADDR registers and see if they are
		loaded with the correct MAC
	*/

	//**********Advanced Initialisations************//
	// (1): Initialise PHY layer registers
	//	ENC28_writePhy(PHLCON, PHLCON_LED);
	//ENC28_writePhy(PHCON1, PHCON1_PDPXMD);
	ENC28_writePhy(PHCON1, PHCON1_PDPXMD);
	
	ENC28_writePhy(PHCON2, 0x00);
	//ENC28_writePhy(PHCON2, PHCON2_HDLDIS);
	/* Preferred half duplex: LEDA: Link status LEDB: Rx/Tx activity */
	ENC28_writePhy(PHLCON, ENC28J60_LAMPS_MODE);
	//ENC28_writePhy(PHLCON, 0x476);
	//ENC28_writePhy(PHIE, PHIE_PGEIE | PHIE_PLNKIE);

	// (2): Enable Rx interrupt line
	ENC28_setBank(ECON1);

	
	ENC28_writeReg8(ECON1, 0x00);
	//ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_AUTOINC);


	//ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_DMAIF | EIR_LINKIF |
	//		 EIR_TXIF | EIR_TXERIF | EIR_RXERIF | EIR_PKTIF);

	//ENC28_writeOp(ENC28J60_BIT_FIELD_SET, EIE,  EIE_INTIE | EIE_PKTIE | EIE_LINKIE 
	//			|EIE_TXIE | EIE_TXERIE | EIE_RXERIE);

	ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

	ENC28_writeOp(ENC28J60_BIT_FIELD_SET, EIR, EIR_PKTIF);

	// not used in Arduino code
	// ENC28_writeOp(ENC28J60_BIT_FIELD_SET, EIR, EIR_PKTIF);

	/*uint8_t rev = ENC28_readReg8(EREVID);
	// microchip forgot to step the number on the silicon when they
	// released the revision B7. 6 is now rev B7. We still have
	// to see what they do when they release B8. At the moment
	// there is no B8 out yet
	if (rev == 6){
		++rev; // implement arduino's revision value return.
	
    	SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS_14);
	}*/
	delay(1);
	return 0;
}

bool ENC28J60_isLinkUp(void)
{
   return (ENC28_readPhyByte(PHSTAT2) >> 2) & 1;
}

void ENC28J60_enableBroadcast(bool temporary)
{
   ENC28_writeReg8(ERXFCON, ENC28_readReg8(ERXFCON) | ERXFCON_BCEN);
   if (!temporary)
      ENC28J60_broadcast_enabled = true;
}

void ENC28J60_disableBroadcast(bool temporary)
{
   if (!temporary)
      ENC28J60_broadcast_enabled = false;
   if (!ENC28J60_broadcast_enabled)
      ENC28_writeReg8(ERXFCON, ENC28_readReg8(ERXFCON) & ~ERXFCON_BCEN);
}

#if 0

#else
static void ENC28_writeBuf(uint8_t *data, uint16_t len)
{
   uint8_t spiData[2];
   // enable chip
   enableChip();

   spiData[0] = ENC28J60_WRITE_BUF_MEM;
   //HAL_SPI_Transmit(&hspi, spiData, 1, 100);
   spi_write(SPI2_BASE, spiData, 1);

   //	spiData[1] = 0xFF;
   //	HAL_SPI_Transmit(&hspi, &spiData[1], 1, 100);
   //spi_write(SPI2_BASE, spiData[1], 1);

   //HAL_SPI_Transmit(&hspi, data, len, 100);
   spi_write(SPI2_BASE, data, len);

   // disable chip
   disableChip();
}


void ENC28_packetSend(uint8_t *buf, uint16_t len)
{
   uint8_t retry = 0;

   /*while (1)
   {
      // latest errata sheet: DS80349C
      // always reset transmit logic (Errata Issue 12)
      // the Microchip TCP/IP stack implementation used to first check
      // whether TXERIF is set and only then reset the transmit logic
      // but this has been changed in later versions; possibly they
      // have a reason for this; they don't mention this in the errata
      // sheet
      ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
      ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
      ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF | EIR_TXIF);

      // prepare new transmission
      if (retry == 0)
      {
         ENC28_writeReg16(EWRPT, TXSTART_INIT);
         ENC28_writeReg16(ETXND, TXSTART_INIT + len);
         ENC28_writeOp(ENC28J60_WRITE_BUF_MEM, 0, 0); //line 485 enc28j60.cpp
         ENC28_writeBuf(buf, len);
      }

      // initiate transmission
      ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);

      // wait until transmission has finished; referring to the data sheet and
      // to the errata (Errata Issue 13; Example 1) you only need to wait until either
      // TXIF or TXERIF gets set; however this leads to hangs; apparently Microchip
      // realized this and in later implementations of their tcp/ip stack they introduced
      // a counter to avoid hangs; of course they didn't update the errata sheet
      uint16_t count = 0;
      while ((ENC28_readReg8(EIR) & (EIR_TXIF | EIR_TXERIF)) == 0 && ++count < 1000U)
         ;
      if (!(ENC28_readReg8(EIR) & EIR_TXERIF) && count < 1000U)
      {
         // no error; start new transmission
         break;
      }

      // cancel previous transmission if stuck
      ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
      break;

      //retry++; // from Arduino enc28j60.cpp
   }*/
   while(!ENC28_readOp(ENC28J60_READ_CTRL_REG,ECON1)&ECON1_TXRTS)
	{
		if(ENC28_readReg8(EIR)&EIR_TXERIF)
		{
			ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON1,ECON1_TXRST);
			ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, ECON1,ECON1_TXRST);
		}
	}
	ENC28_writeReg16(EWRPTL, TXSTART_INIT);
	ENC28_writeReg16(ETXNDL, TXSTART_INIT+len);
	ENC28_writeBuf((uint8_t*)"\x00", 1);
	ENC28_writeBuf(buf, len);
	ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON1,ECON1_TXRTS);
}

void readBuf(uint8_t *data, uint16_t len)
{
   //   uint8_t nextbyte;
   uint8_t spiData[2];

   enableChip();
   if (len != 0)
   {
      spiData[0] = ENC28J60_READ_BUF_MEM;
      //HAL_SPI_Transmit(&hspi, spiData, 1, 100);
      spi_write(SPI2_BASE, spiData, 1);
      //HAL_SPI_Receive(&hspi, data, len, 100);
      spi_read(SPI2_BASE, data, len);
   }
   disableChip();
}

uint16_t ENC28J60_packetReceive(uint8_t *buf, int max_len)
{
   static bool unreleasedPacket = false;
   uint16_t len = 0;

   //if (unreleasedPacket)
   //{
   //   if (gNextPacketPtr == 0)
   //      ENC28_writeReg16(ERXRDPT, RXSTOP_INIT);
   //   else
   //      ENC28_writeReg16(ERXRDPT, gNextPacketPtr - 1);
   //   unreleasedPacket = false;
  	// }

	static uint8_t d_t = 0;
	if(ENC28_readReg8(EPKTCNT) != 0){
		while (ENC28_readReg8(EPKTCNT) != 0)
		{
			d_t = 0;
			//gNextPacketPtr = ENC28_readReg16(ERXRDPTL);
			ENC28_writeReg16(ERDPTL, gNextPacketPtr);

			typedef struct
			{
				uint16_t nextPacket;
				uint16_t byteCount;
				uint16_t status;
			} header;

			header h;
			readBuf((uint8_t *)&h, sizeof(header));

			gNextPacketPtr = h.nextPacket;
			len = h.byteCount - 4; //remove the CRC count
			if (len > max_len )
				len = max_len;
			if ((h.status & 0x80) == 0)
				len = 0;
			else
			{
				SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS_15);
				
				readBuf(buf, len);
			}
			buf[len] = 0;
			
				if(gNextPacketPtr == RXSTART_INIT)
			//|| (gNextPacketPtr -1 > RXSTOP_INIT))
				{
					ENC28_writeReg16(ERXRDPTL, RXSTOP_INIT-1);
					//gNextPacketPtr = RXSTOP_INIT;
				}
				else
				{
					ENC28_writeReg16(ERXRDPTL, gNextPacketPtr-1);
					//gNextPacketPtr = gNextPacketPtr-1;
				}
				//enc28j60_writeOp(ENC28J60_BIT_FIELD_SET,ECON2,ECON2_PKTDEC);
			//unreleasedPacket = true;

			ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

			//ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_PKTIF);
		}
	}
	else
	{

		//ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_PKTIF);
		//d_t++;
		//if(d_t > 200)
		//{
		//	d_t = 0;
		//	ENC28_writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
		//	delay(1);
			//del();

			// (3): Wait untill Clock is ready
		//	while (!ENC28_readOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY)
		//		;
		//}
		
		uint8_t stateEconRxen = ENC28_readReg8(ECON1) & ECON1_RXEN;
       	// ESTAT.BUFFER rised on TX or RX error
       	// I think the test of this register is not necessary - EIR.RXERIF state checking may be enough
       	uint8_t stateEstatBuffer = ENC28_readReg8(ESTAT) & ESTAT_BUFFER;
       	// EIR.RXERIF set on RX error
       	uint8_t stateEirRxerif = ENC28_readReg8(EIR) & EIR_RXERIF;

		//uint8_t rev = ENC28_readReg8(EREVID);
		SET_BIT(GPIOE->BRR, GPIO_BRR_BR_15);
		// microchip forgot to step the number on the silicon when they
		// released the revision B7. 6 is now rev B7. We still have
		// to see what they do when they release B8. At the moment
		// there is no B8 out yet
		/*if (rev == 6){
			++rev; // implement arduino's revision value return.
			static i = 0;
			if(i == 0)
			{
				SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS_14);
				SET_BIT(GPIOE->BRR, GPIO_BRR_BR_12);
				i++;
			}
			else
			{
				i--;
				SET_BIT(GPIOE->BRR, GPIO_BRR_BR_14);
				SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS_12);
			}
			delay(1);
		}*/
		if ((!stateEconRxen) || (stateEstatBuffer && stateEirRxerif))//(!stateEconRxen) //|| (stateEstatBuffer && stateEirRxerif)) 
		{
        	//Serial.println ("ENC28J60 reinit");
			
			ENC28_writeReg16(ERXSTL, RXSTART_INIT);
			ENC28_writeReg16(ERXRDPTL, RXSTART_INIT);
			ENC28_writeReg16(ERXNDL, RXSTOP_INIT);
			ENC28_writeReg16(ERDPTL, RXSTART_INIT);
			ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_RXERIF);
			ENC28_writeOp(ENC28J60_BIT_FIELD_CLR, ESTAT, ESTAT_BUFFER);
			ENC28_writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
			gNextPacketPtr = RXSTART_INIT;

			//ENC28_writeReg16(ERXRDPTL, RXSTART_INIT);
        	//eth_reset();
			static i = 0;
			if(i == 0)
			{
				SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS_14);
				SET_BIT(GPIOE->BRR, GPIO_BRR_BR_12);
				i++;
			}
			else
			{
				i--;
				SET_BIT(GPIOE->BRR, GPIO_BRR_BR_14);
				SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS_12);
			}

			//ENC28_writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
			//delay(1);
			//del();

			// (3): Wait untill Clock is ready
			//while (!ENC28_readOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY)
					;

			delay(1);
       	}
	}
	return len;
}

#endif


