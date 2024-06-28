#ifndef PROTO_PACKET_H
#define PROTO_PACKET_H

#include <stdint.h>

//defined buffer USART address
#define USART_BUFF_RX_ADDR 0x2000000
#define USART_BUFF_TX_ADDR 0x3000000

//defined buffer for SPI address
#define SPI_BUFF_RX_ADDR 0x4000000
#define SPI_BUFF_TX_ADDR 0x5000000

//defined buffer for I2C address
#define I2C_BUFF_RX_ADDR 0x6000000
#define I2C_BUFF_TX_ADDR 0x7000000

//defined protocol start bytes
#define PROTO_START1 0x44
#define PROTO_START2 0x55


//define packet protocol type
#define ETHERNET_TYPE 0x20
#define GET_INTERFACE_TYPE 0x23
#define GET_INTERFACE_SETTING_TYPE 0x24
#define GET_INTERFACE_WORK_TYPE 0x25

//define unknow packet code
#define UNKNOW_TYPE 0xDF
#define UNKNOW_INTERFACE 0x46

//define send packet buffer size
#define SEND_BUFF_SIZE 0x0FFF

//define interface type
#define USART_TYPE 0x01
#define SPI_TYPE 0x02
#define I2C_TYPE 0x03
#define CAN_TYPE 0x04
#define GPIO_TYPE 0x05
#define PWM_TYPE 0x06

//define interface type
#define CONFIG_PACK 0x01
#define CTRL_PACK 0x02
#define DATA_WRDATA_PACK 0x03
#define DATA_GETD_PACK 0x04
#define DATA_RDATA_PACK 0x05

//define return value
#define CONFIG_OK 0x02
#define CONFIG_ERR 0x01

#define CTRL_ON 0x02
#define CTRL_OFF 0x01

#define SEND_OK 0x02
#define SEND_ERR 0x01

#define READ_OK 0x02
#define READ_ERR 0x01

//defined interface type
enum IfaceType
{
    uart, spi, i2c, can
};

typedef struct
{
    uint8_t pack_start1;
    uint8_t pack_start2;
    uint8_t pack_type;
    //uint16_t lenght;
    //uint8_t* data;
} pack_header;

typedef struct
{
    pack_header pack;
    uint8_t cmd_code;
    uint8_t inter_count;
    uint8_t uart_inter_count;
} cmd_header;

typedef struct
{
    uint8_t pack_type;
    uint8_t interface_type;
    uint8_t interface_num;
} interface_header;


typedef struct
{
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gw[4];
    uint8_t mac[6];
} ethernet_get_data;


typedef struct
{
    //uint8_t mode;
    uint32_t baud_rate;
    uint8_t bits;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t parity_type;
    uint8_t port_num;
} uart_config_data;

typedef struct
{
    uint8_t number;
    uint8_t mode;
    uint16_t baud_rate;
    uint8_t frame_bit;
} spi_config_packet;

typedef struct
{
    uint8_t number;
    uint8_t mode;
    uint16_t baud_rate;
} i2c_config_packet;

typedef struct
{
    uint8_t number;
    uint8_t mode;
    uint16_t baud_rate;
} can_config_packet;

typedef struct 
{
    pack_header head;
    uint8_t interface_type;
    uint8_t interface_num;
    uint16_t lenght;
    uint8_t data[];
} data_packet;



#endif
