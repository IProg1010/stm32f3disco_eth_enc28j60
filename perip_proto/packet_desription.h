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

//defined interface type
enum IfaceType
{
    uart, spi, i2c, can
};

typedef struct
{
    uint8_t pack_start;
    uint8_t pack_type;
    uint16_t lenght;
    uint8_t data[];
} pack_header;

typedef struct
{
    uint8_t number;
    uint8_t mode;
    uint16_t baud_rate;
    uint8_t bits;
    uint8_t stop_bits;
    uint8_t parity;
} uart_config_packet;

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
    uint8_t interface_config[];
} config_packet;

typedef struct 
{
    pack_header head;
    uint8_t interface_type;
    uint8_t interface_num;
    uint16_t lenght;
    uint8_t data[];
} data_packet;


typedef struct 
{
    uint8_t interface_type;
    void (*interface_conf) (uint8_t* data, uint16_t length);
    void (*data_write) (uint8_t* data, uint16_t length);
    void (*data_read) (uint8_t* data, uint16_t length);
} function_table;	


#endif
