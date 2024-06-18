#include "proto_handle.h"
#include "usart.h"
#include "spi.h"
#include "i2c.h"

void proto_init()
{
    f_table[0].interface_type = USART_INTERFACE;
    f_table[0].data_read = spi_write;

    f_table[1].interface_type = SPI_INTERFACE;    
    f_table[1].data_read = spi_write_data;
    
    f_table[2].interface_type = I2C_INTERFACE;    
    f_table[2].data_read = spi_write;

    config_packet* config = (config_packet*) data;
    //config_packet->
}

void proto_handle(uint8_t* data, uint16_t length)
{

}

void proto_data_write(uint8_t* data, uint16_t length)
{

}

void proto_data_read(uint8_t* data, uint16_t length)
{

}