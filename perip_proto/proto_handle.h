#ifndef PROTO_HANDLE_H
#define PROTO_HANDLE_H

#include <stdint.h>
#include "packet_desription.h"

//uint8_t new_data_found = 0x00;

//protocol initialization function
void proto_init();

//protocol recv/send data function
void proto_recv(uint8_t* packet, uint16_t len);
void proto_send(uint8_t* packet, uint16_t len);
void proto_poll();
void proto_packet_parser(uint8_t* packet, uint16_t len);

struct inter_descript 
{
    uint8_t interface_type;
    uint8_t interface_num;
    
    uint8_t wr_flag;
    uint8_t wr_data_len;
    uint8_t buff_write[256];

    uint8_t rd_flag;
    uint8_t rd_data_len;
    uint8_t buff_read[256];

    uint32_t int_base_addr;
    uart_config_data* config_ptr;

    void (*interface_conf) (uint8_t* conf, uint32_t int_base_addr);
    void (*data_write) (uint32_t base_addr, uint8_t* data, uint16_t length, void (*write_end_callback) (uint16_t len));
    void (*data_read) (uint32_t base_addr, uint8_t* data, uint16_t length,  void (*read_end_callback) (uint16_t len));
    void (*write_end_callback) (uint16_t len);
    void (*read_end_callback) (uint16_t len);
    void (*interface_ctrl) (uint8_t* ctrl_data, uint32_t int_base_addr);
};

struct proto_interface_table 
{
    uint8_t interface_types;
};

#endif
