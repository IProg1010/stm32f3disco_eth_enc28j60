#include "proto_handle.h"
#include "usart.h"
#include "spi.h"
#include "i2c.h"
#include "F3_MACROS.h"

struct inter_descript f_table[5];
uint8_t interface_count = 2;

uint16_t data_found = 0;
uint8_t send_packet[SEND_BUFF_SIZE];

void (*interface_conf)(uint8_t* conf, uint32_t int_base_addr);
void (*interface_ctrl)(uint8_t* ctrl_data, uint32_t int_base_addr);
void (*data_write)(uint32_t base_addr, uint8_t* data, uint16_t length, void (*write_end_callback) (uint16_t len));
void (*data_read)(uint32_t base_addr, uint8_t* data, uint16_t length, void (*read_end_callback) (uint16_t len));

void uart_conf(uint8_t* conf, uint32_t int_base_addr);
void uart_control(uint8_t* ctrl_data, uint32_t int_base_addr);

void uart1_write_end(uint16_t len);
void uart1_read_end(uint16_t len);

void uart2_write_end(uint16_t len);
void uart2_read_end(uint16_t len);
//void uart_conf(uint8_t* data, uint16_t len, struct inter_descript* descript);
//void config_packet_send(uint8_t* conf, uint32_t int_base_addr);

void proto_init()
{
    //uart1 config
    f_table[0].interface_type = 0x01;
    f_table[0].interface_num = 0x01;
    f_table[0].int_base_addr = USART1_BASE;

    f_table[0].interface_conf = &uart_conf;
    f_table[0].interface_ctrl = &uart_control;
    f_table[0].data_write = &usart_write;
    f_table[0].data_read = &usart_read;

    f_table[0].write_end_callback = &uart1_write_end;
    f_table[0].read_end_callback = &uart1_read_end;
    
    //uart2 config
    f_table[1].interface_type = 0x01;
    f_table[1].interface_num = 0x02;
    f_table[0].int_base_addr = USART2_BASE;

    f_table[1].interface_conf = &uart_conf;
    f_table[1].interface_ctrl = &uart_control;
    f_table[1].data_write = &usart_write;
    f_table[1].data_read = &usart_read;

    f_table[1].write_end_callback = &uart2_write_end;
    f_table[1].read_end_callback = &uart2_read_end;
}

void proto_handle(uint8_t* data, uint16_t length)
{

}

void proto_recv(uint8_t* packet, uint16_t len)
{

}

void proto_send(uint8_t* packet, uint16_t len)
{
    //pack_header* pack = packet;
    proto_packet_parser(packet, len);
}

void proto_packet_parser(uint8_t* packet, uint16_t len)
{
    pack_header* pack = packet;
    
    uint8_t typeasd;
    uint8_t f = 0x00;  
    uint8_t flag = 0x00;  
    interface_header* interface_pack;
    if(pack->pack_start1 == PROTO_START1 && pack->pack_start2 == PROTO_START2)
    {
        switch(pack->pack_type)
        {
            case ETHERNET_TYPE:

                break;
            case GET_INTERFACE_TYPE:

                send_packet[0] = PROTO_START1;
                send_packet[1] = PROTO_START2;
                send_packet[2] = pack->pack_type;
                send_packet[3] = interface_count;
                for(int i = 0; i < interface_count; i++)
                {    
                    send_packet[4+2*i] = f_table[i].interface_type; 
                    send_packet[4+2*i+1] = f_table[i].interface_num;
                }
                data_found = 4+interface_count*2;

                break;
            case GET_INTERFACE_SETTING_TYPE:

                break;
            case GET_INTERFACE_WORK_TYPE:
                             
                interface_pack = packet+sizeof(pack_header);
                for(int i = 0; i < interface_count; i++)
                {
                    if(f_table[i].interface_type == interface_pack->interface_type
                        && f_table[i].interface_num == interface_pack->interface_num)
                    {
                        if(interface_pack->pack_type == CONFIG_PACK)
                        {
                            interface_conf = f_table[i].interface_conf;
                            uint8_t* conf = packet+sizeof(pack_header)+sizeof(interface_pack);
                            interface_conf(conf, f_table[i].int_base_addr);

                            //if(flag == 0x01 && f == 0x00)
                            //{
                                send_packet[0] = PROTO_START1;
                                send_packet[1] = PROTO_START2;
                                send_packet[2] = pack->pack_type;
                                send_packet[3] = interface_pack->pack_type;
                                send_packet[4] = interface_pack->interface_type;
                                send_packet[5] = interface_pack->interface_num;
                                send_packet[6] = CONFIG_OK;
                                data_found = 7;
                            //}
                            /*else
                            { 
                                send_packet[0] = PROTO_START1;
                                send_packet[1] = PROTO_START2;
                                send_packet[2] = pack->pack_type;
                                send_packet[3] = interface_pack->interface_type;
                                send_packet[4] = interface_pack->interface_num;
                                send_packet[5] = interface_pack->pack_type;
                                send_packet[6] = CONFIG_ERR;
                                data_found = 7;
                            }*/
                        }
                        else if(interface_pack->pack_type == CTRL_PACK)
                        {
                            interface_ctrl = f_table[i].interface_ctrl;
                            uint8_t* ctrl = packet+sizeof(pack_header)+sizeof(interface_pack);
                            interface_conf(ctrl, f_table[i].int_base_addr);

                            send_packet[0] = PROTO_START1;
                            send_packet[1] = PROTO_START2;
                            send_packet[2] = pack->pack_type;
                            send_packet[3] = interface_pack->pack_type;
                            send_packet[4] = interface_pack->interface_type;
                            send_packet[5] = interface_pack->interface_num;
                            if(ctrl == 0x01)
                                send_packet[6] = CTRL_ON;
                            else
                                send_packet[6] = CTRL_OFF;
                            
                            data_found = 7;
                        }
                        else if(interface_pack->pack_type == DATA_WRDATA_PACK)
                        {
                            data_write = f_table[i].data_write;
                            uint16_t* len = packet+sizeof(pack_header)+sizeof(interface_pack);
      
                            uint8_t* data = packet+sizeof(pack_header)+sizeof(interface_pack)+sizeof(uint16_t);
                            for(int i = 0; i < len; i++)
                            {
                                f_table[i].buff_write[i] = *(packet+sizeof(pack_header)+sizeof(interface_pack)+sizeof(uint16_t)+i);
                            }

                            data_write(f_table[i].int_base_addr, f_table[i].buff_write, *len, f_table[i].write_end_callback); 
                        }
                        else if(interface_pack->pack_type == DATA_GETD_PACK)
                        {
                            data_read = f_table[i].data_read;
                            uint16_t* len = packet+sizeof(pack_header)+sizeof(interface_pack);
                            //data_read(f_table[i].buff_read, len);
                            data_read(f_table[i].int_base_addr, f_table[i].buff_read, *len, f_table[i].read_end_callback);
                        }
                        else
                        {
                            f = 0x02;        
                        }

                        flag = 0x01;
                    }
                }
                
                

                break;

            default: //unknow packet code
                data_found = 1;

                send_packet[0] = PROTO_START1;
                send_packet[1] = PROTO_START2;
                send_packet[2] = pack->pack_type;
                send_packet[3] = UNKNOW_TYPE;
                data_found = 4;
                
                break;
        }
    }

}

void proto_poll()
{
    for(int i = 0; i < interface_count; i++)
    {
        if(f_table[i].rd_flag == 0x01)
        {
            send_packet[0] = PROTO_START1;
            send_packet[1] = PROTO_START2;
            send_packet[2] = GET_INTERFACE_WORK_TYPE;
            send_packet[3] = DATA_RDATA_PACK;
            send_packet[4] = f_table[i].interface_type;
            send_packet[5] = f_table[i].interface_num;
            send_packet[6] = READ_OK;
            send_packet[7] = f_table[i].rd_data_len >> 8 & 0x00FF;
            send_packet[8] = f_table[i].rd_data_len & 0x00FF;

            for(int i = 0; i < f_table[i].rd_data_len; i++)
                send_packet[9+i] = f_table[i].buff_read[i];
            
            data_found = 9+f_table[i].rd_data_len;

            break;
        }

        if(f_table[i].wr_flag == 0x01)
        {
            send_packet[0] = PROTO_START1;
            send_packet[1] = PROTO_START2;
            send_packet[2] = GET_INTERFACE_WORK_TYPE;
            send_packet[3] = DATA_WRDATA_PACK;
            send_packet[4] = f_table[i].interface_type;
            send_packet[5] = f_table[i].interface_num;
            send_packet[6] = SEND_OK;
            send_packet[7] = f_table[i].rd_data_len >> 8 & 0x00FF;
            send_packet[8] = f_table[i].rd_data_len & 0x00FF;

            data_found = 9;

            break;
        }
    }
}

void uart_conf(uint8_t* conf, uint32_t int_base_addr)
{
    usart_config* config = (usart_config*) conf;
    usart_init(int_base_addr, config);
}

void uart_control(uint8_t* ctrl_data, uint32_t int_base_addr)
{
    usart_enable(int_base_addr, *ctrl_data);
}


void uart1_write_end(uint16_t len)
{
    uint8_t wr_flag;
    f_table[0].wr_data_len = len;
}
void uart1_read_end(uint16_t len)
{
    f_table[0].rd_flag = 0x01;
    f_table[0].rd_data_len = len;
}


void uart2_write_end(uint16_t len)
{
    f_table[1].rd_flag = 0x01;
    f_table[1].rd_data_len = len;
}

void uart2_read_end(uint16_t len)
{
    f_table[1].rd_flag = 0x01;
    f_table[1].rd_data_len = len;
}