#ifndef PROTO_HANDLE_H
#define PROTO_HANDLE_H

#include <stdint.h>
#include "packet_desription.h"

uint8_t new_data_found = 0x00;

//protocol initialization function
void proto_init();

//protocol recv/send data function
void proto_recv(uint8_t* packet, uint16_t len);
void proto_send(uint8_t* packet, uint16_t len);

function_table f_table[3];

#endif
