#ifndef __TCP_SERVER_RAW_H
#define __TCP_SERVER_RAW_H

#include "../../lwip/src/include/lwip/opt.h"
#include "../../lwip/src/include/lwip/debug.h"
#include "../../lwip/src/include/lwip/stats.h"
#include "../../lwip/src/include/lwip/tcp.h"

#if LWIP_TCP

static struct tcp_pcb *server_pcb;

enum echo_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

struct echo_state
{
  u8_t state;
  u8_t retries;
  struct tcp_pcb *pcb;
  /* pbuf (chain) to recycle */
  struct pbuf *p;
};

void server_init(void);
err_t server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void server_error(void *arg, err_t err);
err_t server_poll(void *arg, struct tcp_pcb *tpcb);
err_t server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
void server_send(struct tcp_pcb *tpcb, struct echo_state *es);
void server_close(struct tcp_pcb *tpcb, struct echo_state *es);

#endif

#endif