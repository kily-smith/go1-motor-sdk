#include "go1_transport.h"
#include <string.h>
int go1_send_recv(Go1Transport *io,const Go1MotorCmd *cmd,Go1MotorData *data,uint32_t timeout_ms) {
 uint8_t wire[GO1_TX_SIZE],byte; Go1Parser p; uint32_t start,elapsed; int r;
 if(data) memset(data,0,sizeof(*data));
 if(!io||!cmd||!data||!io->rx_reset||!io->tx||!io->rx_byte||!io->millis||!timeout_ms||timeout_ms>0x7fffffffu||go1_pack(cmd,wire)) return -1;
 start=io->millis(io->context);
 if(io->rx_reset(io->context)<0) return -2;
 elapsed=io->millis(io->context)-start;if(elapsed>=timeout_ms)return -3;
 if(io->tx(io->context,wire,sizeof(wire),timeout_ms-elapsed)<0)return -2;
 go1_parser_init(&p);
 while((elapsed=io->millis(io->context)-start)<timeout_ms) {
  r=io->rx_byte(io->context,&byte,timeout_ms-elapsed);if(r<0)return -2;if(r==0)return -3;
  if(go1_parser_feed(&p,byte,cmd->id,data))return 0;
 }
 return -3;
}
