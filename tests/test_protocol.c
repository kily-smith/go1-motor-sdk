#include "go1_motor.h"
#include "go1_transport.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
static const uint8_t stop1[34]={0xfe,0xee,0x01,0xba,0x00,0xff,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0xd8,0x10,0x64,0x75};
/* Captured from the real motor after ID2 power-cycle, not synthesized. */
static const uint8_t reply[78]={0xfe,0xee,0x02,0x01,0x00,0x00,0x28,0x00,0x00,0xf8,0x38,0x41,0x00,0x00,0xbd,0xff,0x34,0x9e,0xc8,0x3b,0x00,0x00,0x00,0x00,0x00,0x00,0xf7,0xff,0x00,0x00,0x55,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x62,0x2d,0x07,0x00,0x00,0x00,0x46,0x93,0xf3,0x59};
typedef struct { unsigned now, index, reset, sent; int respond; } Fake;
static int fake_reset(void *v){Fake *f=v;f->reset++;return 0;}
static int fake_tx(void *v,const uint8_t *p,size_t n,uint32_t timeout){Fake *f=v;(void)timeout;assert(n==34&&p[2]==2);f->sent++;return 0;}
static int fake_rx(void *v,uint8_t *b,uint32_t timeout){Fake *f=v;(void)timeout;f->now++;if(!f->respond)return 0;if(f->index<sizeof(reply)){*b=reply[f->index++];return 1;}return 0;}
static uint32_t fake_clock(void *v){return ((Fake*)v)->now;}
int main(void){uint8_t tx[34],bad[78];Go1MotorCmd c={0};Go1MotorData d;Go1Parser p;size_t i;int found=0;
 assert(fabsf(go1_query_gear_ratio()-6.33f)<0.00001f);
 c.id=1;assert(!go1_pack(&c,tx)&&!memcmp(tx,stop1,34));
 assert(!go1_unpack(reply,78,2,&d)&&d.correct&&d.motor_id==2&&d.mode==0&&d.temp==40&&d.merror==0&&d.q_raw==13909);
 memcpy(bad,reply,78);bad[12]^=1;assert(go1_unpack(bad,78,2,&d)<0&&!d.correct);assert(go1_unpack(reply,78,1,&d)<0);
 go1_parser_init(&p);for(i=0;i<34;i++)assert(!go1_parser_feed(&p,stop1[i],2,&d));for(i=0;i<78;i++)assert(!go1_parser_feed(&p,bad[i],2,&d));for(i=0;i<78;i++)found+=go1_parser_feed(&p,reply[i],2,&d);assert(found==1&&d.q_raw==13909);
 c.mode=10;c.tau=-0.25f;c.dq=2;c.kp=0.5f;c.kd=1;assert(!go1_pack(&c,tx));assert(tx[12]==0xc0&&tx[13]==0xff&&tx[14]==0&&tx[15]==1);
 c.kp=-1;assert(go1_pack(&c,tx)<0);c.kp=0;c.dq=10000;assert(go1_pack(&c,tx)<0);c.dq=0;c.mode=0;assert(go1_pack(&c,tx)<0);
 {Fake f={0,0,0,0,1};Go1Transport io={&f,fake_reset,fake_tx,fake_rx,fake_clock};Go1MotorCmd cmd={0};cmd.id=2;assert(go1_send_recv(&io,&cmd,&d,100)==0&&d.q_raw==13909&&f.reset==1&&f.sent==1);f.respond=0;assert(go1_send_recv(&io,&cmd,&d,100)==-3&&!d.correct);}
 puts("real frame CRC, packing, stream recovery, transport and invalid inputs passed");return 0;}
