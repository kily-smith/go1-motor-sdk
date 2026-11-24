#include "go1_motor.h"
#include <math.h>
#include <string.h>
#define PI_D 3.14159265358979323846
static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static uint32_t u32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static int32_t i32(const uint8_t *p) { uint32_t x=u32(p); return x<=INT32_MAX?(int32_t)x:(int32_t)((int64_t)x-4294967296LL); }
static int16_t i16(const uint8_t *p) { uint16_t x=u16(p); return x<=INT16_MAX?(int16_t)x:(int16_t)((int32_t)x-65536); }
static void put16(uint8_t *p,int32_t x) { uint16_t v=(uint16_t)x; p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p,uint32_t x) { p[0]=(uint8_t)x;p[1]=(uint8_t)(x>>8);p[2]=(uint8_t)(x>>16);p[3]=(uint8_t)(x>>24); }
uint32_t go1_crc32(const uint8_t *d,size_t n) { uint32_t c=0xffffffffu; size_t i; int bit; if (!d || n%4) return 0; for(i=0;i<n;i+=4) { c^=u32(d+i); for(bit=0;bit<32;bit++) c=(c<<1)^((c&0x80000000u)?0x04c11db7u:0); } return c; }
static int scaled(float v,double scale,double lo,double hi,int32_t *out) { double x=(double)v*scale; if(!isfinite(x)||x<lo||x>hi) return -1; *out=(int32_t)x; return 0; }
int go1_pack(const Go1MotorCmd *c,uint8_t out[GO1_TX_SIZE]) {
 int32_t t,w,q,kp,kd;
 if(!c||!out||c->id>14||(c->mode!=GO1_MODE_STOP&&c->mode!=GO1_MODE_FOC)) return -1;
 if(c->mode==GO1_MODE_STOP && (c->tau!=0||c->dq!=0||c->q!=0||c->kp!=0||c->kd!=0)) return -1;
 if(c->kp<0||c->kd<0||scaled(c->tau,256,-32768,32767,&t)||scaled(c->dq,128,-32768,32767,&w)||scaled(c->q,16384/(2*PI_D),-2147483648.0,2147483647.0,&q)||scaled(c->kp,2048,0,32767,&kp)||scaled(c->kd,1024,0,32767,&kd)) return -2;
 memset(out,0,GO1_TX_SIZE);out[0]=0xfe;out[1]=0xee;out[2]=c->id;out[3]=0xba;out[4]=c->mode;out[5]=0xff;out[24]=2;
 put16(out+12,t);put16(out+14,w);put32(out+16,(uint32_t)q);put16(out+20,kp);put16(out+22,kd);put32(out+30,go1_crc32(out,28));return 0;
}
int go1_unpack(const uint8_t *b,size_t n,uint8_t id,Go1MotorData *d) {
 uint32_t lw; float f;
 if(d) memset(d,0,sizeof(*d));
 if(!b||!d||id>14||n!=GO1_RX_SIZE||b[0]!=0xfe||b[1]!=0xee||b[2]!=id) return -1;
 if(go1_crc32(b,72)!=u32(b+74)) return -2;
 lw=u32(b+16);memcpy(&f,&lw,sizeof(f));
 d->motor_id=b[2];d->mode=b[4];d->temp=(int8_t)b[6];d->merror=b[7];d->tau_raw=i16(b+12);d->dq_raw=i16(b+14);d->q_raw=i32(b+30);
 d->tau=d->tau_raw/256.0f;d->dq=d->dq_raw/128.0f;d->q=(float)(d->q_raw*(2*PI_D/16384));d->LW=f;d->correct=1;return 0;
}
void go1_parser_init(Go1Parser *p) { if(p) memset(p,0,sizeof(*p)); }
int go1_parser_feed(Go1Parser *p,uint8_t b,uint8_t id,Go1MotorData *out) {
 if(!p||!out||id>14) return 0;
 if(p->used==0 && b!=0xfe) return 0;
 if(p->used==1 && b!=0xee) {p->used=(b==0xfe)?1:0;return 0;}
 p->buf[p->used++]=b;
 if(p->used<GO1_RX_SIZE) return 0;
 if(go1_unpack(p->buf,GO1_RX_SIZE,id,out)==0) {p->used=0;return 1;}
 /* Preserve overlapping headers after invalid candidates. */
 {size_t i;for(i=1;i<GO1_RX_SIZE;i++) if(p->buf[i]==0xfe && (i+1==GO1_RX_SIZE||p->buf[i+1]==0xee)) break; p->used=GO1_RX_SIZE-i;memmove(p->buf,p->buf+i,p->used);}
 return 0;
}
