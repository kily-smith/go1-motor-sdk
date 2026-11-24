#ifndef GO1_TRANSPORT_H
#define GO1_TRANSPORT_H
#include "go1_motor.h"
/* Board supplies transaction callbacks. rx_reset MUST discard buffered old input.
 * tx must not return until last stop bit leaves UART and the RS485 driver is
 * returned to RX. rx_byte returns 1 for a byte, 0 on timeout, <0 on error.
 * millis must be monotonic modulo 2^32 and advance in these callbacks. */
typedef struct {
 void *context;
 int (*rx_reset)(void *context);
 int (*tx)(void *context,const uint8_t *data,size_t len,uint32_t timeout_ms);
 int (*rx_byte)(void *context,uint8_t *out,uint32_t timeout_ms);
 uint32_t (*millis)(void *context);
} Go1Transport;
/* 0 success; -1 invalid/pack, -2 I/O, -3 timeout. Stop on lost feedback is
   application responsibility; this function does not run a control loop. */
int go1_send_recv(Go1Transport *io,const Go1MotorCmd *cmd,Go1MotorData *data,uint32_t timeout_ms);
#endif
