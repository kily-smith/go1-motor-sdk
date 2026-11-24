/* Porting template, not compiled until you select an STM32 HAL family and board.
 * Define GO1_STM32_HAL_HEADER e.g. "stm32f4xx_hal.h" and provide DE GPIO.
 * 4.8 Mbps UART feasibility depends on the UART clock and hardware. */
#ifdef GO1_STM32_HAL_HEADER
#include GO1_STM32_HAL_HEADER
#include "go1_transport.h"
typedef struct { UART_HandleTypeDef *uart; GPIO_TypeDef *de_port; uint16_t de_pin; } Go1Hal;
static uint32_t tick(void *ctx) { (void)ctx; return HAL_GetTick(); }
static int reset_rx(void *ctx) {
 Go1Hal *h=(Go1Hal*)ctx; uint8_t byte; uint32_t start=HAL_GetTick();
 /* Bounded draining; board-specific UART error/overrun clearing may be needed. */
 while((HAL_GetTick()-start)<5u) {
  HAL_StatusTypeDef s=HAL_UART_Receive(h->uart,&byte,1,1);
  if(s==HAL_TIMEOUT) return 0;
  if(s!=HAL_OK) return -1;
 }
 return -1;
}
static int tx(void *ctx,const uint8_t *p,size_t n,uint32_t timeout) {
 Go1Hal *h=(Go1Hal*)ctx; HAL_StatusTypeDef result;
 HAL_GPIO_WritePin(h->de_port,h->de_pin,GPIO_PIN_SET);
 result=HAL_UART_Transmit(h->uart,(uint8_t*)p,(uint16_t)n,timeout);
 /* HAL_UART_Transmit waits for TC in standard blocking HAL implementations;
    confirm this on the selected family before switching DE. */
 HAL_GPIO_WritePin(h->de_port,h->de_pin,GPIO_PIN_RESET);
 return result==HAL_OK?0:-1;
}
static int rx(void *ctx,uint8_t *p,uint32_t timeout) {
 Go1Hal *h=(Go1Hal*)ctx;HAL_StatusTypeDef s=HAL_UART_Receive(h->uart,p,1,timeout);
 return s==HAL_OK?1:(s==HAL_TIMEOUT?0:-1);
}
Go1Transport go1_hal_transport(Go1Hal *h) { Go1Transport io={h,reset_rx,tx,rx,tick};return io; }
#endif
