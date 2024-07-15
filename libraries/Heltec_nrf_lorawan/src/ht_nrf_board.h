#ifndef __HT_NRF_BOARD_H__
#define __HT_NRF_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "HardwareSerial.h"
#include "Arduino.h"
#include "driver/board.h"
#include "driver/debug.h"
#include "driver/rtc-board.h"
#include "nrfx_gpiote.h"
#include "driver/lorawan_spi.h"
#include <InternalFileSystem.h>
#include <flash/flash_nrf5x.h>

uint8_t SpiInOut(Spi_t *obj, uint8_t outData );
void rtc_timer_start(uint32_t t);
void rtc_timer_stop();
void rtc_timer_init();
void TimerIrqHandler( void );
void wakefromsleep();
void boardInit(bool debug_enable,uint8_t serial_port,uint32_t baud);
uint64_t getID();


#ifdef __cplusplus
}
#endif

#endif
