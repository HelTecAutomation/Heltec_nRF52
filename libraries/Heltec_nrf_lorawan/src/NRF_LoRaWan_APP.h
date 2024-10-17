#ifndef LoRaWan_APP_H
#define LoRaWan_APP_H

#include <stdio.h>
#include "loramac/LoRaMac.h"
#include "loramac/utilities.h"
#include "./LoRaWan_102.h"
#include "HardwareSerial.h"
#include "Arduino.h"
#include "driver/board.h"
#include "driver/debug.h"
#include "driver/board-config.h"

#include "driver/rtc-board.h"
#include "nrfx_gpiote.h"
#include "driver/lorawan_spi.h"
#include "ht_nrf_board.h"


#include "loramac/region/RegionCN470.h"
#include "loramac/region/RegionIN865.h"
#include "loramac/region/RegionEU868.h"
#include "loramac/region/RegionUS915.h"
#include "loramac/region/RegionAU915.h"
#include "loramac/region/RegionKR920.h"
#include "loramac/region/RegionAS923.h"
#include "loramac/region/RegionEU433.h"
#include "loramac/region/Region.h"


enum eDeviceState_LoraWan
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
};

enum eDeviceState_Lora
{
    LORA_INIT,
    LORA_SEND,
    LORA_RECEIVE,
    LORA_CAD,
    MCU_SLEEP,
};


extern uint8_t devEui[];
extern uint8_t appEui[];
extern uint8_t appKey[];
extern uint8_t nwkSKey[];
extern uint8_t appSKey[];
extern uint32_t devAddr;
extern uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];
extern uint8_t appDataSize;
extern uint8_t appPort;
extern uint8_t power_for_noAdr;
extern uint32_t txDutyCycleTime;
extern bool overTheAirActivation;
extern LoRaMacRegion_t loraWanRegion;
extern bool loraWanAdr;
extern bool isTxConfirmed;
extern uint32_t appTxDutyCycle;
extern DeviceClass_t loraWanClass;
extern uint8_t confirmedNbTrials;
extern uint16_t userChannelsMask[6];

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

class LoRaWanClass{
public:
  void init(DeviceClass_t lorawanClass,LoRaMacRegion_t region);
  void join();
  void send();
  void cycle(uint32_t dutyCycle);
  void sleep(DeviceClass_t classMode);
  void setDefaultDR(int8_t dataRate);
  void ifskipjoin();
  void generateDeveuiByChipID();
};


extern const char * region_str[15];
extern const uint8_t region_array[15];
extern const uint8_t def_rx2dr[15];
extern const uint32_t def_rx2freq[15]; 
extern uint8_t rx2dr_type;
extern uint8_t rx2freq_type;
extern uint8_t region_index;

extern enum eDeviceState_LoraWan deviceState;

extern "C" bool SendFrame( void );
extern "C" void turnOnRGB(uint32_t color,uint32_t time);
extern "C" void turnOffRGB(void);
extern "C" bool checkUserAt(char * cmd, char * content);
extern "C" void downLinkAckHandle();
extern "C" void downLinkDataHandle(McpsIndication_t *mcpsIndication);
extern "C" void lwan_dev_params_update( void );
extern "C" void dev_time_updated( void );
extern "C" void print_Hex(uint8_t *para,uint8_t size);
extern "C" uint8_t getHardwareVersion();
extern "C" void setHardwareVersion(uint8_t hard_ver);
extern String channeltoString();

/*
extern "C" uint8_t SpiInOut(Spi_t *obj, uint8_t outData );
extern "C" void rtc_timer_start(uint32_t t);
extern "C" void rtc_timer_stop();
extern "C" void rtc_timer_init();
extern "C" void TimerIrqHandler( void );
extern "C" void wakefromsleep();
extern "C" void rtc_timer_process();
extern "C" void boardInit();
*/

extern LoRaWanClass LoRaWAN;

#endif
