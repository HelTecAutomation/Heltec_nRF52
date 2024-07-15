/* Heltec Automation Ping Pong communication test example
 *
 * Function:
 * 1. Ping Pong communication in two esp32 device.
 * 
 * Description:
 * 1. Only hardware layer communicate, no LoRaWAN protocol support;
 * 2. Download the same code into two esp32 devices, then they will begin Ping Pong test each other;
 * 3. This example is for esp32 hardware basic test.
 *
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 */

#include "Arduino.h"
#include "heltec_nrf_lorawan.h"


#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             10        // 20 dBm
#define TX_TIMEOUT                                  10        // seconds (MAX value)
static RadioEvents_t RadioEvents;

void OnRadioTxTimeout( void )
{
    // Restarts continuous wave transmission when timeout expires
    Radio.SetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
}
void setup() {
  // put your setup code here, to run once:
  boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
  RadioEvents.TxTimeout = OnRadioTxTimeout;
  Radio.Init( &RadioEvents );

  Radio.SetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
}

void loop() {
  // put your main code here, to run repeatedly:
    TimerLowPowerHandler();
    Radio.IrqProcess();
}