#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include "Arduino.h"
#include "SPI.h" 
#include "heltec_nrf_lorawan.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
// #include "Adafruit_TinyUSB.h"
#include "TinyGPS.h"
#include "gps_test.h"
#define TEST_MODE 0
#define BATTER_MIN 3800
#define BATTER_MAX 4100

#define HARD_VERSION 2
TinyGPSPlus GPS;
#define VGNSS_CTRL PIN_GPS_EN
extern void ble_slave_start();
extern void ble_center_start();
Adafruit_ST7735 tft = Adafruit_ST7735(&SPI1,PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);



#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER1                            28      // 20 dBm
#define TX_TIMEOUT                                  10        // seconds (MAX value)
/********************************* lora  *********************************************/
#define RF_FREQUENCY_1                                868100000 // Hz
#define RF_FREQUENCY_2                                870900000 // Hz

#define TX_OUTPUT_POWER                             28        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

#define USERKEY 42


uint8_t txpacket[BUFFER_SIZE];
uint8_t rxpacket[BUFFER_SIZE];


static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

uint8_t error_io[50];
int size_error=0;
bool io_error=false;
bool gps_test_start=false;
uint8_t error_count = 0;

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

States_t state;
int16_t maxTxRssi=-255;
int16_t maxRxRssi=-255;

String packSize = "--";
String packet;

bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
uint64_t chipId = 0;
char chipIdText[17] = {0};
uint8_t rx_cnt=0;
int test_mode=0;
bool loratimeout=false;
volatile bool intosleep=false; // Short-press request, consumed by loop().
uint32_t rtc_cnt_1s,rtc_cnt_2s;
uint16_t volt;

void OnRadioTxTimeout( void )
{
  Radio.SetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER1, TX_TIMEOUT );
}

static void st7735Start()
{
	pinMode(PIN_TFT_VDD_CTL,OUTPUT);
	pinMode(PIN_TFT_LEDA_CTL,OUTPUT);
	digitalWrite(PIN_TFT_VDD_CTL,TFT_VDD_ENABLE);  
  digitalWrite(PIN_TFT_LEDA_CTL,TFT_LEDA_ENABLE);
  delay(1000);
	tft.initR(INITR_MINI160x80_PLUGIN);
	tft.setRotation(1);
	tft.setSPISpeed(40000000);
	tft.fillScreen(ST7735_BLACK);
	tft.setTextSize(2);
	tft.setTextWrap(false);
  tft.invertDisplay(true);
}

static void st7735Stop()
{
    SPI1.end();
    digitalWrite(PIN_TFT_CS,0);
    digitalWrite(PIN_TFT_DC,0);
    digitalWrite(PIN_TFT_RST,0);
		digitalWrite(PIN_TFT_VDD_CTL,!TFT_VDD_ENABLE);
		digitalWrite(PIN_TFT_LEDA_CTL,!TFT_LEDA_ENABLE);
    // Do not drive signals into the unpowered display.
    pinMode(PIN_TFT_CS,ANALOG);
    pinMode(PIN_TFT_DC,ANALOG);
    pinMode(PIN_TFT_RST,ANALOG);
    pinMode(PIN_SPI1_MOSI,ANALOG);
    pinMode(PIN_SPI1_SCK,ANALOG);
}

static void makeChipIdText() {
  chipId = (((uint64_t)NRF_FICR->DEVICEID[1]) << 32) | NRF_FICR->DEVICEID[0];
  snprintf(chipIdText, sizeof(chipIdText), "%08lX%08lX",
           (unsigned long)NRF_FICR->DEVICEID[1],
           (unsigned long)NRF_FICR->DEVICEID[0]);
}

void OnTxDone( void )
{
	debug_printf("TX done......\r\n");
	state=STATE_RX;
}

void OnTxTimeout( void )
{
  Radio.Sleep( );
  debug_printf("TX Timeout......\r\n");
	state=STATE_TX;
}

void show32kStatus()
{
  if(rtc_cnt_1s==rtc_cnt_2s&&rtc_cnt_1s<500)
  {
    tft.setTextColor(ST77XX_RED);
    packet ="32K ERROR "+String(rtc_cnt_1s)+" "+String(rtc_cnt_2s);
    tft.setCursor(0,45);
    tft.println(packet);
    debug_printf("32K ERROR %d %d\r\n",rtc_cnt_1s,rtc_cnt_2s);
    error_count = 1;
  }
  else
  {
    packet ="32K OK "+String(rtc_cnt_1s)+" "+String(rtc_cnt_2s);
    tft.setCursor(0,45);
    tft.println(packet);
    debug_printf("32K OK %d %d\r\n",rtc_cnt_1s,rtc_cnt_2s);
  }
}
extern int8_t blerssi;
void showStatus(int value)
{
  int n = 0,m = 0,c = 0;
  if(intosleep)
  return;
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(1);
  delay(100);
  debug_printf("ShowStatus:\r\n");
 
  uint32_t t=millis()/1000;
  tft.setTextColor(ST77XX_GREEN);
  packet ="t:"+String(t);
  tft.setCursor(0,0);
  tft.println(packet);

  packet ="bat:"+String(volt);
  tft.setCursor(90,0);
  tft.println(packet);
  debug_printf("t: %d               bat: %d \r\n",t,volt);
 
  if(value)
  {
    n = abs(maxTxRssi);
    m = abs(maxRxRssi);
    c = abs(n - m);
    if(c > 15)
    {
      debug_printf("Lora Rssi: %d %d XX\r\n",maxTxRssi,maxRxRssi);
      tft.setTextColor(ST77XX_RED);
      error_count = 2;
    }

    packet ="Lora Rssi:"+String(maxTxRssi)+" "+String(maxRxRssi);
    tft.setCursor(0,15);
    tft.println(packet);
    debug_printf("Lora Rssi: %d %d OK\r\n",maxTxRssi,maxRxRssi);

    tft.setTextColor(ST77XX_GREEN);
    packet ="BLE Rssi:"+String(blerssi);
    tft.setCursor(0,30);
    tft.println(packet);
    debug_printf("BLE Rssi: %d\r\n",blerssi);
  }
  else
  {
    tft.setTextColor(ST77XX_GREEN);
    packet ="LoRa 3";
    tft.setCursor(0,30);
    tft.println(packet);
    debug_printf("LoRa Error\r\n");
    error_count = 3;
  }
  
  show32kStatus();
  delay(200);
  if(error_count == 0)
  {
    debug_printf("Test result:OK\r\n");
    debug_printf("Test OK!\r\n");
    pinMode(LED,OUTPUT);
    digitalWrite(LED, HIGH);  
  }
  else
  {
    debug_printf("Test result:Failed\r\n");
    debug_printf("Test Error!\r\n");
    pinMode(LED,OUTPUT);
    while(!intosleep)
    {
      digitalWrite(LED, HIGH); 
      delay(500);
      digitalWrite(LED, LOW); 
      delay(500);
    }
  }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  Radio.Sleep( );
  debug_printf("RX done......");
  debug_printf("Rx size : %d , rssi : %d, snr : %d\r\n",size,rssi,snr);

  if(test_mode==0)
  {
    state=STATE_TX;
    if(size==10)
    {
      uint64_t rxchipid=*((uint64_t *)payload);
      if(rxchipid==chipId)
      {
        int16_t txrssi=*(int16_t *)(payload+8);
        debug_printf("rx own mes,rssi :%d\r\n",txrssi);
        if(maxTxRssi<txrssi)
          maxTxRssi=txrssi;
        if(maxRxRssi<rssi)
          maxRxRssi=rssi;
        rx_cnt++;
        if(rx_cnt>=3)
        {
          showStatus(1);
          state=LOWPOWER;
        }
      }
    }
  }
  else
  {
    if(size==8)
    {
      state=STATE_TX;
      memcpy(txpacket,payload,8);
      memcpy(txpacket+8,(uint8_t *)&rssi,2);
    }
    else
    {
      Radio.Rx(0);
    }
  }
}

void OnRxTimeout()
{
  Radio.Sleep();
  state=STATE_TX;
  debug_printf("RX Timeout......\r\n");
}

void OnRxError()
{
  Radio.Sleep();
  state=STATE_TX;
  debug_printf("RX Error......\r\n");
}

void lora_init(void)
{
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxError = OnRxError;
  RadioEvents.RxTimeout = OnRxTimeout;
  Radio.Init( &RadioEvents );
  srand1( Radio.Random( ) );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
	state=STATE_TX;
}



uint8_t IO_test[27]={31,29,2,45,10,9,22,43,20,17,15,13,33,32,24,25,23,36,34,38,39,37,35,4,27,8,7};
TaskHandle_t checkUserkey1kHandle = NULL;
void intodeepsleep()
{
  // Only loop() closes peripherals; the key task no longer accesses them.
  vTaskSuspend(checkUserkey1kHandle);
  uint8_t softDeviceEnabled = 0;
  sd_softdevice_is_enabled(&softDeviceEnabled);
  if(softDeviceEnabled)
  {
    Bluefruit.Scanner.stop();
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    Bluefruit.autoConnLed(false);
  }
  Radio.Sleep();
  gps_test_start=false;
  Serial2.end();
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL,!GPS_EN_ACTIVE);
  int adcin = 3;
  int adcvalue = 0;
  float mv_per_lsb = 3000.0F / 4096.0F;  // 10-bit ADC with 3.6V input range
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  pinMode(47, OUTPUT);
  digitalWrite(47, 1);
  delay(10);
  for(int i=0;i<50;i++)
  {
    adcvalue += analogRead(adcin);
  }
  digitalWrite(47, 0);
  volt = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9/50);
  tft.fillScreen(ST77XX_BLACK);
  if(volt<BATTER_MIN|volt>BATTER_MAX)
  {
    tft.setTextColor(ST77XX_RED);
  }
  else
  {
    tft.setTextColor(ST77XX_GREEN);
  }
  tft.setTextSize(2);
  packet ="battery:"+String(volt);
  tft.setCursor(0,30);
  tft.println(packet);
  delay(1000);

  if(volt<BATTER_MIN || volt>BATTER_MAX)
  {
    debug_printf("battery voltage out of range: %u mV; system off cancelled\r\n", (unsigned)volt);
    // Keep the red result until reset; returning would retry via intosleep.
    while(1)
    {
      delay(portMAX_DELAY);
    }
  }

  debug_printf("into system off\r\n");
  st7735Stop();
  // R4/R9 pull these controls to Vext; release them after switching it off.
  pinMode(PIN_TFT_LEDA_CTL,ANALOG);
  pinMode(VGNSS_CTRL,ANALOG);
  SPI.end();
  pinMode(LORA_PA_POWER, OUTPUT);
  digitalWrite(LORA_PA_POWER,LOW);
  pinMode(LORA_PA_CSD,ANALOG);
  pinMode(LORA_PA_CTX,ANALOG);
  pinMode(RADIO_DIO_1,ANALOG);
  // Keep chip-select and reset inactive so the radio stays asleep.
  digitalWrite(RADIO_NSS,HIGH);
  pinMode(RADIO_NSS,OUTPUT);
  digitalWrite(RADIO_RESET,HIGH);
  pinMode(RADIO_RESET,OUTPUT);
  pinMode(RADIO_BUSY,ANALOG);
  pinMode(PIN_SPI_MISO,ANALOG);
  pinMode(PIN_SPI_MOSI,ANALOG);
  pinMode(PIN_SPI_SCK,ANALOG);
  pinMode(PIN_GPS_RESET,ANALOG);
  pinMode(PIN_GPS_PPS,ANALOG);
  pinMode(PIN_SERIAL2_RX,ANALOG);
  pinMode(PIN_SERIAL2_TX,ANALOG);
  pinMode(PIN_BAT_ADC_CTL, OUTPUT);
  digitalWrite(PIN_BAT_ADC_CTL,LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,!LED_STATE_ON);
  pinMode(LED,ANALOG); // A queued BLE blink must not drive the LED again.
  pinMode(35,ANALOG); // BLE test result output.
  pinMode(USERKEY,ANALOG); // No button wakeup is configured.
  Serial1.end();
  pinMode(PIN_SERIAL1_RX,ANALOG);
  pinMode(PIN_SERIAL1_TX,ANALOG);
  // Release all fixture signals, including SDA/SCL, after stopping their buses.
  for(uint8_t i=0;i<sizeof(IO_test)/sizeof(IO_test[0]);i++)
  {
    pinMode(IO_test[i],ANALOG);
  }
  Serial.end();

  if(softDeviceEnabled)
  {
    sd_power_system_off();
  }
  else
  {
    NRF_POWER->SYSTEMOFF = 1;
  }
  // System OFF does not return; never resume tests if it returns under debug.
  while(1)
  {
    delay(portMAX_DELAY);
  }
}

uint32_t gps_start_time=0;
void checkUserkey(void *pvParameters)
{
  uint32_t keydowntime;
  pinMode(USERKEY,INPUT);
  while(1)
  {
    delay(1);
    if(digitalRead(USERKEY)==0)
    {
      keydowntime=millis();
      debug_printf("key down : %u\r\n",keydowntime);
      delay(10);
      while(digitalRead(USERKEY)==0){
        delay(1);
        if( (millis()-keydowntime)>1000 )
        {
          break;
        }
      }
      if( (millis()-keydowntime)>1000 )
      {
        debug_printf("GPS START");
        tft.setTextSize(2);
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(20,30);
        tft.print("GPS START");
        Bluefruit.Scanner.stop();
        pinMode(35,ANALOG);
        gps_test_start=true;
        pinMode(LED ,OUTPUT);
	      digitalWrite(LED, HIGH); 
        delay(500);
        digitalWrite(LED, LOW); 
        while(digitalRead(USERKEY)==0){delay(1);}
      }
      else
      {
        intosleep=true;
        // Retry until loop() takes over, including a request racing with
        // TimerLowPowerHandler() just before it suspends the loop task.
        while(1)
        {
          resumeLoop();
          delay(1);
        }
      }
    }
  }
}


void IO_OUTPUT_LEDON_TEST()
{
  uint8_t e=0;
  for(e=0;e<27;e++)
  {
    pinMode(IO_test[e] ,OUTPUT);
    digitalWrite(IO_test[e], HIGH); 
    delay(100);
  }
}
void IO_OUTPUT_LEDOFF_FAST_TEST()
{
  uint8_t e=0;
  for(e=26;e<27;e--)
  {
    pinMode(IO_test[e] ,OUTPUT);
    digitalWrite(IO_test[e], LOW); 
    delay(5);
  }
}
void IO_OUTPUT_LEDOFF_SLOW_TEST()
{
  uint8_t e=0;
  for(e=26;e<27;e--)
  {
    pinMode(IO_test[e] ,OUTPUT);
    digitalWrite(IO_test[e], LOW); 
    delay(50);
  }
}
void IO_TEST()
{
  IO_OUTPUT_LEDOFF_FAST_TEST();
  IO_OUTPUT_LEDON_TEST();
  delay(1000);
  IO_OUTPUT_LEDOFF_SLOW_TEST();
}

uint32_t read8(uint8_t bits, uint8_t dummy, uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t dc, uint8_t rst)
{
    uint32_t ret = 0;
    uint8_t SDAPIN = mosi;
    pinMode(SDAPIN, INPUT_PULLUP);
    digitalWrite(dc, HIGH);
    for (int i = 0; i < dummy; i++) { // any dummy clocks
        digitalWrite(sck, HIGH);
        delay(1);
        digitalWrite(sck, LOW);
        delay(1);
    }
    for (int i = 0; i < bits; i++) { // read results
        ret <<= 1;
        delay(1);
        if (digitalRead(SDAPIN))
            ret |= 1;
        ;
        digitalWrite(sck, HIGH);
        delay(1);
        digitalWrite(sck, LOW);
    }
    return ret;
}

void write9(uint8_t val, uint8_t dc_val, uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t dc, uint8_t rst)
{
    pinMode(mosi, OUTPUT);
    digitalWrite(dc, dc_val);
    for (int i = 0; i < 8; i++) { // send command
        digitalWrite(mosi, (val & 0x80) != 0);
        delay(1);
        digitalWrite(sck, HIGH);
        delay(1);
        digitalWrite(sck, LOW);
        val <<= 1;
    }
}

uint32_t readwrite8(uint8_t cmd, uint8_t bits, uint8_t dummy, uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t dc, uint8_t rst)
{
    digitalWrite(cs, LOW);
    write9(cmd, 0, cs, sck, mosi, dc, rst);
    uint32_t ret = read8(bits, dummy, cs, sck, mosi, dc, rst);
    digitalWrite(cs, HIGH);
    return ret;
}

uint32_t get_st7735_id(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t dc, uint8_t rst)  
{
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    pinMode(cs, OUTPUT);
    pinMode(sck, OUTPUT);
    pinMode(mosi, OUTPUT);
    pinMode(dc, OUTPUT);
    pinMode(rst, OUTPUT);
    digitalWrite(rst, LOW); // Hardware Reset
    delay(10);
    digitalWrite(rst, HIGH);
    delay(10);

    uint32_t ID = 0;
    ID = readwrite8(0x04, 24, 1, cs, sck, mosi, dc, rst);
    ID = readwrite8(0x04, 24, 1, cs, sck, mosi, dc, rst); // ST7789 needs twice
    return ID;
}

uint32_t a,b;
uint32_t lcd_id;
uint8_t sp_a = TX_OUTPUT_POWER1,sp_b = 0;
void setup()
{
  Serial.begin(115200);
  InternalFS.begin();
  pinMode(26, OUTPUT);
  digitalWrite(26,1);
  boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
  chipId=getID();
  delay(500);
  Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipId>>32));//print High 2 bytes
  Serial.printf("%08X\r\n",(uint32_t)chipId);//print Low 4bytes.
  Serial.printf("start\r\n");
  if(!intosleep && !gps_test_start)
  {
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    Radio.Init( &RadioEvents );
    for(sp_b = 0;sp_b < 3;sp_b++)
    {
      Radio.IrqProcess( );
      Radio.SetTxContinuousWave( RF_FREQUENCY, sp_a, TX_TIMEOUT );
      sp_a += 2;
      if(sp_a > 32)
      {
        sp_a = TX_OUTPUT_POWER1;
      }
      delay(250);
      Radio.Sleep();
      delay(250);
    } 
  }
  xTaskCreate(checkUserkey, "checkUserkey1Task", 2048, NULL, 1, &checkUserkey1kHandle);
  lcd_id = get_st7735_id(22,20,17,15,13);
  st7735Start();
  if(lcd_id == 0xFFFFFF)
  {
    IO_TEST();
  }
  lora_init();
  if(!intosleep && !gps_test_start)
  {
    tft.fillScreen(ST77XX_RED);
    delayMicroseconds(500000);
    a=NRF_RTC1->COUNTER;
    delayMicroseconds(500000);
    b=NRF_RTC1->COUNTER;
    rtc_cnt_1s=b-a;
    debug_printf("%d\r\n",rtc_cnt_1s);
  }
  if(!intosleep && !gps_test_start)
  {
    tft.fillScreen(ST77XX_BLUE);
    NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;
    NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
    NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
    delayMicroseconds(500000);
    a=NRF_RTC1->COUNTER;
    delayMicroseconds(500000);
    b=NRF_RTC1->COUNTER;
    rtc_cnt_2s=b-a;
    debug_printf("%d\r\n",rtc_cnt_2s);
  }
  if(!intosleep && !gps_test_start)
  {
    tft.fillScreen(ST77XX_GREEN);
    delay(500);
  }
  if(!intosleep && !gps_test_start)
  {
    tft.setTextSize(2);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_BLUE);
    tft.setCursor(0,30);
    tft.print("RF_FR:868");
    delay(1000);
  }
  
  if(!intosleep && !gps_test_start)
  {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_GREEN);
    packet ="LORA MODE "+String(test_mode);
    delay(100);
    debug_printf("LORA MODE %d\r\n",test_mode);
    tft.setCursor(10, 30);
    tft.println(packet);
  }

  if(!intosleep && !gps_test_start)
  {
    int adcin = 3;
    int adcvalue = 0;
    float mv_per_lsb = 3000.0F / 4096.0F;  // 10-bit ADC with 3.6V input range
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    pinMode(47, OUTPUT);
    digitalWrite(47, 1);
    delay(10);
    for(int i=0;i<50;i++)
    {
      adcvalue += analogRead(adcin);
    }
    digitalWrite(47, 0);
    volt = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9/50);
  #if (TEST_MODE==1)
    ble_slave_start();
    test_mode=1;
  #else
    ble_center_start();
  #endif
  }
}

uint32_t gps_get_time=0;

bool gps_started=false;
void loop()
{
  delay(1);
  if(intosleep)
  {
    intodeepsleep();
    return;
  }
  if(gps_test_start)
  {
    if(gps_started==false)
    {
      gps_started=true;
      Serial2.begin(115200);
      pinMode(26,OUTPUT);
      digitalWrite(26,1);  //pin26  Vext_3V3
      delay(10);
      pinMode(6,OUTPUT);  //pin6   VGNSS_CTRL
      digitalWrite(6,0);
      pinMode(46,OUTPUT); 
      digitalWrite(46,0); //pin46  GNSS_RST
      delay(100);
      digitalWrite(46,1);
      initGsvCaptors();
      gps_start_time=millis();
    }
    gps_test();
    return;
  }

  
  if(test_mode)
  {
    switch(state)
    {
      case STATE_TX:
        Radio.SetChannel( RF_FREQUENCY_2 );
        debug_printf("loraMode 1:into TX mode\r\n");
        Radio.Send( txpacket, 10 );
        state=LOWPOWER;
        break;
      case STATE_RX:
        Radio.SetChannel( RF_FREQUENCY_1 );
        debug_printf("loraMode 1:into RX mode\r\n");
        Radio.Rx( 0 );
        state=LOWPOWER;
        break;
      case LOWPOWER:
        TimerLowPowerHandler();
        Radio.IrqProcess();
        break;
      default:
        break;
    }
  }
  else
  {
    if(millis()>60000 && rx_cnt<3)
    {
      if(loratimeout==false)
      {
        loratimeout=true;
        showStatus(0);
      }
      return;
    }
    switch(state)
    {
      case STATE_TX:
        if(!intosleep)
        {
          delay(1000+randr(0,100));
          if(intosleep)
          {
            break;
          }
          Radio.SetChannel( RF_FREQUENCY_1 );
          debug_printf("loraMode 0:into TX mode\r\n");
          Radio.Send( (uint8_t *)&chipId, 8 );
        }
        state=LOWPOWER;
        break;
      case STATE_RX:
        if(!intosleep)
        {
          Radio.SetChannel( RF_FREQUENCY_2 );
          debug_printf("loraMode 0:into RX mode\r\n");
          Radio.Rx( 1000 );
        }
        state=LOWPOWER;
        break;
      case LOWPOWER:
        TimerLowPowerHandler();
        Radio.IrqProcess();
        break;
      default:
        break;
    }
  }
}
