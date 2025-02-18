/*
 * HelTec Automation(TM) WIFI_LoRa_32 factory test code, witch includ
 * follow functions:
 * 
 * - Basic OLED function test;
 * 
 * - Basic serial port test(in baud rate 115200);
 * 
 * - LED blink test;
 * 
 * - WIFI connect and scan test;
 * 
 * - LoRa Ping-Pong test (DIO0 -- GPIO26 interrup check the new incoming messages);
 * 
 * - Timer test and some other Arduino basic functions.
 *
 * by Aaron.Lee from HelTec AutoMation, ChengDu, China
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
*/

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include "Arduino.h"
#include "SPI.h" 
#include "heltec_nrf_lorawan.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
// #include "Adafruit_TinyUSB.h"
#include "TinyGPS.h"

#define TEST_MODE 0
#define BATTER_MIN 3800
#define BATTER_MAX 4100

#define HARD_VERSION 2
TinyGPSPlus GPS;

#define LED 35
#define LED_ON_VALUE LOW
extern void ble_slave_start();
extern void ble_center_start();
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1,PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

/********************************* lora  *********************************************/
#define RF_FREQUENCY_1                                868100000 // Hz
#define RF_FREQUENCY_2                                870900000 // Hz

#define TX_OUTPUT_POWER                             10        // dBm

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
uint64_t chipid;
uint8_t rx_cnt=0;
int test_mode=0;
bool loratimeout=false;
bool intosleep=false;
uint32_t rtc_cnt_1s,rtc_cnt_2s;
uint16_t volt;

static void st7789Start()
{
		pinMode(PIN_TFT_VDD_CTL,OUTPUT);
		pinMode(PIN_TFT_LEDA_CTL,OUTPUT);
		digitalWrite(PIN_TFT_VDD_CTL,TFT_VDD_ENABLE);  //set pin 3 low to enable tft vdd
    digitalWrite(PIN_TFT_LEDA_CTL,TFT_LEDA_ENABLE);//set pin 3 low to enable tft ledA
    tft.init(135, 240);                            // Init ST7789 240x135
    tft.setRotation(3);
    tft.setSPISpeed(40000000);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);//12*16
    tft.setTextWrap(false);
}

static void st7789Stop()
{
    SPI1.end();
    digitalWrite(PIN_TFT_CS,0);
    digitalWrite(PIN_TFT_DC,0);
    digitalWrite(PIN_TFT_RST,0);
		digitalWrite(PIN_TFT_VDD_CTL,!TFT_VDD_ENABLE);
		digitalWrite(PIN_TFT_LEDA_CTL,!TFT_LEDA_ENABLE);
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
  if(rtc_cnt_1s==rtc_cnt_2s&&rtc_cnt_1s!=524)
  {
    tft.setTextColor(ST77XX_RED);
    packet ="32K ERROR "+String(rtc_cnt_1s)+" "+String(rtc_cnt_2s);
    tft.setCursor(0,75);
    tft.println(packet);
    Serial1.printf("32K ERROR %d %d\r\n",rtc_cnt_1s,rtc_cnt_2s);
  }
  else
  {
    packet ="32K OK "+String(rtc_cnt_1s)+" "+String(rtc_cnt_2s);
    tft.setCursor(0,75);
    tft.println(packet);
    Serial1.printf("32K OK %d %d\r\n",rtc_cnt_1s,rtc_cnt_2s);
  }
}
extern int8_t blerssi;
void showStatus(int value)
{
  int n = 0,m = 0,c = 0;
  if(intosleep)
    return;
  debug_printf("LoRa %d %d\r\n",maxTxRssi,maxRxRssi);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  Serial1.printf("\r\n\r\n\r\n");
  if(value)
  {
    n = abs(maxTxRssi);
    m = abs(maxRxRssi);
    c = abs(n - m);
    // if(c > 10)
    // {
    //   tft.setTextColor(ST77XX_RED);
    //   if(c > 15)
    //   {
    //     Serial1.printf("\r\nLora test XX\r\n");
    //   }
    // }
    if(c > 15)
    {
      Serial1.printf("\r\nLora test XX\r\n");
      tft.setTextColor(ST77XX_RED);
    }
    packet ="Lora Rssi:"+String(maxTxRssi)+" "+String(maxRxRssi);
    tft.setCursor(0,25);
    tft.println(packet);
    Serial1.printf("Lora Rssi: %d %d\r\n",maxTxRssi,maxRxRssi);

    tft.setTextColor(ST77XX_GREEN);
    packet ="BLE Rssi:"+String(blerssi);
    tft.setCursor(0,50);
    tft.println(packet);
    Serial1.printf("BLE Rssi: %d\r\n",blerssi);
  }
  else
  {
    tft.setTextColor(ST77XX_RED);
    packet ="LoRa Error";
    tft.setCursor(0,37);
    tft.println(packet);
    Serial1.printf("LoRa Error\r\n");
  }
  uint32_t t=millis()/1000;
  tft.setTextColor(ST77XX_GREEN);
  packet ="t:"+String(t);
  tft.setCursor(0,0);
  tft.println(packet);
  Serial1.printf("t: %d\r\n",t);

  packet ="bat:"+String(volt);
  tft.setCursor(120,0);
  tft.println(packet);
  Serial1.printf("bat: %d\r\n",volt);

  show32kStatus();
  digitalWrite(LED, LED_ON_VALUE);  

  
  if(!size_error)
  {
    tft.setTextColor(ST77XX_GREEN);
    packet ="IO OK";
    tft.setCursor(0,100);
    tft.println(packet);
    debug_printf("%s\r\n",packet.c_str());
    Serial1.printf("IO OK\r\n");
  }
  else
  {
    tft.setTextColor(ST77XX_RED);
    packet ="IO Error:";
    Serial1.printf("IO Error:");
    for(int i=0;i<size_error;i++)
    {
      packet+=String(error_io[i]);
      Serial1.printf(" %d",error_io[i]);
      packet+=" ";
    }
    debug_printf("%s\r\n",packet.c_str());
    tft.setCursor(0,100);
    tft.println(packet);
    Serial1.printf("\r\n\r\n\r\n");
  }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  Radio.Sleep( );
  debug_printf("RX done......");
  debug_printf("Rx size : %d , rssi : %d, snr : %d\r\n",size,rssi,snr);
//  for(int i=0;i<size;i++)
//  {
//    debug_printf("%02X ",payload[i]);
//  }
//  debug_printf("\r\n");
  if(test_mode==0)
  {
    state=STATE_TX;
    if(size==10)
    {
      uint64_t rxchipid=*((uint64_t *)payload);
      if(rxchipid==chipid)
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



TaskHandle_t checkUserkey1kHandle = NULL;
void intodeepsleep()
{
  debug_printf("into deep sleep\r\n");
  if(gps_test_start)
  {
    gps_test_start=false;
    Serial2.end();
    digitalWrite(21,0);
    digitalWrite(38,0);
  }
  debug_printf("into deep sleep1\r\n");
  intosleep=true;
  digitalWrite(LED, !LED_ON_VALUE);  

  int adcin = 4;
  int adcvalue = 0;
  float mv_per_lsb = 3000.0F / 4096.0F;  // 10-bit ADC with 3.6V input range
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  pinMode(6, OUTPUT);
  digitalWrite(6, 1);
  delay(10);
  for(int i=0;i<50;i++)
  {
    adcvalue += analogRead(adcin);
  }
  digitalWrite(6, 0);
  volt = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9/50);
  tft.fillScreen(ST77XX_BLACK);
  if(volt<BATTER_MIN|volt>BATTER_MAX)
  {
    tft.setTextColor(ST77XX_RED);
    turnOnRGB(0x000020,0); //RGB_RED
  }
  else
  {
    tft.setTextColor(ST77XX_GREEN);
  }
  tft.setTextSize(2);
  packet ="battery:"+String(volt);
  tft.setCursor(0,60);
  tft.println(packet);
  delay(1000);
  if(volt<BATTER_MIN|volt>BATTER_MAX)
  {
    
  }
  else
  {
    st7789Stop();
  }
  intosleep=true;
  vTaskSuspend(checkUserkey1kHandle);
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
        tft.setTextSize(3);
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(30,60);
        tft.print("GPS START");
        Bluefruit.Scanner.stop();
        pinMode(35,ANALOG);
        gps_test_start=true;
        pinMode(LED ,OUTPUT);
	      digitalWrite(LED, !LED_ON_VALUE);  
        delay(1500);
        pinMode(LED ,OUTPUT);
        digitalWrite(LED, LED_ON_VALUE); 
        while(digitalRead(USERKEY)==0){delay(1);}
        // pinMode(38,OUTPUT);
        // digitalWrite(38,0);
        // delay(100);
        // digitalWrite(38,1);
      }
      else
      {
        Bluefruit.Scanner.stop();
        intodeepsleep();
      }
    }
  }
}


uint8_t io_h[]={3,15,18,24,25,42,9};
uint8_t io_l[]={6,17,20};
uint8_t test_io[]={32,33,16,13,28,30,29,31,45,47,5,46,44,7,8,34,36,37,38,39,9,10};

bool io_in_array(uint8_t pin,uint8_t * array,uint8_t size)
{
  for(int i=0;i<size;i++)
  {
    if(pin==array[i])
    {
      return true;
    }
  }
  return false;
}


void add_error_io(uint8_t pin)
{
  for(int i=0;i<size_error;i++)
  {
    if(error_io[i]==pin)
    {
      return;
    }
  }
  error_io[size_error]=pin;
  size_error++;
  // size_error+=1;
  io_error=true;
}

void checkGPIO()
{
  //reset lora;
  pinMode(25,OUTPUT);
  digitalWrite(25,0);
  delay(10);
  digitalWrite(25,1);
  // 3:1 tft ctl
  // 15:1 tft LEDA ctl
  // 18:1 RST
  // 24:1 lora cs
  // 25:1 lora rst
  // 42:1 user key
  
  uint8_t io_h_size=sizeof(io_h);
  for(int i=2;i<=47;i++)     //全部拉低
  {
    pinMode(i,INPUT_PULLDOWN);
  }
  delay(10);

  if(digitalRead(9)==0)
  {
    io_h_size-=1;
  }
  for(int i=2;i<=47;i++)
  {
    debug_printf("0 GPIO %d:%d\r\n",i,digitalRead(i));
  }
  for(int i=2;i<=47;i++)
  {
    if(io_in_array(i,io_h,io_h_size))  
    {
      if(digitalRead(i)!=1)
      {
        debug_printf("Error : IO %d Unexpected LOW\r\n",i);
        add_error_io((uint8_t)i);
      }
    }
    else
    {
      if(digitalRead(i)!=0)
      {
        debug_printf("Error : IO %d Unexpected HIGH\r\n",i);
        add_error_io((uint8_t)i);
      }
    }
  }

  for(int i=2;i<=47;i++)
  {
    if(io_in_array(i,io_h,io_h_size)==false) //本身非上拉状态的，拉高，再检测其他引脚是否被拉高，如果有表示引脚相连
    {
      pinMode(i,OUTPUT);
      digitalWrite(i,HIGH);
      delay(10);
      for(int j=2;j<=47;j++)
      {
        if(io_in_array(j,io_h,io_h_size)==false)
        {
          if(digitalRead(j)==1 && j!=i)
          {
            if(i==21&&j==14) //21:vext , 14 RGB.
            {
              continue;
            }
            debug_printf("%d GPIO %d:%d\r\n",i,j,digitalRead(j));
            debug_printf("Error: IO %d connected to IO %d\r\n",j,i);
            add_error_io((uint8_t)i);
            add_error_io((uint8_t)j);
          }
        }
      }
      digitalWrite(i,LOW);
      pinMode(i,INPUT_PULLDOWN);
    }
  }

  // 6:0 adc ctl
  // 17:0 lora busy
  // 20:0 lora dio1
   
  for(int i=2;i<=47;i++)
  {
    pinMode(i,INPUT_PULLUP);
  }
  delay(10);

  for(int i=2;i<=47;i++)
  {
    debug_printf("1 GPIO %d:%d\r\n",i,digitalRead(i));
  }
  for(int i=2;i<=47;i++)
  {
    if(io_in_array(i,io_l,sizeof(io_l)))
    {
      if(digitalRead(i)!=0)
      {
        debug_printf("Error : IO %d Unexpected HIGH\r\n",i);
        add_error_io((uint8_t)i);
      }
    }
    else
    {
      if(digitalRead(i)!=1)
      {
        debug_printf("Error : IO %d Unexpected LOW\r\n",i);
        add_error_io((uint8_t)i);
      }
    }
  }

  for(int i=2;i<=47;i++)
  {
    if(io_in_array(i,io_l,sizeof(io_l))==false) //本身非下拉状态的，拉低，再检测其他引脚是否被拉低，如果有表示引脚相连
    {
      pinMode(i,OUTPUT);
      digitalWrite(i,LOW);
      delay(10);
      for(int j=2;j<=47;j++)
      {
        if(io_in_array(j,io_l,sizeof(io_l))==false)
        {
          if(digitalRead(j)==0 && j!=i)
          {
            if(i==21&&j==14) //21:vext , 14 RGB.
            {
              continue;
            }
            debug_printf("%d GPIO %d:%d\r\n",i,j,digitalRead(j));
            debug_printf("Error: IO %d connected to IO %d\r\n",j,i);
            add_error_io((uint8_t)i);
            add_error_io((uint8_t)j);
          }
        }
      }
      digitalWrite(i,HIGH);
      pinMode(i,INPUT_PULLUP);
    }
  }

  for(int i=0;i<size_error;i++)
  {
    debug_printf("error io : %d\r\n",error_io[i]);
  }
}

uint32_t a,b;
void setup()
{
    // boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
  InternalFS.begin();
  checkGPIO();
  for(int i=0;i<sizeof(test_io);i++)
  {
    pinMode(test_io[i],OUTPUT);
    digitalWrite(test_io[i],0);
  }
  pinMode(21,OUTPUT);
  digitalWrite(21,1);
  // delay(1000);

  boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
  debug_printf("start\r\n");
  // uint8_t hard_ver=getHardwareVersion();
  // debug_printf("hardware version : V%d\r\n",hard_ver);
  // if(hard_ver==0)
  // {
  //   debug_printf("new device,set hardware version to %d\r\n",HARD_VERSION);
  //   setHardwareVersion(HARD_VERSION);
  //   hard_ver=getHardwareVersion();
  //   debug_printf("hardware version : V%d\r\n",hard_ver);
  // }
  for(int i=2;i<=47;i++)
  {
    pinMode(i,ANALOG);
  }

  for(int i=0;i<size_error;i++)
  {
    debug_printf("error io : %d\r\n",error_io[i]);
  }

  xTaskCreate(checkUserkey, "checkUserkey1Task", 2048, NULL, 1, &checkUserkey1kHandle);
  st7789Start();
  lora_init();

  if(!intosleep && !gps_test_start)
  {
    tft.fillScreen(ST77XX_RED);
    turnOnRGB(0x200000,0);
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
    turnOnRGB(0x000020,0);
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
    turnOnRGB(0x002000,0);
    delay(1000);
  }
  if(!intosleep && !gps_test_start)
  {
    tft.setTextSize(2);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_BLUE);
    tft.setCursor(0,60);
    tft.print("RF_FREQUENCY:HF");
    delay(1000);
  }
  
  turnOffRGB();
  if(!intosleep && !gps_test_start)
  {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_GREEN);
    packet ="LORA MODE "+String(test_mode);
    tft.setCursor(tft.width()/2-packet.length()/2*12, tft.height()/2-16/2);
    tft.println(packet);
    pinMode(LED ,OUTPUT);
    digitalWrite(LED, !LED_ON_VALUE);  
  }

  if(!intosleep && !gps_test_start)
  {
    int adcin = 4;
    int adcvalue = 0;
    float mv_per_lsb = 3000.0F / 4096.0F;  // 10-bit ADC with 3.6V input range
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    pinMode(6, OUTPUT);
    digitalWrite(6, 1);
    delay(10);
    for(int i=0;i<50;i++)
    {
      adcvalue += analogRead(adcin);
    }
    digitalWrite(6, 0);
    volt = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9/50);
  #if (TEST_MODE==1)
    ble_slave_start();
    test_mode=1;
  #else
    ble_center_start();
  #endif
  }
}

bool first_get_location=true;
uint32_t gps_get_time=0;

bool gps_started=false;
void loop()
{
  delay(1);
  if(gps_test_start)
  {
    if(gps_started==false)
    {
      gps_started=true;
      Serial2.begin(9600);
      pinMode(21,OUTPUT);
      digitalWrite(21,1);
      delay(10);
      pinMode(38,OUTPUT);
      digitalWrite(38,0);
      delay(100);
      digitalWrite(38,1);
      gps_start_time=millis();
    }
    uint32_t starttime = millis();
    while( (millis()-starttime) < 1000 )
    {
      while (Serial2.available() > 0)
      {
        uint8_t a=Serial2.read();
        // Serial.write(a);
        GPS.encode(a);
      }
    }

    tft.setTextSize(1);
    tft.fillScreen(ST77XX_BLACK);

    if(gps_get_time!=0)
    {
      if(gps_get_time<90)
      {
        tft.setTextColor(ST77XX_GREEN);
        turnOnRGB(0x002000,0);
      }
      else if(gps_get_time<180)
      {
        tft.setTextColor(ST77XX_YELLOW);
        turnOnRGB(0x202000,0);
      }
      else
      {
        tft.setTextColor(ST77XX_RED);
        turnOnRGB(0x200000,0);
      }
      tft.setTextSize(2);
      tft.setCursor(200,100);
      tft.println(String(gps_get_time));
      tft.setTextSize(1);
    }
    else
    {
      uint32_t t=millis()-gps_start_time;
      tft.setTextSize(2);
      tft.setCursor(200,100);
      tft.println(String(t/1000));
      tft.setTextSize(1);
    }

    if( GPS.location.age() < 1000 )
    {
      pinMode(LED,OUTPUT);
      digitalWrite(LED, LED_ON_VALUE);  
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(220,0);
      tft.println("A");
      if(first_get_location)
      {
          first_get_location=false;
          gps_get_time=(millis()-gps_start_time)/1000;
      }
    }
    else
    {
      tft.setCursor(220,0);
      tft.setTextColor(ST77XX_WHITE);
      tft.println("V");
      pinMode(LED,OUTPUT);
      digitalWrite(LED, !LED_ON_VALUE);  
    }

    
    char str[30];
    int index = sprintf(str,"%02d-%02d-%02d",GPS.date.year(),GPS.date.day(),GPS.date.month());
    str[index] = 0;
    tft.setCursor(0,0);
    tft.println(str);

    index = sprintf(str,"%02d:%02d:%02d",GPS.time.hour(),GPS.time.minute(),GPS.time.second(),GPS.time.centisecond());
    str[index] = 0;
    tft.setCursor(120,0);
    tft.println(str);


    index = sprintf(str,"lat :  %d.%d",(int)GPS.location.lat(),fracPart(GPS.location.lat(),4));
    str[index] = 0;
    tft.setCursor(0,20);
    tft.println(str);

    index = sprintf(str,"lon : %d.%d",(int)GPS.location.lng(),fracPart(GPS.location.lng(),4));
    str[index] = 0;
    tft.setCursor(0,40);
    tft.println(str);

    index = sprintf(str,"alt: %d.%d",(int)GPS.altitude.meters(),fracPart(GPS.altitude.meters(),2));
    str[index] = 0;
    tft.setCursor(0,60);
    tft.println(str);

    index = sprintf(str,"hdop: %d.%d",(int)GPS.hdop.hdop(),fracPart(GPS.hdop.hdop(),2));
    str[index] = 0;
    tft.setCursor(0,80);
    tft.println(str);

    index = sprintf(str,"speed: %d.%d km/h",(int)GPS.speed.kmph(),fracPart(GPS.speed.kmph(),3));
    str[index] = 0;
    tft.setCursor(0,100);
    tft.println(str);
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
          Radio.SetChannel( RF_FREQUENCY_1 );
          debug_printf("loraMode 0:into TX mode\r\n");
          Radio.Send( (uint8_t *)&chipid, 8 );
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
        if(intosleep)
        {
          Radio.Sleep();
        }
        TimerLowPowerHandler();
        Radio.IrqProcess();
        break;
      default:
        break;
    }
  }
}
