 #include <Adafruit_LittleFS.h>
#include <Arduino_GFX_Library.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <Wire.h>
#include <bluefruit.h>
#include "Arduino.h"
#include "TinyGPS.h"
#include "heltec_nrf_lorawan.h"

#if !defined(HELTEC_MESH_POCKET_V2)
#error "Mesh_Pocket_V2_V1.0 is intended for Mesh Pocket V2 (HT-mesh-pocket-v2)."
#endif

#ifndef RGB565_RED
#define RGB565_RED 0xF800
#endif
#ifndef RGB565_GREEN
#define RGB565_GREEN 0x07E0
#endif
#ifndef RGB565_BLUE
#define RGB565_BLUE 0x001F
#endif
#ifndef RGB565_BLACK
#define RGB565_BLACK 0x0000
#endif
#ifndef RGB565_WHITE
#define RGB565_WHITE 0xFFFF
#endif
#ifndef RGB565_YELLOW
#define RGB565_YELLOW 0xFFE0
#endif

#define ST7735_BLACK RGB565_BLACK
#define ST7735_BLUE RGB565_BLUE
#define ST7735_GREEN RGB565_GREEN
#define ST7735_RED RGB565_RED
#define ST7735_WHITE RGB565_WHITE
#define ST7735_YELLOW RGB565_YELLOW

/********************************* lora *********************************************/
#define RF_FREQUENCY_1 868100000 // Hz
#define RF_FREQUENCY_2 870900000 // Hz

#define TX_OUTPUT_POWER 10 // dBm

#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 30

#define USERKEY PIN_BUTTON2
#define KEY PIN_BUTTON1
#define SERIAL_BAUDRATE 115200
#define BLE_Rssi_MIN -70
#define BLE_Rssi_MAX -30

#define HARD_VERSION 2
TinyGPSPlus GPS;

extern void ble_slave_start();
extern void ble_center_start();
extern int8_t blerssi;

Arduino_DataBus *bus = new Arduino_HWSPI(PIN_TFT_DC, PIN_TFT_CS, &SPI1, true);
Arduino_NV3001B st7735(bus, PIN_TFT_RST, NV3001B_ROTATION, NV3001B_IPS,
                       NV3001B_PANEL_WIDTH, NV3001B_PANEL_HEIGHT, 0, 0, 0, 0);

uint8_t txpacket[BUFFER_SIZE];
uint8_t rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

int size_error = 0;
bool gps_test_start = false;
uint8_t error_count = 0;

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
} States_t;

States_t state;
int16_t maxTxRssi = -255;
int16_t maxRxRssi = -255;

String packSize = "--";
String packet;

bool receiveflag = false;
uint64_t chipId = 0;
char chipIdText[17] = {0};
uint8_t rx_cnt = 0;
int test_mode = 0;
bool loratimeout = false;
bool intosleep = false;
uint32_t rtc_cnt_1s, rtc_cnt_2s;
uint16_t volt;
uint8_t Buzzer_state = false;

static void makeChipIdText()
{
  chipId = (((uint64_t)NRF_FICR->DEVICEID[1]) << 32) | NRF_FICR->DEVICEID[0];
  snprintf(chipIdText, sizeof(chipIdText), "%08lX%08lX",
           (unsigned long)NRF_FICR->DEVICEID[1],
           (unsigned long)NRF_FICR->DEVICEID[0]);
}

static void initDisplay()
{
  pinMode(PIN_TFT_VDD_CTL, OUTPUT);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
  digitalWrite(PIN_TFT_VDD_CTL, TFT_VDD_ENABLE);
  delay(100);
  pinMode(PIN_IO_CSA, OUTPUT);
  digitalWrite(PIN_IO_CSA, HIGH);
  delay(100);
  digitalWrite(PIN_TFT_LEDA_CTL, TFT_LEDA_ENABLE);
  delay(100);

  if (!st7735.begin(1000000))
  {
    Serial.println("st7735.begin() failed");
  }
  st7735.setRotation(NV3001B_ROTATION);
  st7735.setTextWrap(false);
  st7735.setTextSize(1);
  st7735.fillScreen(ST7735_BLACK);
}


static void waitForButton1Test()
{
  pinMode(KEY, INPUT_PULLUP);

  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextSize(2);
  st7735.setTextColor(ST7735_WHITE);
  st7735.setCursor(10, 18);
  st7735.println("Button1");
  st7735.setCursor(10, 46);
  st7735.println("Test");
  st7735.setTextSize(1);
  st7735.setTextColor(ST7735_GREEN);
  st7735.setCursor(10, 82);
  st7735.println("Press Button1");
  st7735.setCursor(10, 96);
  st7735.println("to continue");
  debug_printf("Button1 test: wait press\r\n");

  while (digitalRead(KEY) != LOW)
  {
    delay(1);
  }
  delay(20);
  while (digitalRead(KEY) == LOW)
  {
    delay(1);
  }

  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextSize(2);
  st7735.setTextColor(ST7735_GREEN);
  st7735.setCursor(10, 40);
  st7735.println("Button1 OK");
  debug_printf("Button1 test: OK\r\n");
  delay(500);
}

void OnTxDone(void)
{
  debug_printf("TX done......\r\n");
  state = STATE_RX;
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  debug_printf("TX Timeout......\r\n");
  state = STATE_TX;
}

void showStatus(int value)
{
  int n = 0, m = 0, c = 0;
  if (intosleep)
    return;
  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextColor(ST7735_GREEN);
  st7735.setTextSize(1);
  delay(100);
  debug_printf("ShowStatus:\r\n");

  uint32_t t = millis() / 1000;
  st7735.setTextColor(ST7735_GREEN);
  packet = "t:" + String(t);
  st7735.setCursor(0, 0);
  st7735.println(packet);

  packet = "bat:NA";
  st7735.setCursor(90, 0);
  st7735.println(packet);
  debug_printf("t: %d          bat: NA \r\n", t);

  if (value)
  {
    n = abs(maxTxRssi);
    m = abs(maxRxRssi);
    c = abs(n - m);
    if (c > 15)
    {
      debug_printf("Lora Rssi: %d %d XX\r\n", maxTxRssi, maxRxRssi);
      st7735.setTextColor(ST7735_RED);
      error_count = 2;
    }
    else
    {
      st7735.setTextColor(ST7735_GREEN);
    }
    packet = "Lora Rssi:" + String(maxTxRssi) + " " + String(maxRxRssi);
    st7735.setCursor(0, 20);
    st7735.println(packet);
    debug_printf("Lora Rssi: %d %d OK\r\n", maxTxRssi, maxRxRssi);

    packet = "BLE Rssi:" + String(blerssi);
    debug_printf("BLE Rssi: %d\r\n", blerssi);
    if (blerssi > BLE_Rssi_MAX || blerssi < BLE_Rssi_MIN)
    {
      st7735.setTextColor(ST7735_RED);
      error_count = 3;
    }
    else
    {
      st7735.setTextColor(ST7735_GREEN);
    }
    st7735.setCursor(0, 35);
    st7735.println(packet);
  }
  else
  {
    st7735.setTextColor(ST7735_RED);
    packet = "LoRa Error";
    st7735.setCursor(0, 20);
    st7735.println(packet);
    debug_printf("LoRa Error\r\n");
    error_count = 4;
  }
  delay(200);
  if (error_count == 0)
  {
    debug_printf("Test result:OK\r\n");
    debug_printf("Test OK!\r\n");
    packet = "Test Results: Pass";
    st7735.setTextColor(ST7735_GREEN);
    st7735.setCursor(0, 65);
    st7735.println(packet);
    while (1)
    {
      delay(50);
    }
  }
  else
  {
    debug_printf("Test result:Failed\r\n");
    debug_printf("Test Error!\r\n");
    packet = "Test Results: Fail";
    st7735.setTextColor(ST7735_RED);
    st7735.setCursor(0, 65);
    st7735.println(packet);
    while (1)
    {
      delay(500);
    }
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Radio.Sleep();
  debug_printf("RX done......");
  debug_printf("Rx size : %d , rssi : %d, snr : %d\r\n", size, rssi, snr);

  if (test_mode == 0)
  {
    state = STATE_TX;
    if (size == 10)
    {
      uint64_t rxchipid = *((uint64_t *)payload);
      if (rxchipid == chipId)
      {
        int16_t txrssi = *(int16_t *)(payload + 8);
        debug_printf("rx own mes,rssi :%d\r\n", txrssi);
        if (maxTxRssi < txrssi)
          maxTxRssi = txrssi;
        if (maxRxRssi < rssi)
          maxRxRssi = rssi;
        rx_cnt++;
        if (rx_cnt >= 3)
        {
          showStatus(1);
          state = LOWPOWER;
        }
      }
    }
  }
  else
  {
    if (size == 8)
    {
      state = STATE_TX;
      memcpy(txpacket, payload, 8);
      memcpy(txpacket + 8, (uint8_t *)&rssi, 2);
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
  state = STATE_TX;
  debug_printf("RX Timeout......\r\n");
}

void OnRxError()
{
  Radio.Sleep();
  state = STATE_TX;
  debug_printf("RX Error......\r\n");
}

void lora_init(void)
{
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxError = OnRxError;
  RadioEvents.RxTimeout = OnRxTimeout;
  Radio.Init(&RadioEvents);
  srand1(Radio.Random());
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  state = STATE_TX;
}

TaskHandle_t checkUserkey1kHandle = NULL;
void intodeepsleep()
{
  debug_printf("into deep sleep\r\n");
  volt = 0;
  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextColor(ST7735_GREEN);
  st7735.setTextSize(1);
  packet = "battery:NA";
  st7735.setCursor(0, 30);
  st7735.println(packet);

  Radio.Sleep();
  SPI.end();
  SPI1.end();
  Wire.end();
  Serial1.end();

  pinMode(PIN_GPS_EN, OUTPUT);
  digitalWrite(PIN_GPS_EN, !GPS_EN_ACTIVE);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
  digitalWrite(PIN_TFT_LEDA_CTL, !TFT_LEDA_ENABLE);
  pinMode(PIN_TFT_VDD_CTL, OUTPUT);
  digitalWrite(PIN_TFT_VDD_CTL, !TFT_VDD_ENABLE);
  pinMode(PIN_IO_CSA, OUTPUT);
  digitalWrite(PIN_IO_CSA, LOW);

  delay(100);

  nrf_gpio_cfg_default(PIN_BUTTON1);
  nrf_gpio_cfg_default(PIN_BUTTON2);
  nrf_gpio_cfg_default(PIN_SPI_MISO);
  nrf_gpio_cfg_default(PIN_SPI_MOSI);
  nrf_gpio_cfg_default(PIN_SPI_SCK);
  nrf_gpio_cfg_default(PIN_SPI1_MISO);
  nrf_gpio_cfg_default(PIN_SPI1_MOSI);
  nrf_gpio_cfg_default(PIN_SPI1_SCK);
  nrf_gpio_cfg_default(PIN_WIRE_SDA);
  nrf_gpio_cfg_default(PIN_WIRE_SCL);
  nrf_gpio_cfg_default(PIN_GPS_PPS);
  nrf_gpio_cfg_default(PIN_GPS_RESET);
  nrf_gpio_cfg_default(GPS_TX_PIN);
  nrf_gpio_cfg_default(GPS_RX_PIN);
  nrf_gpio_cfg_default(PIN_TFT_CS);
  nrf_gpio_cfg_default(PIN_TFT_DC);
  nrf_gpio_cfg_default(PIN_TFT_MOSI);
#if defined(PIN_TFT_MISO) && (PIN_TFT_MISO >= 0)
  nrf_gpio_cfg_default(PIN_TFT_MISO);
#endif
  nrf_gpio_cfg_default(PIN_TFT_SCK);
  nrf_gpio_cfg_default(PIN_TFT_RST);
  nrf_gpio_cfg_default(PIN_TFT_LEDA_CTL);
  nrf_gpio_cfg_default(PIN_TFT_VDD_CTL);
  nrf_gpio_cfg_default(SX126X_CS);
  nrf_gpio_cfg_default(SX126X_DIO1);
  nrf_gpio_cfg_default(SX126X_BUSY);
  nrf_gpio_cfg_default(SX126X_RESET);

  // nrf_gpio_cfg_default(PIN_IO_CSA);

  Serial.flush();
  Serial.end();
  sd_power_system_off();

  intosleep = true;
  vTaskSuspend(checkUserkey1kHandle);
}

uint32_t gps_start_time = 0;
void checkUserkey(void *pvParameters)
{
  uint32_t keydowntime;
  pinMode(USERKEY, INPUT);
  while (1)
  {
    delay(1);
    if (digitalRead(USERKEY) == 0)
    {
      keydowntime = millis();
      debug_printf("key down : %u\r\n", keydowntime);
      delay(10);
      while (digitalRead(USERKEY) == 0)
      {
        delay(1);
        if ((millis() - keydowntime) > 1000)
        {
          break;
        }
      }
      if ((millis() - keydowntime) > 1000)
      {
        debug_printf("GPS START");
        st7735.setTextSize(2);
        st7735.fillScreen(ST7735_BLACK);
        st7735.setTextColor(ST7735_WHITE);
        st7735.setCursor(25, 30);
        st7735.print("GPS START");
        Bluefruit.Scanner.stop();
        gps_test_start = true;
        delay(1500);
        while (digitalRead(USERKEY) == 0)
        {
          delay(1);
        }
      }
      else
      {
        Bluefruit.Scanner.stop();
        intodeepsleep();
      }
    }
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  InternalFS.begin();
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, 115200);
  makeChipIdText();
  Serial.print(",CHIPID=");
  Serial.println(chipIdText);
  debug_printf("start\r\n");
  initDisplay();
  waitForButton1Test();
  xTaskCreate(checkUserkey, "checkUserkey1Task", 2048, NULL, 1, &checkUserkey1kHandle);
  lora_init();

  if (!intosleep && !gps_test_start)
  {
    st7735.fillScreen(ST7735_RED);
    delay(500);
  }
  if (!intosleep && !gps_test_start)
  {
    st7735.fillScreen(ST7735_BLUE);
    delay(500);
  }

  if (!intosleep && !gps_test_start)
  {
    st7735.setTextSize(2);
    st7735.fillScreen(ST7735_BLACK);
    st7735.setTextColor(ST7735_BLUE);
    st7735.setCursor(5, 30);
    st7735.print("FREQUENCY:HF");
    delay(1000);
  }

  if (!intosleep && !gps_test_start)
  {
    st7735.fillScreen(ST7735_BLACK);
    st7735.setTextColor(ST7735_GREEN);
    packet = "LORA MODE " + String(test_mode);
    delay(100);
    debug_printf("LORA MODE %d\r\n", test_mode);
    st7735.setCursor(st7735.width() / 2 - packet.length() / 2 * 12, st7735.height() / 2 - 16 / 2);
    st7735.println(packet);
  }

  if (!intosleep && !gps_test_start)
  {
    volt = 0;
#if (TEST_MODE == 1)
    ble_slave_start();
    test_mode = 1;
#else
    ble_center_start();
#endif
  }
}

bool first_get_location = true;
uint32_t gps_get_time = 0;

bool gps_started = false;
void loop()
{
  delay(1);
  if (gps_test_start)
  {
    if (gps_started == false)
    {
      gps_started = true;
      pinMode(PIN_GPS_EN, OUTPUT);
      digitalWrite(PIN_GPS_EN, GPS_EN_ACTIVE);
      pinMode(PIN_GPS_RESET, OUTPUT);
      digitalWrite(PIN_GPS_RESET, GPS_RESET_MODE);
      delay(50);
      digitalWrite(PIN_GPS_RESET, !GPS_RESET_MODE);
      delay(PERIPHERAL_WARMUP_MS);
      Serial1.setPins(PIN_SERIAL1_RX, PIN_SERIAL1_TX);
      Serial1.begin(GPS_BAUDRATE);
      gps_start_time = millis();
    }
    uint32_t starttime = millis();
    while ((millis() - starttime) < 1000)
    {
      while (Serial1.available() > 0)
      {
        uint8_t a = Serial1.read();
        GPS.encode(a);
      }
    }

    st7735.setTextSize(1);
    st7735.fillScreen(ST7735_BLACK);

    if (gps_get_time != 0)
    {
      if (gps_get_time < 90)
      {
        st7735.setTextColor(ST7735_GREEN);
      }
      else if (gps_get_time < 180)
      {
        st7735.setTextColor(ST7735_YELLOW);
      }
      else
      {
        st7735.setTextColor(ST7735_RED);
      }
      st7735.setTextSize(1);
      st7735.setCursor(120, 66);
      st7735.println(String(gps_get_time));
      st7735.setTextSize(1);
    }
    else
    {
      uint32_t t = millis() - gps_start_time;
      st7735.setTextSize(1);
      st7735.setCursor(120, 66);
      st7735.println(String(t / 1000));
      st7735.setTextSize(1);
    }

    if (GPS.location.age() < 1000)
    {
      st7735.setTextColor(ST7735_GREEN);
      st7735.setCursor(150, 0);
      st7735.println("A");
      if (first_get_location)
      {
        first_get_location = false;
        gps_get_time = (millis() - gps_start_time) / 1000;
      }
    }
    else
    {
      st7735.setCursor(150, 0);
      st7735.setTextColor(ST7735_WHITE);
      st7735.println("V");
    }

    char str[30];
    int index = sprintf(str, "%02d-%02d-%02d", GPS.date.year(), GPS.date.day(), GPS.date.month());
    str[index] = 0;
    st7735.setCursor(0, 0);
    st7735.println(str);

    index = sprintf(str, "%02d:%02d:%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second(), GPS.time.centisecond());
    str[index] = 0;
    st7735.setCursor(80, 0);
    st7735.println(str);

    index = sprintf(str, "lat :  %d.%d", (int)GPS.location.lat(), fracPart(GPS.location.lat(), 4));
    str[index] = 0;
    st7735.setCursor(0, 18);
    st7735.println(str);

    index = sprintf(str, "lon : %d.%d", (int)GPS.location.lng(), fracPart(GPS.location.lng(), 4));
    str[index] = 0;
    st7735.setCursor(0, 30);
    st7735.println(str);

    index = sprintf(str, "alt: %d.%d", (int)GPS.altitude.meters(), fracPart(GPS.altitude.meters(), 2));
    str[index] = 0;
    st7735.setCursor(0, 42);
    st7735.println(str);

    index = sprintf(str, "hdop: %d.%d", (int)GPS.hdop.hdop(), fracPart(GPS.hdop.hdop(), 2));
    str[index] = 0;
    st7735.setCursor(0, 54);
    st7735.println(str);

    index = sprintf(str, "speed: %d.%d km/h", (int)GPS.speed.kmph(), fracPart(GPS.speed.kmph(), 3));
    str[index] = 0;
    st7735.setCursor(0, 66);
    st7735.println(str);
    return;
  }

  if (test_mode)
  {
    switch (state)
    {
    case STATE_TX:
      Radio.SetChannel(RF_FREQUENCY_2);
      debug_printf("loraMode 1:into TX mode\r\n");
      Radio.Send(txpacket, 10);
      state = LOWPOWER;
      break;
    case STATE_RX:
      Radio.SetChannel(RF_FREQUENCY_1);
      debug_printf("loraMode 1:into RX mode\r\n");
      Radio.Rx(0);
      state = LOWPOWER;
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
    if (millis() > 60000 && rx_cnt < 3)
    {
      if (loratimeout == false)
      {
        loratimeout = true;
        showStatus(0);
      }
      return;
    }
    switch (state)
    {
    case STATE_TX:
      if (!intosleep)
      {
        delay(1000 + randr(0, 100));
        Radio.SetChannel(RF_FREQUENCY_1);
        debug_printf("loraMode 0:into TX mode\r\n");
        Radio.Send((uint8_t *)&chipId, 8);
      }
      state = LOWPOWER;
      break;
    case STATE_RX:
      if (!intosleep)
      {
        Radio.SetChannel(RF_FREQUENCY_2);
        debug_printf("loraMode 0:into RX mode\r\n");
        Radio.Rx(5000);
      }
      state = LOWPOWER;
      break;
    case LOWPOWER:
      if (intosleep)
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
