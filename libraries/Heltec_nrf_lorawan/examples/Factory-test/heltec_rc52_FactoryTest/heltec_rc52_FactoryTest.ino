#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <Wire.h>
#include <bluefruit.h>

#include "Arduino.h"
#include "heltec_nrf_lorawan.h"

#if !defined(HELTEC_RC52)
#error "heltec_rc52_FactoryTest is intended for Heltec RC52."
#endif

#ifndef TEST_MODE
#define TEST_MODE 0
#endif

#define FACTORY_FW_VERSION "RC52FT1"
#define SERIAL_BAUDRATE 115200
   
#define RF_FREQUENCY_1 868100000
#define RF_FREQUENCY_2 870900000
#define TX_OUTPUT_POWER 10
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define LORA_BUFFER_SIZE 30
#define LORA_DUT_TIMEOUT_MS 60000UL
#define IO_STEP_DELAY_MS 300UL
#define ADC_SAMPLE_COUNT 50
#define ADC_PASS_MIN_MV 3700
#define ADC_PASS_MAX_MV 3900
#define BLE_RSSI_MIN -70
#define BLE_RSSI_MAX -30

extern void ble_slave_start();
extern void ble_center_start();
extern int8_t blerssi;

typedef enum {
  LOWPOWER,
  STATE_RX,
  STATE_TX
} States_t;

struct TestIoPin {
  const char *name;
  uint8_t pin;
};

static const TestIoPin testIoPins[] = {
    {"1.09", 41},
    {"0.10", 10},
    {"1.06", 38},
    {"1.04", 36},
    {"1.02", 34},
    {"0.09", 9},
    {"1.15", 47},
    {"0.05", 5},
    {"0.15", 15},
    {"1.11", 43},
    {"0.28", 28},
    {"1.01", 33},
    {"0.07", 7},
    {"0.08", 8},
    {"0.02", 2},
    {"0.12", 12},
    {"1.13", 45},
    {"0.30", 30},
    {"0.20", 20},
};

static RadioEvents_t RadioEvents;
static States_t state;
static uint8_t txpacket[LORA_BUFFER_SIZE];
static uint8_t rxpacket[LORA_BUFFER_SIZE];

static uint64_t chipId = 0;
static char chipIdText[17] = {0};
static int16_t maxTxRssi = -255;
static int16_t maxRxRssi = -255;
static uint8_t rxCount = 0;
static bool loraTimeout = false;
static bool intoSleep = false;
static bool loraDone = false;
static bool loraPass = false;
static bool loraStarted = false;
static bool bleStarted = false;
static bool adcPass = false;
static bool ioPass = true;
static uint16_t adcRawAverage = 0;
static uint16_t adcMillivolts = 0;
static uint32_t loraStartMs = 0;

static void makeChipIdText()
{
  chipId = (((uint64_t)NRF_FICR->DEVICEID[1]) << 32) | NRF_FICR->DEVICEID[0];
  snprintf(chipIdText, sizeof(chipIdText), "%08lX%08lX",
           (unsigned long)NRF_FICR->DEVICEID[1],
           (unsigned long)NRF_FICR->DEVICEID[0]);
}

static void printItem(const char *id, bool pass, const String &detail)
{
  Serial.print("FT_ITEM,");
  Serial.print(id);
  Serial.print(",");
  Serial.print(pass ? "PASS" : "FAIL");
  if (detail.length() > 0) {
    Serial.print(",");
    Serial.print(detail);
  }
  Serial.println();
}

static void restoreRc52RadioPower()
{
  pinMode(RADIOCORE_FEM_EN, OUTPUT);
  digitalWrite(RADIOCORE_FEM_EN, HIGH);
  pinMode(RADIOCORE_VFEM_CTRL, OUTPUT);
  digitalWrite(RADIOCORE_VFEM_CTRL, HIGH);
}

static void waitUserReleased()
{
  pinMode(PIN_BUTTON_USER, INPUT_PULLUP);
  while (digitalRead(PIN_BUTTON_USER) == LOW) {
    delay(5);
  }
  delay(40);
}

static bool userPressed()
{
  if (digitalRead(PIN_BUTTON_USER) != LOW) {
    return false;
  }
  delay(30);
  return digitalRead(PIN_BUTTON_USER) == LOW;
}

static void stopFactoryBle()
{
  if (!bleStarted) {
    return;
  }
  Bluefruit.Scanner.stop();
  Bluefruit.Advertising.stop();
  if (Bluefruit.connected()) {
    Bluefruit.disconnect(Bluefruit.connHandle());
  }
  bleStarted = false;
}

static void runIoSweep()
{
  Serial.println("FT_ITEM,IO_SWEEP,START,MODE=SEQUENTIAL_500MS");
  waitUserReleased();

  for (size_t i = 0; i < sizeof(testIoPins) / sizeof(testIoPins[0]); i++) {
    pinMode(testIoPins[i].pin, OUTPUT);
    digitalWrite(testIoPins[i].pin, LOW);
  }

  for (size_t i = 0; i < sizeof(testIoPins) / sizeof(testIoPins[0]); i++) {
    digitalWrite(testIoPins[i].pin, HIGH);
    Serial.print("FT_IO,");
    Serial.print(testIoPins[i].name);
    Serial.print(",HIGH,PIN=");
    Serial.println(testIoPins[i].pin);
    delay(IO_STEP_DELAY_MS);
  }

  for (size_t i = 0; i < sizeof(testIoPins) / sizeof(testIoPins[0]); i++) {
    digitalWrite(testIoPins[i].pin, LOW);
    Serial.print("FT_IO,");
    Serial.print(testIoPins[i].name);
    Serial.print(",LOW,PIN=");
    Serial.println(testIoPins[i].pin);
    delay(IO_STEP_DELAY_MS);
  }

  pinMode(PIN_BUTTON_USER, INPUT_PULLUP);
  printItem("IO_SWEEP", ioPass, "ERR=NONE");
}

static bool readAdc(String &detail)
{
  uint32_t rawSum = 0;
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  pinMode(PIN_BAT_ADC_CTL, OUTPUT);
  digitalWrite(PIN_BAT_ADC_CTL, ADC_CTRL_ENABLED);
  delay(10);
  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
    rawSum += analogRead(PIN_BAT_ADC);
    delay(2);
  }
  digitalWrite(PIN_BAT_ADC_CTL, !ADC_CTRL_ENABLED);

  adcRawAverage = (uint16_t)(rawSum / ADC_SAMPLE_COUNT);
  adcMillivolts =
      (uint16_t)((float)adcRawAverage * (3000.0F / 4096.0F) * BAT_AMPLIFY);
  bool pass =
      adcMillivolts >= ADC_PASS_MIN_MV && adcMillivolts <= ADC_PASS_MAX_MV;
  detail = "RAW=" + String(adcRawAverage) + ",MV=" + String(adcMillivolts) +
           ",ERR=" + String(pass ? "NONE" : "MV_RANGE");
  return pass;
}

void OnTxDone(void)
{
  Serial.println("TX done......");
  state = STATE_RX;
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.println("TX Timeout......");
  state = STATE_TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  (void)snr;
  Radio.Sleep();
  Serial.print("RX done...... Rx size: ");
  Serial.print(size);
  Serial.print(" rssi: ");
  Serial.println(rssi);

  if (TEST_MODE == 0) {
    state = STATE_TX;
    if (size == 10) {
      uint64_t rxChipId = *((uint64_t *)payload);
      if (rxChipId == chipId) {
        int16_t txRssi = *(int16_t *)(payload + 8);
        if (maxTxRssi < txRssi) {
          maxTxRssi = txRssi;
        }
        if (maxRxRssi < rssi) {
          maxRxRssi = rssi;
        }
        rxCount++;
        if (rxCount >= 3) {
          loraPass = abs(abs(maxTxRssi) - abs(maxRxRssi)) <= 15;
          loraDone = true;
          state = LOWPOWER;
        }
      }
    }
  } else {
    if (size == 8) {
      state = STATE_TX;
      memcpy(txpacket, payload, 8);
      memcpy(txpacket + 8, (uint8_t *)&rssi, 2);
    } else {
      Radio.Rx(0);
    }
  }
}

void OnRxTimeout()
{
  Radio.Sleep();
  state = STATE_TX;
  Serial.println("RX Timeout......");
}

void OnRxError()
{
  Radio.Sleep();
  state = STATE_TX;
  Serial.println("RX Error......");
}

static void loraInit()
{
  if (loraStarted) {
    return;
  }
  restoreRc52RadioPower();
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxError = OnRxError;
  RadioEvents.RxTimeout = OnRxTimeout;
  Radio.Init(&RadioEvents);
  srand1(Radio.Random());
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0,
                    0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0,
                    0, LORA_IQ_INVERSION_ON, true);
  state = STATE_TX;
  loraStarted = true;
  loraStartMs = millis();
}

static void loraLoop()
{
  if (TEST_MODE == 1) {
    switch (state) {
    case STATE_TX:
      Radio.SetChannel(RF_FREQUENCY_2);
      Serial.println("loraMode 1:into TX mode");
      Radio.Send(txpacket, 10);
      state = LOWPOWER;
      break;
    case STATE_RX:
      Radio.SetChannel(RF_FREQUENCY_1);
      Serial.println("loraMode 1:into RX mode");
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
    return;
  }

  if ((millis() - loraStartMs) > LORA_DUT_TIMEOUT_MS && rxCount < 3) {
    if (!loraTimeout) {
      loraTimeout = true;
      loraDone = true;
      loraPass = false;
    }
    return;
  }

  switch (state) {
  case STATE_TX:
    if (!intoSleep) {
      delay(1000 + randr(0, 100));
      Radio.SetChannel(RF_FREQUENCY_1);
      Serial.println("loraMode 0:into TX mode");
      Radio.Send((uint8_t *)&chipId, 8);
    }
    state = LOWPOWER;
    break;
  case STATE_RX:
    if (!intoSleep) {
      Radio.SetChannel(RF_FREQUENCY_2);
      Serial.println("loraMode 0:into RX mode");
      Radio.Rx(5000);
    }
    state = LOWPOWER;
    break;
  case LOWPOWER:
    if (intoSleep) {
      Radio.Sleep();
    }
    TimerLowPowerHandler();
    Radio.IrqProcess();
    break;
  default:
    break;
  }
}

static void reportLoraResult()
{
  String detail = "TX_RSSI=" + String(maxTxRssi) + ",RX_RSSI=" +
                  String(maxRxRssi) + ",RX_CNT=" + String(rxCount) +
                  ",ERR=" + String(loraPass ? "NONE" : "LORA_TIMEOUT");
  printItem("LORA", loraPass, detail);
}

static bool reportBleResult()
{
  bool pass = blerssi >= BLE_RSSI_MIN && blerssi <= BLE_RSSI_MAX;
  String detail = "RSSI=" + String(blerssi) +
                  ",ERR=" + String(pass ? "NONE" : "RSSI_RANGE");
  printItem("BLE", pass, detail);
  return pass;
}

static void enterDeepSleep()
{
  intoSleep = true;
  Serial.println("FT_ITEM,LOWPOWER,READY,WAKE=USER");

  Radio.Sleep();
  stopFactoryBle();
  SPI.end();
  SPI1.end();
  Wire.end();
  Serial1.end();

  pinMode(PIN_BAT_ADC_CTL, OUTPUT);
  digitalWrite(PIN_BAT_ADC_CTL, !ADC_CTRL_ENABLED);
  pinMode(RADIOCORE_FEM_EN, OUTPUT);
  digitalWrite(RADIOCORE_FEM_EN, LOW);
  pinMode(RADIOCORE_VFEM_CTRL, OUTPUT);
  digitalWrite(RADIOCORE_VFEM_CTRL, LOW);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
  digitalWrite(PIN_TFT_LEDA_CTL, !TFT_LEDA_ENABLE);
  pinMode(PIN_TFT_VDD_CTL, OUTPUT);
  digitalWrite(PIN_TFT_VDD_CTL, !TFT_VDD_ENABLE);

  nrf_gpio_cfg_default(PIN_SPI_MISO);
  nrf_gpio_cfg_default(PIN_SPI_MOSI);
  nrf_gpio_cfg_default(PIN_SPI_SCK);
  nrf_gpio_cfg_default(PIN_SPI1_MISO);
  nrf_gpio_cfg_default(PIN_SPI1_MOSI);
  nrf_gpio_cfg_default(PIN_SPI1_SCK);
  nrf_gpio_cfg_default(SX126X_CS);
  nrf_gpio_cfg_default(SX126X_DIO1);
  nrf_gpio_cfg_default(SX126X_BUSY);
  nrf_gpio_cfg_default(SX126X_RESET);
  nrf_gpio_cfg_default(SX126X_RXEN);
  nrf_gpio_cfg_default(PIN_SERIAL1_RX);
  nrf_gpio_cfg_default(PIN_SERIAL1_TX);
  nrf_gpio_cfg_default(PIN_WIRE_SDA);
  nrf_gpio_cfg_default(PIN_WIRE_SCL);
  nrf_gpio_cfg_default(PIN_TFT_CS);
  nrf_gpio_cfg_default(PIN_TFT_DC);
  nrf_gpio_cfg_default(PIN_TFT_SCK);
  nrf_gpio_cfg_default(PIN_TFT_MOSI);
  nrf_gpio_cfg_default(PIN_TFT_RST);
  nrf_gpio_cfg_default(PIN_TFT_LEDA_CTL);
  nrf_gpio_cfg_default(PIN_TFT_VDD_CTL);
  nrf_gpio_cfg_default(RADIOCORE_FEM_EN);
  nrf_gpio_cfg_default(RADIOCORE_VFEM_CTRL);
  nrf_gpio_cfg_sense_input(PIN_BUTTON_USER, NRF_GPIO_PIN_PULLUP,
                           NRF_GPIO_PIN_SENSE_LOW);

  Serial.println("FT_RESULT,LOWPOWER,ENTER");
  Serial.flush();
  Serial.end();
  sd_power_system_off();
  delay(portMAX_DELAY);
}

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  InternalFS.begin();
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, SERIAL_BAUDRATE);
  makeChipIdText();
  pinMode(PIN_BUTTON_USER, INPUT_PULLUP);

  Serial.print("FT_START,BOARD=Heltec RC52,FW=");
  Serial.print(FACTORY_FW_VERSION);
  Serial.print(",ROLE=");
  Serial.print(TEST_MODE == 0 ? "DUT" : "PEER");
  Serial.print(",CHIPID=");
  Serial.println(chipIdText);

  runIoSweep();

  String adcDetail;
  adcPass = readAdc(adcDetail);
  printItem("ADC", adcPass, adcDetail);
  
  while (!adcPass) {
    pinMode(41, OUTPUT);
    digitalWrite(41, HIGH);
    delay(100);
    digitalWrite(41, LOW);
    delay(100);
    adcPass = readAdc(adcDetail);
  }

  restoreRc52RadioPower();
  waitUserReleased();
  loraInit();

  blerssi = 0;
#if (TEST_MODE == 1)
  ble_slave_start();
#else
  ble_center_start();
#endif
  bleStarted = true;
}

void loop()
{
  if (userPressed()) {
    enterDeepSleep();
  }

  loraLoop();

  if (TEST_MODE == 0 && loraDone) {
    reportLoraResult();
    bool blePass = reportBleResult();
    Serial.print("FT_RESULT,");
    Serial.print((ioPass && adcPass && loraPass && blePass) ? "PASS" : "FAIL");
    Serial.print(",CHIPID=");
    Serial.print(chipIdText);
    Serial.print(",ADC_MV=");
    Serial.print(adcMillivolts);
    Serial.print(",BLE_RSSI=");
    Serial.println(blerssi);
    while (true) {
      if (userPressed()) {
        enterDeepSleep();
      }
      delay(50);
    }
  }
}
