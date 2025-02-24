#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include "heltec_nrf_lorawan.h"
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

const int echoPin = 16;
const int trigPin = 13;
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

/* OTAA para*/
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };
uint8_t appSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };
uint32_t devAddr = (uint32_t)0x007e6ae4;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_US915;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = 1;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 1;

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse width of the echo signal
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (in cm)
  long distance = duration * 0.0343 / 2;

  // Print the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Ultrasonic Sensor");

  // Display the distance on the TFT screen
  tft.setCursor(10, 50);  // Set cursor for the distance text
  tft.fillRect(10, 50, 200, 40, ST77XX_BLACK);  // Clear previous data
  tft.print("Distance: ");
  tft.print(distance);
  tft.println(" cm");


  appDataSize = 0; 
  unsigned char*puc;
  appData[appDataSize++] = 0x00; // parent ID
  appData[appDataSize++] = 0x00; // parent ID
  appData[appDataSize++] = 0x05; // sensor length
  appData[appDataSize++] = 0x00; 

  puc = (unsigned char *)(&distance); 
  appData[appDataSize++] = puc[3];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  Serial.print("Sending data: ");
  for (int i = 0; i < appDataSize; i++) {
      Serial.print(appData[i], HEX);
      Serial.print(" ");
  }
  Serial.println();

        // Ensure that the total size does not exceed the maximum limit
        if (appDataSize > LORAWAN_APP_DATA_MAX_SIZE) {
            appDataSize = LORAWAN_APP_DATA_MAX_SIZE;
        }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
  debug_printf("start\r\n");

  pinMode(PIN_TFT_VDD_CTL, OUTPUT); // TFT VDD ENABLE
  digitalWrite(PIN_TFT_VDD_CTL, 0);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT); // LEDA ENABLE
  digitalWrite(PIN_TFT_LEDA_CTL, 0);

  Serial.print(F("Hello! ST77xx TFT Test"));

  tft.init(135, 240);  // Init ST7789 240x135
  tft.setRotation(3);  // Set screen rotation

  tft.setSPISpeed(40000000);  // SPI speed

  Serial.println(F("Initialized"));

  tft.fillScreen(ST77XX_BLACK);  // Clear the screen
  tft.setTextColor(ST77XX_WHITE);  // Set text color to white
  tft.setTextSize(2);  // Set text size
  tft.setCursor(50, 60);  // Set cursor position
  tft.print(F("Hello"));  // Print "Hello" on the screen
  delay(2000);
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
        LoRaWAN.init(loraWanClass, loraWanRegion);
        //both set join DR and DR when ADR off
        LoRaWAN.setDefaultDR(3);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}