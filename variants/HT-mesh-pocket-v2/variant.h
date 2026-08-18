/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_HELTEC_MESH_POCKET_V2_
#define _VARIANT_HELTEC_MESH_POCKET_V2_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)
#define USE_LFXO

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HARD_VERSION_ADDR    (0xED000 + 7 * 4096 - 16 - 1)
#define HT_LICENSE_ADDR      (0xED000 + 7 * 4096 - 16)
#define HT_LICENSE_ADDR_BASE (0xED000 + 6 * 4096)

// Number of pins defined in PinDescription array
#define PINS_COUNT         (48)
#define NUM_DIGITAL_PINS   (48)
#define NUM_ANALOG_INPUTS  (1)
#define NUM_ANALOG_OUTPUTS (0)

// LEDs
#define PIN_LED1     (0 + 21) // There is no user LED; keep a compatibility alias only.
#define PIN_NEOPIXEL -1
#define NEOPIXEL_NUM 0
#define LED_BUILTIN  PIN_LED1
#define LED_RED      PIN_LED1
#define LED_BLUE     PIN_LED1
#define LED_GREEN    PIN_LED1
#define LED_STATE_ON 1

// Buttons
#define PIN_BUTTON1 (32 + 10)
#define PIN_BUTTON2 (32 + 11)
#define PIN_BUTTON_USER PIN_BUTTON2

#define ADC_RESOLUTION 14

/*
 * I2C
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA (0 + 13)
#define PIN_WIRE_SCL (0 + 5)

/*
 * TFT display pins
 */
#define NV3001B_CS           (0 + 16)
#define NV3001B_MOSI         (32 + 7)
#define NV3001B_RESET        (0 + 24)
#define NV3001B_RS           (32 + 0)
#define NV3001B_SCK          (32 + 1)
#define NV3001B_MISO         (-1)
#define NV3001B_PWR          (0 + 9)
#define NV3001B_PWR_ON       LOW
#define NV3001B_ROTATION     3
#define NV3001B_IPS          true
#define NV3001B_PANEL_WIDTH  128
#define NV3001B_PANEL_HEIGHT 220

#define TFT_BL           (0 + 25)
#define TFT_BL_ON        HIGH
#define TFT_BACKLIGHT_ON TFT_BL_ON

#define PIN_TFT_CS       NV3001B_CS
#define PIN_TFT_RST      NV3001B_RESET
#define PIN_TFT_DC       NV3001B_RS
#define PIN_TFT_MOSI     NV3001B_MOSI
#define PIN_TFT_MISO     NV3001B_MISO
#define PIN_TFT_SCK      NV3001B_SCK
#define PIN_TFT_VDD_CTL  NV3001B_PWR
#define TFT_VDD_ENABLE   NV3001B_PWR_ON
#define PIN_TFT_LEDA_CTL TFT_BL
#define TFT_LEDA_ENABLE  TFT_BL_ON

/*
 * LoRa radio
 */
#define USE_SX1262
#define SX126X_CS    (0 + 14)
#define LORA_CS      SX126X_CS
#define SX126X_DIO1  (0 + 31)
#define SX126X_BUSY  (0 + 29)
#define SX126X_RESET (0 + 11)
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

// nRF52840 has only one high-speed SPIM3 instance; assign it to SPI1 for TFT.
#define SPI_32MHZ_INTERFACE 1

// SPI for LoRa
#define PIN_SPI_MISO (0 + 2)
#define PIN_SPI_MOSI (32 + 15)
#define PIN_SPI_SCK  (32 + 13)

static const uint8_t SS   = SX126X_CS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI1 for TFT.
#define PIN_TFT_SPI_MISO_DUMMY (0 + 3)
#define PIN_SPI1_MISO PIN_TFT_SPI_MISO_DUMMY
#define PIN_SPI1_MOSI NV3001B_MOSI
#define PIN_SPI1_SCK  NV3001B_SCK

static const uint8_t SS1   = NV3001B_CS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

/*
 * GPS pins
 */
#define GPS_UC6580
#define GPS_BAUDRATE 115200
#define PIN_GPS_EN (0 + 22)
#define GPS_EN_ACTIVE LOW
#define PIN_GPS_RESET (32 + 6)
#define GPS_RESET_MODE LOW
#define PERIPHERAL_WARMUP_MS 1000
#define PIN_GPS_PPS (0 + 10)
#define GPS_RX_PIN   (32 + 2)
#define GPS_TX_PIN   (32 + 4)
#define GPS_THREAD_INTERVAL 50

#define PIN_SERIAL1_RX GPS_RX_PIN
#define PIN_SERIAL1_TX GPS_TX_PIN

#define PIN_9006_TX  (0 + 15)
#define PIN_9006_RX  (0 + 26)
#define PIN_9006_IO1 (0 + 4)
#define PIN_9006_IO2 (32 + 9)
#define PIN_CW_INT   (0 + 12)
#define PIN_IO_CSA (0 + 20)

// #define PIN_SERIAL2_RX PIN_9006_RX
// #define PIN_SERIAL2_TX PIN_9006_TX

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
