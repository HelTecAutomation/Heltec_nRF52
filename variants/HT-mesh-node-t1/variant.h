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

#ifndef _VARIANT_HELTEC_MESH_NODE_T1_
#define _VARIANT_HELTEC_MESH_NODE_T1_

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
#define PIN_LED1     (0 + 16)
#define PIN_NEOPIXEL -1
#define NEOPIXEL_NUM 0
#define LED_BUILTIN  PIN_LED1
#define LED_RED      PIN_LED1
#define LED_BLUE     PIN_LED1
#define LED_GREEN    PIN_LED1
#define LED_STATE_ON 0

// Buttons
#define PIN_BUTTON1 (32 + 10)
#define PIN_BUTTON2 (0 + 14)

/*
 * Analog pins
 */
#define PIN_A0 (0 + 5)

static const uint8_t A0 = PIN_A0;
#define ADC_RESOLUTION 14

/*
 * I2C
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA (32 + 3)
#define PIN_WIRE_SCL (0 + 10)

#define PIN_SENSOR_EN        (32 + 6)
#define PIN_SENSOR_EN_ACTIVE LOW
// #define ICM_42607P_INT_PIN (32 + 1)
// #define ICM_42607P_INT2_PIN (32 + 7)

/*
 * TFT display
 */
#define ST7735_CS    (0 + 12)
#define ST7735_RS    (0 + 22)
#define ST7735_SDA   (0 + 24)
#define ST7735_SCK   (32 + 0)
#define ST7735_RESET (0 + 20)
#define ST7735_MISO  (0 + 25)
#define ST7735_BUSY  -1
#define ST7735_BL    (0 + 15)
#define VTFT_CTRL    (0 + 13)

#define SPI_FREQUENCY 80000000
#define SPI_READ_FREQUENCY 16000000
#define SCREEN_ROTATE
#define TFT_HEIGHT 160
#define TFT_WIDTH 80
#define TFT_OFFSET_X 24
#define TFT_OFFSET_Y 0
#define TFT_INVERT false
#define SCREEN_TRANSITION_FRAMERATE 3
#define DISPLAY_FORCE_SMALL_FONTS

#define PIN_TFT_CS       ST7735_CS
#define PIN_TFT_RST      ST7735_RESET
#define PIN_TFT_DC       ST7735_RS
#define PIN_TFT_VDD_CTL  VTFT_CTRL
#define TFT_VDD_ENABLE   0
#define PIN_TFT_LEDA_CTL ST7735_BL
#define TFT_LEDA_ENABLE  0

/*
 * LoRa radio
 */
#define USE_SX1262
#define SX126X_CS    (32 + 11)
#define LORA_CS      SX126X_CS
#define SX126X_DIO1  (0 + 31)
#define SX126X_BUSY  (0 + 29)
#define SX126X_RESET (0 + 2)
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

// nRF52840 has only one high-speed SPIM3 instance; assign it to SPI1 for TFT.
#define SPI_32MHZ_INTERFACE 1

// SPI for LoRa
#define PIN_SPI_MISO (0 + 3)
#define PIN_SPI_MOSI (32 + 14)
#define PIN_SPI_SCK  (32 + 13)

static const uint8_t SS   = SX126X_CS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI1 for TFT
#define PIN_SPI1_MISO ST7735_MISO
#define PIN_SPI1_MOSI ST7735_SDA
#define PIN_SPI1_SCK  ST7735_SCK

static const uint8_t SS1   = ST7735_CS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

/*
 * GPS pins
 */
#define GPS_UC6580
#define GPS_BAUDRATE 115200
#define PIN_GPS_RESET (0 + 26)
#define GPS_RESET_MODE LOW
#define PIN_GPS_EN (0 + 4)
#define GPS_EN_ACTIVE LOW
#define PERIPHERAL_WARMUP_MS 1000
#define PIN_GPS_PPS (32 + 9)
#define GPS_TX_PIN (0 + 7)
#define GPS_RX_PIN (0 + 8)
#define GPS_THREAD_INTERVAL 50

#define PIN_SERIAL1_RX GPS_RX_PIN
#define PIN_SERIAL1_TX GPS_TX_PIN

/*
 * Buzzer
 */
#define PIN_BUZZER (0 + 9)
#define PIN_BUZZER_VOLTAGE_MULTIPLIER_1 (32 + 2)
#define PIN_BUZZER_VOLTAGE_MULTIPLIER_2 (32 + 5)

/*
 * Battery
 */
#define ADC_CTRL 11
#define ADC_CTRL_ENABLED HIGH
#define BATTERY_PIN 5
#define PIN_BAT_ADC BATTERY_PIN
#define PIN_BAT_ADC_CTL ADC_CTRL
#define BAT_AMPLIFY 4.916F


#define HAS_RTC 0

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
