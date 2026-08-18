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

#ifndef _VARIANT_HELTEC_RC52_
#define _VARIANT_HELTEC_RC52_

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
#define PIN_LED1     -1
#define PIN_NEOPIXEL -1
#define NEOPIXEL_NUM 0
#define LED_BUILTIN  PIN_LED1
#define LED_BLUE     PIN_LED1
#define LED_STATE_ON 1

// Buttons
#define PIN_BUTTON1 (32 + 10) // P1.10, external pull-up, active low
#define PIN_BUTTON_USER PIN_BUTTON1

/*
 * Analog pins
 */
#define PIN_A0 (0 + 31)

static const uint8_t A0 = PIN_A0;
#define ADC_RESOLUTION 14

/*
 * I2C
 */
#define WIRE_INTERFACES_COUNT 1
#define I2C_NO_RESCAN

#define PIN_WIRE_SDA (0 + 6)
#define PIN_WIRE_SCL (0 + 29)

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX (0 + 7) // Header UART RX
#define PIN_SERIAL1_TX (0 + 8) // Header UART TX

/*
 * TFT display connector pins
 */
#define RADIOCORE_TFT_SCK  (0 + 30)
#define RADIOCORE_TFT_MOSI (32 + 3)
#define RADIOCORE_TFT_CS   (32 + 5)
#define RADIOCORE_TFT_DC   (0 + 28)
#define RADIOCORE_TFT_RST  (0 + 10)
#define RADIOCORE_TFT_EN   (32 + 13)
#define RADIOCORE_TFT_BL   (0 + 9)

#define TFT_EN_ON  LOW
#define TFT_EN_OFF HIGH
#define TFT_BL_ON  HIGH
#define TFT_BL_OFF LOW

#define PIN_TFT_CS       RADIOCORE_TFT_CS
#define PIN_TFT_RST      RADIOCORE_TFT_RST
#define PIN_TFT_DC       RADIOCORE_TFT_DC
#define PIN_TFT_MOSI     RADIOCORE_TFT_MOSI
#define PIN_TFT_MISO     -1
#define PIN_TFT_SCK      RADIOCORE_TFT_SCK
#define PIN_TFT_VDD_CTL  RADIOCORE_TFT_EN
#define TFT_VDD_ENABLE   TFT_EN_ON
#define PIN_TFT_LEDA_CTL RADIOCORE_TFT_BL
#define TFT_LEDA_ENABLE  TFT_BL_ON

/*
 * LoRa radio
 */
#define USE_SX1262
#define SX126X_CS    (0 + 13)
#define LORA_CS      SX126X_CS
#define SX126X_DIO1  (0 + 11)
#define SX126X_BUSY  (0 + 24)
#define SX126X_RESET (32 + 0)
#define SX126X_RXEN  (32 + 7) // HT-RA62A LNA_Ctrl
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

#define RADIOCORE_FEM_EN    (0 + 26) // HT-RA62A FEM_EN
#define RADIOCORE_VFEM_CTRL (0 + 16) // FEM regulator enable

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

// nRF52840 has only one high-speed SPIM3 instance; assign it to SPI1 for TFT.
#define SPI_32MHZ_INTERFACE 1

// SPI for LoRa
#define PIN_SPI_MISO (0 + 14)
#define PIN_SPI_MOSI (0 + 22)
#define PIN_SPI_SCK  (0 + 25)

static const uint8_t SS   = SX126X_CS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI1 for TFT. Display MISO is not wired; SPIClass still needs a valid pin.
#define PIN_SPI1_MISO (0 + 12)
#define PIN_SPI1_MOSI RADIOCORE_TFT_MOSI
#define PIN_SPI1_SCK  RADIOCORE_TFT_SCK

static const uint8_t SS1   = RADIOCORE_TFT_CS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

/*
 * Battery
 */
#define ADC_CTRL (0 + 4)
#define ADC_CTRL_ENABLED HIGH
#define BATTERY_PIN (0 + 31) // P0.31/AIN7
#define PIN_BAT_ADC BATTERY_PIN
#define PIN_BAT_ADC_CTL ADC_CTRL
#define BAT_AMPLIFY 4.9F

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
