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

#ifndef _VARIANT_CLUE52840_
#define _VARIANT_CLUE52840_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)
#define USE_LFXO    // Board uses LFCLK crystal oscillator


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus
#define HARD_VERSION_ADDR    (0xED000+7*4096-16-1) 
#define HT_LICENSE_ADDR      (0xED000+7*4096-16) 
#define HT_LICENSE_ADDR_BASE (0xED000+6*4096) 
// Number of pins defined in PinDescription array
#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (8)
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs
#define PIN_LED1             (35)
#define PIN_NEOPIXEL         (14)
#define NEOPIXEL_NUM         2

#define LED_BUILTIN          PIN_LED1

#define LED_RED              PIN_LED1
#define LED_BLUE             PIN_LED1

#define LED_STATE_ON         1         // State when LED is litted

// Buttons
#define PIN_BUTTON1          (5)    // Button A
#define PIN_BUTTON2          (11)   // Button B

// Microphone
#define PIN_PDM_DIN           35
#define PIN_PDM_CLK           36
#define PIN_PDM_PWR           -1  // not used

// Buzzer
#define PIN_BUZZER            46

/*
 * Analog pins
 */
#define PIN_A0               (21)
#define PIN_A1               (22)
#define PIN_A2               (23)
#define PIN_A3               (24)
#define PIN_A4               (25)
#define PIN_A5               (26)
#define PIN_A6               (27)
#define PIN_A7               (28)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
#define ADC_RESOLUTION    14

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX       (9)
#define PIN_SERIAL1_TX       (10)

#define PIN_SERIAL2_RX       (37)
#define PIN_SERIAL2_TX       (39)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

// nRF52840 has only one SPIM3 runing at highspeed 32Mhz
// This assign SPIM3 to either: SPI (0), SPI1 (1).
// If not defined, default to 0 or SPI.
// #define SPI_32MHZ_INTERFACE  1

// SPI
#define PIN_SPI_MISO         (23)
#define PIN_SPI_MOSI         (22)
#define PIN_SPI_SCK          (19)

static const uint8_t SS   = (16);
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#if (SPI_INTERFACES_COUNT==2)
// SPI1
#define PIN_SPI1_MISO         (-1)
#define PIN_SPI1_MOSI         (2)
#define PIN_SPI1_SCK          (12)

static const uint8_t SS1   = (30);
static const uint8_t MOSI1 = PIN_SPI1_MOSI ;
static const uint8_t MISO1 = PIN_SPI1_MISO ;
static const uint8_t SCK1  = PIN_SPI1_SCK ;
#endif

/*
 * Wire Interfaces
 */
#if (SPI_INTERFACES_COUNT==1)
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA        (2)
#define PIN_WIRE_SCL        (12)
#else
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA        (15)
#define PIN_WIRE_SCL        (16)
#endif
// QSPI Pins
/*
#define PIN_QSPI_SCK         46
#define PIN_QSPI_CS          47
#define PIN_QSPI_IO0         44
#define PIN_QSPI_IO1         45
#define PIN_QSPI_IO2         32
#define PIN_QSPI_IO3         33
*/

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   MX25R1635F
#define EXTERNAL_FLASH_USE_QSPI
// 	// HT_E0213A367(40 _rst, 13 _dc, 41 _cs, 11 _busy, 12 _sck, 2 _mosi, int8_t _miso, uint32_t _freq = 6000000, DISPLAY_GEOMETRY g = GEOMETRY_250_122)

// On-board TFT display
#define PIN_TFT_CS        41
#define PIN_TFT_RST       40 // Or set to -1 and connect to Arduino RESET pin
#define PIN_TFT_DC        13

#define PIN_VEXT_CTL      21
#define VEXT_ENABLE       1
#define PIN_TFT_VDD_CTL   3
#define TFT_VDD_ENABLE    0
#define PIN_TFT_LEDA_CTL  11
#define TFT_LEDA_ENABLE   1

#define PIN_BAT_ADC     4
#define PIN_BAT_ADC_CTL 6
#define BAT_AMPLIFY     4.9

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
