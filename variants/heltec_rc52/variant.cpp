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

#include "variant.h"
#include "Arduino.h"
#include "nrf.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

const uint32_t g_ADigitalPinMap[] = {
    // P0 - pins 0 and 1 are hardwired for xtal and should never be enabled.
    0xff, 0xff, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16,   17,   18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,

    // P1
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

void initVariant()
{
  pinMode(RADIOCORE_FEM_EN, OUTPUT);
  digitalWrite(RADIOCORE_FEM_EN, HIGH);

  pinMode(RADIOCORE_VFEM_CTRL, OUTPUT);
  digitalWrite(RADIOCORE_VFEM_CTRL, HIGH);

  pinMode(PIN_TFT_VDD_CTL, OUTPUT);
  digitalWrite(PIN_TFT_VDD_CTL, !TFT_VDD_ENABLE);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
  digitalWrite(PIN_TFT_LEDA_CTL, !TFT_LEDA_ENABLE);
  pinMode(PIN_TFT_CS, OUTPUT);
  digitalWrite(PIN_TFT_CS, HIGH);
}

void variant_shutdown()
{
  digitalWrite(RADIOCORE_FEM_EN, LOW);
  digitalWrite(RADIOCORE_VFEM_CTRL, LOW);

  digitalWrite(PIN_TFT_LEDA_CTL, !TFT_LEDA_ENABLE);
  digitalWrite(PIN_TFT_VDD_CTL, !TFT_VDD_ENABLE);

  nrf_gpio_cfg_default(PIN_BUTTON1);
  nrf_gpio_cfg_default(PIN_TFT_LEDA_CTL);
  nrf_gpio_cfg_default(PIN_TFT_VDD_CTL);
  nrf_gpio_cfg_default(PIN_TFT_CS);
  nrf_gpio_cfg_default(PIN_TFT_DC);
  nrf_gpio_cfg_default(PIN_TFT_SCK);
  nrf_gpio_cfg_default(PIN_TFT_MOSI);
  nrf_gpio_cfg_default(PIN_TFT_RST);

  nrf_gpio_cfg_default(SX126X_CS);
  nrf_gpio_cfg_default(SX126X_DIO1);
  nrf_gpio_cfg_default(SX126X_BUSY);
  nrf_gpio_cfg_default(SX126X_RESET);
  nrf_gpio_cfg_default(SX126X_RXEN);

  nrf_gpio_cfg_default(PIN_SPI_MISO);
  nrf_gpio_cfg_default(PIN_SPI_MOSI);
  nrf_gpio_cfg_default(PIN_SPI_SCK);

  nrf_gpio_cfg_default(PIN_SPI1_MISO);
  nrf_gpio_cfg_default(PIN_SPI1_MOSI);
  nrf_gpio_cfg_default(PIN_SPI1_SCK);

  nrf_gpio_cfg_default(RADIOCORE_FEM_EN);
  nrf_gpio_cfg_default(RADIOCORE_VFEM_CTRL);
}
