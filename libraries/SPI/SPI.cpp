/*
 * SPI Master library for nRF5x.
 * Copyright (c) 2015 Arduino LLC
 * Copyright (c) 2016 Sandeep Mistry All right reserved.
 * Copyright (c) 2019 Ha Thach for Adafruit Industries. All right reserved.
 * Copyright (c) 2024 Harry Zhang All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

// Due to nRF52832 Errata with SPIM, we will only use SPIM for 840 variant
// https://infocenter.nordicsemi.com/topic/errata_nRF52832_Rev2/ERR/nRF52832/Rev2/latest/anomaly_832_58.html
#ifdef NRF52840_XXAA

#include "SPI.h"
#include <Arduino.h>
#include <wiring_private.h>
#include <assert.h>

#include <Adafruit_TinyUSB.h> // for Serial

SPIClass::SPIClass(NRF_SPIM_Type *p_spi, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI)
{
  initialized = false;
  assert(p_spi != NULL);

  _spim.p_reg = p_spi;

  // not used by SPIM module
  _p_spi = NULL;

  // SPIM0 & SPIM1 are used for I2C
#if NRFX_SPIM0_ENABLED
  if ( NRF_SPIM0 == p_spi ) {
    _spim.drv_inst_idx = NRFX_SPIM0_INST_IDX;
  }
#endif

#if NRFX_SPIM1_ENABLED
  if ( NRF_SPIM1 == p_spi ) {
    _spim.drv_inst_idx = NRFX_SPIM1_INST_IDX;
  }
#endif

#if NRFX_SPIM2_ENABLED
  if ( NRF_SPIM2 == p_spi ) {
    _spim.drv_inst_idx = NRFX_SPIM2_INST_IDX;
  }
#endif

#if NRFX_SPIM3_ENABLED
  if ( NRF_SPIM3 == p_spi ) {
    _spim.drv_inst_idx = NRFX_SPIM3_INST_IDX;
  }
#endif

  // pins
  _uc_pinMiso = g_ADigitalPinMap[uc_pinMISO];
  _uc_pinSCK = g_ADigitalPinMap[uc_pinSCK];
  _uc_pinMosi = g_ADigitalPinMap[uc_pinMOSI];

  _dataMode = SPI_MODE0;
  _bitOrder = NRF_SPIM_BIT_ORDER_MSB_FIRST;
}

void SPIClass::begin()
{
  if (initialized) return;
  if(_mutex_spi == NULL)
  {
    _mutex_spi = xSemaphoreCreateMutex();
    if(_mutex_spi == NULL) {
      Serial1.println("_mutex_spi LOCK error.");
      return; // no new driver was installed
    }
  }
  _mutex_unlock();
  initialized = true;

  _mutex_lock();
  nrfx_spim_config_t cfg =
  {
    .sck_pin        = _uc_pinSCK,
    .mosi_pin       = _uc_pinMosi,
    .miso_pin       = _uc_pinMiso,
    .ss_pin         = NRFX_SPIM_PIN_NOT_USED,
    .ss_active_high = false,
    .irq_priority   = 3,
    .orc            = 0xFF,
    // default setting 4 Mhz, Mode 0, MSB first
    .frequency      = NRF_SPIM_FREQ_4M,
    .mode           = NRF_SPIM_MODE_0,
    .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,
  };

  _dataMode = SPI_MODE0;
  _bitOrder = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  // blocking
  nrfx_spim_init(&_spim, &cfg, NULL, NULL);

  // highspeed SPIM should set SCK and MOSI to high drive
  nrf_gpio_cfg(_uc_pinSCK,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0H1,
               NRF_GPIO_PIN_NOSENSE);

  nrf_gpio_cfg(_uc_pinMosi,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0H1,
               NRF_GPIO_PIN_NOSENSE);
               
  _mutex_unlock();
}

void SPIClass::end()
{
  if (!initialized) return;
  _mutex_lock();
  nrfx_spim_uninit(&_spim);
  initialized = false;
  vSemaphoreDelete(_mutex_spi);
  _mutex_spi = NULL;
  _mutex_unlock();
}

void SPIClass::usingInterrupt(int /*interruptNumber*/)
{
}

void SPIClass::beginTransaction(SPISettings settings)
{
  if (!initialized) return;

  _mutex_lock();
  nrf_spim_disable(_spim.p_reg);
  this->_dataMode = settings.dataMode;
  this->_bitOrder = (settings.bitOrder == MSBFIRST ? NRF_SPIM_BIT_ORDER_MSB_FIRST : NRF_SPIM_BIT_ORDER_LSB_FIRST);
  nrf_spim_configure(_spim.p_reg, (nrf_spim_mode_t) _dataMode, (nrf_spim_bit_order_t) _bitOrder);
  _mutex_unlock();
  setClockDivider(F_CPU / settings.clockFreq);
  _mutex_lock();
  nrf_spim_enable(_spim.p_reg);
  _mutex_unlock();
}

void SPIClass::endTransaction(void)
{
  if (!initialized) return;

  _mutex_lock();
  nrf_spim_disable(_spim.p_reg);
  _mutex_unlock();
}

void SPIClass::setPins(uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI)
{
  _uc_pinMiso = g_ADigitalPinMap[uc_pinMISO];
  _uc_pinSCK = g_ADigitalPinMap[uc_pinSCK];
  _uc_pinMosi = g_ADigitalPinMap[uc_pinMOSI];
}

void SPIClass::setBitOrder(BitOrder order)
{
  _mutex_lock();
  this->_bitOrder = (order == MSBFIRST ? NRF_SPIM_BIT_ORDER_MSB_FIRST : NRF_SPIM_BIT_ORDER_LSB_FIRST);
  nrf_spim_configure(_spim.p_reg, (nrf_spim_mode_t) _dataMode, (nrf_spim_bit_order_t) _bitOrder);
  _mutex_unlock();
}

void SPIClass::setDataMode(uint8_t mode)
{
  _mutex_lock();

  this->_dataMode = mode;
  nrf_spim_configure(_spim.p_reg, (nrf_spim_mode_t) _dataMode, (nrf_spim_bit_order_t) _bitOrder);
  _mutex_unlock();
}

void SPIClass::setClockDivider(uint32_t div)
{
  if (!initialized) return;

  _mutex_lock();

  nrf_spim_frequency_t clockFreq;

  // Adafruit Note: nrf52 run at 64MHz
  if (div >= SPI_CLOCK_DIV512) {
    clockFreq = NRF_SPIM_FREQ_125K;
  } else if (div >= SPI_CLOCK_DIV256) {
    clockFreq = NRF_SPIM_FREQ_250K;
  } else if (div >= SPI_CLOCK_DIV128) {
    clockFreq = NRF_SPIM_FREQ_500K;
  } else if (div >= SPI_CLOCK_DIV64) {
    clockFreq = NRF_SPIM_FREQ_1M;
  } else if (div >= SPI_CLOCK_DIV32) {
    clockFreq = NRF_SPIM_FREQ_2M;
  } else if (div >= SPI_CLOCK_DIV16) {
    clockFreq = NRF_SPIM_FREQ_4M;
  } else if (div >= SPI_CLOCK_DIV8) {
    clockFreq = NRF_SPIM_FREQ_8M;
  } else {
    if ( _spim.p_reg == NRF_SPIM3 )
    {
      if (div >= SPI_CLOCK_DIV4) {
        clockFreq = NRF_SPIM_FREQ_16M;
      }else {
        clockFreq = NRF_SPIM_FREQ_32M;
      }
    }else
    {
      clockFreq = NRF_SPIM_FREQ_8M;
    }
  }

  nrf_spim_frequency_set(_spim.p_reg, clockFreq);
  _mutex_unlock();
}

void SPIClass::transfer(const void *tx_buf, void *rx_buf, size_t count)
{
  if (!initialized) return;

  _mutex_lock();

  const uint8_t* tx_buf8 = (const uint8_t*) tx_buf;
  uint8_t* rx_buf8 = (uint8_t*) rx_buf;

  while (count)
  {
    // each transfer can only up to 64KB (16-bit) bytes
    const size_t xfer_len = min(count, (size_t) UINT16_MAX);

    nrfx_spim_xfer_desc_t xfer_desc =
    {
      .p_tx_buffer = tx_buf8,
      .tx_length   = tx_buf8 ? xfer_len : 0,

      .p_rx_buffer = rx_buf8,
      .rx_length   = rx_buf8 ? xfer_len : 0,
    };

    nrfx_spim_xfer(&_spim, &xfer_desc, 0);

    count -= xfer_len;
    if (tx_buf8) tx_buf8 += xfer_len;
    if (rx_buf8) rx_buf8 += xfer_len;
  }
  _mutex_unlock();
}

void SPIClass::transfer(void *buf, size_t count)
{
  transfer(buf, buf, count);
}

byte SPIClass::transfer(uint8_t data)
{
  transfer(&data, 1);
  return data;
}

uint16_t SPIClass::transfer16(uint16_t data) {

  union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;

  t.val = data;

  if (_bitOrder == NRF_SPIM_BIT_ORDER_LSB_FIRST) {
    t.lsb = transfer(t.lsb);
    t.msb = transfer(t.msb);
  } else {
    t.msb = transfer(t.msb);
    t.lsb = transfer(t.lsb);
  }

  return t.val;
}

void SPIClass::attachInterrupt() {
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
  // Should be disableInterrupt()
}

// default to 0
#ifndef SPI_32MHZ_INTERFACE
#define SPI_32MHZ_INTERFACE 0
#endif

#if SPI_32MHZ_INTERFACE == 0
  #define _SPI_DEV    NRF_SPIM3 // 32 Mhz
  #define _SPI1_DEV   NRF_SPIM2

#elif SPI_32MHZ_INTERFACE == 1
  #define _SPI_DEV    NRF_SPIM2
  #define _SPI1_DEV   NRF_SPIM3 // 32 Mhz

#else
  #error "not supported yet"
#endif

#if SPI_INTERFACES_COUNT >= 1
// use SPIM3 for highspeed 32Mhz
SPIClass SPI(_SPI_DEV,  PIN_SPI_MISO,  PIN_SPI_SCK,  PIN_SPI_MOSI);
#endif

#if SPI_INTERFACES_COUNT >= 2
SPIClass SPI1(_SPI1_DEV, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI);
#endif

#endif // NRF52840_XXAA
