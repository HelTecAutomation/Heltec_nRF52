/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2024 Harry Zhang All right reserved.

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

#include <nrf.h>

#include "Arduino.h"
#include "wiring_private.h"
#include "nrf_gpiote.h"
#include "nrfx_gpiote.h"

#include <string.h>

#if defined(NRF52) || defined(NRF52_SERIES)
#define NUMBER_OF_GPIO_TE 8
#else
#define NUMBER_OF_GPIO_TE 4
#endif

#ifdef GPIOTE_CONFIG_PORT_Msk
#define GPIOTE_CONFIG_PORT_PIN_Msk (GPIOTE_CONFIG_PORT_Msk | GPIOTE_CONFIG_PSEL_Msk)
#else
#define GPIOTE_CONFIG_PORT_PIN_Msk GPIOTE_CONFIG_PSEL_Msk
#endif

static voidFuncPtr callbacksInt[NUMBER_OF_GPIO_TE];
static bool callbackDeferred[NUMBER_OF_GPIO_TE];
static int8_t channelMap[NUMBER_OF_GPIO_TE];
static int enabled = 0;

/* Configure I/O interrupt sources */
static void __initialize()
{
  nrfx_gpiote_init(6);

  return;
  memset(callbacksInt, 0, sizeof(callbacksInt));
  memset(channelMap, -1, sizeof(channelMap));
  memset(callbackDeferred, 0, sizeof(callbackDeferred));

  NVIC_DisableIRQ(GPIOTE_IRQn);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_SetPriority(GPIOTE_IRQn, 3);
  NVIC_EnableIRQ(GPIOTE_IRQn);
}

/*
 * Modified by Harry Zhang(Heltec Automation)
 * \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 *        Replaces any previous function that was attached to the interrupt.
 *
 * \return Interrupt Mask
 * 
 */
int attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
	if (!enabled) {
	  __initialize();
	  enabled = 1;
	}

	if (pin >= PINS_COUNT) {
	  return 0;
	}

	pin = g_ADigitalPinMap[pin];
	
	nrfx_gpiote_in_config_t config=NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
	
	switch (mode) {
	  case CHANGE:
		config.sense=NRF_GPIOTE_POLARITY_TOGGLE;
		break;
	
	  case FALLING:
		config.sense=NRF_GPIOTE_POLARITY_HITOLO;
		//config.pull=NRF_GPIO_PIN_PULLUP;
		break;
	
	  case RISING:
		config.sense=NRF_GPIOTE_POLARITY_LOTOHI;
		//config.pull=NRF_GPIO_PIN_PULLDOWN;
		break;
	
	  default:
		return 0;
	}
	uint32_t pin_tmp=pin;
	NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_tmp);
	config.pull=(reg->PIN_CNF[pin_tmp]&GPIO_PIN_CNF_PULL_Msk)>>GPIO_PIN_CNF_PULL_Pos;
	nrfx_gpiote_in_init(pin,&config,callback);
	nrfx_gpiote_in_event_enable(pin,true);

	return 1;
}

/*
 * Modified by Harry Zhang(Heltec Automation)
 * \brief Turns off the given interrupt.
 */
void detachInterrupt(uint32_t pin)
{
  if (pin >= PINS_COUNT) {
    return;
  }
  
  pin = g_ADigitalPinMap[pin];
  nrfx_gpiote_in_uninit(pin);
  nrfx_gpiote_in_event_enable(pin,false);
  return ;
}

/*
void GPIOTE_IRQHandler()
{
#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordEnterISR();
#endif

  // Read this once (not 8x), as it's a volatile read
  // across the AHB, which adds up to 3 cycles.
  uint32_t const enabledInterruptMask = NRF_GPIOTE->INTENSET;
  for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
    // only process where the interrupt is enabled and the event register is set
    // check interrupt enabled mask first, as already read that IOM value, to
    // reduce delays from AHB (16MHz) reads.
    if ( 0 == (enabledInterruptMask & (1 << ch))) continue;
    if ( 0 == NRF_GPIOTE->EVENTS_IN[ch]) continue;

    // If the event was set and interrupts are enabled,
    // call the callback function only if it exists,
    // but ALWAYS clear the event to prevent an interrupt storm.
    if (channelMap[ch] != -1 && callbacksInt[ch]) {
      if ( callbackDeferred[ch] ) {
        // Adafruit defer callback to non-isr if configured so
        ada_callback(NULL, 0, callbacksInt[ch]);
      } else {
        callbacksInt[ch]();
      }
    }

    // clear the event
    NRF_GPIOTE->EVENTS_IN[ch] = 0;
  }
#if __CORTEX_M == 0x04
  // See note at nRF52840_PS_v1.1.pdf section 6.1.8 ("interrupt clearing")
  // See also https://gcc.gnu.org/onlinedocs/gcc/Volatiles.html for why
  // using memory barrier instead of read of an unrelated volatile
  __DSB(); __NOP();__NOP();__NOP();__NOP();
#endif

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordExitISR();
#endif
}
 */
