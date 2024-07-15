/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "nrf_cc310/include/crys_ecpki_build.h"

#include "Adafruit_nRFCrypto.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+


//------------- IMPLEMENTATION -------------//
nRFCrypto_ECC_PrivateKey::nRFCrypto_ECC_PrivateKey(void)
{
  _domain = NULL;
}

bool nRFCrypto_ECC_PrivateKey::begin(CRYS_ECPKI_DomainID_t id)
{
  _domain = CRYS_ECPKI_GetEcDomain(id);
  return _domain != NULL;
}

void nRFCrypto_ECC_PrivateKey::end(void)
{
  _domain = NULL;
}

// Export from internal type to raw bytes in Big Endian
// return raw buffer size = keysize + 1 (header)
uint32_t nRFCrypto_ECC_PrivateKey::toRaw(uint8_t* buffer, uint32_t bufsize)
{
  VERIFY_CRYS(CRYS_ECPKI_ExportPrivKey(&_key, buffer, &bufsize), 0);
  return bufsize;
}

// Build public key from raw bytes in Big Endian
bool nRFCrypto_ECC_PrivateKey::fromRaw(uint8_t* buffer, uint32_t bufsize)
{
  VERIFY_CRYS(CRYS_ECPKI_BuildPrivKey(_domain, buffer, bufsize, &_key), false);
  return true;
}
