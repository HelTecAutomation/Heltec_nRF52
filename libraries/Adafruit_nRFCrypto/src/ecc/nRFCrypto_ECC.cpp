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

#include "nrf_cc310/include/crys_ecpki_kg.h"
#include "nrf_cc310/include/crys_ecpki_dh.h"

#include "Adafruit_nRFCrypto.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// ECC
//--------------------------------------------------------------------+

bool nRFCrypto_ECC::genKeyPair(nRFCrypto_ECC_PrivateKey& private_key, nRFCrypto_ECC_PublicKey& public_key)
{
  VERIFY( private_key.getDomain() == public_key.getDomain() );

  CRYS_ECPKI_KG_TempData_t* tempbuf = (CRYS_ECPKI_KG_TempData_t*) rtos_malloc( sizeof(CRYS_ECPKI_KG_TempData_t) );
  VERIFY(tempbuf);

  uint32_t err = CRYS_ECPKI_GenKeyPair(nRFCrypto.Random.getContext(), CRYS_RND_GenerateVector, private_key.getDomain(),
                                       &private_key._key, &public_key._key,
                                       tempbuf, NULL);
  rtos_free(tempbuf);

  VERIFY_CRYS(err, false);
  return true;
}

uint32_t nRFCrypto_ECC::SVDP_DH(nRFCrypto_ECC_PrivateKey& private_key, nRFCrypto_ECC_PublicKey& peer_pubkey, uint8_t* shared_secret, uint32_t bufsize)
{
  CRYS_ECDH_TempData_t* tempbuf = (CRYS_ECDH_TempData_t *) rtos_malloc(sizeof(CRYS_ECDH_TempData_t));
  VERIFY(tempbuf);

  uint32_t err = CRYS_ECDH_SVDP_DH(&peer_pubkey._key, &private_key._key, shared_secret, &bufsize, tempbuf);

  rtos_free(tempbuf);

  VERIFY_CRYS(err, 0);
  return bufsize;
}

nRFCrypto_ECC::nRFCrypto_ECC(void)
{

}

bool nRFCrypto_ECC::begin(void)
{
  return true;
}

void nRFCrypto_ECC::end(void)
{

}

