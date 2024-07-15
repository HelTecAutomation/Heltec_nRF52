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

#ifndef NRFCRYPTO_ECC_H_
#define NRFCRYPTO_ECC_H_

#include "nrf_cc310/include/crys_ecpki_types.h"
#include "nrf_cc310/include/crys_ecpki_error.h"
#include "nrf_cc310/include/crys_ecpki_domain.h"

#include "nRFCrypto_ECC_PublicKey.h"
#include "nRFCrypto_ECC_PrivateKey.h"

class nRFCrypto_ECC
{
  public:
    //------------- Static Members -------------//
    static bool genKeyPair(nRFCrypto_ECC_PrivateKey& private_key, nRFCrypto_ECC_PublicKey& public_key);
    static uint32_t SVDP_DH(nRFCrypto_ECC_PrivateKey& private_key, nRFCrypto_ECC_PublicKey& peer_pubkey, uint8_t* shared_secret, uint32_t bufsize);

  public:
    nRFCrypto_ECC(void);
    bool begin(void);
    void end(void);

  private:

};

#endif /* NRFCRYPTO_ECC_H_ */
