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

#include "Adafruit_nRFCrypto.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// Length of Digest Op mode in bytes
static const uint8_t digest_len_arr[] =
{
  CRYS_HASH_SHA1_DIGEST_SIZE_IN_BYTES,
  CRYS_HASH_SHA224_DIGEST_SIZE_IN_BYTES,
  CRYS_HASH_SHA256_DIGEST_SIZE_IN_BYTES,
  CRYS_HASH_SHA384_DIGEST_SIZE_IN_BYTES,
  CRYS_HASH_SHA512_DIGEST_SIZE_IN_BYTES,
  CRYS_HASH_MD5_DIGEST_SIZE_IN_BYTES
};


//------------- IMPLEMENTATION -------------//
nRFCrypto_Hash::nRFCrypto_Hash(void)
{
  _digest_len = 0;
}

bool nRFCrypto_Hash::begin(CRYS_HASH_OperationMode_t mode)
{
  VERIFY_ERROR( CRYS_HASH_Init(&_context, mode), false );
  if ( mode < CRYS_HASH_NumOfModes ) _digest_len = digest_len_arr[mode];

  return true;
}

bool nRFCrypto_Hash::update(uint8_t data[], size_t size)
{
  VERIFY_ERROR( CRYS_HASH_Update(&_context, data, size), false );
  return true;
}

uint8_t nRFCrypto_Hash::end(uint32_t result[16])
{
  VERIFY_ERROR( CRYS_HASH_Finish(&_context, result), 0);
  return _digest_len;
}



