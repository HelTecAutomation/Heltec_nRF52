/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Marcus Schmid (lanaticasylum.de)
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

uint8_t digestSize(CRYS_HASH_OperationMode_t mode) {
    switch (mode) {
        case CRYS_HASH_SHA1_mode:
            return CRYS_HASH_SHA1_DIGEST_SIZE_IN_BYTES;
        case CRYS_HASH_SHA224_mode:
            return CRYS_HASH_SHA224_DIGEST_SIZE_IN_BYTES;
        case CRYS_HASH_SHA256_mode:
            return CRYS_HASH_SHA256_DIGEST_SIZE_IN_BYTES;
        case CRYS_HASH_SHA384_mode:
            return CRYS_HASH_SHA384_DIGEST_SIZE_IN_BYTES;
        case CRYS_HASH_SHA512_mode:
            return CRYS_HASH_SHA512_DIGEST_SIZE_IN_BYTES;
        case CRYS_HASH_MD5_mode:
            return CRYS_HASH_MD5_DIGEST_SIZE_IN_BYTES;
        default:
            return 0;
    }
}

nRFCrypto_Hmac::nRFCrypto_Hmac(void) { _digest_size = 0; }

bool nRFCrypto_Hmac::begin(CRYS_HASH_OperationMode_t mode, uint8_t *key_ptr,
                           uint16_t keySize) {
    _digest_size = digestSize(mode);
    VERIFY_ERROR(CRYS_HMAC_Init(&_context, mode, key_ptr, keySize), false);
    return true;
}

bool nRFCrypto_Hmac::update(uint8_t data[], size_t size) {
    VERIFY_ERROR(CRYS_HMAC_Update(&_context, data, size), false);
    return true;
}

uint8_t nRFCrypto_Hmac::end(uint32_t result[16]) {
    VERIFY_ERROR(CRYS_HMAC_Finish(&_context, result), 0);
    return _digest_size;
}
