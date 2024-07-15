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

#include "variant.h"
#include "common_inc.h"
#include "utility/utilities.h"

#include "nrf_cc310/include/sns_silib.h"
#include "Adafruit_nRFCrypto.h"

#include <Adafruit_TinyUSB.h> // for Serial

// Only nRF52840 has CC310
#ifndef NRF_CRYPTOCELL
  #error CryptoCell CC310 is not supported on this board
#endif

// https://devzone.nordicsemi.com/f/nordic-q-a/40825/nrf52840-using-the-cryptocell310-with-a-preemptive-os
// The cc310 lib doesn't work nicely with FreeRTOS, we either to
// - Mask interrupts with most CC310 API with taskENTER_CRITICAL/taskEXIT_CRITICAL or
// - Use no-interrupt version of lib c310 (which is what we did)
//
// Until Nordic "fixes" their implementation, we are better to use no-interrupt version
#define USE_CC310_LIB_NO_INTERRUPT

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

Adafruit_nRFCrypto nRFCrypto;

//------------- IMPLEMENTATION -------------//

Adafruit_nRFCrypto::Adafruit_nRFCrypto(void)
{
  _begun = false;
}

bool Adafruit_nRFCrypto::begin(void)
{
  // skip if already called begin before
  if (_begun) return true;
  _begun = true;

#ifndef USE_CC310_LIB_NO_INTERRUPT
  NVIC_SetPriority(CRYPTOCELL_IRQn, 2);
  NVIC_EnableIRQ(CRYPTOCELL_IRQn);
#endif

  VERIFY_ERROR(SaSi_LibInit(), false);
  VERIFY( Random.begin() );

  return true;
}

void Adafruit_nRFCrypto::end(void)
{
  // skipped if not begin-ed
  if (!_begun) return;
  _begun = false;

  SaSi_LibFini();
  Random.end();

#ifndef USE_CC310_LIB_NO_INTERRUPT
  NVIC_DisableIRQ(CRYPTOCELL_IRQn);
#endif
}

//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_DEBUG

static lookup_entry_t const _str_crys_lookup[] =
{
  { .key =  CRYS_ECPKI_ILLEGAL_DOMAIN_ID_ERROR                           , .data = "ECPKI_ILLEGAL_DOMAIN_ID" },
  { .key =  CRYS_ECPKI_DOMAIN_PTR_ERROR                                  , .data = "ECPKI_DOMAIN_PTR" },
  { .key =  CRYS_ECPKI_GEN_KEY_INVALID_PRIVATE_KEY_PTR_ERROR             , .data = "ECPKI_GEN_KEY_INVALID_PRIVATE_KEY_PTR" },
  { .key =  CRYS_ECPKI_GEN_KEY_INVALID_PUBLIC_KEY_PTR_ERROR              , .data = "ECPKI_GEN_KEY_INVALID_PUBLIC_KEY_PTR" },
  { .key =  CRYS_ECPKI_GEN_KEY_INVALID_TEMP_DATA_PTR_ERROR               , .data = "ECPKI_GEN_KEY_INVALID_TEMP_DATA_PTR" },
  { .key =  CRYS_ECPKI_RND_CONTEXT_PTR_ERROR                             , .data = "ECPKI_RND_CONTEXT_PTR" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_COMPRESSION_MODE_ERROR          , .data = "ECPKI_BUILD_KEY_INVALID_COMPRESSION_MODE" },
  { .key =  CRYS_ECPKI_BUILD_KEY_ILLEGAL_DOMAIN_ID_ERROR                 , .data = "ECPKI_BUILD_KEY_ILLEGAL_DOMAIN_ID" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_PRIV_KEY_IN_PTR_ERROR           , .data = "ECPKI_BUILD_KEY_INVALID_PRIV_KEY_IN_PTR" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_USER_PRIV_KEY_PTR_ERROR         , .data = "ECPKI_BUILD_KEY_INVALID_USER_PRIV_KEY_PTR" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_PRIV_KEY_SIZE_ERROR             , .data = "ECPKI_BUILD_KEY_INVALID_PRIV_KEY_SIZE" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_PRIV_KEY_DATA_ERROR             , .data = "ECPKI_BUILD_KEY_INVALID_PRIV_KEY_DATA" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_PUBL_KEY_IN_PTR_ERROR           , .data = "ECPKI_BUILD_KEY_INVALID_PUBL_KEY_IN_PTR" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_USER_PUBL_KEY_PTR_ERROR         , .data = "ECPKI_BUILD_KEY_INVALID_USER_PUBL_KEY_PTR" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_PUBL_KEY_SIZE_ERROR             , .data = "ECPKI_BUILD_KEY_INVALID_PUBL_KEY_SIZE" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_PUBL_KEY_DATA_ERROR             , .data = "ECPKI_BUILD_KEY_INVALID_PUBL_KEY_DATA" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_CHECK_MODE_ERROR                , .data = "ECPKI_BUILD_KEY_INVALID_CHECK_MODE" },
  { .key =  CRYS_ECPKI_BUILD_KEY_INVALID_TEMP_BUFF_PTR_ERROR             , .data = "ECPKI_BUILD_KEY_INVALID_TEMP_BUFF_PTR" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_INVALID_USER_PUBL_KEY_PTR_ERROR   , .data = "ECPKI_EXPORT_PUBL_KEY_INVALID_USER_PUBL_KEY_PTR" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_ILLEGAL_COMPRESSION_MODE_ERROR    , .data = "ECPKI_EXPORT_PUBL_KEY_ILLEGAL_COMPRESSION_MODE" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_INVALID_EXTERN_PUBL_KEY_PTR_ERROR , .data = "ECPKI_EXPORT_PUBL_KEY_INVALID_EXTERN_PUBL_KEY_PTR" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_INVALID_PUBL_KEY_SIZE_PTR_ERROR   , .data = "ECPKI_EXPORT_PUBL_KEY_INVALID_PUBL_KEY_SIZE_PTR" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_INVALID_PUBL_KEY_SIZE_ERROR       , .data = "ECPKI_EXPORT_PUBL_KEY_INVALID_PUBL_KEY_SIZE" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_ILLEGAL_DOMAIN_ID_ERROR           , .data = "ECPKI_EXPORT_PUBL_KEY_ILLEGAL_DOMAIN_ID" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_ILLEGAL_VALIDATION_TAG_ERROR      , .data = "ECPKI_EXPORT_PUBL_KEY_ILLEGAL_VALIDATION_TAG" },
  { .key =  CRYS_ECPKI_EXPORT_PUBL_KEY_INVALID_PUBL_KEY_DATA_ERROR       , .data = "ECPKI_EXPORT_PUBL_KEY_INVALID_PUBL_KEY_DATA" },
  { .key =  CRYS_ECPKI_EXPORT_PRIV_KEY_INVALID_USER_PRIV_KEY_PTR_ERROR   , .data = "ECPKI_EXPORT_PRIV_KEY_INVALID_USER_PRIV_KEY_PTR" },
  { .key =  CRYS_ECPKI_EXPORT_PRIV_KEY_INVALID_EXTERN_PRIV_KEY_PTR_ERROR , .data = "ECPKI_EXPORT_PRIV_KEY_INVALID_EXTERN_PRIV_KEY_PTR" },
  { .key =  CRYS_ECPKI_EXPORT_PRIV_KEY_ILLEGAL_VALIDATION_TAG_ERROR      , .data = "ECPKI_EXPORT_PRIV_KEY_ILLEGAL_VALIDATION_TAG" },
  { .key =  CRYS_ECPKI_EXPORT_PRIV_KEY_INVALID_PRIV_KEY_SIZE_PTR_ERROR   , .data = "ECPKI_EXPORT_PRIV_KEY_INVALID_PRIV_KEY_SIZE_PTR" },
  { .key =  CRYS_ECPKI_EXPORT_PRIV_KEY_INVALID_PRIV_KEY_SIZE_ERROR       , .data = "ECPKI_EXPORT_PRIV_KEY_INVALID_PRIV_KEY_SIZE" },
  { .key =  CRYS_ECPKI_EXPORT_PRIV_KEY_INVALID_PRIV_KEY_DATA_ERROR       , .data = "ECPKI_EXPORT_PRIV_KEY_INVALID_PRIV_KEY_DATA" },
  { .key =  CRYS_ECPKI_BUILD_DOMAIN_ID_IS_NOT_VALID_ERROR                , .data = "ECPKI_BUILD_DOMAIN_ID_IS_NOT_VALID" },
  { .key =  CRYS_ECPKI_BUILD_DOMAIN_DOMAIN_PTR_ERROR                     , .data = "ECPKI_BUILD_DOMAIN_DOMAIN_PTR" },
  { .key =  CRYS_ECPKI_BUILD_DOMAIN_EC_PARAMETR_PTR_ERROR                , .data = "ECPKI_BUILD_DOMAIN_EC_PARAMETR_PTR" },
  { .key =  CRYS_ECPKI_BUILD_DOMAIN_EC_PARAMETR_SIZE_ERROR               , .data = "ECPKI_BUILD_DOMAIN_EC_PARAMETR_SIZE" },
  { .key =  CRYS_ECPKI_BUILD_DOMAIN_COFACTOR_PARAMS_ERROR                , .data = "ECPKI_BUILD_DOMAIN_COFACTOR_PARAMS" },
  { .key =  CRYS_ECPKI_BUILD_DOMAIN_SECURITY_STRENGTH_ERROR              , .data = "ECPKI_BUILD_DOMAIN_SECURITY_STRENGTH" },
  { .key =  CRYS_ECPKI_BUILD_SCA_RESIST_ILLEGAL_MODE_ERROR               , .data = "ECPKI_BUILD_SCA_RESIST_ILLEGAL_MODE" },
  { .key =  CRYS_ECPKI_PKI_INTERNAL_ERROR                                , .data = "ECPKI_PKI_INTERNAL" },
  { .key =  CRYS_ECDH_SVDP_DH_INVALID_PARTNER_PUBL_KEY_PTR_ERROR         , .data = "ECDH_SVDP_DH_INVALID_PARTNER_PUBL_KEY_PTR" },
  { .key =  CRYS_ECDH_SVDP_DH_PARTNER_PUBL_KEY_VALID_TAG_ERROR           , .data = "ECDH_SVDP_DH_PARTNER_PUBL_KEY_VALID_TAG" },
  { .key =  CRYS_ECDH_SVDP_DH_INVALID_USER_PRIV_KEY_PTR_ERROR            , .data = "ECDH_SVDP_DH_INVALID_USER_PRIV_KEY_PTR" },
  { .key =  CRYS_ECDH_SVDP_DH_USER_PRIV_KEY_VALID_TAG_ERROR              , .data = "ECDH_SVDP_DH_USER_PRIV_KEY_VALID_TAG" },
  { .key =  CRYS_ECDH_SVDP_DH_INVALID_SHARED_SECRET_VALUE_PTR_ERROR      , .data = "ECDH_SVDP_DH_INVALID_SHARED_SECRET_VALUE_PTR" },
  { .key =  CRYS_ECDH_SVDP_DH_INVALID_TEMP_DATA_PTR_ERROR                , .data = "ECDH_SVDP_DH_INVALID_TEMP_DATA_PTR" },
  { .key =  CRYS_ECDH_SVDP_DH_INVALID_SHARED_SECRET_VALUE_SIZE_PTR_ERROR , .data = "ECDH_SVDP_DH_INVALID_SHARED_SECRET_VALUE_SIZE_PTR" },
  { .key =  CRYS_ECDH_SVDP_DH_INVALID_SHARED_SECRET_VALUE_SIZE_ERROR     , .data = "ECDH_SVDP_DH_INVALID_SHARED_SECRET_VALUE_SIZE" },
  { .key =  CRYS_ECDH_SVDP_DH_ILLEGAL_DOMAIN_ID_ERROR                    , .data = "ECDH_SVDP_DH_ILLEGAL_DOMAIN_ID" },
  { .key =  CRYS_ECDH_SVDP_DH_NOT_CONCENT_PUBL_AND_PRIV_DOMAIN_ID_ERROR  , .data = "ECDH_SVDP_DH_NOT_CONCENT_PUBL_AND_PRIV_DOMAIN_ID" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_DOMAIN_ID_ERROR                      , .data = "ECDSA_SIGN_INVALID_DOMAIN_ID" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_USER_CONTEXT_PTR_ERROR               , .data = "ECDSA_SIGN_INVALID_USER_CONTEXT_PTR" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_USER_PRIV_KEY_PTR_ERROR              , .data = "ECDSA_SIGN_INVALID_USER_PRIV_KEY_PTR" },
  { .key =  CRYS_ECDSA_SIGN_ILLEGAL_HASH_OP_MODE_ERROR                   , .data = "ECDSA_SIGN_ILLEGAL_HASH_OP_MODE" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_MESSAGE_DATA_IN_PTR_ERROR            , .data = "ECDSA_SIGN_INVALID_MESSAGE_DATA_IN_PTR" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_MESSAGE_DATA_IN_SIZE_ERROR           , .data = "ECDSA_SIGN_INVALID_MESSAGE_DATA_IN_SIZE" },
  { .key =  CRYS_ECDSA_SIGN_USER_CONTEXT_VALIDATION_TAG_ERROR            , .data = "ECDSA_SIGN_USER_CONTEXT_VALIDATION_TAG" },
  { .key =  CRYS_ECDSA_SIGN_USER_PRIV_KEY_VALIDATION_TAG_ERROR           , .data = "ECDSA_SIGN_USER_PRIV_KEY_VALIDATION_TAG" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_SIGNATURE_OUT_PTR_ERROR              , .data = "ECDSA_SIGN_INVALID_SIGNATURE_OUT_PTR" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_SIGNATURE_OUT_SIZE_PTR_ERROR         , .data = "ECDSA_SIGN_INVALID_SIGNATURE_OUT_SIZE_PTR" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_SIGNATURE_OUT_SIZE_ERROR             , .data = "ECDSA_SIGN_INVALID_SIGNATURE_OUT_SIZE" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_IS_EPHEMER_KEY_INTERNAL_ERROR        , .data = "ECDSA_SIGN_INVALID_IS_EPHEMER_KEY_INTERNAL" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_EPHEMERAL_KEY_PTR_ERROR              , .data = "ECDSA_SIGN_INVALID_EPHEMERAL_KEY_PTR" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_RND_CONTEXT_PTR_ERROR                , .data = "ECDSA_SIGN_INVALID_RND_CONTEXT_PTR" },
  { .key =  CRYS_ECDSA_SIGN_INVALID_RND_FUNCTION_PTR_ERROR               , .data = "ECDSA_SIGN_INVALID_RND_FUNCTION_PTR" },
  { .key =  CRYS_ECDSA_SIGN_SIGNING_ERROR                                , .data = "ECDSA_SIGN_SIGNING" },
  { .key =  CRYS_ECDSA_VERIFY_INVALID_DOMAIN_ID_ERROR                    , .data = "ECDSA_VERIFY_INVALID_DOMAIN_ID" },
  { .key =  CRYS_ECDSA_VERIFY_INVALID_USER_CONTEXT_PTR_ERROR             , .data = "ECDSA_VERIFY_INVALID_USER_CONTEXT_PTR" },
  { .key =  CRYS_ECDSA_VERIFY_INVALID_SIGNER_PUBL_KEY_PTR_ERROR          , .data = "ECDSA_VERIFY_INVALID_SIGNER_PUBL_KEY_PTR" },
  { .key =  CRYS_ECDSA_VERIFY_ILLEGAL_HASH_OP_MODE_ERROR                 , .data = "ECDSA_VERIFY_ILLEGAL_HASH_OP_MODE" },
  { .key =  CRYS_ECDSA_VERIFY_INVALID_SIGNATURE_IN_PTR_ERROR             , .data = "ECDSA_VERIFY_INVALID_SIGNATURE_IN_PTR" },
  { .key =  CRYS_ECDSA_VERIFY_INVALID_SIGNATURE_SIZE_ERROR               , .data = "ECDSA_VERIFY_INVALID_SIGNATURE_SIZE" },
  { .key =  CRYS_ECDSA_VERIFY_INVALID_MESSAGE_DATA_IN_PTR_ERROR          , .data = "ECDSA_VERIFY_INVALID_MESSAGE_DATA_IN_PTR" },
  { .key =  CRYS_ECDSA_VERIFY_INVALID_MESSAGE_DATA_IN_SIZE_ERROR         , .data = "ECDSA_VERIFY_INVALID_MESSAGE_DATA_IN_SIZE" },
  { .key =  CRYS_ECDSA_VERIFY_USER_CONTEXT_VALIDATION_TAG_ERROR          , .data = "ECDSA_VERIFY_USER_CONTEXT_VALIDATION_TAG" },
  { .key =  CRYS_ECDSA_VERIFY_SIGNER_PUBL_KEY_VALIDATION_TAG_ERROR       , .data = "ECDSA_VERIFY_SIGNER_PUBL_KEY_VALIDATION_TAG" },
  { .key =  CRYS_ECDSA_VERIFY_INCONSISTENT_VERIFY_ERROR                  , .data = "ECDSA_VERIFY_INCONSISTENT_VERIFY" },
  { .key =  CRYS_ECC_ILLEGAL_PARAMS_ACCORDING_TO_PRIV_ERROR              , .data = "ECC_ILLEGAL_PARAMS_ACCORDING_TO_PRIV" },
  { .key =  CRYS_ECC_ILLEGAL_HASH_MODE_ERROR                             , .data = "ECC_ILLEGAL_HASH_MODE" },
  { .key =  CRYS_ECPKI_INVALID_RND_FUNC_PTR_ERROR                        , .data = "ECPKI_INVALID_RND_FUNC_PTR" },
  { .key =  CRYS_ECPKI_INVALID_RND_CTX_PTR_ERROR                         , .data = "ECPKI_INVALID_RND_CTX_PTR" },
  { .key =  CRYS_ECPKI_INVALID_DOMAIN_ID_ERROR                           , .data = "ECPKI_INVALID_DOMAIN_ID" },
  { .key =  CRYS_ECPKI_INVALID_PRIV_KEY_TAG_ERROR                        , .data = "ECPKI_INVALID_PRIV_KEY_TAG" },
  { .key =  CRYS_ECPKI_INVALID_PUBL_KEY_TAG_ERROR                        , .data = "ECPKI_INVALID_PUBL_KEY_TAG" },
  { .key =  CRYS_ECPKI_INVALID_DATA_IN_PASSED_STRUCT_ERROR               , .data = "ECPKI_INVALID_DATA_IN_PASSED_STRUCT" },
  { .key =  CRYS_ECIES_INVALID_PUBL_KEY_PTR_ERROR                        , .data = "ECIES_INVALID_PUBL_KEY_PTR" },
  { .key =  CRYS_ECIES_INVALID_PUBL_KEY_TAG_ERROR                        , .data = "ECIES_INVALID_PUBL_KEY_TAG" },
  { .key =  CRYS_ECIES_INVALID_PRIV_KEY_PTR_ERROR                        , .data = "ECIES_INVALID_PRIV_KEY_PTR" },
  { .key =  CRYS_ECIES_INVALID_PRIV_KEY_TAG_ERROR                        , .data = "ECIES_INVALID_PRIV_KEY_TAG" },
  { .key =  CRYS_ECIES_INVALID_PRIV_KEY_VALUE_ERROR                      , .data = "ECIES_INVALID_PRIV_KEY_VALUE" },
  { .key =  CRYS_ECIES_INVALID_KDF_DERIV_MODE_ERROR                      , .data = "ECIES_INVALID_KDF_DERIV_MODE" },
  { .key =  CRYS_ECIES_INVALID_KDF_HASH_MODE_ERROR                       , .data = "ECIES_INVALID_KDF_HASH_MODE" },
  { .key =  CRYS_ECIES_INVALID_SECRET_KEY_PTR_ERROR                      , .data = "ECIES_INVALID_SECRET_KEY_PTR" },
  { .key =  CRYS_ECIES_INVALID_SECRET_KEY_SIZE_ERROR                     , .data = "ECIES_INVALID_SECRET_KEY_SIZE" },
  { .key =  CRYS_ECIES_INVALID_CIPHER_DATA_PTR_ERROR                     , .data = "ECIES_INVALID_CIPHER_DATA_PTR" },
  { .key =  CRYS_ECIES_INVALID_CIPHER_DATA_SIZE_PTR_ERROR                , .data = "ECIES_INVALID_CIPHER_DATA_SIZE_PTR" },
  { .key =  CRYS_ECIES_INVALID_CIPHER_DATA_SIZE_ERROR                    , .data = "ECIES_INVALID_CIPHER_DATA_SIZE" },
  { .key =  CRYS_ECIES_INVALID_TEMP_DATA_PTR_ERROR                       , .data = "ECIES_INVALID_TEMP_DATA_PTR" },
  { .key =  CRYS_ECIES_INVALID_EPHEM_KEY_PAIR_PTR_ERROR                  , .data = "ECIES_INVALID_EPHEM_KEY_PAIR_PTR" }
};

static lookup_table_t const _str_crys_table =
{
  .count = arrcount(_str_crys_lookup),
  .items = _str_crys_lookup
};

const char* dbg_strerr_crys (int32_t err_id)
{
  return (const char*) lookup_find(&_str_crys_table, (uint32_t) err_id);
}

#endif

