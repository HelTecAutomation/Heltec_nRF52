#include "Adafruit_nRFCrypto.h"
#include <Adafruit_TinyUSB.h> // for Serial

/* Input data for testing
 * Same data is stored in input_data.bin file, to verify the result, run hash sum on your PC
 * and compare the result with this sketch
 *
 *  $ sha1sum input_data.bin
 *    9f9da10ec23735930089a8f89b34f7b5d267903e
 *
 *  $ sha224sum input_data.bin
 *    68abe34d09a758be6b2fb3a7a997983a639687099d35406f927a5cc5
 *
 *  $ sha256sum input_data.bin
 *    75cfb39b62c474921e2aad979c210f8b69180a9d58e9f296a4b9904ae6e7aa40
 *
 *  $ sha512sum input_data.bin
 *    e3979c6296e282af04619992f71addfefd118be26626cedd715edced36b87058f868b316e725b24e1e7f661ce2935e44ba4deea62afa3e13188071403a2f1463
 */
uint8_t input_data[] =
{
  0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
  0x65, 0xa2, 0x32, 0xd6, 0xbc, 0xd0, 0xf9, 0x39, 0xed, 0x1f, 0xe1, 0x28, 0xc1, 0x3b, 0x0e, 0x1b
};

nRFCrypto_Hash hash;

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  while( !Serial) delay(10);
  Serial.println("nRFCrypto Hash example");

  nRFCrypto.begin();

  test_hash(CRYS_HASH_SHA1_mode   , "SHA-1");
  test_hash(CRYS_HASH_SHA224_mode , "SHA-224");
  test_hash(CRYS_HASH_SHA256_mode , "SHA-256");
  test_hash(CRYS_HASH_SHA512_mode , "SHA-512");

// Note: SHA384 and MD5 currently cause hardfault
//  test_hash(CRYS_HASH_SHA384_mode , "SHA384");
//  test_hash(CRYS_HASH_MD5_mode    , "MD5");

  nRFCrypto.end();

}

void test_hash(uint32_t mode, const char* modestr)
{
  uint32_t result[16];
  uint8_t  result_len; // depending on Hash mode

  hash.begin(mode);
  hash.update(input_data, sizeof(input_data));
  result_len = hash.end(result);

  Serial.print("                      ");
  Serial.flush();
  Serial.println(modestr);

  Serial.printBuffer( (uint8_t*) result, result_len, ' ', 16);

  Serial.println();
  Serial.println();

  Serial.flush();
}

void loop()
{
  digitalToggle(LED_BUILTIN);
  delay(1000);
}
