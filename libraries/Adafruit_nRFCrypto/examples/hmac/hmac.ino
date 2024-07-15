#include "Adafruit_nRFCrypto.h"
#include <Adafruit_TinyUSB.h> // for Serial

/* Input data for testing
 * to verify the result, run openssl on your PC
 * and compare the result with this sketch
 *
 *  $ echo -ne "Hello World!" | openssl dgst -sha1 -hmac "secret"
 *    (stdin)= 5efed98b0787c83f9cb0135ba283c390ca49320e
 *
 *  $ echo -ne "Hello World!" | openssl dgst -sha224 -hmac "secret"
 *    (stdin)= d1f9ff720c7558366f2b67e672b4f4ee7cb58b5803cd9e46380123f2
 *
 *  $ echo -ne "Hello World!" | openssl dgst -sha256 -hmac "secret"
 *    (stdin)= 6fa7b4dea28ee348df10f9bb595ad985ff150a4adfd6131cca677d9acee07dc6
 *
 *  $ echo -ne "Hello World!" | openssl dgst -sha512 -hmac "secret"
 *    (stdin)= fd3716eb8e1903ecb8deb9320f1baecf0f8dbe04e4c4c71e64fbb37080222ca42deb0b646cabeb53e269a2446507fb5442961656f8007959bd330ef48df0e13e
*/
uint8_t input_data[] = // "Hello World!"
{
  0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20,
  0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21
};

uint8_t key[] = // "secret"
{
  0x73, 0x65, 0x63, 0x72, 0x65, 0x74
};

nRFCrypto_Hmac hmac;

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  while( !Serial) delay(10);
  Serial.println("nRFCrypto Hmac example");

  nRFCrypto.begin();

  test_hmac(CRYS_HASH_SHA1_mode   , "SHA-1");
  test_hmac(CRYS_HASH_SHA224_mode , "SHA-224");
  test_hmac(CRYS_HASH_SHA256_mode , "SHA-256");
  test_hmac(CRYS_HASH_SHA512_mode , "SHA-512");

  nRFCrypto.end();
}

void test_hmac(CRYS_HASH_OperationMode_t mode, const char* modestr)
{
  uint32_t result[16];
  uint8_t  result_len; // depending on Hash mode

  hmac.begin(mode, key, sizeof(key));
  hmac.update(input_data, sizeof(input_data));
  result_len = hmac.end(result);

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
