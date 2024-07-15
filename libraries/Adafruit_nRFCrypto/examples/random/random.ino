#include "Adafruit_nRFCrypto.h"
#include <Adafruit_TinyUSB.h> // for Serial

// Bufer to hold randomized data
uint8_t  buffer[32];

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

//  while(!Serial) delay(10);
  Serial.println("nRFCrypto Random example");
  delay(10);

  nRFCrypto.begin();
}

void loop()
{
  // Generate new random 32 bytes every 1 second
  nRFCrypto.Random.generate(buffer, sizeof(buffer));

  // Print to Serial
  Serial.printBuffer(buffer, sizeof(buffer));
  Serial.println();

  digitalToggle(LED_BUILTIN);
  delay(1000);
}
