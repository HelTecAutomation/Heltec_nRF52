// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library
// This sketch shows use of the "new" operator with Adafruit_NeoPixel.
// It's helpful if you don't know NeoPixel settings at compile time or
// just want to store this settings in EEPROM or a file on an SD card.

#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
int pin = 14; // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
int numPixels = 2; // Popular NeoPixel ring size

// NeoPixel color format & data rate. See the strandtest example for
// information on possible values.
int pixelFormat = NEO_GRB + NEO_KHZ800;

// Rather than declaring the whole NeoPixel object here, we just create
// a pointer for one, which we'll then allocate later...
Adafruit_NeoPixel *pixels;

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

void setup() {

  pinMode(21,OUTPUT);//Enable Vext
  digitalWrite(21,1);
 
  // Then create a new NeoPixel object dynamically with these values:
  pixels = new Adafruit_NeoPixel(numPixels, pin, pixelFormat);

  // Going forward from here, code works almost identically to any other
  // NeoPixel example, but instead of the dot operator on function calls
  // (e.g. pixels.begin()), we instead use pointer indirection (->) like so:
  pixels->begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  // You'll see more of this in the loop() function below.
}

void loop() {

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<numPixels; i++) { 

    pixels->clear(); // Set all pixel colors to 'off'
    // pixels->Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels->setPixelColor(i, pixels->Color(255, 0, 0)); //Red

    pixels->show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop

  }

  for(int i=0; i<numPixels; i++) { 
    pixels->clear();
    pixels->setPixelColor(i, pixels->Color(0, 255, 0)); //Green
    pixels->show();   
    delay(DELAYVAL); 

  }
  for(int i=0; i<numPixels; i++) { 
    pixels->clear();
    pixels->setPixelColor(i, pixels->Color(0, 0, 255)); //Blue
    pixels->show();   
    delay(DELAYVAL); 
    
  }
}
