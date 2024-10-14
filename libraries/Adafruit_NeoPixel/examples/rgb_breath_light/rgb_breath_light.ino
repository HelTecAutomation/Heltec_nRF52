// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#define PIN        14 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 2 // Popular NeoPixel ring size

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// #define ORANGE 0
// #define RED 1
// #define BLUE_GREEN 2
// #define GREEN 3
typedef enum
{
    ORANGE,
    RED,
    BLUE_GREEN,
    GREEN
}Color_t;

Color_t color;

#define VALUE 51

void RGB_Color(uint8_t color, uint8_t value)
{
  switch(color)
  {
    case ORANGE:
    {
      pixels.setPixelColor(0, pixels.Color(value * 5, value, 0));
      pixels.setPixelColor(1, pixels.Color(value * 5, value, 0));
      pixels.show();
      break;
    }
    case RED:
    {
      pixels.setPixelColor(0, pixels.Color(value * 5, 0, 0));
      pixels.setPixelColor(1, pixels.Color(value * 5, 0, 0));
      pixels.show();
      break;
    }
    case BLUE_GREEN:
    {
      pixels.setPixelColor(0, pixels.Color(0, value * 5, value * 2));
      pixels.setPixelColor(1, pixels.Color(0, value * 5, value * 2));
      pixels.show();
      break;
    }
      case GREEN:
    {
      pixels.setPixelColor(0, pixels.Color(0, value * 5, 0));
      pixels.setPixelColor(1, pixels.Color(0, value * 5, 0));
      pixels.show();
      break;
    }
  }
}

void RGB_Breath(uint8_t color, uint8_t value)
{
  uint8_t i = 0;

  while(i <= value)
  {
    RGB_Color(color, i++);
    delay(50);
    if(i == value)
    {
      while(i <= value)
      {
        RGB_Color(color, i--);
        delay(50);
      }
    }
  }
}

void setup() {
  pinMode(21,OUTPUT);//Enable Vext
  digitalWrite(21,1);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
}

void loop()
{
  RGB_Color(RED, VALUE);
  delay(2000);
  RGB_Color(ORANGE, VALUE);
  delay(2000);
  RGB_Color(BLUE_GREEN, VALUE);
  delay(2000);
  RGB_Color(GREEN, VALUE);
  delay(2000);
  RGB_Breath(RED, VALUE);
  RGB_Breath(ORANGE, VALUE);
  RGB_Breath(BLUE_GREEN, VALUE);
  RGB_Breath(GREEN, VALUE);
}
