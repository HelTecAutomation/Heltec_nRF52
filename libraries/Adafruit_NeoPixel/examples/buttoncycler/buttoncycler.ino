// Simple demonstration on using an input device to trigger changes on your
// NeoPixels. Wire a momentary push button to connect from ground to a
// digital IO pin. When the button is pressed it will change to a new pixel
// animation. Initial state has all pixels off -- press the button once to
// start the first animation. As written, the button does not interrupt an
// animation in-progress, it works only when idle.

#include <Adafruit_NeoPixel.h>

// Digital IO pin connected to the button. This will be driven with a
// pull-up resistor so the switch pulls the pin to ground momentarily.
// On a high -> low transition the button press logic will execute.
#define BUTTON_PIN  11

#define PIXEL_PIN   14  // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 16  // Number of NeoPixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

boolean oldState = HIGH;
int     mode     = 0;    // Currently-active animation mode, 0-9

void setup() {
  Serial.begin(115200);

  //Pull up button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  //Enable NeoPixels VDD
  pinMode(3,OUTPUT); 
  digitalWrite(3,1);

  //Enable NeoPixels Vext
  pinMode(21,OUTPUT);
  digitalWrite(21,1);

  // Initialize NeoPixel strip object (REQUIRED)
  strip.begin(); 

  // Initialize all pixels to 'off'
  strip.show();  
}

void loop() {
  boolean newState = digitalRead(BUTTON_PIN);

  //Print button status
  Serial.print("Button State: ");
  if (newState == LOW) {
    Serial.println("HIGH (Not Pressed)");
    colorWipe(strip.Color(0, 255, 0));  // green
  } else {
    Serial.println("LOW (Pressed)");
    colorWipe(strip.Color(255, 0, 0));  // red

  //Update button status
  oldState = newState;  
 }
}

void colorWipe(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);  // Set the color of each pixel
  }
  strip.show();  // Display color
}