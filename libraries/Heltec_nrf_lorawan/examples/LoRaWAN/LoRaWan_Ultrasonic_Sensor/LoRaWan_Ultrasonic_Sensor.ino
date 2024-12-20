#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

const int echoPin = 16;
const int trigPin = 13;

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

void setup(void) {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(PIN_TFT_VDD_CTL, OUTPUT); // TFT VDD ENABLE
  digitalWrite(PIN_TFT_VDD_CTL, 0);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT); // LEDA ENABLE
  digitalWrite(PIN_TFT_LEDA_CTL, 0);

  Serial.print(F("Hello! ST77xx TFT Test"));

  tft.init(135, 240);  // Init ST7789 240x135
  tft.setRotation(3);  // Set screen rotation

  tft.setSPISpeed(40000000);  // SPI speed

  Serial.println(F("Initialized"));

  tft.fillScreen(ST77XX_BLACK);  // Clear the screen
  tft.setTextColor(ST77XX_WHITE);  // Set text color to white
  tft.setTextSize(2);  // Set text size
  tft.setCursor(50, 60);  // Set cursor position
  tft.print(F("Hello"));  // Print "Hello" on the screen
  delay(2000);
}

void loop() {
  // Clear screen and show title
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Ultrasonic Sensor");

  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse width of the echo signal
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (in cm)
  long distance = duration * 0.0343 / 2;

  // Print the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Display the distance on the TFT screen
  tft.setCursor(10, 50);  // Set cursor for the distance text
  tft.fillRect(10, 50, 200, 40, ST77XX_BLACK);  // Clear previous data
  tft.print("Distance: ");
  tft.print(distance);
  tft.println(" cm");

  delay(1000);  // Update once every second
}