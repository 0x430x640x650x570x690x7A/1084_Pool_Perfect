/*
   Link to product: https://www.makerfocus.com/products/4pcs-i2c-oled-0-96-inch-display-module-with-du-pont-wire-40-pin-for-arduino-uno-r3-stm?_pos=2&_sid=b2a715dd4&_ss=r
   Link to tutorial: https://randomnerdtutorials.com/guide-for-oled-display-with-arduino/

   Details: I2C OLED 0.96 Inch OLED Display Module IIC SSD1306 128 64 LCD White for Arduino UNO R3 STM
*/

//#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

float pHReading = 5.2;
float chlorineReading = 2.5;
byte TemperatureReading = 100;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("BootingSystem");
  display.display();
  delay(2000);
}

void loop() {
  display.setTextSize(2);
  display.clearDisplay();
  display.setCursor(0, 10);
  // Display static text
  //display.println("Main Pump: ");
  //  display.println(statusMainPump);
  //display.println("Chemical Levels: ");
  //  display.println(chemicalLevels);
  display.print("pH:   ");
  display.println(pHReading);
  display.print("Cl:   ");
  display.println(chlorineReading);
    display.print("Temp: ");
  display.println(TemperatureReading);
  display.display();
}
