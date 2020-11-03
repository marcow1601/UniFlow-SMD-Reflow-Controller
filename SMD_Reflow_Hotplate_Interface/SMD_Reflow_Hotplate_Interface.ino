#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();

  delay(2000);
  
}

void loop() {

  display.drawRoundRect(0, 0, 128, 43, 5, SSD1306_WHITE);

  display.drawRoundRect(0, 43, 32, 21, 5, SSD1306_WHITE);
  display.drawRoundRect(96, 43, 32, 21, 5, SSD1306_WHITE);

  display.setTextSize(4);      
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 8);     
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  display.write('2');
  display.write('0');
  display.write('8');

  display.setTextSize(1);
  display.setCursor(55, 50);

  display.write('2');
  display.write('4');
  display.write('0');

 
  display.display();

  delay(1000);


}
