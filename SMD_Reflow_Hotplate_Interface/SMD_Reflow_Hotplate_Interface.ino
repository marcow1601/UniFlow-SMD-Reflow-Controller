#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

long time_start = 0;
double temp_storage[128] = {0};
long last_storage = 0;

void drawFooter(String content){
  display.drawLine(0, 53, display.width(), 53, SSD1306_WHITE);

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(10, 55);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  for(int i=0; i<content.length(); i++){
    display.write(content.charAt(i));
  }

  display.display();
  
}

void drawGraph(){
  for(int i=0; i<128; i++){
    if(temp_storage[i]>0){
      display.drawPixel(i, map(temp_storage[i],20,250,51,0), SSD1306_WHITE);
    }
  }

  display.display();
}

void addSampleTemp(){
  if(millis()-time_start < 80000){
    temp_storage[int((millis()-time_start)/(240000.0/128.0))] = 20.0+1.5*(millis()-time_start)/1000.0;
  }
  
}

void setup() {
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();

  drawFooter(String(132.50)+" --> "+String(150.0));

  delay(2000);

  time_start = millis();
  
}

void loop() {
 /* display.clear();
  
  if(millis()-last_storage > 240000.0/128.0){
    addSampleTemp();

    last_storage=millis();
  }
*/

}
