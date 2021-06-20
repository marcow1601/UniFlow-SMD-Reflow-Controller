#include "max6675.h"
#include <PID_v1.h>

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <NeoPixelBus.h>

#include <ESP32Encoder.h>

// General defines

#define RELAY_WINDOW          2500
#define GFX_REFRESH_TIME      1000
#define TEMP_OFFSET           10
#define ALARM_TIMEOUT         10*60*1000   // 10 minutes in milliseconds

#define FORCERESET_EEPROM     false

// ESP32 GPIOs

#define THERMO_DO             19
#define THERMO_CS             5
#define THERMO_CLK            18

#define INDICATORS            4

#define ENCODER_CLK           33
#define ENCODER_DT            34
#define ENCODER_SW            35

#define DIP_4                 32
#define DIP_3                 27
#define DIP_2                 26
#define DIP_1                 25

#define RELAY_PIN             13
#define GPIO_14               14
#define GPIO_15               15
#define GPIO_16               16
#define GPIO_17               17

#define BUZZER                12


  /*******
  0: Config menu
  1: Standby
  2: Ambient -> Preheat
  3: Preheat
  4: Preheat -> Reflow
  5: Reflow
  6: Reflow -> Ambient (Off)
  ********/
bool configurationCompleted = false;
volatile int reflow_cycle = 0;

double disabled_temp = 0;
double standby_temp = 50;
double preheat_temp = 150;
double reflow_temp = 240;
double steady_slope = 0;
double preheat_slope = 1.5;
double critical_slope = 1.5;
double cooldown_slope = -3;

double slope_setpoint;
double input_slope = 0;
double output_pid_slope;

double temp_setpoint;
float previous_temps[] = {0,0,0,0,0};
double input_temp;
double output_pid_temp;

float output_pid_series;

ESP32Encoder encoder;

volatile boolean encClicked = false;
volatile unsigned long lastClick;
#define CLICK_BOUNCE 250

MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

/***
 * Setpoint and tuning parameter struct for persistent storage in EEPROM
 ***/
struct {
  float tempPID[3];
  float slopePID[3];
  
  float setpoints[4][10];
  
} persistence;

const struct {
  float tempPID[3] = {5.7, 0.15, 100.0};
  float slopePID[3] = {200, 0, 0};
  
  float setpoints[4][10] = {{0,50,150,240,0,1.5,1.5,-3,30,60},{0,50,150,240,0,1.5,1.5,-3,30,60},{0,50,150,240,0,1.5,1.5,-3,30,60},{0,50,150,240,0,1.5,1.5,-3,30,60}};
  
} persistenceDefault;

//Specify the PID links and initial tuning parameters
PID PID_temp(&input_temp, &output_pid_temp, &temp_setpoint, persistenceDefault.tempPID[0], persistenceDefault.tempPID[1], persistenceDefault.tempPID[2], DIRECT);
PID PID_slope(&input_slope, &output_pid_slope, &slope_setpoint, persistenceDefault.slopePID[0], persistenceDefault.slopePID[1], persistenceDefault.slopePID[2], DIRECT);

int activeReflowProfile = -1;

long startTime = 0;
unsigned long windowStartTime;
long last_cycle_change = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> indicators(2, INDICATORS);
RgbwColor orange(130, 60, 0, 0);
RgbwColor red(200, 0 , 0 , 0);
RgbwColor white(0, 0, 0, 150);
RgbwColor black(0, 0, 0, 0);

long last_draw = 0;
int next_pixel_idx = 0;

volatile int menu_setting = 0;

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile boolean fetchRequired = false;


void alarm(){

  digitalWrite(RELAY_PIN, LOW);   // Turn off the heating element
  
  while(true){  // can only be exited by resetting the UniFlow Controller
    digitalWrite(BUZZER, HIGH);
    indicators.SetPixelColor(0, white);
    indicators.SetPixelColor(1, red);
    indicators.Show();
    delay(250);

    digitalWrite(BUZZER, LOW);
    indicators.SetPixelColor(0, red);
    indicators.SetPixelColor(1, white);
    indicators.Show();
    delay(250);
    
    yield();  // make sure watchdog does not auto-reset the controller without user action
  }
}

void enc_SW(){
  if((long)(micros() - lastClick) >= CLICK_BOUNCE*1000){
    Serial.println("Click");
    encClicked = true;
    lastClick = micros(); 
  }
  else Serial.println("Bounce");
}

int readEncoder(){
  return (int)encoder.getCount();
}
int readAndResetEncoder(){
  int encoderPos = (int)encoder.getCount();
  encoder.clearCount();
    
  return encoderPos;
}
void setEncoder(int pos){
  encoder.setCount(pos);
}

void drawInterface(){
  display.clearDisplay();

  display.drawRoundRect(0, 0, 128, 43, 5, SSD1306_WHITE);

  display.drawRoundRect(0, 43, 32, 21, 5, SSD1306_WHITE);
  display.drawRoundRect(96, 43, 32, 21, 5, SSD1306_WHITE);
  
  display.setTextSize(4);      
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 8);     

  display.println(input_temp);
  
  display.setTextSize(1);
  display.setCursor(55, 50);

  display.println(temp_setpoint);
    
  display.display();
}

void parameterConfiguration(String pName, float* parameter, float increments){
  int encoderPos = readAndResetEncoder();
  
  while(!encClicked){
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0,0);
    display.println(pName);

    display.setTextSize(3);
    display.setCursor(20,15);
    display.println((*parameter)+increments*readEncoder());
    
    display.display();
 
    yield(); // Prevent watchdog timeout
  }

  *parameter += increments*readAndResetEncoder();
  setEncoder(encoderPos);
  encClicked = false;
}

void configurationMenu(){
  display.clearDisplay();

  /*
   *  Main menu
   */   
  if(menu_setting == 0){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("Main menu"));
    
    display.setTextSize(1);
    
    if(readEncoder()%4 == 0){
      display.fillRoundRect(0, 10, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(0, 10, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,12);
    display.println(F("General settings"));
        
    if(readEncoder()%4 == 1){
      display.fillRoundRect(0, 23, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(0, 23, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,25);
    display.println(F("Setpoints"));
        
    if(readEncoder()%4 == 2){
      display.fillRoundRect(0, 36, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(0, 36, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,38);
    display.println(F("PID parameters"));

    if(readEncoder()%4 == 3){
      display.fillRoundRect(20, 49, 88, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(20, 49, 88, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(25,51);
    display.println(F("Start Reflow"));
      
    if(encClicked){
      encClicked = false;
      if(readEncoder()%4 < 3) menu_setting = readAndResetEncoder()%4 + 1;
      else configurationCompleted = true;
    }
  }

  /*
   * General settings
   */
  else if(menu_setting == 1){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("General settings"));

    if(encClicked){
      encClicked = false;
      menu_setting = 0;
    }
  }

  /*
   * Setpoints
   */
  else if(menu_setting == 2){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("Setpoints"));

    if(readEncoder()%5 == 0){
      display.fillRoundRect(0, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(0, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(6,15);
    display.println(F("Profile 1"));

    if(readEncoder()%5 == 1){
      display.fillRoundRect(64, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(64, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(70,15);
    display.println(F("Profile 2"));

    if(readEncoder()%5 == 2){
      display.fillRoundRect(0, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(0, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(6,34);
    display.println(F("Profile 3"));

    if(readEncoder()%5 == 3){
      display.fillRoundRect(64, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(64, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(70,34);
    display.println(F("Profile 4"));

    if(readEncoder()%5 == 4){
      display.fillRoundRect(20, 49, 88, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(20, 49, 88, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(25,51);
    display.println(F("Back to main"));

    if(encClicked){
      encClicked = false;
      if(readEncoder()%5<4) menu_setting = readAndResetEncoder()%4 + 20;
      else {
        setEncoder(1);
        menu_setting = 0;
      }
    }
  }
  if(menu_setting >= 20 && menu_setting <= 29){

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    if(menu_setting == 20) display.println(F("Setpoints - Profile 1"));
    else if(menu_setting == 21) display.println(F("Setpoints - Profile 2"));
    else if(menu_setting == 22) display.println(F("Setpoints - Profile 3"));
    else if(menu_setting == 23) display.println(F("Setpoints - Profile 4"));

    display.setCursor(0,12);
    display.println(F("<-Save"));
    
    display.drawLine(20,60,40,40,SSD1306_WHITE);
    display.drawLine(40,40,60,40,SSD1306_WHITE);
    display.drawLine(60,40,80,20,SSD1306_WHITE);
    display.drawLine(80,20,105,20,SSD1306_WHITE);
    display.drawLine(105,20,127,42,SSD1306_WHITE);

    if(readEncoder() % 9 == 0) display.drawLine(0,54,19,54,SSD1306_WHITE);
    else if(readEncoder() % 9 == 1) display.drawLine(30,59,50,59,SSD1306_WHITE);
    else if(readEncoder() % 9 == 2) display.drawLine(13,33,33,33,SSD1306_WHITE);
    else if(readEncoder() % 9 == 3) display.drawLine(42,30,62,30,SSD1306_WHITE);
    else if(readEncoder() % 9 == 4) {
      display.setRotation(1);
      display.drawLine(30,48,50,48,SSD1306_WHITE);
      display.setRotation(0);
    }
    else if(readEncoder() % 9 == 5) display.drawLine(53,13,73,13,SSD1306_WHITE);
    else if(readEncoder() % 9 == 6) display.drawLine(87,10,107,10,SSD1306_WHITE);
    else if(readEncoder() % 9 == 7) {
      display.setRotation(1);
      display.drawLine(30,24,50,24,SSD1306_WHITE);
      display.setRotation(0);
    }
    else if(readEncoder() % 9 == 8) {
      display.drawLine(0,21,35,21,SSD1306_WHITE);    
    }

    display.setCursor(1,56);
    display.println(String(int(persistence.setpoints[menu_setting%20][1])) + "C");

    display.setCursor(13,35);
    display.println(String(int(persistence.setpoints[menu_setting%20][2])) + "C");

    display.setCursor(42,32);
    display.println(String(int(persistence.setpoints[menu_setting%20][8])) + "s");

    display.setCursor(53,15);
    display.println(String(int(persistence.setpoints[menu_setting%20][3])) + "C");

    display.setCursor(87,12);
    display.println(String(int(persistence.setpoints[menu_setting%20][9])) + "s");

    //Slopes
    
    display.setCursor(30,50);
    display.println(String(persistence.setpoints[menu_setting%20][5],1));

    display.setRotation(1);
    
    display.setCursor(30,50);
    display.println(String(persistence.setpoints[menu_setting%20][6],1));
    
    display.setCursor(30,15);
    display.println(String(persistence.setpoints[menu_setting%20][7],1));

    display.setRotation(0);
    
    if(encClicked){
      encClicked = false;
      
      if(readEncoder() % 9 == 0) parameterConfiguration(String("Standby temperature"), &persistence.setpoints[menu_setting%20][1], 1.0);
      else if(readEncoder() % 9 == 1) parameterConfiguration(String("Amb.-Preheat slope"), &persistence.setpoints[menu_setting%20][5], 0.1);
      else if(readEncoder() % 9 == 2) parameterConfiguration(String("Preheat temperature"), &persistence.setpoints[menu_setting%20][2], 1.0);
      else if(readEncoder() % 9 == 3) parameterConfiguration(String("Preheat time"), &persistence.setpoints[menu_setting%20][8], 1.0);
      else if(readEncoder() % 9 == 4) parameterConfiguration(String("Pr.heat-Reflow slope"), &persistence.setpoints[menu_setting%20][6], 0.1);
      else if(readEncoder() % 9 == 5) parameterConfiguration(String("Reflow temperature"), &persistence.setpoints[menu_setting%20][3], 1.0);
      else if(readEncoder() % 9 == 6) parameterConfiguration(String("Reflow time"), &persistence.setpoints[menu_setting%20][9], 1.0);
      else if(readEncoder() % 9 == 7) parameterConfiguration(String("Cooldown slope"), &persistence.setpoints[menu_setting%20][7], 0.1);

      else if(readEncoder() % 9 == 8){
        EEPROM.put(0,persistence);
        EEPROM.commit();
        setEncoder(menu_setting%20);
        menu_setting = 2;
      }
    }    
  }


  /*
   * PID parameters
   */
  else if(menu_setting == 3){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    
    display.println(F("Temperature"));
    display.setCursor(70,0);
    display.println(F("Slope"));

     if(readEncoder()%7 == 0){
      display.fillRoundRect(0, 10, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(0, 10, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,12);
    display.println(F("P:"));
    display.setCursor(25,12);
    char p_temp[6];
    dtostrf(persistence.tempPID[0], 4, 2, p_temp);
    display.println(p_temp);
        
    if(readEncoder()%7 == 1){
      display.fillRoundRect(0, 23, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(0, 23, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,25);
    display.println(F("I: "));
    display.setCursor(25,25);
    char i_temp[6];
    dtostrf(persistence.tempPID[1], 4, 2, i_temp);
    display.println(i_temp);
        
    if(readEncoder()%7 == 2){
      display.fillRoundRect(0, 36, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(0, 36, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,38);
    display.println(F("D: "));
    display.setCursor(25,38);
    char d_temp[6];
    dtostrf(persistence.tempPID[2], 4, 2, d_temp);
    display.println(d_temp);

    if(readEncoder()%7 == 3){
      display.fillRoundRect(64, 10, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(64, 10, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(74,12);
    display.println(F("P: "));
    display.setCursor(89,12);
    char p_slope[6];
    dtostrf(persistence.slopePID[0], 4, 2, p_slope);
    display.println(p_slope);

    if(readEncoder()%7 == 4){
      display.fillRoundRect(64, 23, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(64, 23, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(74,25);
    display.println(F("I: "));
    display.setCursor(89,25);
    char i_slope[6];
    dtostrf(persistence.slopePID[1], 4, 2, i_slope);
    display.println(i_slope);

    if(readEncoder()%7 == 5){
      display.fillRoundRect(64, 36, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(64, 36, 64, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(74,38);
    display.println(F("D: "));
    display.setCursor(89,38);
    char d_slope[6];
    dtostrf(persistence.slopePID[2], 4, 2, d_slope);
    display.println(d_slope);

    if(readEncoder()%7 == 6){
      display.fillRoundRect(20, 49, 88, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(20, 49, 88, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(25,51);
    display.println(F("<- Save"));

    
    if(encClicked){
      encClicked = false;

      if(readEncoder()%7 == 0) parameterConfiguration(String("Temp Controller - P"), &persistence.tempPID[0], 0.1);
      else if(readEncoder()%7 == 1) parameterConfiguration(String("Temp Controller - I"), &persistence.tempPID[1], 0.05);
      else if(readEncoder()%7 == 2) parameterConfiguration(String("Temp Controller - D"), &persistence.tempPID[2], 1.0);
      else if(readEncoder()%7 == 3) parameterConfiguration(String("Slope Controller - P"), &persistence.slopePID[0], 1.0);
      else if(readEncoder()%7 == 4) parameterConfiguration(String("Slope Controller - I"), &persistence.slopePID[1], 0.05);
      else if(readEncoder()%7 == 5) parameterConfiguration(String("Slope Controller - D"), &persistence.slopePID[2], 1.0);
      
      else if(readEncoder()%7 == 6){
        EEPROM.put(0,persistence);
        EEPROM.commit();
        
        setEncoder(2);
        menu_setting = 0;
      }
      
    }
  }
}

void IRAM_ATTR fetchData(){
  portENTER_CRITICAL_ISR(&timerMux);
  fetchRequired = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

double calculateLinRegSlope(void){
  double sum_x = 0;
  double sum_y = 0;
  double sum_x2 = 0;
  //double sum_y2 = 0;
  double sum_xy = 0;
  
  double x[]={0,0.5,1.0,1.5,2.0};
  int n = 5;
  
  for(int i = 0; i<5; i++){
    sum_x += x[i];
    sum_y += previous_temps[i];
    sum_x2 += x[i] * x[i];
    //sum_y2 += previous_temps[i] * previous_temps[i];
    //Serial.println(x[i] * previous_temps[i]);
    sum_xy = sum_xy + x[i] * previous_temps[i]; 
  }
  
  double m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
  //double b = (sum_x2 * sum_y - sum_x * sum_xy) / (n * sum_x2 - sum_x * sum_x);
  return m;
}

void changeReflowToCycle(int cycle){
  /*******
  0: Config menu
  1: Standby
  2: Ambient -> Preheat
  3: Preheat
  4: Preheat -> Reflow
  5: Reflow
  6: Reflow -> Ambient (Off)
  ********/
  switch (cycle) {    
    case 1:
      temp_setpoint = persistence.setpoints[activeReflowProfile-1][1];
      slope_setpoint = persistence.setpoints[activeReflowProfile-1][5];
      break;
    case 2:
      temp_setpoint = persistence.setpoints[activeReflowProfile-1][2];
      slope_setpoint = persistence.setpoints[activeReflowProfile-1][5];
      break;
    case 3:
      temp_setpoint = persistence.setpoints[activeReflowProfile-1][2];
      slope_setpoint = persistence.setpoints[activeReflowProfile-1][4];
      break;
    case 4:
      temp_setpoint = persistence.setpoints[activeReflowProfile-1][3];
      slope_setpoint = persistence.setpoints[activeReflowProfile-1][6];
      break;
    case 5:
      temp_setpoint = persistence.setpoints[activeReflowProfile-1][3];
      slope_setpoint = persistence.setpoints[activeReflowProfile-1][4];
      break;
    case 6:
      temp_setpoint = persistence.setpoints[activeReflowProfile-1][1];
      slope_setpoint = persistence.setpoints[activeReflowProfile-1][7];
      break;
    default:
      temp_setpoint = persistence.setpoints[activeReflowProfile-1][0];
      slope_setpoint = persistence.setpoints[activeReflowProfile-1][4];
      break;
  }

  last_cycle_change = millis();
  reflow_cycle = cycle;
}

int getActiveReflowProfile() {
  Serial.println("Determining reflow profile: ");  
  int profile = -1;
  uint8_t dip_1, dip_2, dip_3, dip_4;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  display.println(F("Profile selection"));

  display.drawRect(10, 10, 108, 41, SSD1306_WHITE);
  
  display.drawRect(20, 12, 21, 30, SSD1306_WHITE);
  display.drawRect(41, 12, 21, 30, SSD1306_WHITE);
  display.drawRect(62, 12, 21, 30, SSD1306_WHITE);
  display.drawRect(83, 12, 21, 30, SSD1306_WHITE);

  display.setCursor(29, 43);
  display.println("1");
  display.setCursor(50, 43);
  display.println("2");
  display.setCursor(71, 43);
  display.println("3");
  display.setCursor(92, 43);
  display.println("4");  
  
  display.display();
  
  while(!encClicked || profile < 0){
    if(encClicked) encClicked = false;

    display.fillRect(23, 13, 15, 26, SSD1306_BLACK);
    display.fillRect(44, 13, 15, 26, SSD1306_BLACK);
    display.fillRect(65, 13, 15, 26, SSD1306_BLACK);
    display.fillRect(86, 13, 15, 26, SSD1306_BLACK);
    
    dip_1 = !digitalRead(DIP_1);
    if(dip_1) display.fillRect(23, 14, 15, 13, SSD1306_WHITE);
    else      display.fillRect(23, 26, 15, 13, SSD1306_WHITE);
    
    dip_2 = !digitalRead(DIP_2);
    if(dip_2) display.fillRect(44, 14, 15, 13, SSD1306_WHITE);
    else      display.fillRect(44, 26, 15, 13, SSD1306_WHITE);
    
    dip_3 = !digitalRead(DIP_3);
    if(dip_3) display.fillRect(65, 14, 15, 13, SSD1306_WHITE);
    else      display.fillRect(65, 26, 15, 13, SSD1306_WHITE);
    
    dip_4 = !digitalRead(DIP_4);
    if(dip_4) display.fillRect(86, 14, 15, 13, SSD1306_WHITE);
    else      display.fillRect(86, 26, 15, 13, SSD1306_WHITE);


    display.fillRect(0, 52, 128, 12, SSD1306_BLACK);
    display.setCursor(11,52);
    
    if((dip_1+dip_2+dip_3+dip_4) == 1){
      if(dip_1 == 1) profile = 1;
      else if(dip_2 == 1) profile = 2;
      else if(dip_3 == 1) profile = 3;
      else if(dip_4 == 1) profile = 4;

      display.println(F("Confirm with Click"));
    }
    else if((dip_1+dip_2+dip_3+dip_4) > 1){
      display.println(F("Only one switch up"));
    }
    else {
      display.println(F("Flip one switch up"));
    }

    display.display();

    delay(250);
    yield();
  }
  
  Serial.print("Selected profile: ");
  Serial.println(profile);
  
  return profile;
}
  
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000L);
  delay(500);

  Serial.println("Welcome to UniFlow");
  Serial.flush();

  pinMode(DIP_1, INPUT);
  pinMode(DIP_2, INPUT);
  pinMode(DIP_3, INPUT);
  pinMode(DIP_4, INPUT);
  
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  /*pinMode(GPIO_14, INPUT/OUTPUT);
  pinMode(GPIO_15, INPUT/OUTPUT);
  pinMode(GPIO_16, INPUT/OUTPUT);
  pinMode(GPIO_17, INPUT/OUTPUT);*/

  pinMode(ENCODER_SW, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW), enc_SW, RISING);

  ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachSingleEdge(ENCODER_DT, ENCODER_CLK);
  encoder.clearCount();

  Serial.println("completed!");
  
  /*#################################
   * 0.96" OLED Setup
   ##################################*/

   Serial.print("SSD1306 OLED Initialization... ");
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    //Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  
  display.setTextSize(1);
  display.setCursor(30, 10); 
  display.println(F("Welcome to"));

  display.setTextSize(2);
  display.setCursor(20, 30); 
  display.println(F("UniFlow"));
  
  display.display();

  Serial.println("completed!");

  Serial.println("Bootup sequence completed!");
  
  /*#################################
   * SK6812 LED Setup
   ##################################*/

  indicators.Begin();
  indicators.SetPixelColor(0, white);
  indicators.SetPixelColor(1, black);
  indicators.Show();
  delay(250);
  indicators.SetPixelColor(0, black);
  indicators.SetPixelColor(1, white);
  indicators.Show();
  delay(250);
  indicators.SetPixelColor(0, white);
  indicators.SetPixelColor(1, black);
  indicators.Show();
  delay(250);
  indicators.SetPixelColor(0, black);
  indicators.SetPixelColor(1, white);
  indicators.Show();
  delay(250);
  indicators.SetPixelColor(0, orange);
  indicators.SetPixelColor(1, orange);
  indicators.Show();
  delay(1500);

  
  /*#################################
   * Program initialization
   ##################################*/

  EEPROM.begin(sizeof(persistence));
  EEPROM.get(0,persistence);
  
  if(FORCERESET_EEPROM || !(persistence.tempPID[0]>0 && persistence.tempPID[0]<1000)){
    memcpy(&persistence, &persistenceDefault, sizeof(persistenceDefault));
    EEPROM.put(0,persistence);
    EEPROM.commit();
  }

  Serial.println("EEPROM data retrieved");
  
  //initialize the variables we're linked to
  changeReflowToCycle(0);

  Serial.println("Entering process parameter configuration");

  configurationMenu();
  display.display();

   // In setup/standby mode
  while(reflow_cycle == 0){
    
    configurationMenu();
    configurationMenu();

    display.display();
    

    if(configurationCompleted){
      activeReflowProfile = getActiveReflowProfile();
      changeReflowToCycle(1);
    }
    
    delay(100);
  }

  display.clearDisplay();
  display.display();

  windowStartTime = millis();
  startTime = millis();

  for(int i = 0; i<5; i++){ previous_temps[i] = thermocouple.readCelsius(); }

    /*#################################
   * Setup PID controllers
   ##################################*/

  PID_temp.SetTunings(persistence.tempPID[0], persistence.tempPID[1], persistence.tempPID[2]);
  PID_slope.SetTunings(persistence.slopePID[0], persistence.slopePID[1], persistence.slopePID[2]);
   
  //tell the PID to range between 0 and the full window size
  //PID_temp.SetOutputLimits(-output_zero_offset, RELAY_WINDOW-output_zero_offset);
  PID_temp.SetOutputLimits(0, 100);
  PID_temp.SetSampleTime(RELAY_WINDOW);

  PID_slope.SetOutputLimits(0,100);
  PID_slope.SetSampleTime(RELAY_WINDOW);

  /*#################################
   * Hardware timer setup
   ##################################*/
  timer = timerBegin(0,80,true); // Use timer 0, prescaler 80 => increment each microsecond, count up
  timerAttachInterrupt(timer, &fetchData, true); // Attach ISR, edge interrupt
  timerAlarmWrite(timer, 500000, true); // 500ms timer period, auto reload
  timerAlarmEnable(timer); // enable timer
    
  //turn the PID on
  PID_temp.SetMode(AUTOMATIC);
  PID_slope.SetMode(AUTOMATIC);

  
  
}

void loop() {  
  // Shift last 5 temps to left and add current temp
  if(fetchRequired){

    input_temp = thermocouple.readCelsius() + TEMP_OFFSET;
    portENTER_CRITICAL(&timerMux);
    fetchRequired = false;
    portEXIT_CRITICAL(&timerMux);
    
    for(int i=0; i<4; i++){
      previous_temps[i] = previous_temps[i+1];
    }
    previous_temps[4] = input_temp;

  
    // Calculate average slope of last 5 temps
    input_slope = calculateLinRegSlope();
  }
  

  if(millis() - last_draw > GFX_REFRESH_TIME){
    drawInterface();
   
    last_draw = millis();

    
  }

  if(reflow_cycle == 1 && input_temp >= temp_setpoint && (millis() - last_cycle_change > 60000)) changeReflowToCycle(2);
  else if((reflow_cycle == 2 || reflow_cycle == 4) && input_temp >= temp_setpoint-5) changeReflowToCycle(reflow_cycle+1);
  else if(reflow_cycle == 3 && (millis() - last_cycle_change > 20000)) changeReflowToCycle(4);
  //else if(reflow_cycle == 5 && (millis() - last_cycle_change > 30000)) changeReflowToCycle(6);
    
  
    
  //Serial.println(input, 2);

  // Do PID calculations
  PID_temp.Compute();
  PID_slope.Compute();

  output_pid_series = ((output_pid_temp/100.0) * (output_pid_slope/100.0))*100.0;
  
  int relay_dc = map(output_pid_series, 0, 100, 0, RELAY_WINDOW);
   
  
  if (millis() - windowStartTime > RELAY_WINDOW)
  { 
        
    /*Serial.print(input_temp, 2);
    Serial.print("°C , ");
    Serial.print(input_slope, 4);
    Serial.print("°C/s , ");
    Serial.print(output_pid_temp);
    Serial.print("% , ");
    Serial.print(output_pid_slope);
    Serial.print("% , ");
    Serial.print(relay_dc, 2);
    Serial.println("ms");*/

    //time to shift the Relay Window
    windowStartTime = millis();
  }
  
  if (relay_dc > millis() - windowStartTime){
    digitalWrite(RELAY_PIN,HIGH);
  }
  else{
    digitalWrite(RELAY_PIN,LOW);
  }

  // Trigger alarm state if cycle takes longer than ALARM_TIMEOUT
  if(millis() - startTime >= ALARM_TIMEOUT) alarm();
  
  
}
