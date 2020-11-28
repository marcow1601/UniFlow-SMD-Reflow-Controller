#include "max6675.h"
#include <PID_v1.h>

extern "C" {
#include "user_interface.h"
}

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_NeoPixel.h>

#include <MCP23017.h>

// General defines

#define RELAY_WINDOW          2500
#define GFX_REFRESH_TIME      1000
#define TEMP_OFFSET           10
#define MCP_ADDRESS 0x20

// ESP12 GPIOs

#define THERMO_DO             12
#define THERMO_CS             2
#define THERMO_CLK            14

#define INDICATORS            15

#define MCP_INT               13

// MCP23017 GPIOs

#define ENCODER_CLK           0   // GPA0
#define ENCODER_DT            1   // GPA1
#define ENCODER_SW            2   // GPA2

#define DIP_4                 3   // GPA3
#define DIP_3                 4   // GPA4
#define DIP_2                 5   // GPA5
#define DIP_1                 6   // GPA6

#define RELAY_PIN             0   // GPB0
#define GPIO_B1               1   // GPB1
#define GPIO_B2               2  // GPB2
#define GPIO_B3               3  // GPB3
#define GPIO_B4               4  // GPB4


  /*******
  0: Config menu
  1: Standby
  2: Ambient -> Preheat
  3: Preheat
  4: Preheat -> Reflow
  5: Reflow
  6: Reflow -> Ambient (Off)
  ********/
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
volatile boolean temp_updated = false;
volatile float input_isr;
double input_temp;
double output_pid_temp;

float output_pid_series;

MCP23017 mcp(MCP_ADDRESS);

byte intCapReg;
volatile uint8_t encLastInternal = 0;
volatile int16_t encoderPositionISR = 0;
int16_t encoderPosition = 0;

volatile boolean encClicked = false;

MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

//Specify the links and initial tuning parameters
float Kp_temp=5.7, Ki_temp=0.15, Kd_temp=100;     // P 5.7   I 0.1   D  73
PID PID_temp(&input_temp, &output_pid_temp, &temp_setpoint, Kp_temp, Ki_temp, Kd_temp, DIRECT);

float Kp_slope=200, Ki_slope=0, Kd_slope=0;
PID PID_slope(&input_slope, &output_pid_slope, &slope_setpoint, Kp_slope, Ki_slope, Kd_slope, DIRECT);

/***
 * Setpoint and tuning parameter struct for persistent storage in EEPROM
 ***/
struct {
  float tempPID[3];
  float slopePID[3];
  
  float setpoints[4][8];
  
} persistence;

const struct {
  float tempPID[3] = {5.7, 0.15, 100.0};
  float slopePID[3] = {200, 0, 0};
  
  float setpoints[4][8] = {{0,50,150,240,0,1.5,1.5,-3},{0,50,150,240,0,1.5,1.5,-3},{0,50,150,240,0,1.5,1.5,-3},{0,50,150,240,0,1.5,1.5,-3}};
  
} persistenceDefault;

long time_start = 0;
unsigned long windowStartTime;
long last_cycle_change = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
String content;

Adafruit_NeoPixel indicators = Adafruit_NeoPixel(2, INDICATORS, NEO_GRBW + NEO_KHZ800);

long last_draw = 0;
int next_pixel_idx = 0;

volatile int menu_setting = 0;

os_timer_t Timer1;


/*void updateGraph(float temperature){
  
  display.drawPixel(next_pixel_idx, map(temperature,20,250,51,0), SSD1306_WHITE);

  next_pixel_idx++;

  display.display();
}*/

void drawInterface(){
  display.clearDisplay();

  display.drawRoundRect(0, 0, 128, 43, 5, SSD1306_WHITE);

  display.drawRoundRect(0, 43, 32, 21, 5, SSD1306_WHITE);
  display.drawRoundRect(96, 43, 32, 21, 5, SSD1306_WHITE);
  
  display.setTextSize(4);      
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 8);     

  content = String(int(input_temp));

  for(int i=0; i<content.length(); i++){
    display.write(content.charAt(i));
  }
  
  display.setTextSize(1);
  display.setCursor(55, 50);

  content = String(int(temp_setpoint));

  for(int i=0; i<content.length(); i++){
    display.write(content.charAt(i));
  }
  
  display.display();
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
    
    if(readEncoder()%3 == 0){
      display.fillRoundRect(0, 10, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(0, 10, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,12);
    display.println(F("General settings"));
        
    if(readEncoder()%3 == 1){
      display.fillRoundRect(0, 23, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(0, 23, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,25);
    display.println(F("Setpoints"));
        
    if(readEncoder()%3 == 2){
      display.fillRoundRect(0, 36, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    else {
      display.drawRoundRect(0, 36, 128, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(10,38);
    display.println(F("PID parameters"));
      
    if(encClicked){
      delay(150);
      encClicked = false;
      menu_setting = readAndResetEncoder()%3 + 1;
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
      delay(150);
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

    if(readEncoder()%4 == 0){
      display.fillRoundRect(0, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(0, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(6,15);
    display.println(F("Profile 1"));

    if(readEncoder()%4 == 1){
      display.fillRoundRect(64, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(64, 10, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(70,15);
    display.println(F("Profile 2"));

    if(readEncoder()%4 == 2){
      display.fillRoundRect(0, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(0, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(6,34);
    display.println(F("Profile 3"));

    if(readEncoder()%4 == 3){
      display.fillRoundRect(64, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
    }
    else {
      display.drawRoundRect(64, 29, 63, 18, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(70,34);
    display.println(F("Profile 4"));

    if(encClicked){
      delay(150);
      encClicked = false;
      menu_setting = readAndResetEncoder()%4 + 20;
    }
  }

  else if(menu_setting == 20){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("Setpoints - Profile 1"));



    if(encClicked){
      delay(150);
      encClicked = false;
      menu_setting = 2;
    }
  }
  else if(menu_setting == 21){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("Setpoints - Profile 2"));


    if(encClicked){
      delay(150);
      encClicked = false;
      menu_setting = 2;
    }
  }
  else if(menu_setting == 22){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("Setpoints - Profile 3"));


    if(encClicked){
      delay(150);
      encClicked = false;
      menu_setting = 2;
    }
  }
  else if(menu_setting == 23){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("Setpoints - Profile 4"));


    if(encClicked){
      delay(150);
      encClicked = false;
      menu_setting = 2;
    }
  }

  /*
   * PID parameters
   */
  else if(menu_setting == 3){
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("PID parameters"));

    if(encClicked){
      delay(150);
      encClicked = false;
      menu_setting = 0;
    }
  }
  

  display.display();  
}

void fetchData(void *pArg){
  input_isr = thermocouple.readCelsius();
  temp_updated = true;
  //Serial.println(input_isr, 2);
}

void mcpInterruptISR(){
  handleMCPInterrupt(); // Interrupt-event needs to be decoded immediately to catch fast rotation of rotary encoder
}

void handleMCPInterrupt(){
  
  byte intFlagReg, eventPin; 
  intFlagReg = mcp.getIntFlag(A);
  eventPin = log(intFlagReg)/log(2);
  intCapReg = mcp.getIntCap(A);

  if(((~intCapReg)>>2) & 1 == 1) enc_SW();
  
  else {
    uint8_t encInternal = intCapReg & 3;
    if(encLastInternal == 0 && encInternal == 3) ++encoderPositionISR;
    else if(encLastInternal == 2 && encInternal == 1) --encoderPositionISR;

    encLastInternal = encInternal;
  }
  
}


void enc_SW(){
  encClicked = true;
}

int readEncoder(){
  noInterrupts();
  encoderPosition += encoderPositionISR;
  encoderPositionISR = 0;
  interrupts();

  return encoderPosition;
}
int readAndResetEncoder(){
  int encoder;
  noInterrupts();
  encoder = encoderPosition+encoderPositionISR;
  encoderPositionISR = 0;
  interrupts();
  encoderPosition = 0;

  return encoder;
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
      temp_setpoint = standby_temp;
      slope_setpoint = preheat_slope;
      break;
    case 2:
      temp_setpoint = preheat_temp;
      slope_setpoint = preheat_slope;
      break;
    case 3:
      temp_setpoint = preheat_temp;
      slope_setpoint = steady_slope;
      break;
    case 4:
      temp_setpoint = reflow_temp;
      slope_setpoint = critical_slope;
      break;
    case 5:
      temp_setpoint = reflow_temp;
      slope_setpoint = steady_slope;
      break;
    case 6:
      temp_setpoint = standby_temp;
      slope_setpoint = cooldown_slope;
      break;
    default:
      temp_setpoint = disabled_temp;
      slope_setpoint = steady_slope;
      break;
  }

  last_cycle_change = millis();
  reflow_cycle = cycle;
}
  
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000L);
  delay(500);

  Serial.println("Welcome to UniFlow");

  /*#################################
   * MCP23017 I2C Port Expander Setup
   ##################################*/

  Serial.print("MCP23017 Initialization... ");
  mcp.Init();

  pinMode(MCP_INT, INPUT);

  mcp.setPinMode(ENCODER_CLK, A, 0);
  mcp.setPinMode(ENCODER_DT, A, 0);
  mcp.setPinMode(ENCODER_SW, A, 0);

  mcp.setPinMode(DIP_1, A, 0);
  mcp.setPinMode(DIP_2, A, 0);
  mcp.setPinMode(DIP_3, A, 0);
  mcp.setPinMode(DIP_4, A, 0);
  
  mcp.setPinMode(RELAY_PIN, B, 1);
  
  /*mcp.pinMode(GPIO_B1, B, 1);
  mcp.pinMode(GPIO_B2, B, 1);
  mcp.pinMode(GPIO_B3, B, 1);
  mcp.pinMode(GPIO_B4, B, 1);*/

  attachInterrupt(digitalPinToInterrupt(MCP_INT), mcpInterruptISR, FALLING);

  mcp.setInterruptPinPol(LOW); // set INTA and INTB active-low
  delay(10);
  mcp.setInterruptOnChangePort(B00000101, A); // set Port A pins 0 and 2 as interrupt pins
  

  intCapReg = mcp.getIntCap(A); // ensures that existing interrupts are cleared

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
  content = "Welcome to";
  
  for(int i=0; i<content.length(); i++){
    display.write(content.charAt(i));
  }

  display.setTextSize(2);
  display.setCursor(20, 30); 
  content = "UniFlow";

  for(int i=0; i<content.length(); i++){
    display.write(content.charAt(i));
  }
  
  display.display();

  Serial.println("completed!");

  Serial.println("Bootup sequence completed!");
  
  /*#################################
   * SK6812 LED Setup
   ##################################*/

  indicators.begin();
  indicators.setPixelColor(0, indicators.Color(0,0,0,150));
  indicators.setPixelColor(1, indicators.Color(0,0,0,0));
  indicators.show();
  delay(250);
  indicators.setPixelColor(0, indicators.Color(0,0,0,0));
  indicators.setPixelColor(1, indicators.Color(0,0,0,150));
  indicators.show();
  delay(250);
  indicators.setPixelColor(0, indicators.Color(0,0,0,150));
  indicators.setPixelColor(1, indicators.Color(0,0,0,0));
  indicators.show();
  delay(250);
  indicators.setPixelColor(0, indicators.Color(0,0,0,0));
  indicators.setPixelColor(1, indicators.Color(0,0,0,150));
  indicators.show();
  delay(250);
  indicators.setPixelColor(0, indicators.Color(130,60,0,0));
  indicators.setPixelColor(1, indicators.Color(130,60,0,0));
  indicators.show();
  delay(1500);

  
  /*#################################
   * Program initialization
   ##################################*/

  Serial.println("Entering process parameter configuration");

  EEPROM.begin(sizeof(persistence));
  EEPROM.get(0,persistence);
  
  if(!(persistence.tempPID[0]>0 && persistence.tempPID[0]<1000)){
    memcpy(&persistence, &persistenceDefault, sizeof(persistenceDefault));
    EEPROM.put(0,persistence);
    EEPROM.commit();
  }
  
  //initialize the variables we're linked to
  changeReflowToCycle(0);

   // In setup/standby mode
  while(reflow_cycle == 0){
    
    intCapReg = mcp.getIntCap(A); // Make sure MCP interrupts are cleared
    
    configurationMenu();
    
  }

  display.clearDisplay();
  display.display();
  
  windowStartTime = millis();
  time_start = millis();

  

  for(int i = 0; i<5; i++){ previous_temps[i] = thermocouple.readCelsius(); }

  //tell the PID to range between 0 and the full window size
  //PID_temp.SetOutputLimits(-output_zero_offset, RELAY_WINDOW-output_zero_offset);
  PID_temp.SetOutputLimits(0, 100);
  PID_temp.SetSampleTime(RELAY_WINDOW);

  PID_slope.SetOutputLimits(0,100);
  PID_slope.SetSampleTime(RELAY_WINDOW);
  
  // Fetch sensor data every 500ms
  os_timer_setfn(&Timer1, fetchData, NULL);
  os_timer_arm(&Timer1, 500, true);
  
  //turn the PID on
  PID_temp.SetMode(AUTOMATIC);
  PID_slope.SetMode(AUTOMATIC);

  
  
}

void loop() {  
  
  noInterrupts();
  input_temp = input_isr + TEMP_OFFSET;
  interrupts();

  if(millis() - last_draw > GFX_REFRESH_TIME){
    drawInterface();
   
    last_draw = millis();

    
  }

  if(reflow_cycle == 1 && input_temp >= temp_setpoint && (millis() - last_cycle_change > 60000)) changeReflowToCycle(2);
  else if((reflow_cycle == 2 || reflow_cycle == 4) && input_temp >= temp_setpoint-5) changeReflowToCycle(reflow_cycle+1);
  else if(reflow_cycle == 3 && (millis() - last_cycle_change > 20000)) changeReflowToCycle(4);
  //else if(reflow_cycle == 5 && (millis() - last_cycle_change > 30000)) changeReflowToCycle(6);
    
  // Shift last 5 temps to left and add current temp
  if(temp_updated){
    for(int i=0; i<4; i++){
      previous_temps[i] = previous_temps[i+1];
    }
    previous_temps[4] = input_temp;

    temp_updated = false;
  
    // Calculate average slope of last 5 temps
    input_slope = calculateLinRegSlope();
  }
    
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
    mcp.setPin(RELAY_PIN,B,HIGH);
  }
  else{
    mcp.setPin(RELAY_PIN,B,LOW);
  }
  
  
  
  
}
