#include "max6675.h"
#include <PID_v1.h>
#include <TimerOne.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Encoder.h>




#define RELAY_PIN             6
#define RELAY_WINDOW          2500

#define THERMO_DO             12
#define THERMO_CS             10
#define THERMO_CLK            13

#define GFX_TIME_PER_PIXEL    1875  //240.000ms/128px

#define ENCODER_CLK           4
#define ENCODER_DT            3
#define ENCODER_SW            2

/*******
0: Standby
1: Ambient -> Preheat
2: Preheat
3: Preheat -> Reflow
4: Reflow
5: Reflow -> Ambient (Off)
********/
int reflow_cycle = 0;

double slope_setpoint;
float previous_temp = 0;
float temp = 0;
double input_slope = 0;
float previous_temps[] = {0,0,0,0,0};
volatile boolean temp_updated = false;
double output_pid_slope;

double temp_setpoint;
volatile float input_isr;
double input_temp;
double output_pid_temp;

float output_pid_series;

int relay_dc = 0;

int output_zero_offset = 300;

long last_telem = 0;

MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

//Specify the links and initial tuning parameters
float Kp_temp=5, Ki_temp=0, Kd_temp=70;
PID PID_temp(&input_temp, &output_pid_temp, &temp_setpoint, Kp_temp, Ki_temp, Kd_temp, DIRECT);

float Kp_slope=200, Ki_slope=0, Kd_slope=0;
PID PID_slope(&input_slope, &output_pid_slope, &slope_setpoint, Kp_slope, Ki_slope, Kd_slope, DIRECT);


unsigned long windowStartTime;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

long time_start = 0;
long last_draw = 0;
int next_pixel_idx = 0;

Encoder encoder(ENCODER_DT, ENCODER_CLK);
int oldPosition  = -999;

void drawFooter(float temperature){
  display.drawLine(0, 53, display.width(), 53, SSD1306_WHITE);

  for(int x=0; x<128; x++){
    for(int y=54; y<64; y++){
      display.drawPixel(x, y, SSD1306_BLACK);
    }
  }

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(10, 55);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
 
  String content = String(temperature);
  content += " --> ";
  content += String(temp_setpoint);
  
  for(int i=0; i<content.length(); i++){
    display.write(content.charAt(i));
  }

  display.display();
  
}

void updateGraph(float temperature){
  
  display.drawPixel(next_pixel_idx, map(temperature,20,250,51,0), SSD1306_WHITE);

  next_pixel_idx++;

  display.display();
}

void fetchData(void){
  input_isr = thermocouple.readCelsius();
  temp_updated = true;
  //Serial.println(input_isr, 2);
}

double calculateLinRegSlope(void){
  double sum_x = 0;
  double sum_y = 0;
  double sum_x2 = 0;
  double sum_y2 = 0;
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
  0: Standby
  1: Ambient -> Preheat
  2: Preheat
  3: Preheat -> Reflow
  4: Reflow
  5: Reflow -> Ambient (Off)
  ********/
  switch (cycle) {
    case 1:
      temp_setpoint = 150;
      slope_setpoint = 1.5;
      break;
    case 2:
      temp_setpoint = 150;
      slope_setpoint = 0;
      break;
    case 3:
      temp_setpoint = 240;
      slope_setpoint = 1.5;
      break;
    case 4:
      temp_setpoint = 240;
      slope_setpoint = 0;
      break;
    case 5:
      temp_setpoint = 25;
      slope_setpoint = -3;
      break;
    default:
      temp_setpoint = 0;
      slope_setpoint = 0;
      break;
  }
}
  
void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  delay(500);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();

  windowStartTime = millis();
  time_start = millis();

  //initialize the variables we're linked to
  changeReflowToCycle(1);

  for(int i = 0; i<5; i++){ previous_temps[i] = thermocouple.readCelsius(); }

  //tell the PID to range between 0 and the full window size
  //PID_temp.SetOutputLimits(-output_zero_offset, RELAY_WINDOW-output_zero_offset);
  PID_temp.SetOutputLimits(0, 100);
  PID_temp.SetSampleTime(RELAY_WINDOW);

  PID_slope.SetOutputLimits(0,100);
  PID_slope.SetSampleTime(RELAY_WINDOW);
  
  // Fetch sensor data every 500ms
  Timer1.initialize(500000);
  Timer1.attachInterrupt(fetchData);

  
  //turn the PID on
  PID_temp.SetMode(AUTOMATIC);
  PID_slope.SetMode(AUTOMATIC);

  
}

void loop() {

  int newPosition = encoder.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
  
  noInterrupts();
  input_temp = input_isr;
  interrupts();

  if(millis() - last_draw > GFX_TIME_PER_PIXEL){
    
    updateGraph(input_temp);
    drawFooter(input_temp);

    last_draw = millis();

    
  }

  if(reflow_cycle == 1 && input_temp >= temp_setpoint) changeReflowToCycle(2);
  
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
  
  relay_dc = map(output_pid_series, 0, 100, 0, RELAY_WINDOW);
  
  //relay_dc = output_pid_temp+output_zero_offset;
  if (millis() - last_telem > 250){
    Serial.print(input_temp);
    Serial.print(" ");
    Serial.println(input_slope*100);

    last_telem = millis();
  }
   
  
  if (millis() - windowStartTime > RELAY_WINDOW)
  { 
    //gradient = (input_temp-previous_temp)/5.0;
    
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

   

    previous_temp = input_temp;

    //time to shift the Relay Window
    windowStartTime = millis();
  }
  
  if (relay_dc > millis() - windowStartTime){
    digitalWrite(RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(RELAY_PIN, LOW);
  }
  
}
