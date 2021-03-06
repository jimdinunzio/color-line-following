/* MIT License

Copyright (c) [2018] [James C DiNunzio, II]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <Adafruit_TCS34725.h>

//www.elegoo.com

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <IRremote.h>

////////// IR REMOTE CODES //////////
#define F 16736925  // FORWARD
#define B 16754775  // BACK
#define L 16720605  // LEFT
#define R 16761405  // RIGHT
#define S 16712445  // STOP
#define UNKNOWN_F 5316027     // FORWARD
#define UNKNOWN_B 2747854299  // BACK
#define UNKNOWN_L 1386468383  // LEFT
#define UNKNOWN_R 553536955   // RIGHT
#define UNKNOWN_S 3622325019  // STOP

#define RECV_PIN  12

enum LineColor {
  LC_BLACK,
  LC_WHITE,
  LC_RED,
  LC_GREEN,
  LC_BLUE,
  LC_UNKNOWN
};

enum Direction {
  D_FORWARD,
  D_LEFT,
  D_RIGHT,
  D_BACK
};

enum RunMode {
  RM_FOLLOWING_LINE,
  RM_VERIFYING_COLOR_CHANGED,
  RM_SEARCHING_FOR_COLOR,
  RM_SEARCHING_FOR_LINE,
  RM_HALTED
};

// order of colored line following
LineColor lc_follow_order[][4] = {{ LC_BLACK, LC_RED, LC_BLUE, LC_GREEN },
                                  { LC_GREEN, LC_BLUE, LC_RED, LC_BLACK }, 
                                  { LC_BLUE, LC_GREEN, LC_GREEN, LC_GREEN}}; 

int lc_follow_index_last = sizeof(lc_follow_order[0]) / sizeof(LineColor) - 1;

// Line color detection logic
#define isItRed(r, g, b) ((r) >= 128 && (r) - (g) > 30)
#define isItBlue(r, g, b) ((b) >= 128)
#define isItGreen(r, g, b) ((g) >= 115 && (g) - (r) > 30 && (b < 105))
#define isItBlack(c) ((c) < 127)
#define isItWhite(c) ((c) > 900)

#define MinNumLoopItersVerifyColorChanged 6
#define MinNumLoopItersVerifyLost 3

// Pins
// Analog
#define AnalogLTL A0
#define AnalogLTR A1

// Digital
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

//Line Tracking IO define, LT_L -> gray wire, LT_R -> orange wire
#define LT_L analogRead(AnalogLTL)
#define LT_R analogRead(AnalogLTR)

// tcs34725 rgb sensor
/* Connect SCL  (white wire)  to analog 5
   Connect SDA  (orange wire)  to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
// redLed -> brown wire
// greenLed -> green wire
// blueLed -> blue wire
#define redLedPin 3
#define greenLedPin 10
#define blueLedPin 13
// for a common anode LED, connect the common pin to +5V -> red wire
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

bool shouldGoRight(LineColor color, int ltl, int ltr)
{
  bool result;
  switch(color)
  {
    case LC_BLACK:
      result = ltr < 530;
      break;
    case LC_BLUE:
      result = ltr < 650 && ltl > 700;
      break;
    default:  // Fall through
    case LC_RED:
      result = ltr < 710 && ltl > 728;
      break;
    case LC_GREEN:
      result = ltr < 650 && ltl > 730;
      break;
  }
  return result;
}

bool shouldGoLeft(LineColor color, int ltl, int ltr)
{
  bool result;
  switch(color)
  {
    case LC_BLACK:
      result = ltl < 660;
      break;
    case LC_BLUE:
      result = ltl < 710 && ltr > 650;
      break;
    default:  // Fall through
    case LC_RED:
      result = ltl < 730 && ltr > 670;
      break;
    case LC_GREEN:
      result = ltl < 710 && ltr > 670;
      break;
  }
  return result;
}

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

#define carStartupSpeed 115
#define carSpeed 100
#define carTurningSpeed 200
#define carSearchTurningSpeed 200
#define carLostLineTurningSpeed 200
#define carCurveInRoadSpeed 80

bool debug = false;
Direction lastDirection = D_FORWARD;

const int interruptPin = 2;
volatile boolean state = false;

//Interrupt Service Routine
void isr() 
{
  state = true;
}

/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

void forward(int speedIn){
  analogWrite(ENA, speedIn);
  analogWrite(ENB, speedIn);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  lastDirection = D_FORWARD;
  if (debug)
    Serial.println("go forward!");
}

void back(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  lastDirection = D_BACK;
  if (debug)
    Serial.println("go back!");
}

void left(int speed){
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  lastDirection = D_LEFT;
  if (debug)
    Serial.println("go left!");
}

void right(int speed){
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  lastDirection = D_RIGHT;
  if (debug)
    Serial.println("go right!");
} 

void stop(){
   digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
   if (debug)
     Serial.println("Stop!");
} 

// initialize last known line color to first in the sequence
int orderIndex = 0;
LineColor last_known_lc = lc_follow_order[orderIndex][0];
RunMode runMode = RM_HALTED; 
int lc_follow_index = 0;
int lc_color_changed_count = 0;
int startupSpeedCount = 4;
int searchForLineCount = 0;
int lostCount = 0;
Direction searchDirection = D_RIGHT;
uint16_t clear = 0, red = 0, green = 0, blue = 0;
int startTime = 0;

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long val;


void setup(){
  // uncomment the following to try using the interrupt feature of the TCS34725 
// pinMode(interruptPin, INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain
// attachInterrupt(digitalPinToInterrupt(interruptPin), isr, FALLING);

  Serial.begin(9600);
  randomSeed(analogRead(3));
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // Set persistence filter to generate an interrupt for every RGB Cycle, regardless of the integration limits
  // uncomment the following to try using the interrupt feature of the TCS34725 
//  tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE); 
//  tcs.setInterrupt(true);
  
  // use these three pins to drive an LED
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  irrecv.enableIRIn();  
  initVars(0);
  runMode = RM_HALTED;
}

void initVars(int orderIndexIn)
{
  orderIndex = orderIndexIn;
  last_known_lc = lc_follow_order[orderIndex][0];
  lastDirection = D_FORWARD;
  runMode = RM_FOLLOWING_LINE;
  lc_follow_index = 0;
  lc_color_changed_count = 0;
  startupSpeedCount = 4;
  searchForLineCount = 0;  
  lostCount = 0;
  searchDirection = random(2) == 0 ? D_RIGHT : D_LEFT;
  clear = 0; red =0; green = 0; blue = 0;
  startTime = 0;
}

void halt()
{
  runMode = RM_HALTED;
  stop();
}

void printColor(int color)
{
  Serial.print(" Color = ");
  if (color == LC_BLACK)
    Serial.print("Black");
  else if (color == LC_WHITE)
    Serial.print("White");
  else if (color == LC_RED)
    Serial.print("Red");
  else if (color == LC_BLUE)
    Serial.print("Blue");
  else if (color == LC_GREEN)
    Serial.print("Green");
  else if (color == LC_UNKNOWN)
    Serial.print("Unknown");
}

bool followingWrongLine(LineColor current_lc)
{
  return (runMode == RM_FOLLOWING_LINE 
          && current_lc != LC_UNKNOWN 
          && current_lc != lc_follow_order[orderIndex][lc_follow_index]);
}

void loop()
{
  LineColor current_lc = LC_UNKNOWN;

  // uncomment the following to try using the interrupt feature of the TCS34725 
//  if (state)
//  {
//    getRawData_noDelay(&red, &green, &blue, &clear);
//    tcs.clearInterrupt();
//    state = false;
//  }

  if (irrecv.decode(&results)){ 
    val = results.value;
    Serial.println(val);
    irrecv.resume();
    switch(val){
      case F: 
      case UNKNOWN_F: initVars(0); break;
      case B: 
      case UNKNOWN_B: initVars(1); break;
      case L: 
      case UNKNOWN_L: debug = !debug; break;
      case R: 
      case UNKNOWN_R: break;
      case S: 
      case UNKNOWN_S: halt(); break;
      default: break;
    }
  }
//  char cmd = Serial.read();
//  if (cmd == 's') 
//  {
//    halt();
//  }
//  else if (cmd == '1' || cmd == 'r')
//  {
//    initVars(0);
//  }
//  else if (cmd == '2')
//  {
//    initVars(1);
//  }
//  else if (cmd == '3')
//  {
//    initVars(2);
//  }
//  else if (cmd == 'd')
//  {
//    debug = !debug;
//  }
  if (runMode == RM_HALTED)
  {
    return;
  }

  // comment out the following if using the interrupt feature of the TCS34725
  tcs.getRawData(&red, &green, &blue, &clear);
  
  //Serial.print("C:\t"); Serial.print(clear);
  //Serial.print("\tR:\t"); Serial.print(red);
  //Serial.print("\tG:\t"); Serial.print(green);
  //Serial.print("\tB:\t"); Serial.print(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  
  //Serial.print("\t");
  //Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  //Serial.println();

  if (debug)
  {
    Serial.print((int)r );
    Serial.print(" ");
    Serial.print((int)g);
    Serial.print(" "); 
    Serial.print((int)b );
    Serial.print(" ");
    Serial.print((int)clear);
    Serial.println(" ");
  }
   
  boolean detectedRed = isItRed(r, g, b);
  boolean detectedGreen = isItGreen(r, g, b);
  boolean detectedBlue = isItBlue(r, g, b);
  boolean detectedBlack = isItBlack(clear);
  boolean detectedWhite = isItWhite(clear);
  
//  Serial.print("detectedRed:\t"); Serial.print(!detectedRed);
//  Serial.print("\tdetectedGreen:\t"); Serial.print(!detectedGreen);
//  Serial.print("\tdetectedBlue:\t"); Serial.print(!detectedBlue);
//  Serial.println();
//  
  digitalWrite(redLedPin, not detectedRed);
  digitalWrite(greenLedPin, not detectedGreen);
  digitalWrite(blueLedPin, not detectedBlue);

  if (detectedRed)
    current_lc = LC_RED;
  else if (detectedBlue)
    current_lc = LC_BLUE;
  else if (detectedGreen)
    current_lc = LC_GREEN;
  else if (detectedBlack)
    current_lc = LC_BLACK;
  else if (detectedWhite)
    current_lc = LC_WHITE;
  else
    current_lc = last_known_lc;

  int ltl, ltr;
  ltl = LT_L;
  ltr = LT_R;

  LineColor check_lc = current_lc;
  // if current line color is unknown or gone to white, use last known color
  if (current_lc == LC_UNKNOWN || current_lc == LC_WHITE)
  {
//    Serial.print("** line color unknown or white, using last known ");
//    printColor(last_known_lc);
//    Serial.println("");
    check_lc = last_known_lc;
  } 
  else // if not white, clear out lost count
  {
    lostCount = 0;
  }

  if (debug)
  {
    Serial.print("LT_R = "); Serial.print(ltr);
    Serial.print(" LT_L = "); Serial.print(ltl);
    printColor(current_lc);
    Serial.print(":  ");
  }
  
  if (runMode == RM_SEARCHING_FOR_COLOR)
  {
	  if (current_lc != lc_follow_order[orderIndex][lc_follow_index])
	  {
	    if (debug)
	      Serial.println("Searching, but have not found next color.");
      if (searchDirection == D_RIGHT)
      {
        right(carSearchTurningSpeed);
      }
      else
      {
        left(carSearchTurningSpeed);
      }
	    return;
	  }
	  else  // current_lc == lc_follow_order[orderIndex][lc_follow_index] 
	  {
      // we've found the new color, return to line following mode
      runMode = RM_FOLLOWING_LINE;
      last_known_lc = lc_follow_order[orderIndex][lc_follow_index]; 
      Serial.print("FOUND next color: ");
      printColor(current_lc);
      Serial.println(". Now returning to line following mode.");
      startupSpeedCount = 2;    
      // fall through to normal line following code
	  }
  }
  else if (runMode == RM_SEARCHING_FOR_LINE)
  {
    if (current_lc != lc_follow_order[orderIndex][lc_follow_index])
    {
      if (debug)
      {
        Serial.print(" Searching, but have not found line color = ");
        printColor(lc_follow_order[orderIndex][lc_follow_index]);
      }
      if (searchForLineCount < 0)
      {
        right(carLostLineTurningSpeed);
      }
      else if (searchForLineCount < 2000)
      {
        left(carLostLineTurningSpeed);
      } 
      else // searchForLineCount > 2000
      { 
        searchForLineCount = -2000;
        right(carLostLineTurningSpeed);
      }
      searchForLineCount += millis() - startTime;
      startTime = millis();
      return;
    }
    else // current_lc == lc_follow_order[orderIndex][lc_follow_index]
    {
      // we've recovered the line!
      searchForLineCount = 0;
      runMode = RM_FOLLOWING_LINE;
      Serial.println("FOUND line, resuming line following.");
      startupSpeedCount = 2;    
      // fall through to normal line following code
    }
  }

 //Logic to detect end blob, not fully tested or confident
  if (lc_follow_index == lc_follow_index_last)
  {
    if (ltl < 660 && ltr < 710)
    { // if last color and hit the color blob, then halt.
      halt();
      return;
    }  
  }

  // Basic line following logic  
  if(shouldGoRight(check_lc, ltl, ltr)) { 
    if (current_lc != LC_WHITE 
        && followingWrongLine(current_lc) 
        && lc_follow_index < lc_follow_index_last)
    {
      runMode = RM_VERIFYING_COLOR_CHANGED;
      Serial.println("runMode now VERIFYING_COLOR_CHANGED");
      lc_color_changed_count = 0;
      return;
    }
    else
    {
      right(carTurningSpeed);
      do {
        ltl = LT_L;
        ltr = LT_R;
      } while (shouldGoRight(check_lc, ltl, ltr));
    }
  }   
  else if(shouldGoLeft(check_lc, ltl, ltr)) {
    if (current_lc != LC_WHITE 
        && followingWrongLine(current_lc) 
        && lc_follow_index < lc_follow_index_last)
    {
      runMode = RM_VERIFYING_COLOR_CHANGED;
      Serial.println("runMode now VERIFYING_COLOR_CHANGED");
      lc_color_changed_count = 0;
      return;      
    }
    else
    {
      left(carTurningSpeed);
      do {
        ltl = LT_L;
        ltr = LT_R;
      } while (shouldGoLeft(check_lc, ltl, ltr));
    }
  }
  else if( /* not left nor right && */ current_lc != LC_WHITE) 
  {
	  if (followingWrongLine(current_lc) 
	      && lc_follow_index < lc_follow_index_last)
	  {
      // Color has changed, verify this over the next n iterations
      runMode = RM_VERIFYING_COLOR_CHANGED;
      Serial.println("runMode now VERIFYING_COLOR_CHANGED");
	    lc_color_changed_count = 0;
      // fall through
	  }
	  else if (runMode == RM_VERIFYING_COLOR_CHANGED)
	  {
      if (current_lc != LC_UNKNOWN)
	    { // if current color != last followed color
        if (current_lc != lc_follow_order[orderIndex][lc_follow_index])
		    {
          lc_color_changed_count++;
          if (debug)
		        Serial.println("color changed count ++");
		      if (lc_color_changed_count >= MinNumLoopItersVerifyColorChanged)
		      {
			      runMode = RM_SEARCHING_FOR_COLOR;
            searchDirection = random(2) == 0 ? D_RIGHT : D_LEFT;
            lc_follow_index = min(lc_follow_index + 1, lc_follow_index_last);
            lc_color_changed_count = 0;
			      Serial.print("runMode now SEARCHING_FOR_COLOR: ");
            printColor(lc_follow_order[orderIndex][lc_follow_index]);
            if (debug)
            {
              Serial.print(", index = ");
              Serial.println(lc_follow_index);
            }
			      return;
          }
		    }
		    else // current_lc == lc_follow_order[orderIndex][lc_follow_index]
        {
          // false alarm color did not stay changed
		      lc_color_changed_count = 0;
		      runMode = RM_FOLLOWING_LINE;
		      if (debug)
		        Serial.println("false alarm, runMode restored to FOLLOWING_LINE");
          // fall through and continue forward
        }
	    }
	  }

    if (startupSpeedCount > 0)
    {
      startupSpeedCount--;
      forward(carStartupSpeed);
    }
    else
    {
      forward(lastDirection != D_FORWARD ? carCurveInRoadSpeed : carSpeed);
    }
      last_known_lc = current_lc;
  } 
  else // current_lc == LC_WHITE 
  {
    // detected white, 
    // if detected white we are lost, so go in the last non-forward direction we were going
    if (lastDirection == D_RIGHT)
    {
      Serial.println("We're lost so trying last direction, right.");
      right(carTurningSpeed);
      lostCount = 0;
    }
    else if (lastDirection == D_LEFT) 
    {
      Serial.println("We're lost so trying last direction, left.");
      left(carTurningSpeed);
      lostCount = 0;
    }
    else 
    {
      if (lostCount > MinNumLoopItersVerifyLost )
      {
        runMode = RM_SEARCHING_FOR_LINE;
        searchForLineCount = -1000;
        startTime = millis();
        right(carLostLineTurningSpeed);
        Serial.println("We're really lost, runMode now SEARCHING_FOR_LINE");
        lostCount = 0;
      }
      else
      {
        lostCount++;
        if (debug)
          Serial.println("We might be lost, lostCount++");
      }
    }
  }
}
