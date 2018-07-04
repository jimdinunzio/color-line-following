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
                                  { LC_BLUE, LC_RED, LC_GREEN, LC_BLACK}, 
                                  { LC_GREEN, LC_BLUE, LC_RED, LC_BLACK }}; 

int lc_follow_index_last = sizeof(lc_follow_order[0]) / sizeof(LineColor) - 1;

// Line color detection logic
#define isItRed(r, g, b) ((r) >= 128 && (r) - (g) > 30)
#define isItBlue(r, g, b) ((b) >= 128)
#define isItGreen(r, g, b) ((g) >= 100 && (g) - (r) > 30)
#define isItBlack(c) ((c) < 150)
#define isItWhite(c) ((c) > 900)

#define MinNumLoopItersVerifyColorChanged 2
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
#define greenLedPin 12
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
      result = ltr < 660 && ltl > 700;
      break;
    default:  // Fall through
    case LC_RED:
      result = ltr < 720 && ltl > 728;
      break;
    case LC_GREEN:
      result = ltr < 660 && ltl > 730;
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
      result = ltl < 720 && ltr > 650;
      break;
    default:  // Fall through
    case LC_RED:
      result = ltl < 740 && ltr > 670;
      break;
    case LC_GREEN:
      result = ltl < 710 && ltr > 670;
      break;
  }
  return result;
}

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);


#define carStartupSpeed 90
#define carSpeed 85
#define carTurningSpeed 150
#define carSearchTurningSpeed 170
#define carLostLineTurningSpeed 170

// For full battery
//#define carStartupSpeed 90
//#define carSpeed 70
//#define carTurningSpeed 135
//#define carSearchTurningSpeed 146

Direction lastDirection = D_FORWARD;

void forward(int speedIn){
  analogWrite(ENA, speedIn);
  analogWrite(ENB, speedIn);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  lastDirection = D_FORWARD;
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
  Serial.println("go right!");
} 

void stop(){
   digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
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

void setup(){
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  
  // use these three pins to drive an LED
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  initVars(0);
  runMode = RM_HALTED;
  tcs.setInterrupt(false);
}

void initVars(int orderIndexIn)
{
  orderIndex = orderIndexIn;
  last_known_lc = lc_follow_order[orderIndexIn][0];
  lastDirection = D_FORWARD;
  runMode = RM_FOLLOWING_LINE;
  lc_follow_index = 0;
  lc_color_changed_count = 0;
  startupSpeedCount = 4;
  searchForLineCount = 0;  
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
  uint16_t clear, red, green, blue;

  char cmd = Serial.read();
  if (cmd == 's') 
  {
    halt();
  }
  else if (cmd == '1' || cmd == 'r')
  {
    initVars(0);
  }
  else if (cmd == '2')
  {
    initVars(1);
  }
  else if (cmd == '3')
  {
    initVars(2);
  }
  if (runMode == RM_HALTED)
  {
    return;    
  }
  
//  tcs.setInterrupt(false);      // turn on LED
  tcs.getRawData(&red, &green, &blue, &clear);
//  tcs.setInterrupt(true);  // turn off LED
  
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
//
//  Serial.print((int)r );
//  Serial.print(" ");
//  Serial.print((int)g);
//  Serial.print(" "); 
//  Serial.print((int)b );
//  Serial.print(" ");
//  Serial.print((int)clear);
//  Serial.print(" ");
//    
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
  delayMicroseconds(150);
  ltl = LT_L;
  delayMicroseconds(150);
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
  Serial.print("LT_R = "); Serial.print(ltr);
  Serial.print(" LT_L = "); Serial.print(ltl);
  printColor(current_lc);
  Serial.print(":  ");
  
  if (runMode == RM_SEARCHING_FOR_COLOR)
  {
	  if (current_lc != lc_follow_order[orderIndex][lc_follow_index])
	  {
	    Serial.println("Searching, but have not found next color.");
      right(carSearchTurningSpeed);
	    return;
	  }
	  else  // current_lc == lc_follow_order[orderIndex][lc_follow_index] 
	  {
      // we've found the new color, return to line following mode
      runMode = RM_FOLLOWING_LINE;
      last_known_lc = lc_follow_order[orderIndex][lc_follow_index]; 
      Serial.println("FOUND next color, now returning to line following mode.");
      startupSpeedCount = 2;    
      // fall through to normal line following code
	  }
  }
  else if (runMode == RM_SEARCHING_FOR_LINE)
  {
    if (current_lc != lc_follow_order[orderIndex][lc_follow_index])
    {
      Serial.print(" Searching, but have not found line.");
      searchForLineCount++;
      if (searchForLineCount < 0)
      {
        right(carLostLineTurningSpeed);
      }
      else if (searchForLineCount < 50)
      {
        left(carLostLineTurningSpeed);
      } 
      else // searchForLineCount > 50
      { 
        searchForLineCount = -50;
        right(carLostLineTurningSpeed);
      }
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
    if (current_lc != LC_WHITE && followingWrongLine(current_lc))
    {
      runMode = RM_VERIFYING_COLOR_CHANGED;
      Serial.println("runMode now VERIFYING_COLOR_CHANGED");
      lc_color_changed_count = 0;
      return;
    }
    else
    {
      right(carTurningSpeed);
    }
  }   
  else if(shouldGoLeft(check_lc, ltl, ltr)) {
    if (current_lc != LC_WHITE && followingWrongLine(current_lc))
    {
      runMode = RM_VERIFYING_COLOR_CHANGED;
      Serial.println("runMode now VERIFYING_COLOR_CHANGED");
      lc_color_changed_count = 0;
      return;      
    }
    else
    {
      left(carTurningSpeed);
    }
  }
  else if(current_lc != LC_WHITE) 
  {
	  if (followingWrongLine(current_lc))
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
		      Serial.println("color changed count ++");
		      if (lc_color_changed_count >= MinNumLoopItersVerifyColorChanged)
		      {
			      runMode = RM_SEARCHING_FOR_COLOR;
			      Serial.println("runMode now SEARCHING_FOR_COLOR");
			      lc_follow_index++;
            lc_color_changed_count = 0;
			      return;
          }
		    }
		    else // current_lc == lc_follow_order[orderIndex][lc_follow_index]
        {
          // false alarm color did not stay changed
		      lc_color_changed_count = 0;
		      runMode = RM_FOLLOWING_LINE;
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
      forward(carSpeed);
    }
//    if (lc_color_changed_count == 0)
      last_known_lc = current_lc;
  } 
  else // current_lc == LC_WHITE 
  {
    // detected white, 
    // if detected white we are lost, so go in the last non-forward direction we were going
    if (lastDirection == D_RIGHT)
    {
      Serial.println("We're lost so trying last direction.");
      right(carTurningSpeed);
    }
    else if (lastDirection == D_LEFT) 
    {
      Serial.println("We're lost so trying last direction.");
      left(carTurningSpeed);
    }
    else 
    {
//      Serial.println("We're lost, halt.");
//      halt();
//      return;
      runMode = RM_SEARCHING_FOR_LINE;
      //stop();
      searchForLineCount = -25;
      Serial.println("We're really lost, runMode now SEARCHING_FOR_LINE");
    }
  }
}

