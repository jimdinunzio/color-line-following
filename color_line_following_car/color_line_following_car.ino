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
  D_RIGHT
};

enum RunMode {
  RM_FOLLOWING_LINE,
  RM_VERIFYING_COLOR_CHANGED,
  RM_SEARCHING_FOR_COLOR
};

// order of colored line following
LineColor lc_follow_order[] = { LC_BLACK, LC_RED, LC_BLUE, LC_GREEN, LC_WHITE };

// Line color detection logic
#define isItRed(r, g, b) ((r) >= 128 && (r) - (g) > 30)
#define isItBlue(r, g, b) ((b) >= 128)
#define isItGreen(r, g, b) ((g) >= 100 && (g) - (r) > 30)
#define isItBlack(c) ((c) < 2000)
#define isItWhite(c) ((c) > 12000)

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

//Line Tracking IO define
#define LT_L analogRead(AnalogLTL)
#define LT_R analogRead(AnalogLTR)


// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redLedPin 3
#define greenLedPin 12
#define blueLedPin 13
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

bool shouldGoRight(LineColor color, int ltl, int ltr)
{
  bool result;
  switch(color)
  {
    case LC_BLACK:
      result = ltr < 440;
      break;
    case LC_BLUE:
      result = ltr < 580 && ltl > 690;
      break;
    default:  // Fall through
    case LC_RED:
      result = ltr < 630 && ltl > 718;
      break;
    case LC_GREEN:
      result = ltr < 570 && ltl > 720;
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
      result = ltl < 620;
      break;
    case LC_BLUE:
      result = ltl < 660 && ltr > 640;
      break;
    default:  // Fall through
    case LC_RED:
      result = ltl < 690 && ltr > 660;
      break;
    case LC_GREEN:
      result = ltl < 680 && ltr > 660;
      break;
  }
  return result;
}

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define carSpeed 75
#define carTurningSpeed 150

void forward(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

void back(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go back!");
}

void left(){
  analogWrite(ENA, carTurningSpeed);
  analogWrite(ENB, carTurningSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go left!");
}

void right(){
  analogWrite(ENA, carTurningSpeed);
  analogWrite(ENB, carTurningSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  Serial.println("go right!");
} 

void stop(){
   digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
   Serial.println("Stop!");
} 

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
}

void halt()
{
  stop();
  Serial.println("Halt!");
  while(1);
}

LineColor last_known_lc = LC_UNKNOWN;
boolean lastColor = false;
Direction lastDirection = D_FORWARD;
RunMode runMode = RM_FOLLOWING_LINE; 
int lc_follow_index = 0;
int lc_color_changed_count = 0;

void loop()
{
  LineColor current_lc = last_known_lc;
  uint16_t clear, red, green, blue;

  if (Serial.read() == 's') {
    halt();
  }

  tcs.setInterrupt(false);      // turn on LED
  
  if (lastDirection == D_FORWARD)
    delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  
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

  //Serial.print((int)r );
  //Serial.print(" ");
  //Serial.print((int)g);
  //Serial.print(" "); 
  //Serial.print((int)b );
  //Serial.print(" ");
  //Serial.print((int)clear);
  //Serial.print(" ");
    
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

  if (detectedBlack)
    current_lc = LC_BLACK;
  else if (detectedWhite)
    current_lc = LC_WHITE;
  else if (detectedRed)
    current_lc = LC_RED;
  else if (detectedBlue)
    current_lc = LC_BLUE;
  else if (detectedGreen)
    current_lc = LC_GREEN;

  int ltl, ltr;
  ltl = LT_L;
  ltr = LT_R;

  LineColor check_lc = current_lc;
  // if current line color is unknown or gone to white, use last known color
  if (current_lc == LC_UNKNOWN || current_lc == LC_WHITE)
    check_lc = last_known_lc;
    
//  Serial.print("LT_R = "); Serial.print(ltr);
//  Serial.print(" LT_L = "); Serial.print(ltl);
  Serial.print(" Color = ");
  if (current_lc == LC_BLACK)
    Serial.print("Black");
  else if (current_lc == LC_WHITE)
    Serial.print("White");
  else if (current_lc == LC_RED)
    Serial.print("Red");
  else if (current_lc == LC_BLUE)
    Serial.print("Blue");
  else if (current_lc == LC_GREEN)
    Serial.print("Green");
  else if (current_lc == LC_UNKNOWN)
    Serial.print("Unknown");
  Serial.print(":  ");
  
  if (runMode == RM_SEARCHING_FOR_COLOR)
  {
	  if (current_lc != lc_follow_order[lc_follow_index])
	  {
	    Serial.println("Searching, but have not found next color.");
	    right();
	    return;
	  }
	  else  // we've found the new color !
	  {
	    Serial.println("FOUND next color, resuming line following.");
	    runMode = RM_FOLLOWING_LINE;
	    // fall through to normal line following code
	  }
  }
  
  // Basic line following logic  
  if(shouldGoRight(check_lc, ltl, ltr)) { 
    right();
    lastDirection = D_RIGHT;
  }   
  else if(shouldGoLeft(check_lc, ltl, ltr)) {
    left();
    lastDirection = D_LEFT;
  }
  else if(current_lc != LC_WHITE) 
  {
	  if (runMode == RM_FOLLOWING_LINE 
	      && current_lc != LC_UNKNOWN 
	      && current_lc != lc_follow_order[lc_follow_index])
	  {
      // Color has changed, verify this over the next 2 iterations
      runMode = RM_VERIFYING_COLOR_CHANGED;
      Serial.println("runMode now VERIFYING_COLOR_CHANGED");
	    lc_color_changed_count = 1;
	  }
	  else if (runMode == RM_VERIFYING_COLOR_CHANGED)
	  {
      if (current_lc != LC_UNKNOWN)
	    {
        if (current_lc != lc_follow_order[lc_follow_index])
		    {
		      lc_color_changed_count++;
		      Serial.println("color changed count ++");
		      if (lc_color_changed_count > 2)
		      {
			      runMode = RM_SEARCHING_FOR_COLOR;
			      Serial.println("runMode now SEARCHING_FOR_COLOR");
		  	    lc_color_changed_count = 0;
			      lc_follow_index++;
			      right();
			      return;
          }
		    }
		    else
        {
          // false alarm color did not stay changed
		      lc_color_changed_count = 0;
		      runMode = RM_FOLLOWING_LINE;
		      Serial.println("false alarm, runMode restored to FOLLOWING_LINE");
          // fall through and continue forward
        }
	    }
	  }
	
    forward();
    lastDirection = D_FORWARD;
    last_known_lc = current_lc;
  }
  else {
    // detected white, stop, and if at last color halt program
    stop();
    if (lastColor)
    {
      halt();
    }
  }
}

