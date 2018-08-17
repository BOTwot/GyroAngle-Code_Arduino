/*
Rotary Encoder - Polling Example
    
The circuit:
* encoder pin A to Arduino pin 2
* encoder pin B to Arduino pin 3
* encoder ground pin to ground (GND)
*/

#include <MD_REncoder.h>

// set up encoder object
MD_REncoder R = MD_REncoder(2, 3);

void setup() 
{
  Serial.begin(57600);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  R.begin();
}

void loop() 
{
  analogWrite(10,125);
  analogWrite(11,0);
  uint8_t x = R.read();
  
  if (x) 
  {
    Serial.print(x == DIR_CW ? "\n+1" : "\n-1");
#if ENABLE_SPEED
    Serial.print("  ");
    Serial.print(R.speed()*60/70);
#endif


  }
}
