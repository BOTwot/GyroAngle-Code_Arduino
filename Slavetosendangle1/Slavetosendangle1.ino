#include <Wire.h>
#include <I2C_Anything.h>
String inString = "";
float angle = 0;
//int st;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Serial.begin(9600);
}
void requestEvent() {
  I2C_writeAnything(angle);
}
void loop() {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar != '\n') {
      inString += (char)inChar;
    }
    else {
      angle = inString.toFloat();
      inString = "";
    }
    Wire.onRequest(requestEvent);
  } 
}

