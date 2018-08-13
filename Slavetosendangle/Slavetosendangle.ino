#include <Wire.h>
#include <I2C_Anything.h>


String inString = "";
float angle;
//int st;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar != '\n') {
      inString += (char)inChar;
    }
    else {
      angle = inString.toFloat();
      inString = "";
    }
  //  st = (int)angle;
    Wire.onRequest(requestEvent);
  }
}

void requestEvent() {
  I2C_writeAnything(angle);
}

