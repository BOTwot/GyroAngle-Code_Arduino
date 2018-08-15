#include <Wire.h>
#include <I2C_Anything.h>


String inString = "";
float angle = 0;
unsigned long time1, time2;
//int st;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Serial.begin(9600);

}

void loop() {
  time1 = millis();
  // put your main code here, to run repeatedly:
  while (!Serial.available())
  {
    time2 = millis();
//    Serial.println("IN not ava");
//    //Serial.println(time2);
//    Serial.println(time2-time1);
    if (time2 - time1 > 2000)
    { angle = 500;
      time1 = time2 = 0;
      break;
    }
  }
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar != '\n') {
      inString += (char)inChar;
    }
    else {
      angle = inString.toFloat();
      inString = "";
    }
    Serial.println(angle);
    //  st = (int)angle;
    Wire.onRequest(requestEvent);
  }
  
}

void requestEvent() {
  I2C_writeAnything(angle);
}

