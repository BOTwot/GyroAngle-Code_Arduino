//this code is for tri-wheel base which has omni wheels, use this code in collaboration with an app made by Kushal Shah to update
//the angle readings from gyroscope of an andriod device.USES I2C PROTOCOL.

#include <I2C_Anything.h>
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "FlySkyIBus.h"
#include <Wire.h>
float heading, declinationAngle;
float minimum, maximum;
float getHeading;

#define OUTPUT_READABLE_YAWPITCHROLL

int speedo, Kp = 4, constant = 6, threshold = 220, pwm = 0, corr, flagg = 0; //Keep constant zero and adjust Kp according to the extent you nedd for corrections
float rande;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

int desired_angle = 0;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
void initg() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  pinMode(INTERRUPT_PIN, INPUT);
}
// reset interrupt flag and get INT_STATUS byte
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  IBus.begin(Serial1);
  Wire.beginTransmission(8);

  initg();
  Serial.print("heading:");
  rande = getHeading;
  Serial.println(rande);
  delay(2000);
  flagg = 0;
  minimum = rande ;
  maximum = rande ;
  Serial.print("max");
  Serial.println(maximum);
  desired_angle = rande;
  Serial.print("min");
  Serial.println(minimum);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}
//this function is called when you have to turn clockwise, pwm depends upon the magnitude of stick moved
void cw() {
  Serial.println("cw");
  pwm = abs(IBus.readChannel(3) - 1500);
  pwm = pwm / 3;
  analogWrite(6, pwm);
  analogWrite(9, 0);
  analogWrite(10, 0);
  analogWrite(11, pwm);
  analogWrite(3, pwm);
  analogWrite(5, 0);
  updateangle();
}
//this function is called when you have to turn counter- clockwise, pwm depends upon the magnitude of stick moved
void ccw() {
  Serial.println("ccw");
  pwm = abs( 1500 - IBus.readChannel(3) );

  pwm = pwm / 3;
  analogWrite(9, pwm);
  analogWrite(6, 0);
  analogWrite(11, 0);
  analogWrite(10, pwm);
  analogWrite(5, pwm);
  analogWrite(3, 0);
  updateangle();
}
//this function is called to update the current angle, be sure to call this function frequently to update the current angle
void updateangle() {
  Wire.requestFrom(8, 4);             // request 4 bytes from slave device #8
  if (Wire.requestFrom(8, 4) == 4)
    I2C_readAnything (getHeading);
  if (abs(desired_angle - getHeading) > 150)
  {
    if (desired_angle > 300)
      desired_angle -= 360;
    else if (getHeading > 300)
      getHeading -= 360;

  }
}
void loop() {
  flagg = 0;      //this vavriable serves as flag to get the desired_angle (previous angle) which  is used for corrections
  updateangle();  // calling in loop to get current angle
  IBus.loop();
  //Condition for stop below
  if ((IBus.readChannel(3) >= 1470 && IBus.readChannel(3) <=  1530) &&
      (IBus.readChannel(0) >= 1470 && IBus.readChannel(0) <=  1530) &&
      (IBus.readChannel(1) >= 1470 && IBus.readChannel(1) <=  1530)) {
    Serial.println("stopp");
    updateangle();
    desired_angle = getHeading;
    analogWrite(9, 0);
    analogWrite(6, 0);
    analogWrite(11, 0);
    analogWrite(10, 0);
    analogWrite(5, 0);
    analogWrite(3, 0);
  }
  //rotation clockwise
  else if (IBus.readChannel(3) > 1535) {
    cw();
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
  }
  //rotation anti-clockwise
  else if (IBus.readChannel(3) < 1465) {
    ccw();
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
  }
  flagg = 0;
  //forward
  while (IBus.readChannel(1) > 1535) {
    //////////////
    IBus.loop();
    updateangle();
    Serial.println("fwddd");
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
    pwm = abs(IBus.readChannel(1) - 1500);
    pwm = pwm / 3;
    if (getHeading > desired_angle) {
      speedo = (getHeading - desired_angle) * Kp + constant;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo;
      analogWrite(5, corr);
      analogWrite(3, 0);
    }
    else if (getHeading < desired_angle) {
      speedo = (desired_angle - getHeading) * Kp + constant;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo ;
      analogWrite(3, corr);
      analogWrite(5, 0);
    }
    analogWrite(6, pwm);
    analogWrite(9, 0);
    analogWrite(10, pwm);
    analogWrite(11, 0);
  }
  flagg = 0;
  //back
  while (IBus.readChannel(1) < 1465) {
    IBus.loop();
    updateangle();
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
    Serial.println("revvvvvvvvvvvddd");
    pwm = abs(1500 - IBus.readChannel(1) );
    pwm = pwm / 3;
    if (getHeading > desired_angle) {
      speedo = (getHeading - desired_angle) * Kp + constant ;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo;
      analogWrite(5, corr);
      analogWrite(3, 0);
    }
    else if (getHeading < desired_angle) {
      speedo = (desired_angle - getHeading) * Kp + constant;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo ;
      analogWrite(3, corr);
      analogWrite(5, 0);
    }
    analogWrite(9, pwm);
    analogWrite(6, 0);
    analogWrite(11, pwm);
    analogWrite(10, 0);
  }
  flagg = 0;
  //right
  while (IBus.readChannel(0) > 1535) {
    IBus.loop();
    updateangle();
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
    pwm = abs(IBus.readChannel(0) - 1500);
    pwm = pwm / 3;
    if (getHeading > desired_angle) {
      speedo = (getHeading - desired_angle) * Kp ;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo;

      analogWrite(3, pwm);
      analogWrite(5, 0);
      analogWrite(9, 0.5 * pwm);
      analogWrite(6, 0);
      analogWrite(11, 0);
      analogWrite(10, 0.5 * pwm + corr);
    }
    else if (getHeading < desired_angle) {
      speedo = (desired_angle - getHeading) * Kp ;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo ;
      analogWrite(3, pwm);
      analogWrite(5, 0);
      analogWrite(9, 0.5 * pwm + corr);
      analogWrite(6, 0);
      analogWrite(11, 0);
      analogWrite(10, 0.5 * pwm);
    }


  }
  flagg = 0;
  //left
  while (IBus.readChannel(0) < 1465) {
    IBus.loop();
    updateangle();
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
    Serial.println("left");
    pwm = abs( 1500 - IBus.readChannel(0));
    pwm = pwm / 3;
    if (getHeading > desired_angle) {
      speedo = (getHeading - desired_angle) * Kp;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo;
      analogWrite(3, 0);
      analogWrite(5, pwm);
      analogWrite(6, 0.5 * pwm); //
      analogWrite(9, 0);
      analogWrite(10, 0);
      analogWrite(11, 0.5 * pwm + corr);
    }
    else if (getHeading < desired_angle) {
      speedo = (desired_angle - getHeading) * Kp ;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo ;

      analogWrite(3, 0);
      analogWrite(5, pwm);
      analogWrite(6, 0.5 * pwm + corr); //
      analogWrite(9, 0);
      analogWrite(10, 0);
      analogWrite(11, 0.5 * pwm);

    }

    Serial.println(pwm);
  }



}
