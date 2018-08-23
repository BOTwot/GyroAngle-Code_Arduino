//this code is for tri-wheel base which has omni wheels, use this code in collaboration with an app made by Kushal Shah to update
//the angle readings from gyroscope of an andriod device.USES I2C PROTOCOL.

#include "AutoPID.h"
#include <I2C_Anything.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "FlySkyIBus.h"
#include <Wire.h>
float getHeading;
double kki = 0.06, kkd = 125, kkp = 2;
double Ki = 0.03, Kd = 125, Kp = 6;
uint8_t minimum = 0, maximum = 175;
#define OUTPUT_READABLE_YAWPITCHROLL

int speedo, pwm1 = 0, pwm2 = 0 , constant = 6, threshold = 220, pwm = 0, corr, flagg = 0; //Keep constant zero and adjust Kp according to the extent you nedd for corrections
float rande;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define NOSE_PIN1 3
#define NOSE_PIN2 5
#define LEFT_PIN1 6
#define LEFT_PIN2 9
#define RIGHT_PIN1 10
#define RIGHT_PIN2 11

int desired_angle = 0;
int rotation = 0;

// reset interrupt flag and get INT_STATUS byte
AutoPID wheel1(&getHeading, &desired_angle, &pwm1, minimum, maximum, Kp, Ki, Kd);
AutoPID wheel2(&getHeading, &desired_angle, &pwm2, minimum, maximum, kkp, kki, kkd);
//AutoPID wheel3(&getHeading, &desired_angle, &pwm1, minimum, maximum, Kp, Ki, Kd);
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  IBus.begin(Serial1);
  Wire.begin();
  wheel1.setTimeStep(50);
  wheel2.setTimeStep(50);
  Wire.beginTransmission(8);
  flagg = 0;
  pinMode(NOSE_PIN1, OUTPUT);
  pinMode(NOSE_PIN2, OUTPUT);
  pinMode(LEFT_PIN1, OUTPUT);
  pinMode(LEFT_PIN2, OUTPUT);
  pinMode(RIGHT_PIN1, OUTPUT);
  pinMode(RIGHT_PIN2, OUTPUT);
}
//this function is called when you have to turn clockwise, pwm depends upon the magnitude of stick moved
void cw() {
  Serial.println("cw");
  pwm = abs(IBus.readChannel(3) - 1500);
  pwm = pwm / 5;
  analogWrite(LEFT_PIN1, pwm);
  analogWrite(LEFT_PIN2, 0);
  analogWrite(RIGHT_PIN1, 0);
  analogWrite(RIGHT_PIN2, pwm);
  analogWrite(NOSE_PIN1, pwm);
  analogWrite(NOSE_PIN2, 0);
  updateangle();
  desired_angle = getHeading;
  rotation = getHeading;
}
//this function is called when you have to turn counter- clockwise, pwm depends upon the magnitude of stick moved
void ccw() {
  Serial.println("ccw");
  pwm = abs( 1500 - IBus.readChannel(3) );

  pwm = pwm / 5;
  analogWrite(LEFT_PIN2, pwm);
  analogWrite(LEFT_PIN1, 0);
  analogWrite(RIGHT_PIN2, 0);
  analogWrite(RIGHT_PIN1, pwm);
  analogWrite(NOSE_PIN2, pwm);
  analogWrite(NOSE_PIN1, 0);
  updateangle();
  desired_angle = getHeading;
  rotation = getHeading;
}
void adjust()
{
  if (getHeading > rotation)
  {
    pwm = 40;
    analogWrite(LEFT_PIN2, pwm);
    analogWrite(LEFT_PIN1, 0);
    analogWrite(RIGHT_PIN2, 0);
    analogWrite(RIGHT_PIN1, pwm);
    analogWrite(NOSE_PIN2, pwm);
    analogWrite(NOSE_PIN1, 0);
    updateangle();
  }
  else if (getHeading < rotation)
  {
    pwm = 40;
    analogWrite(LEFT_PIN1, pwm);
    analogWrite(LEFT_PIN2, 0);
    analogWrite(RIGHT_PIN1, 0);
    analogWrite(RIGHT_PIN2, pwm);
    analogWrite(NOSE_PIN1, pwm);
    analogWrite(NOSE_PIN2, 0);
    updateangle();
  }
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
    while (abs(rotation - getHeading) > 4)
    {
      adjust();
    }
    desired_angle = getHeading;
    analogWrite(LEFT_PIN2, 0);
    analogWrite(LEFT_PIN1, 0);
    analogWrite(RIGHT_PIN2, 0);
    analogWrite(RIGHT_PIN1, 0);
    analogWrite(NOSE_PIN2, 0);
    analogWrite(NOSE_PIN1, 0);

  }
  flagg = 0;
  //rotation clockwise
  if (IBus.readChannel(3) > 1535) {
    cw();
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
  }
  flagg = 0;
  //rotation anti-clockwise
  if (IBus.readChannel(3) < 1465) {
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
    wheel1.run();
    IBus.loop();
    updateangle();
    Serial.println("fwddd");
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
    pwm = abs(IBus.readChannel(1) - 1500);
    pwm = pwm / 2.26;
    if (getHeading > desired_angle) {
      wheel1.run();
      analogWrite(NOSE_PIN2, pwm1);
      analogWrite(NOSE_PIN1, 0);
    }
    else if (getHeading < desired_angle) {
      wheel1.run();
      analogWrite(NOSE_PIN1, pwm1);
      analogWrite(NOSE_PIN2, 0);
    }
    analogWrite(LEFT_PIN1, pwm);
    analogWrite(LEFT_PIN2, 0);
    analogWrite(RIGHT_PIN1, pwm);
    analogWrite(RIGHT_PIN2, 0);
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
    pwm = pwm / 2.26;
    if (getHeading > desired_angle) {
      wheel1.run();
      analogWrite(NOSE_PIN2, pwm1);
      analogWrite(NOSE_PIN1, 0);
    }
    else if (getHeading < desired_angle) {
      wheel1.run();
      analogWrite(NOSE_PIN1, pwm1);
      analogWrite(NOSE_PIN2, 0);
    }
    analogWrite(LEFT_PIN2, pwm);
    analogWrite(LEFT_PIN1, 0);
    analogWrite(RIGHT_PIN2, pwm);
    analogWrite(RIGHT_PIN1, 0);
  }
  flagg = 0;
  //right
  while (IBus.readChannel(0) > 1535) {
    Serial.println("right");
    IBus.loop();
    updateangle();

    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
    pwm = abs(IBus.readChannel(0) - 1500);
    pwm = pwm / 2.26;
    if (getHeading > desired_angle) {
      wheel2.run();
      analogWrite(NOSE_PIN1, pwm);
      analogWrite(NOSE_PIN2, 0);
      analogWrite(LEFT_PIN2, 0.5 * pwm );
      analogWrite(LEFT_PIN1, 0);
      analogWrite(RIGHT_PIN2, 0);
      analogWrite(RIGHT_PIN1, 0.5 * pwm + pwm2);
    }
    else if (getHeading < desired_angle) {
      wheel2.run();
      analogWrite(NOSE_PIN1, pwm + pwm2);
      analogWrite(NOSE_PIN2, 0);
      analogWrite(LEFT_PIN2, 0.5 * pwm );
      analogWrite(LEFT_PIN1, 0);
      analogWrite(RIGHT_PIN2, 0);
      analogWrite(RIGHT_PIN1, 0.5 * pwm);
    }

  }
  flagg = 0;
  //left
  while (IBus.readChannel(0) < 1465) {
    IBus.loop();
    updateangle();
    wheel2.run();
    if (flagg == 0)
    {
      desired_angle = getHeading;
      flagg = 1;
    }
    Serial.println("left");
    pwm = abs( 1500 - IBus.readChannel(0));
    pwm = pwm / 2.26;
    if (getHeading > desired_angle) {
      wheel2.run();
      analogWrite(NOSE_PIN1, 0);
      analogWrite(NOSE_PIN2, pwm + pwm2);
      analogWrite(LEFT_PIN1, 0.5 * pwm); //
      analogWrite(LEFT_PIN2, 0);
      analogWrite(RIGHT_PIN1, 0);
      analogWrite(RIGHT_PIN2, 0.5 * pwm);
    }
    else if (getHeading < desired_angle) {
      wheel2.run();
      analogWrite(NOSE_PIN1, 0);
      analogWrite(NOSE_PIN2, pwm);
      analogWrite(LEFT_PIN1, 0.5 * pwm + pwm2); //
      analogWrite(LEFT_PIN2, 0);
      analogWrite(RIGHT_PIN1, 0);
      analogWrite(RIGHT_PIN2, 0.5 * pwm);

    }


  }



}
