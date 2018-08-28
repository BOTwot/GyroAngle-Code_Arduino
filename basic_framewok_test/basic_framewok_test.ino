//Libraries Included
#include <FlySkyIBus.h>
#include <Wire.h>
#include "AutoPID.h"
#include <I2C_Anything.h>
#include "GyroRead.h"
//Definitions of pins and Variables
#define NOSE_PIN1 3
#define NOSE_PIN2 5
#define LEFT_PIN1 6
#define LEFT_PIN2 9
#define RIGHT_PIN1 10
#define RIGHT_PIN2 11
//Global Varibles declared and somewhat defined
int stcorr1 = 0, stcorr2 = 0, sdlcorr1 = 0, sdlcorr2 = 0, sdrcorr1 = 0, sdrcorr2 = 0;   //Variables for AutoPID only
double stkp = 6, stki = 0.03, stkd = 125, sdkp = 2, sdki = 0.06, sdkd = 125;    //Kp,Ki,Kd for AutoPID Lib
uint8_t stmax = 100, stmin = 0, sdmax = 100, sdmin = 0;    //Variables for min and max adjust pwm
float curangle;
int prevangle = 0, pwmfw, pwmbk, pwmlt, pwmrt, pwm;
//Objects created for included libraries
GyroRead gyro;
AutoPID straight(&curangle, &prevangle, &stcorr1, &stcorr2, stmin, stmax, stkp, stki, stkd);
AutoPID sidewayl(&curangle, &prevangle, &sdlcorr1, &sdlcorr2, sdmin, sdmax, sdkp, sdki, sdkd);
AutoPID sidewayr(&curangle, &prevangle, &sdrcorr1, &sdrcorr2, sdmin, sdmax, sdkp, sdki, sdkd);
//AutoPID adjust
void setup() {
  Serial.begin(115200);
  pinMode(NOSE_PIN1, OUTPUT);
  pinMode(NOSE_PIN2, OUTPUT);
  pinMode(LEFT_PIN1, OUTPUT);
  pinMode(LEFT_PIN2, OUTPUT);
  pinMode(RIGHT_PIN1, OUTPUT);
  pinMode(RIGHT_PIN2, OUTPUT);
  Wire.begin();
  IBus.begin(Serial1);  //Library Function
  gyro.begin(8);        //Library Function
  straight.setTimeStep(50);     //AutoPID Lib Functon
  sidewayl.setTimeStep(50);      //AutoPID Lib Function
  sidewayr.setTimeStep(50);      //AutoPID Lib Function
}
//Functions declaration and definition


void mapping()      //Call the function for mapping the pwm recieved from the remote
{
  pwm = map(pwm, 0, 500, 0, 255);
  pwmfw = map(pwmfw, 0, 500, 0, 255);
  pwmbk = map(pwmbk, 0, 500, 0, 255);
  pwmlt = map(pwmlt, 0, 500, 0, 255);
  pwmrt = map(pwmrt, 0, 500, 0, 255);

}
void updateangle()
{
  curangle = gyro.getAngle();
  prevangle = curangle;
  if (abs(prevangle - curangle) > 150)
  {
    if (prevangle > 300)
      prevangle -= 360;
    else if (curangle > 300)
      curangle -= 360;
  }
}
void stoped()       //Call the function to stop the movement
{
  analogWrite(LEFT_PIN2, 0);
  analogWrite(LEFT_PIN1, 0);
  analogWrite(RIGHT_PIN2, 0);
  analogWrite(RIGHT_PIN1, 0);
  analogWrite(NOSE_PIN2, 0);
  analogWrite(NOSE_PIN1, 0);
  pwmfw = pwmbk = pwmlt = pwmrt = pwm = 0;

}
void forward()      //Call the function for forward movement, adjust is included
{
  if ((stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2) > 0)
  {
    analogWrite(NOSE_PIN1, stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2);
    analogWrite(NOSE_PIN2, 0);
  }
  else if ((stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2) < 0)
  {
    analogWrite(NOSE_PIN1, 0);
    analogWrite(NOSE_PIN2, stcorr2 + pwmlt + sdlcorr2 - stcorr1 - pwmrt - sdrcorr1 );
  }
  if ((pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk) > 0)
  {
    analogWrite(LEFT_PIN1, pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk);
    analogWrite(LEFT_PIN2, 0);
  }
  else if ((pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk) < 0)
  {
    analogWrite(LEFT_PIN1, 0);
    analogWrite(LEFT_PIN2, 0.5 * pwmrt + pwmbk - pwmfw - 0.5 * pwmlt - sdlcorr1 );
  }
  if ((pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk) > 0)
  {
    analogWrite(RIGHT_PIN1, pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk);
    analogWrite(RIGHT_PIN2,  0);
  }
  else if ((pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk) < 0)
  {
    analogWrite(RIGHT_PIN1, 0);
    analogWrite(RIGHT_PIN2, 0.5 * pwmlt + pwmbk - 0.5 * pwmrt - sdrcorr2 - pwmfw);
  }
}
void back()         //Call the function for back movement, adjust is included
{
  straight.run();                  // AutoPID lib function to update the correction pwm
  analogWrite(NOSE_PIN1, stcorr1);
  analogWrite(NOSE_PIN2, stcorr2);
  analogWrite(LEFT_PIN2, pwm);
  analogWrite(LEFT_PIN1, 0);
  analogWrite(RIGHT_PIN2, pwm);
  analogWrite(RIGHT_PIN1, 0);
}
void left()         //Call the function for left movement, adjust is included
{
  sidewayl.run();                   // AutoPID lib function to update the correction pwm
  analogWrite(NOSE_PIN1, 0);
  analogWrite(NOSE_PIN2, pwm + sdlcorr2);
  analogWrite(LEFT_PIN1, 0.5 * pwm + sdlcorr1);
  analogWrite(LEFT_PIN2, 0);
  analogWrite(RIGHT_PIN1, 0);
  analogWrite(RIGHT_PIN2, 0.5 * pwm);
}
void right()        //Call the function for right movement, adjust is included
{
  sidewayr.run();                  // AutoPID lib function to update the correction pwm
  analogWrite(NOSE_PIN1, pwm + sdrcorr1);
  analogWrite(NOSE_PIN2, 0);
  analogWrite(LEFT_PIN1, 0);
  analogWrite(LEFT_PIN2, 0.5 * pwm );
  analogWrite(RIGHT_PIN1, 0.5 * pwm + sdrcorr2);
  analogWrite(RIGHT_PIN2, 0);
}
void cwise()        //Call the function for clock wise movement
{
  analogWrite(NOSE_PIN1, pwm);
  analogWrite(NOSE_PIN2, 0);
  analogWrite(LEFT_PIN1, pwm);
  analogWrite(LEFT_PIN2, 0);
  analogWrite(RIGHT_PIN1, 0);
  analogWrite(RIGHT_PIN2, pwm);
}
void ccwise()        //Call the function for counter-clock wise movement
{
  analogWrite(NOSE_PIN1, 0);
  analogWrite(NOSE_PIN2, pwm);
  analogWrite(LEFT_PIN1, 0);
  analogWrite(LEFT_PIN2, pwm);
  analogWrite(RIGHT_PIN1, pwm);
  analogWrite(RIGHT_PIN2, 0);
}
void loop() {
  IBus.loop();    //Call always in loop to recieve signal from remote
  //Condition for stop below
  if ((IBus.readChannel(3) >= 1465 && IBus.readChannel(3) <=  1530) &&
      (IBus.readChannel(0) >= 1465 && IBus.readChannel(0) <=  1535) &&
      (IBus.readChannel(1) >= 1465 && IBus.readChannel(1) <=  1535))
  {
    stoped();
    updateangle();
  }
  if (IBus.readChannel(3) > 1535)     //rotation clockwise
  {
    pwm = abs(IBus.readChannel(3) - 1500 );
    mapping();
    cwise();
    updateangle();
  }
  if (IBus.readChannel(3) < 1465)     //rotation anti-clockwise
  {
    pwm = abs( 1500 - IBus.readChannel(3) );
    mapping();
    ccwise();
    updateangle();
  }
  if (IBus.readChannel(1) > 1535)     //forward
  {
    straight.run();
    pwmfw = abs(IBus.readChannel(1) - 1500 );
    mapping();
    Serial.println(pwmfw);
    forward();
    curangle = gyro.getAngle();
  }
  if (IBus.readChannel(1) < 1465)     //back
  {
    straight.run();
    pwmbk = abs(1500 - IBus.readChannel(1) );
    mapping();
    Serial.println(pwmbk);
    forward();
    curangle = gyro.getAngle();
  }
  if (IBus.readChannel(0) > 1535)     //right
  {
    sidewayr.run();
    pwmrt = abs(IBus.readChannel(0) - 1500);
    mapping();
    Serial.println(pwmrt);
    forward();
    curangle = gyro.getAngle();
  }
  if (IBus.readChannel(0) < 1465)     //left
  {
    sidewayl.run();
    pwmlt = abs(1500 - IBus.readChannel(0));
    mapping();
    Serial.println(pwmlt);
    forward();
    curangle = gyro.getAngle();
  }
}
