#include <I2C_Anything.h>

#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "FlySkyIBus.h"

#include <Wire.h>
float heading, declinationAngle;
float minimum, maximum;
float getHeading;
//MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

int speedo, Kp = 20, constant = 9, threshold = 220, pwm = 50, corr;
float rande;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


int desired_angle;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//  mpuInterrupt = true;
//}

void initg() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  //Serial.begin(115200);

  
  pinMode(INTERRUPT_PIN, INPUT);


//  devStatus = mpu.dmpInitialize();

}

float vv;

  // reset interrupt flag and get INT_STATUS byte


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);
IBus.begin(Serial);
Wire.beginTransmission(8);

  initg();
  Serial.print("heading:");
  rande = getHeading;
  Serial.println(rande);
  delay(2000);

  minimum = rande ;
  maximum = rande ;
  Serial.print("max");
  Serial.println(maximum);
  desired_angle = rande;
  Serial.print("min");
  Serial.println(minimum);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  analogWrite(6, pwm);
  analogWrite(9, 0);
  analogWrite(10, pwm);
  analogWrite(11, 0);
}
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
}
void ccw() {
  Serial.println("ccw");
  pwm = abs( 1500-IBus.readChannel(3) );
 
  pwm = pwm / 3;
  analogWrite(9, pwm);
  analogWrite(6, 0);
  analogWrite(11, 0);
  analogWrite(10, pwm);
  analogWrite(5, pwm);
  analogWrite(3, 0);
}
void nose_cw() {
  Serial.println("nose cw");
  pwm = abs(IBus.readChannel(0) - 1500);
  pwm = pwm / 3;
  analogWrite(3, pwm);
  analogWrite(5, 0);
}
void nose_ccw() {
  Serial.println("nose ccw");
  pwm = abs( 1500-IBus.readChannel(0));
  pwm = pwm / 3;
  analogWrite(3, 0);
  analogWrite(5, pwm);
}

void loop() {
  
  Wire.requestFrom(8,4);              // request 4 bytes from slave device #8
  if (Wire.requestFrom(8, 4) == 4)
  I2C_readAnything (getHeading);   
  IBus.loop();
  //       Serial.print(receiver_input[1]);
  //        Serial.print("--");
  //
  //      Serial.print(receiver_input[2]);
  //          Serial.print("--");
  //
  //      Serial.print(receiver_input[3]);
  //          Serial.print("--");
  //
  //      Serial.println(receiver_input[4]);

  if ((IBus.readChannel(3) >= 1470 && IBus.readChannel(3) <=  1530) &&
      (IBus.readChannel(0) >= 1470 && IBus.readChannel(0)<=  1530) &&
      (IBus.readChannel(1) >= 1470 && IBus.readChannel(1) <=  1530)) {
          Serial.println("stoppppppppppppppp");
    desired_angle = getHeading;
    analogWrite(9, 0);
    analogWrite(6, 0);
    analogWrite(11, 0);
    analogWrite(10, 0);
    analogWrite(5, 0);
    analogWrite(3, 0);
  }
  else if (IBus.readChannel(3) > 1530) {
    cw();
  }
  else if (IBus.readChannel(3) < 1470) {
    ccw();
  }
  else if (IBus.readChannel(0) >  1530) {
    nose_cw();
  }
  else if (IBus.readChannel(0) < 1470) {
    nose_ccw();
  }
  else if (IBus.readChannel(1) < 1470) {
    //////////////
    Serial.println("fwddd");
    pwm = abs(1500-IBus.readChannel(1) );
    pwm = pwm / 3;
    if (getHeading > desired_angle) {
      speedo = (getHeading - desired_angle) * Kp + constant;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo;
      analogWrite(3, corr);
      analogWrite(5, 0);
      //        Serial.print("+")  ;
      //    Serial.println(corr);

    }
    else if (getHeading < desired_angle) {
      speedo = (desired_angle - getHeading) * Kp + constant;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo ;

      analogWrite(5, corr);
      analogWrite(3, 0);
      //    Serial.print("-")  ;
      //    Serial.println(corr);

    }
    analogWrite(6, pwm);
    analogWrite(9, 0);
    analogWrite(10, pwm);
    analogWrite(11, 0);


  }
  else if (IBus.readChannel(1) >  1530) {
    //////////
    Serial.println("revvvvvvvvvvvddd");
    pwm = abs( IBus.readChannel(1) - 1500);
    pwm = pwm / 3;
    if (getHeading > desired_angle) {
      speedo = (getHeading - desired_angle) * Kp + constant;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo;
      analogWrite(3, corr);
      analogWrite(5, 0);
      //        Serial.print("+")  ;
      //    Serial.println(corr);

    }
    else if (getHeading < desired_angle) {
      speedo = (desired_angle - getHeading) * Kp + constant;
      if (speedo > threshold) {
        speedo = threshold;
      }
      corr = speedo ;

      analogWrite(5, corr);
      analogWrite(3, 0);
      //    Serial.print("-")  ;
      //    Serial.println(corr);

    }
    analogWrite(9, pwm);
    analogWrite(6, 0);
    analogWrite(11, pwm);
    analogWrite(10, 0);
  }


  //    Serial.print(receiver_input[1]);
  //      Serial.print("--");
  //
  //    Serial.print(receiver_input[2]);
  //        Serial.print("--");
  //
  //    Serial.print(receiver_input[3]);
  //        Serial.print("--");
  //
  //    Serial.println(receiver_input[4]);


}

/*
  void loop() {

  //  while(1);
  //  Serial.println(getHeading);
  if (getHeading > rande) {
    speedo = (getHeading - rande) * Kp + constant;
    if (speedo > threshold) {
      speedo = threshold;
    }
    corr = speedo;
    analogWrite(3, corr);
    analogWrite(5, 0);
    //        Serial.print("+")  ;
    //    Serial.println(corr);

  }
  else if (getHeading < rande) {
    speedo = (rande - getHeading) * Kp + constant;
    if (speedo > threshold) {
      speedo = threshold;
    }
    corr = speedo ;

    analogWrite(5, corr);
    analogWrite(3, 0);
    //    Serial.print("-")  ;
    //    Serial.println(corr);

  }
  }*/
