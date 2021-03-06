#ifndef AUTOPID_H
#define AUTOPID_H
#include <Arduino.h>

class AutoPID {
  private:
  
    double _Kp, _Ki, _Kd;
    double _integral, _previousError;

    float *_input;
    int *_setpoint,*_output;
    uint8_t _outputMin, _outputMax;
    
    unsigned long _timeStep, _lastStep;

  public:
     AutoPID(float *input, int *setpoint, int *output, uint8_t outputMin, uint8_t outputMax, double Kp, double Ki, double Kd);
     void setGains(double Kp, double Ki, double Kd);
     void setOutputRange(uint8_t outputMin,uint8_t outputMax);
     void setTimeStep(unsigned long timeStep);
     void run();
     void reset();


};//class AutoPID

#endif
