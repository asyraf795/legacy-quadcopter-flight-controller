#ifndef PIDCntrl_h
#define PIDCntrl_h

#include "Arduino.h"

class PIDCntrl
{
  public:

  PIDCntrl();
  void SetParameters(float mKp, float mKi, float mKd, float mMax);
  float Calculate(float mInput, float mSetpoint);
  void ResetMem();

  private:
  float kp;
  float ki;
  float kd;
  float max;
  float iTerm;
  float lastError;
};


#endif
