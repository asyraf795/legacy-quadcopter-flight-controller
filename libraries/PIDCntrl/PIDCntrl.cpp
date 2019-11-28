#include "Arduino.h"
#include "PIDCntrl.h"

PIDCntrl::PIDCntrl()
{
  kp = 0.0f;
  ki = 0.0f;
  kd = 0.0f;
  max = 0.0f;
  iTerm = 0.0f;
  lastError = 0.0f;
}

void PIDCntrl::SetParameters(float mKp, float mKi, float mKd, float mMax)
{
  kp = mKp;
  ki = mKi;
  kd = mKd;
  max = mMax;
}

float PIDCntrl::Calculate(float mInput, float mSetpoint)
{
  float error = mSetpoint - mInput;
  iTerm += ki * error;
  if(iTerm > max) {
    iTerm = max;
  } else if (iTerm < -max) {
    iTerm = -max;
  }

  float output = kp * error + iTerm + kd * (error - lastError);
  if(output > max) {
    output = max;
  } else if (output < -max) {
    output = -max;
  }

  lastError = error;
  return output;
}

void PIDCntrl::ResetMem()
{
  iTerm = 0.0f;
  lastError = 0.0f;
}
