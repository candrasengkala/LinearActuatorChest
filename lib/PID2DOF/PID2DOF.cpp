#include "PID2DOF.h"

PID2DOF::PID2DOF(float P, float I, float D, float N, float b, float c, float Ts) {
  _P = P; _I = I; _D = D; _N = N; _b = b; _c = c; _Ts = Ts;
  _limitsEnabled = false;
  _outMin = 0.0f;
  _outMax = 0.0f;
  _antiWindup = true;
  reset();
}

void PID2DOF::reset() {
  _iState  = 0.0f;
  _eI_prev = 0.0f;
  _dState  = 0.0f;
  _eD_prev = 0.0f;
}

void PID2DOF::setOutputLimits(float outMin, float outMax) {
  _limitsEnabled = true;
  _outMin = outMin;
  _outMax = outMax;
}

void PID2DOF::setAntiWindup(bool enabled) {
  _antiWindup = enabled;
}

void PID2DOF::setGains(float P, float I, float D) {
  _P = P; _I = I; _D = D;
}

void PID2DOF::setWeights(float b, float c) {
  _b = b; _c = c;
}

void PID2DOF::setFilter(float N) {
  _N = N;
}

void PID2DOF::setSampleTime(float Ts) {
  _Ts = Ts;
}

float PID2DOF::compute(float r, float y) {
  // --- Proportional term: P*(b*r - y) ---
  float eP = _b * r - y;
  float uP = _P * eP;

  // --- Integral term: I * Ts/(z-1) * (r - y), forward Euler ---
  // y[k] = y[k-1] + Ts * x[k-1]   =>   uses PREVIOUS error sample
  float eI = r - y;
  float iStateNext = _iState + _Ts * _eI_prev;

  // --- Derivative term: D * N/(1 + N*Ts/(z-1)) * (c*r - y), forward Euler ---
  // y[k] = (1 - N*Ts)*y[k-1] + N*(x[k] - x[k-1])
  float eD = _c * r - y;
  float dStateNext = (1.0f - _N * _Ts) * _dState + _N * (eD - _eD_prev);
  float uD = _D * dStateNext;

  // --- Provisional output (integrator not yet committed, for anti-windup check) ---
  float uUnsat = uP + _I * iStateNext + uD;
  float u = uUnsat;

  bool saturated = false;
  if (_limitsEnabled) {
    if (u > _outMax) { u = _outMax; saturated = true; }
    else if (u < _outMin) { u = _outMin; saturated = true; }
  }

  // Commit integrator state. If anti-windup is enabled and the output is
  // saturated, only keep integrating if that continues to push back toward
  // the limit (conditional integration) — otherwise freeze it.
  bool allowIntegration = true;
  if (_antiWindup && saturated) {
    bool pushingFurtherIntoLimit =
      (uUnsat > _outMax && _eI_prev > 0.0f) ||
      (uUnsat < _outMin && _eI_prev < 0.0f);
    allowIntegration = !pushingFurtherIntoLimit;
  }
  _iState = allowIntegration ? iStateNext : _iState;

  // Commit derivative state and history (derivative filter is not subject
  // to windup, so always commit).
  _dState = dStateNext;

  _eI_prev = eI;
  _eD_prev = eD;

  return u;
}
