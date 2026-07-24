/*
  PID2DOF.h

  Discrete 2-DOF (setpoint-weighted) PID controller with filtered derivative.

  Implements:
    u = P*(b*r - y) + I*Ts/(z-1)*(r - y) + D*N/(1 + N*Ts/(z-1))*(c*r - y)

  where:
    r  = setpoint (reference)
    y  = measured process value
    b  = setpoint weight on proportional term   (0..1 typical, 1 = classic PID)
    c  = setpoint weight on derivative term      (0..1 typical, 0 = derivative on
         measurement only, which avoids "derivative kick")
    N  = derivative filter divisor (higher N = less filtering, N -> inf = pure D)
    Ts = sample time, seconds

  The integral and derivative terms use the "forward Euler" discretization,
  matching MATLAB/Simulink's pidstd2 / Discrete PID Controller (2DOF) block
  with default IFormula/DFormula = ForwardEuler:

    I(z) = I * Ts / (z - 1)
    D(z) = D * N / (1 + N*Ts/(z-1))

  Call compute() once per sample period Ts.
*/

#ifndef PID2DOF_H
#define PID2DOF_H

#include <Arduino.h>

class PID2DOF {
  public:
    // P, I, D  : proportional / integral / derivative gains
    // N        : derivative filter divisor
    // b, c     : setpoint weights for P and D terms
    // Ts       : sample time in seconds (must match your control loop rate)
    PID2DOF(float P, float I, float D, float N, float b, float c, float Ts);

    // Run one control update. Call this exactly every Ts seconds.
    // r = setpoint, y = measured process value. Returns controller output u.
    float compute(float r, float y);

    // Reset internal integrator/derivative/history state (e.g. after a
    // manual->auto bump, actuator saturation recovery, or long pause).
    void reset();

    // Clamp the controller output. Disabled by default.
    void setOutputLimits(float outMin, float outMax);

    // Basic anti-windup: only accumulate the integrator while the
    // unsaturated output is within limits (clamping / conditional integration).
    void setAntiWindup(bool enabled);

    // Live-update tunings without recreating the object.
    void setGains(float P, float I, float D);
    void setWeights(float b, float c);
    void setFilter(float N);
    void setSampleTime(float Ts);

  private:
    float _P, _I, _D, _N, _b, _c, _Ts;

    float _iState;      // integrator accumulator
    float _eI_prev;      // previous (r - y), used by forward-Euler integrator

    float _dState;      // filtered derivative state
    float _eD_prev;      // previous (c*r - y), used by filtered derivative

    bool  _limitsEnabled;
    float _outMin, _outMax;

    bool  _antiWindup;
};

#endif
