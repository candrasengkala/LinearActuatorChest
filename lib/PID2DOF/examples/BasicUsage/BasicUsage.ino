/*
  BasicUsage.ino

  Minimal example: linear actuator position control using PID2DOF.

  Wiring assumption (adjust to your setup):
    - Potentiometer feedback on A0 (0..1023 -> position in your units)
    - PWM drive to motor driver on pin 3 (0..255)

  Tune P, I, D, N, b, c to your system (see notes below).
*/

#include "PID2DOF.h"

// Tunings — replace with your identified/tuned values.
const float P  = 2.0f;
const float I  = 0.8f;
const float D  = 0.05f;
const float N  = 20.0f;   // derivative filter divisor
const float b  = 1.0f;    // 1.0 = full proportional action on setpoint changes
const float c  = 0.0f;    // 0.0 = derivative on measurement only (no D-kick)
const float Ts = 0.01f;   // 10 ms loop -> 100 Hz. MUST match your loop timing.

PID2DOF pid(P, I, D, N, b, c, Ts);

const unsigned long LOOP_MS = (unsigned long)(Ts * 1000.0f);
unsigned long lastLoop = 0;

float setpoint = 512.0f;  // target position, same units as feedback

void setup() {
  Serial.begin(115200);
  pinMode(3, OUTPUT);

  pid.setOutputLimits(-255.0f, 255.0f); // e.g. signed PWM for bidirectional drive
  pid.setAntiWindup(true);
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  lastLoop = now;

  float y = analogRead(A0);              // measured position
  float u = pid.compute(setpoint, y);    // controller output

  // Drive motor (example: sign = direction, magnitude = PWM duty)
  int pwm = (int)fabs(u);
  digitalWrite(4, u >= 0 ? HIGH : LOW);  // direction pin, adjust to your driver
  analogWrite(3, constrain(pwm, 0, 255));

  Serial.print(y);
  Serial.print('\t');
  Serial.println(u);
}
