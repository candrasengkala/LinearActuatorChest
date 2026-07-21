/*
Project Name: Linear Actuator Control with Kalman Filter and PID
Author      : Rizmi Ahmad Raihan
Affiliation : Institut Teknologi Bandung
Description : This project implements a control system for a linear actuator using a Kalman filter for state estimation and a PID controller for precise movement. The system reads position data from a potentiometer, estimates the position and velocity of the actuator, and adjusts the actuator's movement accordingly.
Date        : July 15th, 2026
*/
// === Dependencies ===
#include <Arduino.h>
#include "LinAct_Kalman_Filter.h"
#include "LinAct_Control_Measure.h"
#include "LinAct_Timer.h"
// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_1     true
#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_2     true
  #warning Using Timer1
#else          
  #define USE_TIMER_3     true
  #warning Using Timer3
#endif
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "TimerInterrupt.h"
#include "yapid.h"
// === Pin Definitions ===
#define DIR_PIN 7
#define PWM_PIN 6
#define POT_INPUT A3
// === Timer Interval Definition ===
#define TIMER1_INTERVAL_MS 1
// === Global Variables ===
float position_reading = 0;
float delta_time = 0;
float position_estimate = 0;
float velocity_estimate = 0;
// === PID Controller Parameters ===
float Ts = 1e-3f; // Sampling time in seconds (25 ms)
float Kp = 2.609926872467509;
float Ki = 0.609926872467509;
float Kd = 0; // PID tuning parameters
float N = 0.0; // Integral term for PID control
// === Kalman Filter Parameters ===
float sigma_v_squared = 0.1f; // Process noise variance (tuning parameter)
float r = 0.03375196f; // Measurement noise variance (tuning parameter)
// === Target values for PID control ===
float PID_output = 0;
bool dir;
float velocity_target = -30; // Target velocity for PID control

// === Object Instantiations ===
KalmanFilter filter(&delta_time, &position_reading, &position_estimate, &velocity_estimate, &sigma_v_squared, &r);
LinAct lin_act(PWM_PIN, DIR_PIN, POT_INPUT, &position_reading);
LinAct_Timer timer(&delta_time);
YAPID pid_controller(Ts, Kp, Ki, Kd, N);
// === Helper Functions ===
void print_all() {
  Serial.print("Position: ");
  Serial.print(position_reading);
  Serial.print(" mm, ");
  Serial.print("Delta Time: ");
  Serial.print(delta_time, 6);   // 6 decimal places for microsecond-level resolution
  Serial.println(" s");
  Serial.print("Position Estimate: ");
  Serial.print(position_estimate);
  Serial.print(" mm, ");
  Serial.print("Velocity: ");
  Serial.print(velocity_estimate);
  Serial.println(" mm/s");
}
void pwmTimerHandler(){
  dir = velocity_target < 0;  // HIGH = retract, LOW = extend

  // Feed magnitudes into the PID so the controller isn't confused by a
  // sign flip between target and estimate (e.g. target=-10, estimate=+2
  // during a direction reversal would otherwise read as a huge error).
  float target_mag = fabs(velocity_target);
  float estimate_mag = fabs(velocity_estimate);

  PID_output = pid_controller.Compute1(target_mag, estimate_mag);
  pid_controller.UpdateTime();

  PID_output = fabs(PID_output); // magnitude only; dir carries the sign
  lin_act.move(&PID_output, &dir);

  // Saturation for Position
  if (!dir && position_estimate >= 500.0f) velocity_target = 0;
  else if (dir && position_estimate <= 0.0f) velocity_target = 0;
}
// the loop function runs over and over again forever
void setup() {
  lin_act.instantiate();
  timer.instantiate();
  Serial.begin(115200); // Set to 115200 to handle data matrices smoothly
  pid_controller.SetOutputLimits(0, 50.0f); // cap PID_output magnitude at 50  
  ITimer1.init();
  ITimer1.attachInterrupt(TIMER1_INTERVAL_MS, pwmTimerHandler);
}
void loop() { 
  // --- Fix Delta Time Interval ---
  timer.deltaTime();
  // --- Core Operations ---
  lin_act.measurePosition(); // Drops fresh reading into 'position_reading'
  filter.updateFilter(); // Pulls reading, calculates math, and pushes directly to 'velocity'
  // --- Throttled Printing ---
  static unsigned long lastPrint = 0;
  if (micros() - lastPrint >= 25000) { // Every 25ms
    lastPrint = micros();
    Serial.print("PID_Output: ");
    Serial.println(PID_output);
    Serial.print("Velocity_Target: ");
    Serial.println(velocity_target);
    Serial.print("Velocity_Estimate: ");
    Serial.println(velocity_estimate);
    Serial.print("Direction: ");
    Serial.println(dir);
    // filter.printMatrices(); // Ensure the filter is updated before printing
    // print_all(); // This will now successfully print your changing velocity variable! 
    // filter.stepResponse(&velocity_target); // This will now successfully print your changing velocity variable!
  }
}