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
#include "PID2DOF.h"
// === Pin Definitions ===
#define DIR_PIN 7
#define PWM_PIN 6
#define POT_INPUT A3
// === Timer Interval Definition ===
#define TIMER1_INTERVAL_MS 100
// === Global Variables ===
float position_reading = 0;
float delta_time = 0;
float position_estimate = 0;
float velocity_estimate = 0;
// === PID Controller Parameters ===
float Ts = TIMER1_INTERVAL_MS / 1000.0f; // Sampling time in seconds (100 ms)
float Kp = 3.25049867125114;
float Ki = 0.153823359280193;
float Kd = -2.37818576498243; // PID tuning parameters
float N = 1.27653195864833; // Derivative filter divisor
float b = 1.0f; // Proportional setpoint weight (1.0 = classic PID, matches old YAPID behavior)
float c = 1.0f; // Derivative setpoint weight (1.0 = classic PID, matches old YAPID behavior)
// === Kalman Filter Parameters ===
float sigma_v_squared = 640.0f; // Process noise variance (tuning parameter)
float r = 0.08f; // Measurement noise variance (tuning parameter)
// === Target values for PID control ===
float PID_output = 0;
bool dir;
float position_target = 25; // Target velocity for PID control
// === Kalman Filter Timing ===
unsigned long last_execution_time = 0;
// === Object Instantiations ===
KalmanFilter filter(&delta_time, &position_reading, &position_estimate, &velocity_estimate, &sigma_v_squared, &r);
LinAct lin_act(PWM_PIN, DIR_PIN, POT_INPUT, &position_reading);
LinAct_Timer timer(&delta_time);
PID2DOF pid_controller(Kp, Ki, Kd, N, b, c, Ts);
// === Helper Functions ===
void pwmTimerHandler(){
    timer.deltaTime(); 
    // Everything in here runs exactly once every 100ms
    lin_act.measurePosition(); 
    filter.updateFilter(); 
    
    PID_output = pid_controller.compute(position_target, position_reading);

    PID_output = fabs(PID_output); // magnitude only; dir carries the sign
    dir = PID_output < 0;
    lin_act.move(&PID_output, &dir);

    // Saturation for Position
    if (!dir && position_estimate >= 50.0f) {
        PID_output = 0;
        Serial.print("LATCHED STOP at pos="); Serial.println(position_estimate);
    }
    else if (dir && position_estimate <= 0.0f){
        PID_output = 0;
        Serial.print("LATCHED STOP at pos="); Serial.println(position_estimate);
    } 
}
//the loop function runs over and over again forever
void setup() {
    lin_act.instantiate();
    timer.instantiate();
    Serial.begin(115200);   //Set to 115200 to handle data matrices smoothly
    pid_controller.setOutputLimits(0.0f, 255.0f); // cap PID_output magnitude at 50  
    filter.init();          //seed position_estimate from real hardware first
    ITimer1.init();
    ITimer1.attachInterrupt(TIMER1_INTERVAL_MS, pwmTimerHandler);
    Serial.println("Position_Estimate,PID_Output,Velocity_Target,Velocity_Estimate,Direction");
}
void loop() { 
    static unsigned long lastPrint = 0;
    if (micros() - lastPrint >= 100000) { // Every 25ms
        lastPrint = micros();
        Serial.print(position_estimate);
        Serial.print(",");
        Serial.print(PID_output);
        Serial.print(",");
        Serial.print(position_target);
        Serial.print(",");
        Serial.print(velocity_estimate);
        Serial.print(",");
        Serial.print(dir);
        Serial.println("");
    }
}
