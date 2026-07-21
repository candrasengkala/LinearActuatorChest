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

#define DIR_PIN 7
#define PWM_PIN 6

#define POT_INPUT A3

#define KALMAN_TIMER_ISR_PERIOD 100

float position_reading = 0;
float delta_time = 0;
unsigned long end_time = 0;
unsigned long previous_time = 0; // Add this global variable at the top
float position_estimate = 0;
float velocity = 0;

float sigma_v_squared = 1.0f; // Process noise variance (tuning parameter)
float r = 0.03375196f; // Measurement noise variance (tuning parameter

float pwm_value = 20; // Set the PWM value to 20 (20% duty cycle)
bool direction = LOW; //

KalmanFilter filter(&delta_time, &position_reading, &position_estimate, &velocity, &sigma_v_squared, &r);
LinAct lin_act(PWM_PIN, DIR_PIN, POT_INPUT, &position_reading);
LinAct_Timer timer(&delta_time);

// KalmanFilter kalman_filter(&delta_time, &position_reading);
// the setup function runs once when you press reset or power the board
// Maximum duty cycle for PWM signal: 20% (<50). Do 30 for now. 
// === print_all() function, definition is below this thing ===
void print_all();
// === ISR for kalman filter === 
void kalman_loop();
void setup() {
  lin_act.instantiate();
  timer.instantiate();
  ITimer2.init();
  ITimer2.attachInterrupt(KALMAN_TIMER_ISR_PERIOD, kalman_loop);
  Serial.begin(115200); // Set to 115200 to handle data matrices smoothly
}
void loop() {
    timer.deltaTime(); // It's okay to call it here. This is measured by Timer0.
    lin_act.allRetract(&pwm_value); // Automatically retracts the actuator after each loop iteration
}
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
  Serial.print(velocity);
  Serial.println(" mm/s");
}

void kalman_loop(){
  // --- Fix Delta Time Interval ---
  lin_act.measurePosition(); // Drops fresh reading into 'position_reading'
  filter.updateFilter(); // Pulls reading, calculates math, and pushes directly to 'velocity'
  // --- Throttled Printing ---
  static unsigned long lastPrint = 0;
  if (micros() - lastPrint >= 25000) { // Every 25ms
    lastPrint = micros();
    // filter.printMatrices(); // Ensure the filter is updated before printing
    // print_all(); // This will now successfully print your changing velocity variable! 
    filter.stepResponse(&position_reading, &pwm_value, &delta_time); // This will now successfully print your changing velocity variable!
  }
}