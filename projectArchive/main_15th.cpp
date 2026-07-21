#include <Arduino.h>
#include "LinAct_Kalman_Filter.h"
#include "LinAct_Control_Measure.h"
#include "LinAct_Timer.h"

#define DIR_PIN 7
#define PWM_PIN 6

#define POT_INPUT A3

float position_reading = 0;
float delta_time = 0;
unsigned long end_time = 0;
unsigned long previous_time = 0; // Add this global variable at the top
float position_estimate = 0;
float velocity = 0;

float sigma_v_squared = 0.1f; // Process noise variance (tuning parameter)
float r = 0.03375196f; // Measurement noise variance (tuning parameter

int pwm_value = 20; // Set the PWM value to 20 (20% duty cycle)
bool direction = LOW; //

KalmanFilter filter(&delta_time, &position_reading, &position_estimate, &velocity, &sigma_v_squared, &r);
LinAct lin_act(PWM_PIN, DIR_PIN, POT_INPUT, &position_reading);
LinAct_Timer timer(&delta_time);

// KalmanFilter kalman_filter(&delta_time, &position_reading);
// the setup function runs once when you press reset or power the board
// Maximum duty cycle for PWM signal: 20% (<50). Do 30 for now. 
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

// the loop function runs over and over again forever
void setup() {
  lin_act.instantiate();
  timer.instantiate();
  Serial.begin(115200); // Set to 115200 to handle data matrices smoothly
}

void loop() {
  // --- Fix Delta Time Interval ---
  timer.deltaTime();
  // --- Core Operations ---
  lin_act.allRetract(&pwm_value); // Automatically retracts the actuator after each loop iteration
  lin_act.measurePosition(); // Drops fresh reading into 'position_reading'
  
  filter.updateFilter(); // Pulls reading, calculates math, and pushes directly to 'velocity'

  // --- Throttled Printing ---
  static unsigned long lastPrint = 0;
  if (micros() - lastPrint >= 25000) { // Every 25ms
    lastPrint = micros();
    // filter.printMatrices(); // Ensure the filter is updated before printing
    // print_all(); // This will now successfully print your changing velocity variable! 
    filter.stepResponse(&pwm_value); // This will now successfully print your changing velocity variable!
  }
}