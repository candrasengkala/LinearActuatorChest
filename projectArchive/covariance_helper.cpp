#include <Arduino.h>
#include "LinAct_Kalman_Filter.h"
#include "LinAct_Control_Measure.h"

#define DIR_PIN 7
#define PWM_PIN 6

#define POT_INPUT A3

float position_reading = 0;
float delta_time = 0;
unsigned long end_time = 0;
unsigned long previous_time = 0; // Add this global variable at the top
float position_estimate = 0;
float velocity = 0;


float pwm_value = 20; // Set the PWM value to 20 (20% duty cycle)
bool direction = LOW; //
const unsigned long interval = 10; // 100 milliseconds
unsigned long last_execution_time = 0;

LinAct lin_act(PWM_PIN, DIR_PIN, POT_INPUT, &position_reading);

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
  Serial.begin(115200); // Set to 115200 to handle data matrices smoothly
  previous_time = micros();
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_execution_time >= interval) {
    last_execution_time = current_time; // Save the last time we ran this
  // lin_act.move(&pwm_value, &direction);
    lin_act.covarianceHelper(); // Automatically updates position_reading
  // if (position_reading <= 238) {
  //   pwm_value = 20; // Set the PWM value to 20 (20% duty cycle)
  // }
  // else {
  //   pwm_value = 0; // Stop the actuator if position is above 238 mm
  // }
  }
}