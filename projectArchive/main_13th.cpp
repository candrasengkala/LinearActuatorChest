#include <Arduino.h>
#include "LinAct_Kalman_Filter.h"
#include "Position_Measure.h"

#define DIR_PIN 7
#define PWM_PIN 6

#define POT_INPUT A3

float position_val = 0;
float delta_time = 0;
float end_time = 0;

// KalmanFilter kalman_filter(&delta_time, &position_val);
// the setup function runs once when you press reset or power the board
// Maximum duty cycle for PWM signal: 20% (<50). Do 30 for now. 
void print_all() {
  Serial.print("Position: ");
  Serial.print(position_val);
  Serial.print(" mm, ");
  Serial.print("Delta Time: ");
  Serial.print(delta_time);
  Serial.println(" ms");
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(POT_INPUT, INPUT);
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  delta_time = millis();
  analogWrite(PWM_PIN, 20);
  digitalWrite(DIR_PIN, HIGH);
  measurePosition(analogRead(POT_INPUT), &position_val);
  end_time = millis();
  delta_time = end_time - delta_time;
  print_all();
}
