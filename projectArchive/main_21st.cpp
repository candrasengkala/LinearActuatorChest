#include <Arduino.h>
#include "LinAct_Kalman_Filter.h"
#include "LinAct_Control_Measure.h"
#include "LinAct_Timer.h"

#define DIR_PIN 7
#define PWM_PIN 6
#define POT_INPUT A3

float position_reading = -1.0; // Set to invalid starting reading to avoid false 0 trigger
float delta_time = 0;

float pwm_value_retract;
float pwm_value_extend;
float pwm_value_fed     = 0;

float retract_speed = 5;
float extend_speed = 5;
// PWM = a*v + b
float a = 13.96764;
float b = -0.4061;

float target_position = 20;

bool dir = HIGH;
bool first_run = true;

LinAct lin_act(PWM_PIN, DIR_PIN, POT_INPUT, &position_reading);
LinAct_Timer timer(&delta_time);

unsigned long last_execution_time = 0;
const unsigned long interval = 100; // 100ms update rate

void print_all() {
  Serial.print(last_execution_time);
  Serial.print(",");
  Serial.print(dir);
  Serial.print(",");
  Serial.print(pwm_value_fed);
  Serial.print(",");
  Serial.print(position_reading);
  Serial.println("");
}

void setup() {
  Serial.begin(115200);
  lin_act.instantiate();
  timer.instantiate();
}

void loop() {  
  unsigned long current_time = millis();
  pwm_value_extend = (a * extend_speed) + b;
  pwm_value_retract = (a * retract_speed) + b;
  // Non-blocking timing gate
  if (current_time - last_execution_time >= interval) {
    lin_act.measurePosition();
    last_execution_time = current_time;
    lin_act.backnforthSpeed(target_position, pwm_value_extend, pwm_value_retract, &dir);
    print_all();
  }
}