#include <Arduino.h>
#include "LinAct_Kalman_Filter.h"
#include "LinAct_Control_Measure.h"
#include "LinAct_Timer.h"

#define DIR_PIN 7
#define PWM_PIN 6
#define POT_INPUT A3

// No volatile needed here because we aren't using interrupts!
float position_reading = 0;
float delta_time = 0;
float position_estimate = 0;
float velocity = 0;

float sigma_v_squared = 640.0f; 
float r = 0.08f; 
float pwm_value = 45; 

KalmanFilter filter(&delta_time, &position_reading, &position_estimate, &velocity, &sigma_v_squared, &r);
LinAct lin_act(PWM_PIN, DIR_PIN, POT_INPUT, &position_reading);
LinAct_Timer timer(&delta_time);

unsigned long last_execution_time = 0;
const unsigned long interval = 100; // 100 milliseconds

void setup() {
  Serial.begin(115200);
  lin_act.instantiate();
  timer.instantiate();
  filter.init();
}

void loop() {
  unsigned long current_time = millis();
  // Check if 100ms has passed
  if (current_time - last_execution_time >= interval) {
    timer.deltaTime(); 
    last_execution_time = current_time; // Save the last time we ran this
    // Everything in here runs exactly once every 100ms
    lin_act.measurePosition(); 
    // filter.printMatrices(); 
    filter.updateFilter(); 
    filter.stepResponse(&position_reading, &pwm_value, &delta_time);
  }
  // These run continuously in the background
  lin_act.allExtend(&pwm_value); 
}