#include "LinAct_Control_Measure.h"
#include <Arduino.h>

const float ADC_MIN = 0.0f;       // Minimum ADC value for Arduino UNO
const float ADC_MAX = 1023.0f;    // Maximum ADC value for Arduino
const float POS_MIN_MM = 0.0f;    // Actuator fully retracted (0 mm)
const float POS_MAX_MM = 50.0f;  // Actuator fully extended (500 mm)

LinAct::LinAct(int pwm_pin, int dir_pin, int pot_pin, float* pot) {
    _pwm_pin = pwm_pin;
    _dir_pin = dir_pin;
    _pot_pin = pot_pin;
    _pot = pot; // Store the memory address of the shared position tracking variable
}   

void LinAct::instantiate() {
    pinMode(_pwm_pin, OUTPUT);
    pinMode(_dir_pin, OUTPUT);
    pinMode(_pot_pin, INPUT);
    measurePosition();
}

void LinAct::measurePosition() {
    // 1. Safety Guard: Check if our tracking pointer is valid (not null)
    if (_pot == nullptr) {
        return; 
    }

    // 2. Read the potentiometer value from the analog pin
    int potValue = analogRead(_pot_pin);

    // 3. Sensor Fault Detection: Check analog input boundaries
    if (potValue < ADC_MIN || potValue > ADC_MAX) {
        return; // Early exit on bad hardware readings
    }

    // 4. Linear Interpolation Formula (Mapping ADC to millimeters)
    float calculation = (potValue - ADC_MIN) * (POS_MAX_MM - POS_MIN_MM);
    float mappedPosition = POS_MIN_MM + (calculation / (ADC_MAX - ADC_MIN));

    // 5. Pointer Access: Write directly to the stored memory location!
    *_pot = mappedPosition;
}

void LinAct::move(float* pwm, bool* dir) {
    // Safety Guard: Check if the incoming tracking pointers are valid
    if (pwm == nullptr || dir == nullptr) {
        return; 
    }

    // Set the hardware PWM and direction pin states based on global variables
    analogWrite(_pwm_pin, (int)*pwm);
    digitalWrite(_dir_pin, *dir);
}

void LinAct::covarianceHelper() {
    // Logs raw ADC counts + converted position as CSV.
    // Hold actuator physically still, capture N samples via this,
    // then compute variance(raw_adc) and variance(position_mm) offline -> R.
    int potValue = analogRead(_pot_pin);

    // 3. Sensor Fault Detection: Check analog input boundaries
    if (potValue < ADC_MIN || potValue > ADC_MAX) {
        return; // Early exit on bad hardware readings
    }

    // 4. Linear Interpolation Formula (Mapping ADC to millimeters)
    float calculation = (potValue - ADC_MIN) * (POS_MAX_MM - POS_MIN_MM);
    float mappedPosition = POS_MIN_MM + (calculation / (ADC_MAX - ADC_MIN));

    // 5. Pointer Access: Write directly to the stored memory location!
    *_pot = mappedPosition;

    Serial.print(potValue);
    Serial.print(",");
    Serial.println(mappedPosition, 20);
}

void LinAct::moveToPosition(float target_position, float* pwm) {
    // NOT TO BE USED: THIS IS PRIMITIVE, WITHOUT ANY FORM OF CONTROL LOOP. MAY LEAD TO INSTABILITY. 
    // Safety Guard: Check if the incoming tracking pointer is valid
    if (pwm == nullptr) {
        return; 
    }

    // Safety Guard: Ensure target position is within bounds
    if (target_position < POS_MIN_MM || target_position > POS_MAX_MM) {
        return; 
    }

    // Determine direction based on current position and target position
    bool direction = (*_pot < target_position) ? LOW : HIGH; // LOW for extending, HIGH for retracting

    // Set the PWM and direction
    digitalWrite(_dir_pin, direction);
    (target_position == *_pot)? analogWrite(_pwm_pin, 0) : analogWrite(_pwm_pin, (int)*pwm);
}

void LinAct::allRetract(float* pwm) {
    // Safety Guard: Check if the incoming tracking pointer is valid
    if (pwm == nullptr) {
        return; 
    }

    // Set direction to retract (LOW)
    moveToPosition(POS_MIN_MM, pwm); // Move to fully retracted position
}

void LinAct::allExtend(float* pwm) {
    // Safety Guard: Check if the incoming tracking pointer is valid
    if (pwm == nullptr) {
        return; 
    }

    // Set direction to extend (HIGH)
    moveToPosition(POS_MAX_MM, pwm); // Move to fully extended position
}

void LinAct::backnforthSpeed(float target_position, float extend_pwm, float retract_pwm, bool *dir) {
    // HIGH = retract, LOW = extend
    float midpoint = target_position / 2.0f;
    float fed_pwm;
    if (_first_run) {
        if (*_pot <= 25) {
            fed_pwm = retract_pwm;
            *dir = HIGH;  // extend outward if close to the bottom
        } else {
            fed_pwm = extend_pwm;
            *dir = LOW; // retract
        }
        if ((*_pot <= 0.0f) || (*_pot >= target_position)) {
            _first_run = false;
        }
    } else {
        if (*_pot >= target_position) {
            fed_pwm = retract_pwm;
            *dir = HIGH; // retract
        } else if (*_pot <= 0.0f) {
            fed_pwm = extend_pwm;
            *dir = LOW;  // extend
        }
    }

    move(&fed_pwm, dir);
}