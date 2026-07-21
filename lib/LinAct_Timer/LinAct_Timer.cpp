#include "LinAct_Timer.h"
#include <Arduino.h>

LinAct_Timer::LinAct_Timer(float *delta_time) {
    _delta_time = delta_time;
}
// In your header file (LinAct_Timer.h):
// bool _first_run = true;

int LinAct_Timer::instantiate() {
    _previous_time = micros();
    _first_run = true; 
    return 0;
}

int LinAct_Timer::deltaTime() {
    _end_time = micros();
    
    if (_first_run) {
        _previous_time = _end_time;
        *_delta_time = 0.001f; // Set a sensible default start value (e.g., 1ms)
        _first_run = false;
        return 0;
    }

    *_delta_time = (_end_time - _previous_time) * 1e-6f;
    _previous_time = _end_time;

    // Prevent divide-by-zero or negative values with a microscopic limit, not 1.0s
    if (*_delta_time <= 0.0f) *_delta_time = 1e-6f; 

    return 0;
}