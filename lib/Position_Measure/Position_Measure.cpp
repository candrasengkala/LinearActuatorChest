#include <Arduino.h>
#include "Position_Measure.h"
#include "TimerInterrupt.h"
// Note to self: there's a better and more memory efficient way to do this, but for now, let's keep it simple and straightforward.
// Constants adjusted for Arduino UNO hardware specifications
const float ADC_MIN = 0.0f;       // Minimum ADC value for Arduino UNO
const float ADC_MAX = 1023.0f;    // Maximum ADC value for Arduino UNO (10-bit ADC)
const float POS_MIN_MM = 0.0f;    // Actuator fully retracted (0 mm)
const float POS_MAX_MM = 50.0f;  // Actuator fully extended (50 mm)

int measurePosition(float potValue, float *position) {
    // 1. Safety Guard: Check if the memory pointer is valid (not null)
    if (position == NULL) {
        return -1; // Error: Invalid pointer
    }

    // 2. Sensor Fault Detection: Check analog input boundaries
    if (potValue < ADC_MIN || potValue > ADC_MAX) {
        return -2; // Error: ADC value is outside hardware limits
    }

    // 3. Linear Interpolation Formula (Mapping ADC to millimeters)
    // Pure float math now — no need for the long-cast overflow guard,
    // since float multiplication here won't overflow at these ranges.
    float calculation = (potValue - ADC_MIN) * (POS_MAX_MM - POS_MIN_MM);
    float mappedPosition = POS_MIN_MM + (calculation / (ADC_MAX - ADC_MIN));

    // 4. Pointer Access: Save the calculation result directly to the external variable's memory address
    *position = mappedPosition;

    return 0; // Success!
}