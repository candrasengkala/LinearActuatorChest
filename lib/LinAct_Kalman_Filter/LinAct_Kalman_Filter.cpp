// LinAct_Kalman_Filter.cpp
#include "LinAct_Kalman_Filter.h"
#include <Arduino.h>
#include "BasicLinearAlgebra.h"

// Note: Ensure your header file (.h) defines these as pointers:
// float* _dt; float* _position_reading; float* _position_estimate; float* _velocity;

KalmanFilter::KalmanFilter(float* dt, float* position_reading, float* position_estimate, float* velocity, float *sigma_v_squared, float *r) {
    // 1. Properly save the actual memory ADDRESSES into your pointers
    _dt = dt;
    _position_reading = position_reading;
    _position_estimate = position_estimate;
    _velocity = velocity;
    _r = r;
    _sigma_v_squared = sigma_v_squared;
    // 2. Pull initial values out of the addresses to seed the filter state
    x_hat = {*_position_estimate, 
             *_velocity}; 
    // 3. Initialize stable matrices
    H = {1.0f, 0.0f}; 
    P = {0.0f,   0.0f,
           0.0f, 0.0f};
    // 4. Initialize time-dependent matrices with the baseline values
    Phi = {1.0f, *_dt,
           0.0f,  1.0f};
    setR(_r); // Initialize R from the provided pointer
    setQ(_sigma_v_squared); // Initialize Q from the provided pointer
    K_k = {0.0f, 0.0f};
    Z_k = {*_position_reading}; 
}

void KalmanFilter::init(){
    // 1. Sync the external variables
    *_position_estimate = *_position_reading;
    *_velocity = 0.0f;

    // 2. CRITICAL: Seed the internal Kalman state matrix!
    x_hat = {*_position_reading, 
             0.0f}; 

    // 3. Recommended: Reset the error covariance matrix P
    P = {0.08f,  0.0f,    // Start with a little uncertainty
         0.0f,  400.0f};
}
void KalmanFilter::updateFilter(){
    // Pull fresh data from the external variables via pointers
    float current_dt = *_dt;
    Z_k = {*_position_reading}; 

    // 1. Update Kalman Gain
    K_k = P * ~H * Inverse(H * P * ~H + R); 
    
    // 2. Update State Estimate
    x_hat = x_hat + K_k * (Z_k - H * x_hat);
    getPosition(); // Update the external position estimate variable
    getVelocity(); // Update the external velocity variable
    // 3. Update Covariance of Error in State Estimate
    P = (BLA::Matrix<2, 2>{1.0f, 0.0f,
                           0.0f, 1.0f} - K_k * H) * P;

    // 4. Project into k+1 (Predict next state)
    Phi(0, 1) = current_dt; 
    setQ(_sigma_v_squared); // Rebuild Q based on the new dt   
    setR(_r); // Update R if needed
    x_hat = Phi * x_hat;
    P = Phi * P * ~Phi + Q;
}

void KalmanFilter::getPosition() {
    // Dereference the pointer to write directly to the original variable's memory address
    *_position_estimate = x_hat(0, 0); 
}

void KalmanFilter::getVelocity() {
    // Dereference the pointer to write directly to the original variable's memory address
    *_velocity = x_hat(1, 0); 
}

void KalmanFilter::printMatrices() {
    Serial.println(F("============ KALMAN FILTER MATRICES (6 Decimals) ============"));
    
    Serial.print(F("x_hat [Position; Velocity]:\n"));
    Serial.print(F("[ ")); Serial.print(x_hat(0, 0), 10); Serial.println(F(" ]"));
    Serial.print(F("[ ")); Serial.print(x_hat(1, 0), 10); Serial.println(F(" ]\n"));
    
    Serial.print(F("P (Error Covariance Matrix):\n"));
    Serial.print(F("[ ")); Serial.print(P(0, 0), 10); Serial.print(F(", ")); Serial.print(P(0, 1), 10); Serial.println(F(" ]"));
    Serial.print(F("[ ")); Serial.print(P(1, 0), 10); Serial.print(F(", ")); Serial.print(P(1, 1), 10); Serial.println(F(" ]\n"));
    
    Serial.print(F("K_k (Kalman Gain):\n"));
    Serial.print(F("[ ")); Serial.print(K_k(0, 0), 6); Serial.println(F(" ]"));
    Serial.print(F("[ ")); Serial.print(K_k(1, 0), 6); Serial.println(F(" ]\n"));
    
    Serial.print(F("Z_k (Current Measurement):\n"));
    Serial.print(F("[ ")); Serial.print(Z_k(0, 0), 6); Serial.println(F(" ]\n"));
    
    // This is the crucial one!
    Serial.print(F("Phi (State Transition - Row 1, Col 2 is your dt):\n"));
    Serial.print(F("[ ")); Serial.print(Phi(0, 0), 6); Serial.print(F(", ")); Serial.print(Phi(0, 1), 6); Serial.println(F(" ]"));
    Serial.print(F("[ ")); Serial.print(Phi(1, 0), 6); Serial.print(F(", ")); Serial.print(Phi(1, 1), 6); Serial.println(F(" ]\n"));
    
    Serial.print(F("Q (Process Noise Covariance):\n"));
    Serial.print(F("[ ")); Serial.print(Q(0, 0), 6); Serial.print(F(", ")); Serial.print(Q(0, 1), 6); Serial.println(F(" ]"));
    Serial.print(F("[ ")); Serial.print(Q(1, 0), 6); Serial.print(F(", ")); Serial.print(Q(1, 1), 6); Serial.println(F(" ]\n"));
    
    Serial.print(F("R (Measurement Noise Covariance):\n"));
    Serial.print(F("[ ")); Serial.print(R(0, 0), 6); Serial.println(F(" ]\n"));
    
    Serial.println(F("============================================================="));
}

void KalmanFilter::setQ(float *sigma_v_sq) {
    // Rebuilds Q from a single scalar (sigma_v^2), using the current dt
    // and the piecewise-constant-acceleration structure:
    // Q = sigma_v^2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]
    float dt = *_dt;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    Q(0, 0) = *sigma_v_sq * dt4 / 4.0f;
    Q(0, 1) = *sigma_v_sq * dt3 / 2.0f;
    Q(1, 0) = *sigma_v_sq * dt3 / 2.0f;
    Q(1, 1) = *sigma_v_sq * dt2;
}

void KalmanFilter::setR(float *r) {

    R(0, 0) = *r;
}

void KalmanFilter::stepResponse(float* position_reading, float* pwm_value, float *delta_t) {
    // Prints time_us, pwm, estimated position, estimated velocity as CSV row.
    // Call this once per loop iteration during a step-response capture.
    Serial.print(millis());
    Serial.print(",");
    Serial.print(*delta_t, 6);
    Serial.print(",");
    Serial.print(*pwm_value);
    Serial.print(",");
    Serial.print(*position_reading);
    Serial.print(",");
    Serial.print(*_position_estimate, 4);
    Serial.print(",");
    Serial.println(*_velocity, 4);
}