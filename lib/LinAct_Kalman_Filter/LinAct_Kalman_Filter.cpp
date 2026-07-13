// LinAct_Kalman_Filter.cpp
#include "LinAct_Kalman_Filter.h"
#include <Arduino.h>
#include "BasicLinearAlgebra.h"

// Note: Ensure your header file (.h) defines these as pointers:
// float* _dt; float* _position_reading; float* _position_estimate; float* _velocity;

KalmanFilter::KalmanFilter(float* dt, float* position_reading, float* position_estimate, float* velocity) {
    // 1. Properly save the actual memory ADDRESSES into your pointers
    _dt = dt;
    _position_reading = position_reading;
    _position_estimate = position_estimate;
    _velocity = velocity;

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
    Q = {0.001f * (*_dt),   0.0f,
                    0.0f, 0.001f};

    R = {0.03375196f};
    K_k = {0.0f, 0.0f};
    Z_k = {*_position_reading}; 
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
    Q(0, 0) = 0.001f * current_dt; 
    
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
    Serial.println(F("============ KALMAN FILTER MATRICES ============"));
    
    Serial.print(F("x_hat [Position; Velocity]:\n"));
    Serial.println(x_hat);
    
    Serial.print(F("P (Error Covariance Matrix):\n"));
    Serial.println(P);
    
    Serial.print(F("K_k (Kalman Gain):\n"));
    Serial.println(K_k);
    
    Serial.print(F("Z_k (Current Measurement):\n"));
    Serial.println(Z_k);
    
    Serial.print(F("Phi (State Transition):\n"));
    Serial.println(Phi);
    
    Serial.print(F("Q (Process Noise Covariance):\n"));
    Serial.println(Q);
    
    Serial.print(F("R (Measurement Noise Covariance):\n"));
    Serial.println(R);
    
    Serial.println(F("================================================"));
}