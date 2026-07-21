// KalmanFilter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <BasicLinearAlgebra.h>

class KalmanFilter {
public:
    // Constructor takes memory addresses for tracking inputs and outputs
    KalmanFilter(float* dt, float* position_reading, float* position_estimate, float* velocity, float* sigma_v_squared, float* r); 
    
    // Changes to void because it modifies your external variables directly
    void updateFilter(); 
    void getPosition();
    void getVelocity();
    void init();
    void printMatrices(); // Optional: For debugging purposes
    // Tuning setters
    void setQ(float *sigma_v_sq); // rebuilds Q from single scalar (sigma_v^2)
    void setR(float *r);          // sets measurement noise variance directly
    void stepResponse(float* position_reading, float* pwm_value, float *delta_t);         // Returns time, input PWM, estimated position, and estimated velocity with for writing in CSV.
private:
    // BasicLinearAlgebra types: BLA::Matrix<rows, columns>
    BLA::Matrix<2, 1> K_k;   // Kalman gain matrix (2x1)
    BLA::Matrix<1, 2> H;     // Measurement matrix (1x2)
    BLA::Matrix<2, 1> x_hat; // State estimate vector [position; velocity] (2x1)
    BLA::Matrix<2, 2> P;     // Covariance of error in state estimate (2x2)
    BLA::Matrix<2, 2> Phi;   // State transition matrix (2x2)
    BLA::Matrix<2, 2> Q;     // Covariance of process noise (2x2)
    
    // R and Z_k are scalars wrapped in 1x1 matrices for algebra compatibility
    BLA::Matrix<1, 1> R;     // Covariance of measurement noise (1x1)
    BLA::Matrix<1, 1> Z_k;   // Actual measurement value at time k (1x1)
    
    // CRITICAL FIX: These must be declared as pointers (float*) 
    // so they hold the memory addresses passed into your constructor!
    float* _dt;               
    float* _position_reading;  
    float* _position_estimate; 
    float* _velocity;         
    float* _sigma_v_squared;
    float* _r;
};

#endif