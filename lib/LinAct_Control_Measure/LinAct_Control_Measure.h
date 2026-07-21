#ifndef LIN_ACT_CONTROL_MEASURE_H
#define LIN_ACT_CONTROL_MEASURE_H

class LinAct {
public:
    LinAct(int pwm_pin, int dir_pin, int pot_pin, float* pot);
    void instantiate();
    void measurePosition(); // Changed to void: automatically writes to the stored _pot pointer
    void move(float* pwm, bool* dir);
    void covarianceHelper(); // Optional: For debugging purposes
    void moveToPosition(float target_position, float* pwm); // New method for moving to a specific position
    void allRetract(float* pwm); // New method for moving to the fully retracted position
    void allExtend(float* pwm); // New method for moving to the fully extended position
    void backnforthSpeed(float target_position, float extend_pwm, float retract_pwm, bool *dir);
private:
    int _pwm_pin;
    int _dir_pin;
    int _pot_pin;

    int  _dir           = false;
    bool  _first_run    = true;  // <-- default member initializer, C++11+

    // Pointer variable to hold the memory address of your global position_reading variable
    float* _pot; 
};

#endif