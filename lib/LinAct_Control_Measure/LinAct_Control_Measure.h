#ifndef LIN_ACT_CONTROL_MEASURE_H
#define LIN_ACT_CONTROL_MEASURE_H

class LinAct {
public:
    LinAct(int pwm_pin, int dir_pin, int pot_pin, float* pot);
    void instantiate();
    void measurePosition(); // Changed to void: automatically writes to the stored _pot pointer
    void move(int* pwm, bool* dir);
    void covarianceHelper(); // Optional: For debugging purposes

private:
    int _pwm_pin;
    int _dir_pin;
    int _pot_pin;
    
    // Pointer variable to hold the memory address of your global position_reading variable
    float* _pot; 
};

#endif