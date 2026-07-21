#ifndef LIN_ACT_TIMER_H
#define LIN_ACT_TIMER_H
class LinAct_Timer {
public:
    LinAct_Timer(float *delta_time);
    int instantiate();
    int deltaTime();
private:
    bool _first_run;
    unsigned long _end_time;
    unsigned long _previous_time; // Add this global variable at the top
    float *_delta_time;
};

#endif // LIN_ACT_TIMER_H