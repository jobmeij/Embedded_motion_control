#include <emc/io.h>
#include "../config.h"
#include <cmath>
#include <iostream>
#include <stdio.h>

#ifndef PID_control_H
#define PID_control_H

class PID_control{
private:
    float integrator_value;
    float previous_error_value;
    bool PID_disable;
    bool PID_latchdisable;
    double Kp;
    double Ki;
    double Kd;
    float dt;

public:
    PID_control(){
        PID_latchdisable = false;
        PID_disable = false;
        //dt = 1 / EXECUTION_RATE;
        dt = 0.05;
    }

    void setparams(double Kp_, double Ki_, double Kd_);
    float evaluate(float error_);
    void integrator_reset();
    void disable_latch();
    void disable_fix(bool PID_disable_);
};

#endif
