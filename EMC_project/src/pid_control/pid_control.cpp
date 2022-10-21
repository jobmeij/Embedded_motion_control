#include "pid_control.h"

// sets the parameters of the PID controller. This can not be done for some reason during the init of the function..
void PID_control::setparams(double Kp_, double Ki_, double Kd_){
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

}

// Evaluates the pid controller and calculates the output. For a good performance this function must be called every loop
float PID_control::evaluate(float error_) {
    if (!PID_disable && !PID_latchdisable){
        // P
        float output = error_*Kp;

        //I
        integrator_value += error_*dt;
        output += integrator_value*Ki;

        // D
        output += (error_ - previous_error_value)*dt*Kd;
        previous_error_value = error_;

        return output;
    }
    else{
        if (PID_latchdisable){
            PID_latchdisable = false;
        };
        return 0;
    };

}

// resets the internal value of the integrator to zero
void PID_control::integrator_reset(){
    integrator_value = 0;

}

// Latch disable sets the output to zero for only the next call of PID.evaluate
void PID_control::disable_latch(){
    PID_latchdisable = true;

}

// disable_fix(true) sets the output of all calls for pid.evaluate to zero untill disable_fix(false) is called
void PID_control::disable_fix(bool PID_disable_){
    PID_disable = PID_disable_;
    if (PID_disable == 0){
        PID_disable = 1;
    }
    else{
        PID_disable = 0;
    }
}
