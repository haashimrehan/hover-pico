#include "pico/stdlib.h"
#include "pid_controller.h"

/*
~~~~~~~~~~~~~ PID related functions ~~~~~~~~~~~~~
 - Designed to work with RasPi Pico
 - Use of simple intergrator wind-up prevention.
 - Kp, Ki, Kd, Intergal Limiter for all inputs.
 - Based upon https://www.youtube.com/watch?v=zOByx3Izf5U
*/

// PID coefficents (example implementation):
// Kp: Proportial term (0.5)
// Ki: Integral term (0.05)
// Kd: Derivative term (0.01)
// Tau (0.02)
// Windup limit (1)
void PID_Coefficents(struct PID *pid_struct, float Kp, float Ki, float Kd, float tau, float intergrator_lmt) {
    pid_struct->Kp = Kp;
    pid_struct->Ki = Ki;
    pid_struct->Kd = Kd;
    pid_struct->tau = tau;
    pid_struct->integrator_lmt = intergrator_lmt;
    pid_struct->time_prev = get_absolute_time();
}


// Computes PID, outputs raw PID value around 0 point.
float PID_Compute(absolute_time_t time, struct PID *pid) {
    float time_delta = absolute_time_diff_us(pid->time_prev, time) / 1000000.f; // seconds
    float error = pid->setpoint - pid->measured;

    // ~~~ Proportional ~~~
    float proportional = pid->Kp * error;


    // ~~~ Integral ~~~
    // (previous + current*0.5)
    float intergral  = pid->integrator + (0.5f * pid->Ki * pid->prev_err * time_delta);

    // Clamp it to prevent wind-up https://en.wikipedia.org/wiki/Integral_windup
    if(intergral > +(pid->integrator_lmt) ) 
        intergral = +pid->integrator_lmt;
    else if (intergral < -(pid->integrator_lmt) )
        intergral = -pid->integrator_lmt;

    pid->integrator = intergral;


    // ~~~ Differential ~~~ 
    // Band limited differentiator using the tau term
    // Differential of the measured term, prevents BIG jumps on setpoint change
    float err_diff = -(2.0f * pid->Kd * (pid->measured - pid->prev_measured)
                     +(2.0f * pid->tau - time_delta) * pid->differentiator)
                     /(2.0f * pid->tau + time_delta);


    pid->differentiator = err_diff;

    // Store what we need for next time
    pid->prev_err = error; 
    pid->time_prev = time;
    pid->prev_measured = pid->measured;

    // Output is PID equation: Kp*error + Ki*error_integrated + Kd*error_differential
    float output = proportional + pid->integrator + pid->differentiator;
    return(output); 
}