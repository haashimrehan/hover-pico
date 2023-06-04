// In header to allow for multiple PID controllers.

typedef struct PID {
    float Kp;
    float Ki;
    float Kd;
    float tau;
    float integrator_lmt;
    float setpoint;
    float measured;
    float integrator;
    float differentiator;
    float prev_err;
    float prev_measured;
    absolute_time_t time_prev;
} PID;


// PID coefficents (example implementation):
// Kp: Proportial term (0.5)
// Ki: Integral term (0.05)
// Kd: Derivative term (0.01)
// Tau: Used to smooth setpoint change (0.02)
// Windup limit: Limits integral value to +/- (1)
void PID_Coefficents(struct PID *pid_struct, float Kp, float Ki, float Kd, float tau, float intergrator_lmt);

// Computes PID, outputs raw PID value around 0 point.
float PID_Compute(absolute_time_t time, struct PID *pid);