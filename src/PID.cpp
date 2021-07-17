#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool dynamic)
{
    /**
     * TODO: Initialize PID coefficients (and errors, if needed)
     */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    dynamic = true;

    prev_cte = 0;
    i_error = 0;
    d_error = 0;
    p_error = 0;
}


void PID::UpdateError(double cte)
{
    /**
     * TODO: Update PID errors based on cte.
     */
    p_error = cte;
    d_error = cte-prev_cte;
    i_error += cte;
    double learning_rate = 0.00015;
    if(dynamic)
    {
        Kp += p_error * learning_rate;
        Kd += d_error * 0.00010;
    }

    prev_cte = cte;
}

double PID::TotalError()
{
    /**
     * TODO: Calculate and return the total error
     */
    return (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);
}
