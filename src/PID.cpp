#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(const double Kp_, const double Ki_, const double Kd_)
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    d_error = 0;
    p_error = 0;
    i_error = 0;
}

void PID::UpdateError(const double cte)
{
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() const
{
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}
