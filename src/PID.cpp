#include "PID.h"

PID::PID(const bool optimization):
    _optimization(optimization)
{}

PID::~PID()
{}

void PID::Init(const double Kp, const double Ki, const double Kd)
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    
    _d_error = 0;
    _p_error = 0;
    _i_error = 0;
    
    _n_samples = 0U;
}

bool PID::UpdateError(const double cte, const unsigned n_iterations)
{
    // Update internal storage
    _d_error = cte - _p_error;
    _p_error = cte;
    _i_error += cte;
    
    bool reset = false;
    
    if(_optimization)
    {
        // Collect error for n_iterations
        if(_n_samples < n_iterations)
        {
            _n_samples++;
            std:: cout << _n_samples << std::endl;
        }
        else
        {
            // Set reset flag to true
            reset = true;
            
            // ToDo: Twiddle parameters
            const double Kp = _Kp;
            const double Ki = _Ki;
            const double Kd = _Kd;
            
            // Re-initialize PID controller
            Init(Kp, Ki, Kd);
        }
    }
    
    return reset;
}

double PID::TotalError() const
{
    return -_Kp * _p_error - _Kd * _d_error - _Ki * _i_error;
}
