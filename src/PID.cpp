#include "PID.h"
#include <cmath>

PID::PID(const bool optimization):
_optimization(optimization),
_first_run(true)
{
    // Init
    _best_error = 0;
    
    // Define initial opimization step size
    _optimization_step_size[0] = 0.01;
    _optimization_step_size[1] = 0.0001;
    _optimization_step_size[2] = 0.01;
    
    _optimizationParameter = 0U;
    _optimization_stage = OS_PRE;
}

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
    
    _total_error = 0;
    _n_samples = 0U;
    
    PrintStatus();
}

bool PID::UpdateError(const double cte, const bool validSample, const unsigned n_samples)
{
    // Update internal storage
    _d_error = cte - _p_error;
    _p_error = cte;
    _i_error += cte;
    
    // Set return value intially to false
    bool reset = false;
    
    // Run optimization if requested
    if(_optimization && validSample)
    {
        // Collect error for n_iterations
        if(_n_samples < n_samples)
        {
            // Increase sample counter and sum up total error
            _n_samples++;
            _total_error += std::fabs(cte);
        }
        else
        {
            // Set reset flag to true
            reset = true;
            
            // Check if this was the first run and no reference error is available
            if(_first_run)
            {
                // Set best error to those achieved by the current settings
                _best_error = _total_error;
                
                // Set first_run to false (Can only be reset when new object is created)
                _first_run = false;
            }
            
            // Run twiddle optimization algorithm
            runOptimization();
            
            // Re-call runOptimization
            if(_optimization_stage == OS_PRE)
            {
                runOptimization();
            }
            
        }
    }
    
    return reset;
}

double PID::TotalError() const
{
    return -_Kp * _p_error - _Kd * _d_error - _Ki * _i_error;
}

void PID::runOptimization()
{
    // Twiddle parameters
    double Kp = _Kp;
    double Ki = _Ki;
    double Kd = _Kd;
    
    // Check which parameter should be optimized
    switch(_optimizationParameter)
    {
        case 0U:
            TwiddleParameter(Kp, _optimization_step_size[0U]);
            break;
        case 1U:
            TwiddleParameter(Ki, _optimization_step_size[1U]);
            break;
        case 2U:
            TwiddleParameter(Kd, _optimization_step_size[2U]);
            break;
        // Default case can never be reached
        default:
            _optimizationParameter = 0U;
            break;
    }
    
    // Ensure that _optimizationParameter is between 0 and 2
    _optimizationParameter = (_optimizationParameter % 3U);
    
    // Re-initialize PID controller
    Init(Kp, Ki, Kd);
}

void PID::TwiddleParameter(double &param, double& step)
{
    // Debug Information
    printf("Twiddle Parameter %d in Stage %d\n", _optimizationParameter, _optimization_stage);
    
    // Increase param by step size
    if(_optimization_stage == OS_PRE)
    {
        param += step;
        _optimization_stage = OS_INCREASE;
    }
    else if(_optimization_stage == OS_INCREASE)
    {
        // Check if error was decreased for this parameter
        if(_total_error < _best_error)
        {
            // Update best error and move on
            _best_error = _total_error;
            step *= 1.1;
            
            _optimizationParameter++;
            _optimization_stage = OS_PRE;
        }
        // If results get worse tweak parameter in other direction
        else
        {
            param -= 2.0 * step;
            _optimization_stage = OS_DECREASE;
        }
    }
    else if(_optimization_stage == OS_DECREASE)
    {
        // Check if error was decreased for this parameter
        if(_total_error < _best_error)
        {
            // Update best error and move on
            _best_error = _total_error;
            step *= 1.1;
        }
        else
        {
            // Reset parameter to initial value and move on with lower step size
            param += step;
            step *= 0.9;
        }
        
        _optimizationParameter++;
        _optimization_stage = OS_PRE;
    }
}
