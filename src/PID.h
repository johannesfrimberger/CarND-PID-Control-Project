#ifndef PID_H
#define PID_H

#include <iostream>

class PID
{
public:
    
    /*
     * Constructor
     */
    PID(const bool optimization);
    
    /*
     * Destructor.
     */
    virtual ~PID();
    
    /*
     * Initialize PID.
     */
    void Init(const double Kp, const double Ki, const double Kd);
    
    /*
     * Update the PID error variables given cross track error.
     */
    bool UpdateError(const double cte, const bool validSample, const unsigned n_samples = 400);
    
    /*
     * Calculate the total PID error.
     */
    double TotalError() const;
    
    
    /*
     * Print latest settings of PID controller
     */
    void PrintStatus() const
    {
        std::cout << "PID Controller with parameters: P: " << _Kp <<
        ", I: " << _Ki << ", D: " << _Kd  << std::endl;
    }
    

private:
    
    enum OptimizationStage
    {
        OS_PRE,
        OS_INCREASE,
        OS_DECREASE
    };
    
    void TwiddleParameter(double &param, double& step);
    
    /*
     * Errors
     */
    double _p_error;
    double _i_error;
    double _d_error;
    
    /*
     * Coefficients
     */
    double _Kp;
    double _Ki;
    double _Kd;
    
    /*
     * Parameters and internal storage for optimization
     */
    double _optimization_step_size[3];
    double _total_error;
    double _best_error;
    
    unsigned _n_samples;
    unsigned _optimizationParameter;
    
    bool _optimization;
    bool _first_run;
    
    OptimizationStage _optimization_stage;
};

#endif /* PID_H */
