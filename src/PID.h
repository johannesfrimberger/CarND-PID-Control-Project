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
    bool UpdateError(const double cte, const unsigned n_iterations = 200);
    
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
    
    unsigned _n_samples;
    
    bool _optimization;
};

#endif /* PID_H */
