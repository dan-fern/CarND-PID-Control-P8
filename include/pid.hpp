#ifndef PID_HPP
#define PID_HPP
#include <vector>
#include <iostream>
#include <cmath>


using namespace std;


class PID
{

public:

    /**
    * Constructor
    */
    PID( );

    /**
    * Destructor.
    */
    virtual ~PID( );

    /**
    * Initialize PID.
    * @param (Kp_, Ki_, Kd_) The initial PID coefficients
    */
    void Init( double Kp, double Ki, double Kd );

    /**
    * Update the PID error variables given cross track error.
    * @param cte The current cross track error
    */
    void UpdateError( double cte, bool twiddle = false );

    double total_pid_value_;

    double total_error_;

private:

    /**
    * Calculate the total PID error.
    * @output The total PID error
    */
    double UpdateTotalError( double cte );

    /**
    * Calculate the total PID error.
    * @output The total PID error
    */
    void UpdateTotalPID( );

    /*
    * Convenience function for adding amount (dp) to a PID controller parameter based on index
    */
    void AddToParameterAtIndex( int index, double amount );

    /**
    * PID Coefficients
    */
    double Kp_;
    double Ki_;
    double Kd_;

    /**
    * PID Errors
    */
    double p_error_;
    double i_error_;
    double d_error_;

    double best_error_;

    int step_;
    int parameter_index_;
    int twiddle_frequency;

    bool try_adding_;
    bool try_subtracting_;

    std::vector<double> dp_;

    // number of steps to allow changes to settle, then to evaluate error
    //int n_settle_steps, n_eval_steps;

};

#endif  // PID_HPP
