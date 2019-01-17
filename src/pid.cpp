#include "pid.hpp"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID( )
{

}

PID::~PID( )
{

}

void PID::Init( double Kp, double Ki, double Kd )
{
    /**
    * TODO: Initialize PID coefficients (and errors, if needed)
    */
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;

    total_error_ = 0;

    best_error_ = 9E9;

    step_ = 0;

    parameter_index_ = 2;

    total_pid_value_ = 0;

    twiddle_frequency = 3;

    try_adding_ = false;
    try_subtracting_ = false;

    dp_ = { 0.1 * Kp_, 0.1 * Kd_, 0.1 * Ki_ };
}

void PID::UpdateError( double cte, bool twiddle )
{
    /**
    * TODO: Update PID errors based on cte.
    */
    if( step_ == 0 )
    {
        // to get correct initial d_error
        p_error_ = cte;
    }
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;

    UpdateTotalError( cte );

    // last step in twiddle loop... twiddle it?
    if ( twiddle && step_ % twiddle_frequency == 0 )
    {
        //cout << "step: " << step_ << endl;
        //cout << "total error: " << total_error_ << endl;
        //cout << "best error: " << best_error_ << endl;

        if( total_error_ < best_error_ )
        {
            cout << "improvement!" << endl;

            best_error_ = total_error_;

            //if(step_ !=  n_settle_steps + n_eval_steps) {
            if( step_ == twiddle_frequency )
            {
                // don't do this if it's the first time through
                dp_[ parameter_index_ ] *= 1.1;
            }
            // next parameter
            parameter_index_ = ( parameter_index_ + 1 ) % 3;
            try_adding_ = try_subtracting_ = false;
        }
        if( !try_adding_ && !try_subtracting_ )
        {
            // try adding dp_[i] to params[i]
            AddToParameterAtIndex( parameter_index_, dp_[ parameter_index_ ] );
            try_adding_ = true;
        }
        else if( try_adding_ && !try_subtracting_ )
        {
            // try subtracting dp_[i] from params[i]
            AddToParameterAtIndex( parameter_index_, -2 * dp_[parameter_index_] );
            try_subtracting_ = true;
        }
        else
        {
            // set it back, reduce dp_[i], move on to next parameter
            AddToParameterAtIndex( parameter_index_, dp_[ parameter_index_ ] );
            dp_[ parameter_index_ ] *= 0.9;
            // next parameter
            parameter_index_ = ( parameter_index_ + 1 ) % 3;
            try_adding_ = try_subtracting_ = false;
        }
        total_error_ = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp_ << ", I: " << Ki_ << ", D: " << Kd_ << endl;
    }
    step_++;

    UpdateTotalPID( );

}


double PID::UpdateTotalError( double cte )
{
    /**
    * TODO: Calculate and return the total error
    */
    return total_error_ += sqrt( pow( cte, 2 ) );
}


void PID::UpdateTotalPID( )
{
    total_pid_value_ = - Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
}


void PID::AddToParameterAtIndex( int index, double amount )
{
    if( index == 0 )
    {
        Kp_ += amount;
    }
    else if( index == 1 )
    {
        Kd_ += amount;
    }
    else if( index == 2 )
    {
        Ki_ += amount;
    }
    else
    {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}
