#include "PID.h"

// Initialize the constrcutor 

PID::PID()
{
    Kd      =   0.0; 
    Kp      =   0.0; 
    Ki      =   0.0; 
    state   =   0.0; 
    desX    =   0.0; 
}

PID::~PID()
{
}

/**
 * @brief compute the PID control 
 * 
 * 
 * @return double 
 */
double PID::computeForce()
{
    double error = desX - state; 
    double prev_err = 0.0 ; 
    double sum_err = 0.0 ; 

    double computeForce_ ; 
    
    computeForce_ = Kp*error + Kd*prev_err + Ki*sum_err; 

    prev_err = error; 

    sum_err += error; 


    // check if the sum is higher that 0.5 
    if (sum_err > 0.5)
    {
        sum_err = 0.0; 
    }
    
    return computeForce_; 
}

/**
 * @brief 
 * 
 * @param Kp_ proportional gain 
 * @param Kd_ derivative gain 
 * @param Ki_ integral gain
 */
void PID::setGains(double Kp_, double Kd_, double Ki_)
{
    Kp = Kp_; 
    Kd = Kd_; 
    Ki = Ki_; 
}

/**
 * @brief 
 * 
 * @param x_ set the state
 */
void PID::setState(double x_)
{
    state = x_; 

}

/**
 * @brief set the desX state
 * 
 * @param desx_ 
 */
void PID::setdesX(double desx_)
{
    desX = desx_; 
}

/**
 * @brief 
 * 
 * @return int 
 */
int main()
{
    PID pid; 

    double Ki_g = 0.0; 
    double Kp_g = 1.0; 
    double Kd_g = 0.01; 

    pid.setGains(Kp_g, Kd_g, Ki_g); 

    double X_ = 1.0;
    double desX__ = 0.5; 
 
    pid.setState(X_); 
    pid.setdesX(desX__); 
    
    double output =  pid.computeForce(); 

    std::cout << "pid out "<< output << std::endl; 

    return 0; 
}