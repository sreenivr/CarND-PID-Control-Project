#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_error = 0.0;
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  std::cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << std::endl;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;              // Proportional Error (P)
  i_error += cte;             // Integral Eror (I)
  d_error = cte - prev_error; // Differetial Error (D)
  prev_error = cte;           // Update previous error
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return -(Kp * p_error) -(Ki * i_error) - (Kd * d_error);  // TODO: Add your total error calc here!
}

