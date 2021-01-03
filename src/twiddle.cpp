#include "twiddle.h"
#include <cmath>
#include <limits>
#include <iostream>

void Twiddle::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize PID object
  pid.Init(Kp_, Ki_, Kd_);
  
  p[0] = Kp_;
  p[1] = Ki_;
  p[2] = Kd_;
  
  // Step size
  dp[0] = 0.03;    // P
  dp[1] = 0.0001;  // I
  dp[2] = 0.1;      // D
  
  best_err  = std::numeric_limits<double>::max();
  acc_error = 0.0;
  acc_count = 0;
  i         = 0;
  currentState = State::START;
  //newParamAvailable = false;
}

// TODO: Can we return if new params are available ?
void Twiddle::UpdateError(double cte) {
  // Update the error in PID object
  pid.UpdateError(cte);
  
  acc_error += pow(cte, 2); // Accumulate squared error
  acc_count++;              // Increment num errors accumulated so far
  //newParamAvailable = false;
  if(acc_count >= N) {
    double cte = acc_error / acc_count;
    // trigger the state machine
    FsmEvent(cte); 
    // Reset the accumulated error and count
    acc_error = 0.0;
    acc_count = 0;
  } 
}

double Twiddle::TotalError() {
  return pid.TotalError();
}

/*
bool Twiddle::NewParamAvailable() {
  return newParamAvailable;
}

void Twiddle::GetParams(double &Kp_, double &Ki_, double &Kd_) {
  Kp_ = p[0];
  Ki_ = p[1];
  Kd_ = p[2];
}
*/

// Call the event handler based on the current state
void Twiddle::FsmEvent(double cte){
  std::cout << "Best Error = " << best_err << " cte = " << cte << " i = " << i << " Sum dp = " << (dp[0]+dp[1]+dp[2]) << std::endl;
  
  switch(currentState) {
    case State::START: StartHandler(cte);      break;
    case SEARCH_UP:    SearchUpHandler(cte);   break;
    case SEARCH_DOWN:  SearchDownHandler(cte); break;
    case DONE:         DoneHandler(cte);       break;
  }
}

// Handler for START state
void Twiddle::StartHandler(double cte) {
  best_err = pow(cte, 2); // Set the best error to the first error squared
  p[i] += dp[i];  // Increase p by dp
  pid.Init(p[0], p[1], p[2]);
  //newParamAvailable  = true;
  currentState = State::SEARCH_UP; // Update the current state
}

// We are searching for a optimal paramter by increasing "p"
void Twiddle::SearchUpHandler(double cte) {
  if (cte < best_err) {
    best_err = cte; // Update best error
    dp[i]   *= 1.1; // Increase the step size
    SetNextParam(); // Lets adjust the next parameter
    p[i] += dp[i];  // increase the next parameter by step size
    pid.Init(p[0], p[1], p[2]);
    currentState = State::SEARCH_UP; // Remain in the same state
  } else {
    // cte didn't improve, so lets decrease the p and check the cte
    p[i] -= 2 * dp[i]; // 2* is to compensate for the increment done previously
    pid.Init(p[0], p[1], p[2]);
    //newParamAvailable = true;
    currentState = State::SEARCH_DOWN; // Update the current state to search down
  }
}

void Twiddle::SearchDownHandler(double cte) {
  if(cte < best_err) {
    best_err = cte; // Update best error
    dp[i]   *= 1.1; // Increase the step size
    SetNextParam(); // Lets adjust the next parameter
    p[i] += dp[i];  // increase the next parameter by step size
    pid.Init(p[0], p[1], p[2]);
    currentState = State::SEARCH_UP; // Go to Search up state
  } else {
    // cte didn't improve, lets revert back p to previous value
    p[i] += dp[i];
    pid.Init(p[0], p[1], p[2]);
    //newParamAvailable = true;
    dp[i] *= 0.9;
    SetNextParam(); // Lets adjust the next parameter
    if(dp[0] + dp[1] + dp[2] < threshold) {
      currentState = State::DONE;
    } else {
      p[i] += dp[i];
      pid.Init(p[0], p[1], p[2]);
      currentState = State::SEARCH_UP; // Go to Search up state
    }
  }
  
}

void Twiddle::DoneHandler(double cte) {
  // Nothing to do here
  std::cout << "-------------DONE-----------" << std::endl;
}

// Returns if Twiddle has reached the target
/*
bool Twiddle::IsDone() {
  return (currentState == State::DONE) ? true : false;
}
*/

// Set the next parameter to adjust
void Twiddle::SetNextParam() {
  i++;
  i = i % 3;
}
