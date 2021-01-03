#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"

class Twiddle {
public:
  Twiddle()  { }
  ~Twiddle() { }
  
  void Init(double Kp_, double Ki_, double Kd_);
  void UpdateError(double cte);
  double TotalError();
  
  // Is following required ?
  /*
  bool NewParamAvailable();
  void GetParams(double &Kp_, double &Ki_, double &Kd_);
  bool IsDone();
  */
  
private:
  PID    pid;
  double best_err;
  double p[3];
  double dp[3];
  double acc_error;
  int    acc_count;
  
  const int N = 1000;
  int   i;            // index of the param being tunned
  const double threshold = 0.1;
  //bool newParamAvailable;
  void FsmEvent(double cte);
  
  enum State {
    START,
    SEARCH_UP,
    SEARCH_DOWN,
    DONE
  };
  
  State currentState;
  
  // Event handlers for each state
  void StartHandler(double cte);
  void SearchUpHandler(double cte);
  void SearchDownHandler(double cte);
  void DoneHandler(double cte);
  
  // Helper function
  void SetNextParam();

};

#endif // TWIDDLE_H