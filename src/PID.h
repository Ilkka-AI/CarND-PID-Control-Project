#ifndef PID_H
#define PID_H
#include "Twiddle.h"

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error_sum;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double cte_prev;
  double cte_integral;
  Twiddle twiddle1;
  
  //std::vector<double> tail_integral;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_);
void UpdateCoefs();
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  void TotalError();

  
};

#endif /* PID_H */
