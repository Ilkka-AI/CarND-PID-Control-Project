#ifndef TWIDDLE_H
#define TWIDDLE_H
#include <vector>
using namespace std;


class Twiddle {
public:


int iterations;
double best_total_error_per_frame;
 
 
//  double Kp;
//  double Ki;
//  double Kd;

int twiddle_it;
vector <double> p;
vector <double> dp;
int next_primary_or_secondary_update;

  //std::vector<double> tail_integral;

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize Twiddle.
  */
  void Init(double Kp_i, double Ki_i, double Kd_i);

  /*
  * 
  */
  //void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
//  double TotalError();

  void TwiddleUpdate(double total_error_per_frame);
};

#endif /* TWIDDLE_H */
