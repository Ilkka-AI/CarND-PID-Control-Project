#include "PID.h"
#include <math.h>
#include "Twiddle.h"
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

Kp=Kp_;
Ki=Ki_;
Kd=Kd_;
cte_prev=0;
cte_integral=0;

// the pid has a twiddle object, initialize
twiddle1.Init(Kp_,Ki_,Kd_); 
total_error_sum=0;

// Initial Twiddle parameter update proposal
Kp+=twiddle1.dp[0];
}


void PID::UpdateCoefs(){
// Update new parameters coming from the Twiddle update
Kp=twiddle1.p[0];
Ki=twiddle1.p[1];
Kd=twiddle1.p[2];
// Start driving from the beginning
cte_prev=0;
cte_integral=0;
total_error_sum=0;
}

void PID::UpdateError(double cte) {
// Update error terms at every iteration
p_error=cte;
i_error+=cte;
d_error=cte-cte_prev;
cte_prev=cte;
}


void PID::TotalError() {
// Accumulate the total error
total_error_sum+=p_error*p_error;


}

