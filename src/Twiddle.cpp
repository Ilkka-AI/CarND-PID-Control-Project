#include "Twiddle.h"
#include <math.h>
#include <vector>
using namespace std;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double Kp_i, double Ki_i, double Kd_i) {
  iterations=0;
  // Initial max error. 4.5 is the distance of lane edge from the center
  best_total_error_per_frame=4.5*4.5;
  // Twiddle has primary and secondary update proposals	
  next_primary_or_secondary_update=1;
  // Iterator over the 3 terms
  twiddle_it = 0;
  p.resize(3);
  dp.resize(3);
  for(int i;i<3;i++){
    p[i]=0;
    dp[i]=1;
  }
  p[0]=Kp_i;
  p[1]=Ki_i;
  p[2]=Kd_i;

  // Initial update proposal values
  dp[0]=0.2;
  dp[1]=0.01;
  dp[2]=3;

  // First attempt
  p[twiddle_it]+=dp[twiddle_it];
}

void Twiddle::TwiddleUpdate(double total_error_per_frame){
  iterations=0;

  // If primary update is being tried
  if(next_primary_or_secondary_update==1){
  //If the primary update proposal (addition) decreases error, update value  
    if( total_error_per_frame < best_total_error_per_frame){
      best_total_error_per_frame = total_error_per_frame;
      dp[twiddle_it] *= 1.1;
      next_primary_or_secondary_update=1;
      twiddle_it=(twiddle_it+1) % 3;
      p[twiddle_it] += dp[twiddle_it];
       
  // If the addition does not decrease error, try subtraction, thus go the secondary update       
    }else{
      p[twiddle_it] -= 2 * dp[twiddle_it];
      next_primary_or_secondary_update=2;
      }   
  // If secondary update was tried
  }else{
  // If subtraction worked, accept the proposal
     if(total_error_per_frame < best_total_error_per_frame){
       best_total_error_per_frame = total_error_per_frame;
       dp[twiddle_it] *= 1.1;
     }
     // If not, go to next parameter
     else{
       p[twiddle_it] += dp[twiddle_it];
       dp[twiddle_it] *= 0.9;
     }
  next_primary_or_secondary_update=1;
  twiddle_it=(twiddle_it+1) % 3; 
  // Run the simulation for the next parameter
  p[twiddle_it] += dp[twiddle_it];   
}
    

}

