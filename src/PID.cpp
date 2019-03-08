#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double str_Kp_, double str_Ki_, double str_Kd_, double spd_Kp_, double spd_Ki_, double spd_Kd_) 
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  //Kp = Kp_;
  //Ki = Ki_;
  //Kd = Kd_;
  str_K[0] = str_Kp_;
  str_K[1] = str_Ki_;
  str_K[2] = str_Kd_;

  spd_K[0] = spd_Kp_;
  spd_K[1] = spd_Ki_;
  spd_K[2] = spd_Kd_;
  //p_error = 0.0;
  //i_error = 0.0;
  //d_error = 0.0;
  
  str_err[0] = 0.0;
  str_err[1] = 0.0;
  str_err[2] = 0.0;
  pre_cte    = 0.0;

  spd_err[0]  = 0.0;
  spd_err[1]  = 0.0;
  spd_err[2]  = 0.0;
  pre_spd_err = 0.0;
  dp[0]  = 0.1*str_K[0];
  dp[1]  = 0.1*str_K[1];
  dp[2]  = 0.1*str_K[2];

  time_step=1;
  stabilize_steps = 100;
  twidle_step_cnt = 1500;
  twiddle_flag = true;
  first_run = true;
  par_index = 0;
  tried_adding = tried_subtracting = false;
  best_err = std::numeric_limits<double>::max();

}

void PID::UpdateError(double cte, double spd_dif) 
{
  /**
   * TODO: Update PID errors based on cte.
   */
  str_err[0]  = cte;
  str_err[1] += cte;
  str_err[2]  = cte - pre_cte;
  pre_cte = cte;

  spd_err[0]  = spd_dif;
  spd_err[1] += spd_dif;
  spd_err[2]  = spd_dif - pre_spd_err;
  pre_spd_err = spd_dif;

  /* Twiddle */
  if(time_step % (stabilize_steps + twidle_step_cnt) > stabilize_steps)
  { //only update run error after stabilization period ended
    run_err+= pow(cte,2);
  }

  if((twiddle_flag == true) && (time_step % (stabilize_steps + twidle_step_cnt) == 0))
  {
    cout << "time_step: " << time_step << endl;
    cout << "best_err: " << best_err << endl;
    cout << "run_err: " << run_err << endl;
    if (run_err < best_err)
    {
      cout << "Improvement!" << endl;
      best_err = run_err;
      // next parameter
      if(time_step != (stabilize_steps + twidle_step_cnt))
      {
        dp[par_index] *= 1.1;
      }
      par_index = (par_index + 1) % 3; // move on tho the next parameter
      tried_adding = tried_subtracting = false;
    }
    
    if (!tried_adding && !tried_subtracting) 
    {
      // try adding dp[i] to params[i]
      str_K[par_index] += dp[par_index];
      tried_adding = true;
    }
    else if (tried_adding && !tried_subtracting) 
    {
      // try subtracting dp[i] from params[i]
      str_K[par_index] -= 2*dp[par_index];
      tried_subtracting = true;         
    }
    else 
    {
        // set it back, reduce dp[i], move on to next parameter
      str_K[par_index] += dp[par_index];
      dp[par_index] *= 0.9;
      // next parameter
      par_index = (par_index + 1) % 3; // move on tho the next parameter
      tried_adding = tried_subtracting = false;
    }

    cout << "new parameters" << endl;
    cout << "P: " << str_K[0] << ", I: " << str_K[1] << ", D: " << str_K[2] << endl;        

    run_err = 0; // clear run error 
  } /*(twiddle_flag == true)*/
  time_step++;
}

double PID::TotalError() 
{
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}
