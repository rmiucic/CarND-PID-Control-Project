#include "PID.h"

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

  time_step=1;
  stabilize_steps = 100;
  twidle_sep_cnt = 1500;
  twiddle_flag = true;
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
  //if((twiddle_flag == true) && ()
  //{
  //  if (pid.dp_Kp + pid.dp_Ki + pid.dp_Kd > 0.2)
  //  {
  //    
  //  }
  //} /*(twiddle_flag == true)*/
  time_step++;
}

double PID::TotalError() 
{
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}