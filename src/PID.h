#ifndef PID_H
#define PID_H

class PID 
{
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double str_Kp_, double str_Ki_, double str_Kd_, double spd_Kp_, double spd_Ki_, double spd_Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte, double spd_dif);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 //private:
 
  /**
   * PID Errors
   */
  //double p_error;
  //double i_error;
  //double d_error;
  double pre_cte;
  double pre_spd_err;
  double str_err[3]; //0-P, 1-I, and 2-D errors for steering control
  double spd_err[3]; //0-P, 1-I, and 2-D errors for throtle control

  /**
   * PID Coefficients
   */ 
  //double Kp;
  //double Ki;
  //double Kd;
  double str_K[3]; //0-P, 1-I, and 2-D errors for steering control
  double spd_K[3]; //0-P, 1-I, and 2-D errors for throtle control

  //double dp_Kp;
  //double dp_Ki;
  //double dp_Kd;
  double dp[3];  //0-P, 1-I, and 2-D errors for steering control
  
  int time_step;
  int stabilize_steps;
  int twidle_step_cnt;
  bool twiddle_flag;
  bool first_run;
  int par_index;
  bool tried_adding;
  bool tried_subtracting;
  double best_err;
  double run_err;
};

#endif  // PID_H