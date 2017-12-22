#ifndef ANGULARPDCONTROLLER_H
#define ANGULARPDCONTROLLER_H

class AngularPDController {
private:
  // Stores whether this is fthe first time through, important for initializing
  // time derivatives
  bool first_loop = true;

  // stores last given error
  float last_error_deriv;

  // stores the last angular error float
  float last_angular_error;

  // stores the last outputted command
  float last_command;

  // stores the last computed pterm and dterm
  float last_pterm, last_dterm;

  // struct for holding PD constants
  struct PD_Constants {
  	float Kp;
  	float Ki;
  	float Kd;
  } pd_constants;

public:
  AngularPDController(float kp, float kd);
  float compute_command(const float& error, int dt_micros);
  float compute_command(const float& error, float ang_vel);

  float get_error();
  float get_error_deriv();

  float get_command();

  void get_error_terms(float& pterm, float& dterm);

  void set_gains(float Kp, float Kd);
  void get_gains(float& Kp, float& Kd);
};

#endif /*ANGULARPDCONTROLLER_H*/
