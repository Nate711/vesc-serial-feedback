#include "AngularPDController.h"
#include <Arduino.h>
#include "utils.h"

#define MICROSPERSEC 1000000

/**
 * Constructor for angular proportional derivative controller
 * @param kp [description]
 * @param kd [description]
 */
AngularPDController::AngularPDController(float kp, float kd) {
  pd_constants.Kp = kp;
  pd_constants.Kd = kd;

  last_angular_error = 0.0;
  last_error_deriv = 0.0;
  last_command = 0.0;
  last_pterm = 0.0;
  last_dterm = 0.0;
}

/**
 * Compute the PID output given a new error value and dt.
 * Assumption: requires positive torque to increase encoder value!
 * @param  error     angle error
 * @param  dt_micros number of micros since last measurement
 * @return           PID output, saturated between -1, 1
 */
float AngularPDController::compute_command(float error, int dt_micros) {
  // Compute angular velocity of error
  if(first_loop) {
    last_error_deriv = 0.0;
    first_loop = false;
  } else {
    last_error_deriv = MICROSPERSEC *
                        utils_angle_difference(error, last_angular_error) /
                          dt_micros;
  }

  return compute_command(error, last_error_deriv);
}

/**
 * Compute the PID output given a new error value and dt
 * @param  error     angle error
 * @param  dt_micros number of micros since last measurement
 * @return           PID output, saturated between -1, 1
 */
float AngularPDController::compute_command(float error, float ang_vel) {
  // Compute p term
  float pterm = - pd_constants.Kp * error;

  // Compute d term
  // TODO: Constain / filter D term
  float dterm = - pd_constants.Kd * ang_vel;

  // Compute final command
  float command = pterm + dterm;

  // Constrain the output between -1.0 and 1.0
  // TODO: Check if the arduino constrain is still not working
  command = command > 1.0 ? 1.0 : command;
  command = command < -1.0 ? -1.0 : command;

  // Update memory of last command and components
  last_command = command;
  last_pterm = pterm;
  last_dterm = dterm;
  last_error_deriv = ang_vel;
  last_angular_error = error;

  return last_command;
}

/**
 * Return the last computed angle error
 * @return last computed angle error
 */
float AngularPDController::get_error() {
  return last_angular_error;
}

/**
 * Return the last computed angular velocity
 * @return the last computed angular velocity
 */
float AngularPDController::get_error_deriv() {
  return last_error_deriv;
}

/**
 * Return the last computed normalized command
 * @return the last computed normalized command
 */
float AngularPDController::get_command() {
  return last_command;
}

/**
 * Getter for pterm and dterm
 * @param pterm variable to put pterm in
 * @param dterm variable to put dterm in
 */
void AngularPDController::get_error_terms(float& pterm, float& dterm) {
  pterm = last_pterm;
  dterm = last_dterm;
}

/**
 * Getter for p and d gains
 * @param Kp [description]
 * @param Kd [description]
 */
void AngularPDController::get_gains(float& Kp, float& Kd) {
  Kp = pd_constants.Kp;
  Kd = pd_constants.Kd;
}

/**
 * Setter for p and d gains
 * @param Kp [description]
 * @param Kd [description]
 */
void AngularPDController::set_gains(float Kp, float Kd) {
  pd_constants.Kp = Kp;
  pd_constants.Kd = Kd;
}

//
// float AngularPDController::get_pterm() {
//   return last_pterm;
// }
//
// float AngularPDController::get_dterm() {
//   return last_dterm;
// }
