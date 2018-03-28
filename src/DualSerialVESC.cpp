/*
Copyright 2012-2014 Nathan Kau nathankau@stanford.edu

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "DualSerialVESC.h"
#include "buffer.h"
#include "datatypes.h"
#include "utils.h"
#include "VescUart.h"

/*************** PRIVATE METHODS ****************/

/**
* Converts an angle in the vesc encoder reference frame to a normalized angle
* @param  raw_angle [description]
* @return           [description]
*/
float DualVESC::vesc_to_normalized_angle(float raw_angle,
                                         float encoder_offset,
                                         float encoder_direction) {
  float normalized = raw_angle;

  // correct for encoder direction, ie if + angle is CW or CCW
  if(encoder_direction == -1) {
    normalized = utils_angle_difference(0, normalized);
  }
  normalized += encoder_offset; // add encoder offset

  utils_norm_angle(normalized); // normalize to [0 360)

  return normalized;
}

/**
* Converts an angle in the robot frame to an angle in the vesc encoder frame
* @param  raw_angle [description]
* @return           [description]
*/
float DualVESC::normalized_to_vesc_angle(float normalized_angle,
                                         float encoder_offset,
                                         float encoder_direction) {
  float raw_angle = normalized_angle;

  raw_angle -= encoder_offset; // subtract offset

  // reverse if opposite direction
  if(encoder_direction == -1) {
    raw_angle = utils_angle_difference(0, raw_angle);
  }
  // normalize angle to 0 to 360
  utils_norm_angle(raw_angle);

  return raw_angle;
}

/**
 * TODO DOC
 * @param IA [description]
 * @param IB [description]
 */
void DualVESC::read_current(float& IA, float& IB) {
  IA = last_current_A;
  IB = last_current_B;
}

// TODO move watchdog code elsewhere!!!
/**
* Sends current command to the VESC
*/
void DualVESC::_send_current(float current_A, float current_B) {
  if((vesc_A_watchdog > VESC_TIMEOUT || vesc_B_watchdog > VESC_TIMEOUT) && alive) {
    die();
  }
  else {
    last_current_A = current_A;
    last_current_B = current_B;

    vesc_uart_A.set_current(current_A);
    vesc_uart_B.set_current(current_B);
  }
}


/*************** PUBLIC METHODS ****************/

/**
* Constructs VESC object with initial params
* I dont understand the point of member initializer lists
*
* @param vesc_encoder_reading_period : Tell the VESC object how often it is
*                                      getting new encoder readings.
*                                      If the VESC is sending at 2000hz,
*                                      use 500us
* @param serial_port : reference to serial port to use to communicate to the
*                      teensy
*/
// TODO add serial object to constructor
DualVESC::DualVESC(int _vesc_encoder_period,
                   HardwareSerial* serial_port1,
                   HardwareSerial* serial_port2) :
                   pos_controller_theta(0,0),
                   pos_controller_gamma(0,0),
                   vesc_uart_A(serial_port1),
                   vesc_uart_B(serial_port2){

  // the first time_delta will be large and will give small deg per sec which is ok
  time_last_angle_read = 0;

  vesc_angle_A = 0.0;
  vesc_angle_B = 0.0;
  vesc_vel_A = 0;
  vesc_vel_B = 0;

  vesc_encoder_period = _vesc_encoder_period;
}

/**
 * TODO DOC
 * @return [description]
 */
bool DualVESC::isAlive() {
  return alive;
}

/**
 * TODO DOC
 */
void DualVESC::die() {
  alive = false;
  vesc_uart_A.set_current(0.0);
  vesc_uart_B.set_current(0.0);

  Serial.println("VESC WATCHDOG TRIGGERED");
}

/**
 * TODO: create doc
 */
void DualVESC::reset_watchdogs() {
  vesc_A_watchdog = 0;
  vesc_B_watchdog = 0;
  alive = true;
}

/**
* Processes a given byte over the teensy-vesc serial port.
* TODO: Don't update position if the type of message received wasn't a
* position update
*
* @param rx_data byte from VESC sent over serial bus
*/
void DualVESC::packet_process_byte_A(uint8_t rx_data) {
  if(vesc_uart_A.packet_process_byte(rx_data,0)) {

    update_angle_A(vesc_uart_A.get_rotor_position());

    // Debug code for receiving messages
    // Serial.print(micros());
    // Serial.print("\t");
    // Serial.println(vesc_angle);
  }
}

/**
* Processes a given byte over the teensy-vesc serial port.
* TODO: Don't update position if the type of message received wasn't a
* position update
*
* @param rx_data byte from VESC sent over serial bus
*/
void DualVESC::packet_process_byte_B(uint8_t rx_data) {
  if(vesc_uart_B.packet_process_byte(rx_data,0)) {
    update_angle_B(vesc_uart_B.get_rotor_position());
  }
}

/**
* Sets up the vesc object to talk over this CAN ID channel
* @param _encoder_offset    float, encoder offset
* @param _encoder_direction int
* @param _max_current       float, maximum current to send
*/
void DualVESC::attach(float _encoder_offset_A, int _encoder_direction_A,
                      float _encoder_offset_B, int _encoder_direction_B,
                      float _max_current) {
  encoder_offset_A = _encoder_offset_A;
  encoder_direction_A = _encoder_direction_A;
  encoder_offset_B = _encoder_offset_B;
  encoder_direction_B = _encoder_direction_B;
  max_current = _max_current;
}

/**
* Sends position CAN message to motor to update position hold command.
* Currently only implements VESC-side position hold.
* @param deg normalized target angle in degrees
*/
void DualVESC::write(float theta, float gamma) {
  pid_update(theta,gamma);
}

/**
* Sends CAN message to set current
* @param current desired current in amps
*/
void DualVESC::write_current(float current_A, float current_B) {
  _send_current(current_A, current_B);
}

/**
* Returns the last read normalized motor position in degrees. Note
* that the motor position read is not the commanded position, but
* the actual, last-read motor position
* @return motor position
*/
float DualVESC::read_A() {
  return vesc_to_normalized_angle(vesc_angle_A, encoder_offset_A, encoder_direction_A);
}

/**
* Returns the last read normalized motor position in degrees. Note
* that the motor position read is not the commanded position, but
* the actual, last-read motor position
* @return motor position
*/
float DualVESC::read_B() {
  return vesc_to_normalized_angle(vesc_angle_B, encoder_offset_B, encoder_direction_B);
}

/**
 * Take in an angle received over serial and update the angle of the motor
 * in the operation space
 * @param angle angle received over serial from the vesc
 */
void DualVESC::update_angle_A(float angle) {
  float corrected = angle;
  utils_norm_angle(corrected);

  if (corrected - prev_angle_A > 180) {
    // TODO Document that the initial value of prev angle will fuck up your rot count
    if(prev_angle_A != 0.0) {
      num_rotations_A--;
    }
  } else if (corrected - prev_angle_A < -180) {
    if(prev_angle_A != 0.0) {
      num_rotations_A++;
    }
  }

  vesc_vel_A = (MICROSPERSEC / vesc_encoder_period) * utils_angle_difference(corrected,vesc_angle_A);
  vesc_vel_A *= encoder_direction_A;

  // Update angle state
  vesc_angle_A = (360.0f * num_rotations_A + corrected);
  vesc_angle_A = encoder_direction_A * (vesc_angle_A - encoder_offset_A);

  prev_angle_A = corrected;

  // reset watchdog timer
  vesc_A_watchdog = 0;
}

/**
 * Take in an angle received over serial and update the angle of the motor
 * in the operation space
 * @param angle angle received over serial from the vesc
 */
void DualVESC::update_angle_B(float angle) {
  float corrected = angle;
  utils_norm_angle(corrected);

  // TODO TODO TODO MASSIVE BUG MUST START LEG IN CORRECT POSITION OR BREAKS
  if (corrected - prev_angle_B > 180) {
    if(prev_angle_B != 0.0) {
      num_rotations_B--;
    }
  } else if (corrected - prev_angle_B < -180) {
    if(prev_angle_B != 0.0) {
      num_rotations_B++;
    }
  }

  vesc_vel_B = (MICROSPERSEC / vesc_encoder_period) * utils_angle_difference(corrected,vesc_angle_B);
  vesc_vel_B *= encoder_direction_B;

  // Update angle state
  vesc_angle_B = 360.0f * num_rotations_B + corrected;
  vesc_angle_B = encoder_direction_B * (vesc_angle_B - encoder_offset_B);

  prev_angle_B = corrected;

  // reset watchdog timer
  vesc_B_watchdog = 0;
}

/**
* Prints VESC object state
*/
// TODO debug both controllers
void DualVESC::print_debug() {
  if(last_print_debug > 100) {
    last_print_debug = 0;

    Serial.print("O: ");
    Serial.print(pos_controller_theta.get_command());
    Serial.print(" \tEr: ");
    Serial.print(pos_controller_theta.get_error());
    Serial.print(" \tEr.w:  ");
    Serial.print(pos_controller_theta.get_error_deriv());
    Serial.print(" \tw: ");
    Serial.print(vesc_vel_A);
    Serial.print(" \tKp: ");
    float pterm,dterm;
    pos_controller_theta.get_error_terms(pterm, dterm);
    Serial.print(pterm);
    Serial.print(" \tKd: ");
    Serial.println(dterm);
  }
}

/***** OLD ONBOARD PID CODE *******/
/**
* Compute PID output and send to VESC given a normalized angle set
* point. Uses last given position values.
* @param set_point normalized angle set point
*/
void DualVESC::pid_update_normalized(float set_point) {
  // pid_update(normalized_to_vesc_angle(set_point, offset, dir));
}

/**
 * Calculates theta and gamma from alpha and beta.
 * Edge case issues:
 * 1) When alpha and beta are close together,
 * gamma will fluctate between small values and large values as the links
 * cross. Not sure if this is a problem or just a characteristic of how it must
 * work.
 * TODO: test this code!
 * @param alpha angle of rightmost link to the horizontal, positive angles
 * increase clockwise
 * @param beta  angle of leftmost link to the horizontal, ^
 * @param theta placeholder for theta result
 * @param gamma placeholder for gamma result
 */
void theta_gamma(float alpha, float beta, float& theta, float& gamma) {
  theta = (alpha + beta) * 0.5;
  utils_norm_angle_q1q2(theta);

  gamma = (beta - alpha) * 0.5;
  utils_norm_angle_q1q2(gamma);
}
/**
* Given the two motor angles, return theta, the leg to foot angle
* @param  alpha angle of link on the RIGHT (smaller angle usually)
* @param  beta  angle of link on the LEFT (larger angle usually)
* @return       angle between horizontal and line from hip to foot
*/
float theta(float alpha, float beta) {
  return (alpha + beta) * 0.5;
}

/**
* Given the two motor angles, return gamma, the angle seperation of the legs
* divided by two
* @param  alpha angle of link on the RIGHT (smaller angle usually)
* @param  beta  angle of link on the LEFT (larger angle usually)
* @return       angle between horizontal and line from hip to foot
*/
float gamma(float alpha, float beta) {
  return (beta - alpha) * 0.5;
}

elapsedMillis lastprint_A = 0;
/**
 * TODO: Create doc
 * @param alpha_setpoint [description]
 */
void DualVESC::update_A(float alpha_setpoint) {
  float alpha = vesc_angle_A;
  float alpha_error = alpha - alpha_setpoint;

  // TODO fix encoder direction A multiplication hack
  float alpha_current = encoder_direction_A * max_current *
        pos_controller_theta.compute_command(alpha_error, vesc_encoder_period);
  _send_current(alpha_current,0);

  if(lastprint_A > 500) {
    Serial.print("A: ");
    Serial.print(alpha);
    Serial.print("\tI: ");
    Serial.println(alpha_current);
    lastprint_A = 0;
  }
}

elapsedMillis lastprint = 0;

/**
* Calculates and returns the current value of gamma based on current
* encoder angles.
* @return gamma_deg the current gamma
*/
float DualVESC::get_gamma() {
  // Calculate gamma and return
  float gamma_deg;
  gamma_deg = gamma(vesc_angle_A,vesc_angle_B);
  return gamma_deg;
}

/**
 * Compute PID output and send to VESC. Uses last given values
 * @param theta_setpoint Set point for leg theta angle
 * @param gamma_setpoint  Set point for leg gamma angle
 */
void DualVESC::pid_update(float theta_setpoint, float gamma_setpoint) {
  // alpha is angle of motor with link closest to right horizontal
  // beta is angle of motor with link further away
  // vesc_angle_A is alpha, vesc_angle_B is beta
  // beta should be greater than alpha with direct drive
  //   beta  /\ alpha
  //         \/

  // Calculate theta and gamms
  float theta_deg, gamma_deg;
  // theta_gamma(alpha,beta,theta_deg,gamma_deg);
  theta_deg = theta(vesc_angle_A,vesc_angle_B);
  gamma_deg = gamma(vesc_angle_A,vesc_angle_B);

  float error_theta = theta_deg - theta_setpoint;
  float error_gamma = gamma_deg - gamma_setpoint;

  float theta_current = max_current *
         pos_controller_theta.compute_command(error_theta, vesc_encoder_period);
  float gamma_current = max_current *
         pos_controller_gamma.compute_command(error_gamma, vesc_encoder_period);

  // TODO frame all motor angle conversions as coordinate system changes
  // TODO multiply currents by motor direction so pid based on normalized angles
  // actually work!

  // motor torque = J.T * F
  // J = [0.5, 0.5; -0.5, 0.5]
  float current_A = encoder_direction_A *
                    (0.5 * theta_current - 0.5 * gamma_current);
  float current_B = encoder_direction_B *
                    (0.5 * theta_current + 0.5 * gamma_current);

  // TODO resolve anti cogging and anti stiction 2/27
  // if(current_A > 0) {
  //   current_A += 0.5;
  // } else {
  //   current_A -= 0.5;
  // }

  if(lastprint > 100) {
    Serial.print("AB: ");
    Serial.print(vesc_angle_A);
    Serial.print('\t');
    Serial.print(vesc_angle_B);
    Serial.print("\tThY: ");
    Serial.print(theta_deg);
    Serial.print('\t');
    Serial.print(gamma_deg);
    Serial.print("\tThY Err: ");
    Serial.print(error_theta);
    Serial.print('\t');
    Serial.print(error_gamma);
    Serial.print("\tThY SP: ");
    Serial.print(theta_setpoint);
    Serial.print('\t');
    Serial.print(gamma_setpoint);
    Serial.print("\t Iab: ");
    Serial.print(current_A);
    Serial.print('\t');
    Serial.println(current_B);
    lastprint = 0;
  }

  _send_current(current_A, current_B);
}

/**
* Update kP and kD for teensy-side PID control
* @param kP proportional gain
* @param kD derivative gain
*/
void DualVESC::set_pid_gains(float kP_the, float kD_the, float kP_gam, float kD_gam) {
  pos_controller_theta.set_gains(kP_the, kD_the);
  pos_controller_gamma.set_gains(kP_gam, kD_gam);
}
