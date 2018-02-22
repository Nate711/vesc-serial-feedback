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

  utils_norm_angle_center(normalized); // normalize to [-180 180)

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
  // normalize angle to -180 to 180
  utils_norm_angle_center(raw_angle);

  return raw_angle;
}

/**
* Sends current command to the VESC
*/
void DualVESC::_send_current(float current_A, float current_B) {
  vesc_uart_A.set_current(current_A);
  vesc_uart_B.set_current(current_B);
}


/*************** PUBLIC METHODS ****************/

/**
* Constructs VESC object with initial params
* I dont understand member initializer lists :( but CANtx breaks without it
*
* @param vesc_encoder_reading_period : Tell the VESC object how often it is
*                                      getting new encoder readings.
*                                      If the VESC is sending at 2000hz,
*                                      use 500us
* @param serial_port : reference to serial port to use to communicate to the
*                      teensy
*/
// TODO add serial object to constructor
DualVESC::DualVESC(int vesc_encoder_reading_period,
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

  VESC_ENCODER_PERIOD = vesc_encoder_reading_period;
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
* Updates the VESC objects knowledge of the motor angle
* Takes between 4 and 5 us when using the while loop-based normalize
* angle function
*
* @param angle : measured position in vesc encoder frame. degrees
* automatically normalizes the given angle (no > 180deg moves)
*/
void DualVESC::update_angle_A(float angle) {
  float corrected = angle;

  // convert to -180 180 for safety
  utils_norm_angle_center(corrected);

  // Compute velocity in deg per s
  // This computation is subject to noise!
  // 37-38 us loop time
  // this line takes 6-8 us OUTDATED 12-21

  // Hardcoded sampling rate of 1000hz, make sure to change if changing
  // the send frequency of the VECS

  // TEST THIS SOLUTION TO THE BAD BUG
  vesc_vel_A = (MICROSPERSEC / VESC_ENCODER_PERIOD) * utils_angle_difference(corrected,vesc_angle_A);
  // add lowpass filter didn't help much or made the vibrations worse!
  // Tested 0.5 and 0.8 * temp

  // Update angle state
  vesc_angle_A = corrected;
}

/**
* Updates the VESC objects knowledge of the motor angle
* Takes between 4 and 5 us when using the while loop-based normalize
* angle function
*
* @param angle : measured position in vesc encoder frame. degrees
* automatically normalizes the given angle (no > 180deg moves)
*/
void DualVESC::update_angle_B(float angle) {
  float corrected = angle;

  utils_norm_angle_center(corrected);

  // Compute velocity in deg per s
  // Hardcoded sampling rate of 1000hz, make sure to change if changing
  // the send frequency of the VECS
  vesc_vel_B = (MICROSPERSEC / VESC_ENCODER_PERIOD) * utils_angle_difference(corrected,vesc_angle_B);
  vesc_angle_B = corrected;
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
* Given the two motor angles, return theta, the leg to foot angle
* @param  alpha angle of link on the RIGHT (smaller angle usually)
* @param  beta  angle of link on the LEFT (larger angle usually)
* @return       angle between horizontal and line from hip to foot
*/
float theta(float alpha, float beta) {
  // Takes care of edge case where links are above the horizontal
  // by limiting alpha and beta to [-180 180] degs
  alpha = utils_angle_difference(alpha, 0.0f);
  beta = utils_angle_difference(beta, 0.0f);

  return (alpha + beta) * 0.5;
}

float gamma(float alpha, float beta) {
  // Takes care of edge case where links are above the horizontal
  // by limiting alpha and beta to [-180 180] degs
  alpha = utils_angle_difference(alpha, 0.0f);
  beta = utils_angle_difference(beta, 0.0f);
  return (beta - alpha) * 0.5;
}

/**
* Compute PID output and send to VESC. Uses last given values
*/
// TODO complete decoupled PID
// TODO check directions of current going to motor!!
void DualVESC::pid_update(float theta_setpoint, float gamma_setpoint) {
  // alpha is angle of motor with link closest to right horizontal
  // beta is angle of motor with link further away
  // beta should be greater than alpha with direct drive
  //   beta  /\ alpha
  //         \/
  float alpha = vesc_to_normalized_angle(vesc_angle_A, encoder_offset_A, encoder_direction_A);
  float beta = vesc_to_normalized_angle(vesc_angle_B, encoder_offset_B, encoder_direction_B);

  float error_theta = utils_angle_difference(theta(alpha,beta), theta_setpoint);
  float error_gamma = utils_angle_difference(gamma(alpha,beta), gamma_setpoint);

  float theta_current = max_current *
         pos_controller_theta.compute_command(error_theta, VESC_ENCODER_PERIOD);
  float gamma_current = max_current *
         pos_controller_gamma.compute_command(error_gamma, VESC_ENCODER_PERIOD);

  // TODO frame all motor angle conversions as coordinate system changes
  // TODO multiply currents by motor direction so pid based on normalized angles
  // actually work!

  // motor torque = J.T * F
  // J = [0.5, 0.5; -0.5, 0.5]
  float current_A = 0.5 * theta_current - 0.5 * gamma_current;
  float current_B = 0.5 * theta_current + 0.5 * gamma_current;

  Serial.print(error_theta);
  Serial.print('\t');
  Serial.println(error_gamma);

  // TODO current_B should be multiplied by -1
  // TODO, figure out sign of current and encoder shit
  _send_current(current_A, current_B);
}

/**
* Update kP and kD for teensy-side PID control
* @param kP proportional gain
* @param kD derivative gain
*/
void DualVESC::set_pid_gains(float kP, float kD) {
  pos_controller_theta.set_gains(kP, kD);
  pos_controller_gamma.set_gains(kP, kD);
}
