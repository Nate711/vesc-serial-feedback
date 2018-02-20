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

#ifndef DUAL_SERIAL_VESC_H
#define DUAL_SERIAL_VESC_H

#include <Arduino.h>
#include "AngularPDController.h"
#include "VescUart.h"


class DualVESC {
private:
  // Holds time in micros of the last time the object got an angle measurement
  // It's important that this time is consistent
	long time_last_angle_read;

  // Holds the most recent time delta between measurements in MICROS
  long last_time_delta_micros=30;

  // Offset and direction for encoder
  // it might be a bad idea to have direction multipliers... think about it

  float encoder_offset_A;
  int encoder_direction_A;

	float encoder_offset_B;
  int encoder_direction_B;

  // Default current limit
  float max_current = 0; // amps

  // corrected position and velocity of the motor
  float vesc_angle_A = 0;
	float vesc_angle_B = 0;
  float vesc_vel_A = 0;
	float vesc_vel_B = 0;

	// TODO Serial object to use for comms

  // PID object to control angular position
  AngularPDController pos_controller_A;
	AngularPDController pos_controller_B;

	// Serial object
	VESCUart vesc_uart_A;
	VESCUart vesc_uart_B;

	int VESC_ENCODER_PERIOD;

  // keep track of elapsed milliseconds since last prints
  elapsedMillis last_print_debug=0;
	elapsedMillis print_w=0;



	/**
	 * Converts an angle in the vesc encoder reference frame to a normalized angle
	 * @param  raw_angle [description]
	 * @return           [description]
	 */
	float vesc_to_normalized_angle(float raw_angle, float offset, float dir);
	/**
	 * Converts an angle in the robot frame to an angle in the vesc encoder frame
	 * @param  raw_angle [description]
	 * @return           [description]
	 */
	float normalized_to_vesc_angle(float normalized_angle, float offset, float dir);

	/**
	 * Sends a CAN message to VESC to set the motor current
	 * @param current : desired signed current
	 */
	void _send_current(float current_A, float current_B);

	/**
	 * Sends position CAN message to VESC given an absolute angle
	 * @param pos [description]
	 */
	void _send_position(float theta, float gamma);

	/**
	 * Sends a CAN message with the new pid constants and position to the VESC
	 * @param kp [description]
	 * @param ki [description]
	 * @param kd [description]
	 * @param pos
	 */
	void _send_position_pid_constants(float kp, float ki, float kd,
		float pos);

	/**
	 * Updates rotation state and calculates speed
	 * @param deg absolute encoder angle sent over CAN
	 */
  void update_angle_A(float angle);

	/**
	 * Updates rotation state and calculates speed
	 * @param deg absolute encoder angle sent over CAN
	 */
  void update_angle_B(float angle);


public:
	/**
	 * Constructor. Sets the serial port object and calls constructor
	 * @param serial_port : reference to SERIAL object
	 */
  DualVESC(int encoder_period, HardwareSerial* serial_port1, HardwareSerial* serial_port2);

	/**
	 * Processes the given byte from the serial stream
	 * @param rx_data data to process
	 */
	void packet_process_byte_A(uint8_t rx_data);

	/**
	 * Processes the given byte from the serial stream
	 * @param rx_data data to process
	 */
	void packet_process_byte_B(uint8_t rx_data);

	/**
	 * Sends position CAN message to motor to update position hold command.
	 * Currently only implements VESC-side position hold
	 * @param deg normalized target angle in degrees
	 */
	void write(float theta, float gamma);

	/**
	 * Sends CAN message to set current
	 * @param current desired current in amps
	 */
	void write_current(float current_A, float current_B);
	/**
	 * Sends CAN message to update position PID gains and position
	 * @param kp P term gain
	 * @param ki I term gain
	 * @param kd D term gain
	 * @param pos : normalized target position
	 */
	void write_pos_and_pid_gains(float kp, float ki,
		float kd,float pos);

	/**
	 * Returns the last read normalized motor position in degrees. Note
	 * that the motor position read is not the commanded position, but
	 * the actual, last-read motor position
	 * @return motor position
	 */
	float read_A();

	/**
	 * Returns the last read normalized motor position in degrees. Note
	 * that the motor position read is not the commanded position, but
	 * the actual, last-read motor position
	 * @return motor position
	 */
	float read_B();

	/**
   * Sets up the vesc object to talk over this CAN ID channel
   * @param CANID              integer, channel ID
   * @param _encoder_offset    float, encoder offset
   * @param _encoder_direction int
   * @param _max_current       float, maximum current to send
   */
	void attach(float _encoder_offset_A, int _encoder_direction_A,
							float _encoder_offset_B, int _encoder_direction_B,
							float _max_current);

	/**
	 * De-initializes the VESC object and sends a zero-current command to halt
	 * the VESC
	 */
	void detach();

  /**
   * Not implemented.
   * @param sleep_time [description]
   */
	void reset(int sleep_time);

	/**
	 * Uses teensy-based pid to send current command to VESC given a normalized
	 * angle target
	 * @param set_point [description]
	 */
	void pid_update_normalized(float set_point);

	/**
	 * Same as above except uses absolute angle
	 * @param set_point [description]
	 */
  void pid_update(float theta_sp, float gamma_sp);

	/**
	 * Prints VESC object state
	 */
  void print_debug();

	/**
	 * Update kP and kD for teensy-side PID control
	 * @param kP proportional gain
	 * @param kD derivative gain
	 */
	void set_pid_gains(float kP, float kD);
};

#endif
