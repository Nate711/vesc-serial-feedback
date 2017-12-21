/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _VESCUART_h
#define _VESCUART_h


// TODO add ifdef like above for teensy 3.5
// TODO Allow you to set which serial port to use
#define SERIALIO Serial4
#define DEBUGSERIAL Serial

#include "datatypes.h"
#include "local_datatypes.h"
#include "HardwareSerial.h"


// Borrowed from VESC code, copyright Benjamin Vedder
#define PACKET_MAX_PL_LEN 1024
#define PACKET_RX_TIMEOUT 2

class VESCUart {
private:

	// Serial port to use for this motor
	HardwareSerial *serial_port;

	typedef struct {
		volatile unsigned char rx_state;
		volatile unsigned char rx_timeout;
		void(*send_func)(unsigned char *data, unsigned int len);
		void(*process_func)(unsigned char *data, unsigned int len);
		unsigned int payload_length;
		unsigned char rx_buffer[PACKET_MAX_PL_LEN];
		unsigned char tx_buffer[PACKET_MAX_PL_LEN + 6];
		unsigned int rx_data_ptr;
		unsigned char crc_low;
		unsigned char crc_high;
	} PACKET_STATE_t;

	// TODO: either make code that uses multiple packet handlers or use just one
	PACKET_STATE_t handler_states[1];

	// Keep track of last read motor angle
	float vescuart_rotor_position = 0.0;

	///PackSendPayload Packs the payload and sends it over Serial.
	///Define in a Config.h a SERIAL with the Serial in Arduino Style you want to you
	///@param: payload as the payload [unit8_t Array] with length of int lenPayload
	///@return the number of bytes send

	int pack_send_payload(uint8_t* payload, int lenPay);

public:
	VESCUart(HardwareSerial* port_ptr);

	float get_rotor_position();
	/**
	 * [commands_process_packet description]
	 * @param data [description]
	 * @param len  [description]
	 */
	void commands_process_packet(unsigned char *data, unsigned int len);

	/**
	 * [packet_process_byte description]
	 * @param rx_data     [description]
	 * @param handler_num [description]
	 */
	bool packet_process_byte(uint8_t rx_data, int handler_num);

	/**
	 * Sends a command to the VESC to control the motor current
	 * @param current as float with the current for the motor
	 */
	void set_current(float current);

	/**
	 * Sends a command to VESC to control the motor position
	 * @param position as float with the degrees for the motor
	 */
	void set_position(float position);

	/**
	 * Sends a command to the VESC to control motor braking current
	 * @param brakeCurrent as float with the current for the brake
	 */
	void VescUartSetCurrentBrake(float brakeCurrent);
};

#endif
