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

#include "VescUart.h"
#include "buffer.h"
#include "crc.h"
#include <Arduino.h> //needed for Serial.println
#include <string.h> //needed for memcpy
#include "HardwareSerial.h"


VESCUart::VESCUart(HardwareSerial* port_ptr) {
	serial_port = port_ptr;

  // Hard coded initialize rx_state as 0 for first handler
  // TODO get rid of multiple handlers
	handler_states[0].rx_state = 0;
}
/**
 * Processes one byte of data from any given communication channel.
 * Disabled the timeout behavior for now, which otherwise keeps
 * track of the time since the last byte was sent.
 * @param rx_data     Single byte of data
 * @param handler_num Which packet processer to use
 */
bool VESCUart::packet_process_byte(uint8_t rx_data, int handler_num) {
	switch (handler_states[handler_num].rx_state) {
	case 0:
		if (rx_data == 2) {
			// 1 byte PL len
			handler_states[handler_num].rx_state += 2;
			// handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
			handler_states[handler_num].rx_data_ptr = 0;
			handler_states[handler_num].payload_length = 0;

// TODO: find out what was sending the (3) over serial line, was it crc? command type?
		// } else if (rx_data == 3) {
		// 	// 2 byte PL len
		// 	handler_states[handler_num].rx_state++;
		// 	// handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		// 	handler_states[handler_num].rx_data_ptr = 0;
		// 	handler_states[handler_num].payload_length = 0;
		// 	Serial.println("SC3");
		} else {
			handler_states[handler_num].rx_state = 0;
		}
		break;

	case 1:
		handler_states[handler_num].payload_length = (unsigned int)rx_data << 8;
		handler_states[handler_num].rx_state++;
		// handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 2:
		handler_states[handler_num].payload_length |= (unsigned int)rx_data;
		if (handler_states[handler_num].payload_length > 0 &&
				handler_states[handler_num].payload_length <= PACKET_MAX_PL_LEN) {
			handler_states[handler_num].rx_state++;
			// handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		} else {
			handler_states[handler_num].rx_state = 0;
		}
		break;

	case 3:
		handler_states[handler_num].rx_buffer[handler_states[handler_num].rx_data_ptr++] = rx_data;
		if (handler_states[handler_num].rx_data_ptr == handler_states[handler_num].payload_length) {
			handler_states[handler_num].rx_state++;
		}
		// handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 4:
		handler_states[handler_num].crc_high = rx_data;
		handler_states[handler_num].rx_state++;
		// handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 5:
		handler_states[handler_num].crc_low = rx_data;
		handler_states[handler_num].rx_state++;
		// handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 6:
		if (rx_data == 3) {
			if (crc16(handler_states[handler_num].rx_buffer, handler_states[handler_num].payload_length)
					== ((unsigned short)handler_states[handler_num].crc_high << 8
							| (unsigned short)handler_states[handler_num].crc_low)) {
				// Packet received!
				// Disabled more generalized function pointer code in favor of hard
				// coded function call
				// if (handler_states[handler_num].process_func) {
				// 	handler_states[handler_num].process_func(handler_states[handler_num].rx_buffer,
				// 			handler_states[handler_num].payload_length);
				// }
				commands_process_packet(handler_states[handler_num].rx_buffer,
					handler_states[handler_num].payload_length);
			}
		}
		handler_states[handler_num].rx_state = 0;
		return true;
		break;

	default:
		handler_states[handler_num].rx_state = 0;
		break;
	}
	return false;
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void VESCUart::commands_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id = (COMM_PACKET_ID)data[0];
	int32_t ind = 0;

	// skip the packet id byte
	data++;
	len--;

	switch (packet_id) {
	case COMM_ROTOR_POSITION:
		{
		ind = 0;
		// TODO refactor code so I don't use this global variable
		// set the global rotor position variable
		vescuart_rotor_position = (float)buffer_get_int32(data, &ind) / 100000.0;
		break;
		}
	default:
		break;
	}
}

/**
 * Sends a given the given payload data to the VESC over serial
 * @param  payload [description]
 * @param  lenPay  [description]
 * @return         [description]
 */
int VESCUart::pack_send_payload(uint8_t* payload, int lenPay) {
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = NULL;

	//Sending package
	serial_port->write(messageSend, count);


	//Returns number of send bytes
	return count;
}

/**
 * [VESCUart::set_current description]
 * @param current [description]
 */
void VESCUart::set_current(float current) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT ;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	pack_send_payload(payload, 5);
}

/**
 * [VESCUart::set_position description]
 * @param position [description]
 */
void VESCUart::set_position(float position) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_POS;

	const int MULTIPLIER = 1000000;

	buffer_append_int32(payload, (int32_t)(position * MULTIPLIER), &index);
	pack_send_payload(payload, 5);
}

/**
 * [VESCUart::VescUartSetCurrentBrake description]
 * @param brakeCurrent [description]
 */
void VESCUart::VescUartSetCurrentBrake(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	pack_send_payload(payload, 5);
}

/**
 * [VescUart::get_rotor_position description]
 * @return [description]
 */
float VESCUart::get_rotor_position() {
	return vescuart_rotor_position;
}

// void SerialPrint(uint8_t* data, int len) {
// 	//	DEBUGSERIAL.print("Data to display: "); DEBUGSERIAL.println(sizeof(data));
// 	for (int i = 0; i <= len; i++)
// 	{
// 		DEBUGSERIAL.print(data[i]);
// 		DEBUGSERIAL.print(" ");
// 	}
// 	DEBUGSERIAL.println("");
// }
//
// void SerialPrint(const bldcMeasure& values) {
// 	DEBUGSERIAL.print("avgMotorCurrent: "); DEBUGSERIAL.println(values.avgMotorCurrent);
// 	DEBUGSERIAL.print("avgInputCurrent: "); DEBUGSERIAL.println(values.avgInputCurrent);
// 	DEBUGSERIAL.print("dutyCycleNow: "); DEBUGSERIAL.println(values.dutyCycleNow);
// 	DEBUGSERIAL.print("rpm: "); DEBUGSERIAL.println(values.rpm);
// 	DEBUGSERIAL.print("inputVoltage: "); DEBUGSERIAL.println(values.inpVoltage);
// 	DEBUGSERIAL.print("ampHours: "); DEBUGSERIAL.println(values.ampHours);
// 	DEBUGSERIAL.print("ampHoursCharges: "); DEBUGSERIAL.println(values.ampHoursCharged);
// 	DEBUGSERIAL.print("tachometer: "); DEBUGSERIAL.println(values.tachometer);
// 	DEBUGSERIAL.print("tachometerAbs: "); DEBUGSERIAL.println(values.tachometerAbs);
// }

/*
bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len) {
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{
	case COMM_GET_VALUES:
		ind = 14; //Skipped the first 14 bit.
		values.avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
		values.avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
		values.dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
		values.rpm = buffer_get_int32(message, &ind);
		values.inpVoltage = buffer_get_float16(message, 10.0, &ind);
		values.ampHours = buffer_get_float32(message, 10000.0, &ind);
		values.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
		ind += 8; //Skip 9 bit
		values.tachometer = buffer_get_int32(message, &ind);
		values.tachometerAbs = buffer_get_int32(message, &ind);
		return true;
		break;

	default:
		return false;
		break;
	}
}
*/
