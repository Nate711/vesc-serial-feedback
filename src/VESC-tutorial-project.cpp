/*
  Copyright 2016-2017 Nathan Kau nathankau@stanford.edu

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

#include "VESC-tutorial-project.h"
#include "AngularPDController.h"
#include "buffer.h"
#include "utils.h"
#include "SerialVESC.h"
#include "VescUart.h"

/******** GLOBAL VARISBLES *********/

// TODO: create serial object

// Variable to keep track of the last time a debugging print message was sent
// The elapsedMicros type automatically increments itself every loop execution!
elapsedMillis last_print_shit;

// Variable to keep track of time since 100hz, 500hz, and 1000hz loops
// were executed
// The elapsedMicros type automatically increments itself every loop execution!
elapsedMicros elapsed_100HZ = 0;
elapsedMicros elapsed_500HZ = 0;
elapsedMicros elapsed_1000HZ = 0;

// VESC motor objects
VESC vesc1(&Serial4); // CAN flexcan

// STATE MACHINE STATE VARIABLE
enum controller_state_machine {
	STAGING,
	RUNNING,
	ESTOP
};
controller_state_machine controller_state = STAGING;

// Keep track of when RUNNING was entered
long running_timestamp;

// Keep track of positio and gain targets
struct vesc_pos_gain_command vesc_pos_gain_target = {0.0, 0.0, 0.0};

/********** END GLOBAL VARIABLES ******/

/********* CONSTANTS *********/

// Send position commands at 500hz (1s / 2000us)
// const int UPDATE_PERIOD =  2000; // us
// const int UPDATE_PERIOD =  10000; // us
const int UPDATE_1000HZ = 1100; //us
const int UPDATE_500HZ = 2000; //us
const int UPDATE_100HZ = 10000; //us


// built-in led pin
int led_pin = 13;
#define LED_ON digitalWrite(led_pin,HIGH)
#define LED_OFF digitalWrite(led_pin,LOW)

// Configuration
int COMPUTER_BAUDRATE = 500000;

// 320k, 250k, 200k, 160k, 125k, 100k all work
// 800k, 500k, 400k do not work because 100hz position messages are corrupted, not all received
// 1000k does NOT work, no data received
// When you go back to staging then back to running it stops working
// (causes frozen or 50ms delay)??? This
// happens at 250hz and 115200hz (probably all) when printing time and angle only
// VESC MUST BEGIN TRANSMITTING AFTER TEENSY??
// When vesc goes back into running mode, the message handler is all messed up
// and is stuck in rx_state 0 trying to find the start byte
// Somehow the teensy is getting payload length 3 messages, possibly stay alive ???
// TODO: FIXED: eliminated start byte type 3 (payload length message is 2 bytes,
// aka more than 256 bytes but thats way to fricken long

int VESC_BAUDRATE =  250000;
// Causes 50ms delay between data or just doesnt receive data
// int VESC_BAUDRATE = 115200;

// Unused when pos control is done on the VESC
const float MAX_CURRENT = 20.0; // 30 amps seems the max

// VESC1 settings
const int8_t VESC1_CHANNEL_ID = 0;
const float VESC1_OFFSET = -108; // 108
const int VESC1_DIRECTION = -1;

/******** END OF CONSTANTS ***********/

/**
 * Call this function in the main loop if you want to see the normalized motor angles
 */
void print_shit() {
  if(last_print_shit > 10) {
    last_print_shit -= 10;

    // Serial.println(loop_time);
    Serial.println(vesc1.read());
  }
}

/**
 * Transition to RUNNING state
 */
void transition_to_running() {
  Serial.println("Transitioning to RUNNING");
  running_timestamp = millis();
  controller_state = RUNNING;

	vesc_pos_gain_target.k_d = 0;
	vesc_pos_gain_target.k_p = 0;
	vesc_pos_gain_target.pos = 0;

	Serial4.clear();
}

/**
 * Transition to ESTOP state
 */
void transition_to_ESTOP() {
  Serial.println("Transitioning to ESTOP");
  controller_state = ESTOP;
}

/**
 * Transition to STAGING state
 */
void transition_to_STAGING() {
  Serial.println("Transitioning to STAGING");
  controller_state = STAGING;

  // Program specific resets
  reset_impulse();
}

/**
 * Process serial commands send from a computer
 */
void process_serial() {
  if(Serial.available()) {
		// NOTE: message string variable will not include '\n' char
		// TODO: change this code from blocking code to GOOD, nonblocking code
		String message = Serial.readStringUntil('\n');

		/***** G1 COMMAND ******/
		// Send G1 X[pos as float] P[Kp as float] D[kd value as float] to command
		// the teensy to send a position and pid gain command to the vesc
    // TODO: Add defensive error catching stuff
		if(message[0] == 'G' && message[1] == '1') {
			int x_index = message.indexOf('X');
			int x_end = message.indexOf(' ',x_index);
			String x_string = message.substring(x_index+1, x_end);

			int p_index = message.indexOf('P');
			int p_end = message.indexOf(' ',p_index);
			String kp_string = message.substring(p_index+1, p_end);

			int d_index = message.indexOf('D');
			int d_end = message.indexOf(' ',d_index);
			String kd_string = message.substring(d_index+1, d_end);

			float x = x_string.toFloat();
			float kp = kp_string.toFloat();
			float kd = kd_string.toFloat();

			update_pos_and_gain_target(x,kp,kd);

			print_pos_gain_target();

			return;
		}

		// Send 'e' to start encoder readings
		if(message[0] == 'e') {
			start_encoder_prints();
		}
		if(message[0] == '!' && message[1] == 'e') {
			stop_encoder_prints();
		}

		// Send 's' over serial to permanently ESTOP
		if(message[0] == 's') {
      transition_to_ESTOP();
		}
      // Send 'b' over serial to begin running if not in ESTOP mode
    if(message[0] == 'b') {
      if(controller_state != ESTOP) {
        transition_to_running();
      }
		}
    // Send 'r' over serial to reset this program's vars and restart
    if(message[0] == 'r') {
      transition_to_STAGING();
		}

    // Clear the buffer after the first byte is read.
    // So don't send multiple commands at once and expect any but the first to be executed
    // Serial.clear();
  }
}

void process_VESC_serial() {
  while(Serial4.available()) {
		uint8_t data = Serial4.read();
		vesc1.packet_process_byte(data);
	}
}

/**
 * Handle any position messages from VESCs
 */
void process_CAN_messages() {
  // float last_read_angle;
  // int transmitter_ID;
  //
  // // time to read angle over can is 9 us
  // if(readAngleOverCAN(CANTransceiver, last_read_angle, transmitter_ID)) {
  //   switch(transmitter_ID) {
  //     case VESC1_CHANNEL_ID:
  //       vesc1.update_angle(last_read_angle);
  //       break;
  //   }
  // }
}

/**
 * Initial state, Teensy boots into this state. Idles.
 */
void STAGING_STATE() {
  // vesc1.write_current(0.0f);
}

/**
 * Executes any user code and executes on Serial commands
 */
void RUNNING_STATE() {
  // 1000Hz loop
  if(elapsed_1000HZ > UPDATE_1000HZ) {
    elapsed_1000HZ = 0;

    // encoder_printing();
    // VESC-side position PID control
		// send_vesc_target(vesc1, vesc_pos_gain_target);

		// vesc1.set_pid_gains(vesc_pos_gain_target.k_p,vesc_pos_gain_target.k_d);
		// vesc1.pid_update(vesc_pos_gain_target.pos);

		// Hard-coded values
		// vesc1.set_pid_gains(0.05,0.001);
		// vesc1.pid_update(180.0);
		// Serial.println(vesc1.read());
  }
	// 500Hz loop
	if(elapsed_500HZ > UPDATE_500HZ) {
		elapsed_500HZ = 0;

		// Serial.println(vesc1.read());
		// encoder_printing();
		// encoder_printing();
		// send_vesc_target(vesc1, vesc_pos_gain_target);

	}
	// 100Hz loop
	if(elapsed_100HZ > UPDATE_100HZ) {
		elapsed_100HZ = 0;

		// Serial.println();
		// Serial.println(vesc1.read());
		// vesc1.print_debug();

		// SERIAL POSITION
		float pos = vesc_pos_gain_target.pos;
		vesc1.write(pos);
		// impulse();
    // send_vesc_target(vesc1, vesc_pos_gain_target);
	}
}

/**
 * ESTOP: Stops motor in emergency
 */
void ESTOP_STATE() {
  vesc1.write_current(0.0f);

	// Pause for 100ms
	long now = millis();
	while(millis() - now < 100) {}
}

/**
 * Called when Teensy boots
 */
void setup() {
  // TODO initialize serial4
	Serial4.begin(VESC_BAUDRATE);
	Serial4.clear();

	// Init Serial
	Serial.begin(COMPUTER_BAUDRATE);
  Serial.println("CAN Transmitter Initialized");
	Serial.setTimeout(2); // 2 ms timeout

  // Initialize "power on" led
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);



  // Initialize VESC controller objects
  vesc1.attach(VESC1_CHANNEL_ID,
                  VESC1_OFFSET,
                  VESC1_DIRECTION,
                  MAX_CURRENT);

	// Wait for shit to get set up
  delay(1000);

	Serial4.clear();
}

/**
 * Called indefinitely
 */
void loop() {
		// Important: read any data sent by the VESC
		process_VESC_serial();

    // IMPORTANT: read any commands sent by the computer
    process_serial();

    // IMPORTANT: for some reason the code doesn't work without this function call
    // Probably has to do with a delay thing
    // print_shit();

    switch(controller_state) {
      case STAGING:
        STAGING_STATE();
        break;
      case RUNNING:
        RUNNING_STATE();
        break;
      case ESTOP:
        ESTOP_STATE();
        break;
      default:
        ESTOP_STATE();
        break;
    }
}

/****** APPLICATION CODE ******/

void print_pos_gain_target() {
	Serial.print("x: ");
	Serial.print(vesc_pos_gain_target.pos,1);
	Serial.print("\tk_p: ");
	Serial.print(vesc_pos_gain_target.k_p,3);
	Serial.print("\tk_d: ");
	Serial.println(vesc_pos_gain_target.k_d,4);
}

// For printing encoder readings
static bool print_encoder_readings = false;

/**
 * Start printing vesc1 encoder readings to Serial
 */
void start_encoder_prints() {
	print_encoder_readings = true;
}

/**
 * Stop printing vesc1 encoder readings to Serial
 */
void stop_encoder_prints() {
	print_encoder_readings = false;
}

/**
 * Print [millis] [encoder reading] if print_encoder_readings is true
 */
void encoder_printing() {
	if(print_encoder_readings) {
		Serial.print(millis());
		Serial.print(" ");
		Serial.println(vesc1.read());
	}
}

/**
 * Update the global target variable with the given parameters. Constrain
 * the gain constants to reasonable values:
 * k_p = (0, 1.0)
 * k_d = (0.0001, 0.01)
 * @param pos: Target position [degrees]
 * @param kp: New P gain
 * @param kd: New D gain
 */
void update_pos_and_gain_target(float pos, float kp, float kd) {
	kp = constrain(kp, 0.0, 1.0);
	kd = constrain(kd, 0.0001, 0.01);

	vesc_pos_gain_target.pos = pos;
	vesc_pos_gain_target.k_p = kp;
	vesc_pos_gain_target.k_d = kd;
}

/**
 * Send a position and gain command to a VESC
 * @param vesc: VESC reference
 * @param comm: Position and gain struct command
 */
void send_vesc_target(VESC &vesc, struct vesc_pos_gain_command &comm) {
	vesc.write_pos_and_pid_gains(comm.k_p,
															 0.0,
															 comm.k_d,
															 comm.pos);
}

// Call this function in RUNNING_STATE to move the motor in a sinusoid
void sinusoid() {
  float seconds = (float)(millis()%1000000) / 1000.0;
  float freq = 0.25; // actual freq, not angular freq
  // Must use sinf and not sin??
  float motor_angle = sinf(seconds*2*PI*freq) * 180.0;

  // This particular set of arguments sets Kd to 0.05, Ki to 0, Kd to 0.0005, and target position to 0 degrees
  vesc1.write_pos_and_pid_gains(0.05, 0, 0.0005, motor_angle);
}

static bool impulse_started = false;
static bool impulse_finished = false;
static bool impulsed_printed_finish = false;

void impulse_print_position() {
  Serial.println(vesc1.read());
}

void reset_impulse() {
  impulse_finished = false;
  impulsed_printed_finish = false;
  impulse_started = false;
}

void impulse() {
  // Print IMPULSE-STARTING when the function is first called
  if(!impulse_started) {
    Serial.println("IMPULSE-STARTING");
    impulse_started = true;
  }

  // Print IMPULSE-DONE after the program is finished
  if(impulse_finished && !impulsed_printed_finish) {
    Serial.println("IMPULSE-DONE");
    impulsed_printed_finish = true;
  }

  // Send motor position command based off which time segment we're in
  static long time_running = 0;
  time_running = millis() - running_timestamp;

  // Send to 180 deg and wait for 500ms for the motor to get there)
  if (time_running < 500) {
    vesc1.write(180.0f);
    return;
  }

  // Send zero current after the impulse is done
  if(time_running > 2000) {
    impulse_finished = true;
    LED_ON;
    transition_to_ESTOP();
    return;
  }

  // If execution reaches, here it means we're in the actual impulse stage
  // Print the motor position

  // CAITLIN: using this function makes it not work
  // impulse_print_position();
  // CAITLIN: Printing directly works
  // Does not work when Serial.print
  Serial.println(vesc1.read());

  // Set 180.0 deg position and Wait 1 second to measure resting instability
  if (time_running < 1000) {
    LED_OFF;
    vesc1.write(180.0f);
    return;
  }

  // Send to 90 deg and wait for 500ms
  if(time_running >= 1000 && time_running < 1500) {
    LED_ON;
    // Serial.println("");
    vesc1.write(90.0f);
    return;
  }

  /***** CAITLIN THE BELOW ISNT WORKING ******/
  // THINGS TRIED
  // Filling out 8 byte can message buffer instead of filling out first 4 bytes
  // Printing out CAN message data in binary and making sure its not corrupted
  // Can't make the CAN_messate_t variable global or static :(

  // Send back to 0 deg and wait for 500 ms
  if(time_running >= 1500 && time_running < 2000) {
    // Serial.println("");

    // Without the above println, the LED_OFF which uses digitalWrite DOES WORK
    LED_OFF;

    // Without the above println, the vesc1.write DOES NOT WORK
    vesc1.write(180.0f);
    return;
  }
}

/** NON BLOCKING SERIAL
 * // Example 3 - Receive with start- and end-markers

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

void setup() {
Serial.begin(9600);
Serial.println("<Arduino is ready>");
}

void loop() {
recvWithStartEndMarkers();
showNewData();
}

void recvWithStartEndMarkers() {
static boolean recvInProgress = false;
static byte ndx = 0;
char startMarker = '<';
char endMarker = '>';
char rc;

while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (recvInProgress == true) {
				if (rc != endMarker) {
						receivedChars[ndx] = rc;
						ndx++;
						if (ndx >= numChars) {
								ndx = numChars - 1;
						}
				}
				else {
						receivedChars[ndx] = '\0'; // terminate the string
						recvInProgress = false;
						ndx = 0;
						newData = true;
				}
		}

		else if (rc == startMarker) {
				recvInProgress = true;
		}
}
}

void showNewData() {
if (newData == true) {
		Serial.print("This just in ... ");
		Serial.println(receivedChars);
		newData = false;
}
}
 * @param [name] [description]
 */
