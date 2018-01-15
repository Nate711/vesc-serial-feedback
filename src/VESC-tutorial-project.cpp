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
#include "Config.h"

/********* CONSTANTS *********/

// Send position commands at 500hz (1s / 2000us)
// const int UPDATE_PERIOD =  2000; // us
// const int UPDATE_PERIOD =  10000; // us
const int UPDATE_2000HZ = 500; //us
const int UPDATE_1000HZ = 1000; //us
const int UPDATE_500HZ = 2000; //us
const int UPDATE_100HZ = 10000; //us

// built-in led pin
int led_pin = 13;
#define LED_ON digitalWrite(led_pin,HIGH)
#define LED_OFF digitalWrite(led_pin,LOW)

/******** END OF CONSTANTS ***********/

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
elapsedMicros elapsed_2000HZ = 0;

// VESC motor objects
VESC vesc1(VESC_ENCODER_PERIOD, &VESC1_SERIAL);
VESC vesc2(VESC_ENCODER_PERIOD, &VESC2_SERIAL);

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
 * Prints number of idle and busy loops per second
 *
 * As of commit b7f7583 1-5-18:
 * Sometimes teensy doesn't receive any vesc encoder readings in staging mode
 * Busy:0, Idle: 250k
 *
 * Teensy-VESC bus at 2000hz duplex, 250kbps
 * STAGING state
 * Busy: 20020 +- 10, Idle: 214k +- 20
 * RUNNING phase
 * Busy: 21.9k +- 200, Idle: ~125k +- 300
 * ESTOP state
 * Busy: 11, Idle: 0
 *
 * Vesc to teensy at 2000hz, Teensy to vesc at 1000hz, 250kbps
 * STAGING state
 * Busy: 20020 +- 10, Idle: 214k +- 20
 * RUNNING phase
 * Busy: 22.6k +- 200, Idle: ~128k +- 300
 * ESTOP state
 * Busy: 11, Idle: 0
 *
 * Vesc to teensy at 2000hz, Teensy to vesc at 2000hz, 320kbps
 * STAGING state
 * Busy: 20020 +- 2, Idle: 214k +- 20
 * RUNNING phase
 * Busy: 21.9k +- 200, Idle: ~125k +- 300
 * ESTOP state
 * Busy: 11, Idle: 0
 *
 * TWO VESC to teensy at 2000hz, 320kbps
 * STAGING STATE
 * Busy: 25k, Idle: 186l
 * RUNNING phase
 * Busy: 26k +- 500, Idle: 103k +- 500
 * ESTOP state
 * Busy: 11, Idle: 0
 */
elapsedMillis last_processor_usage_print = 0;
void print_processor_usage(long &_busy_loops, long &_idle_loops) {
	if(last_processor_usage_print > 1000) {
		last_processor_usage_print = 0;

		Serial.print("Busy loops: ");
		Serial.print(_busy_loops);
		Serial.print("\t Idle loops: ");
		Serial.println(_idle_loops);

		// Reset loop counts
		_busy_loops = 0;
		_idle_loops = 0;
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

	VESC1_SERIAL.clear();
	VESC2_SERIAL.clear();
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
 *
 * Returns 1 if serial byte received, 0 otherwise
 */
int process_serial() {
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

			return 1;
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

		return 1;
    // Clear the buffer after the first byte is read.
    // So don't send multiple commands at once and expect any but the first to be executed
    // Serial.clear();
  }
	return 0;
}

int process_VESC_serial() {
	int received = 0;
  while(VESC1_SERIAL.available()) {
		uint8_t data = VESC1_SERIAL.read();
		vesc1.packet_process_byte(data);
		received = 1;
	}

	while(VESC2_SERIAL.available()) {
		uint8_t data = VESC2_SERIAL.read();
		vesc2.packet_process_byte(data);
		received = 1;
	}
	return received;
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
 *
 * Returns 1 if any timed loop (2000hz, 500hz, etc) executed, otherwise 0
 */
int RUNNING_STATE() {
	int executed_code = 0;
	// 2000hz loop
	if(elapsed_2000HZ > UPDATE_2000HZ) {
		elapsed_2000HZ = 0;

		vesc1.set_pid_gains(0.02,0.001);
		vesc1.pid_update(180.0);

		vesc2.set_pid_gains(-0.02, -0.001);
		vesc2.pid_update(180.0);


		executed_code |= 1;
	}
  // 1000Hz loop
  if(elapsed_1000HZ > UPDATE_1000HZ) {
    elapsed_1000HZ = 0;

		// vesc1.set_pid_gains(0.02,0.001);
		// vesc1.pid_update(180.0);

    // encoder_printing();
    // VESC-side position PID control
		// send_vesc_target(vesc1, vesc_pos_gain_target);

		// vesc1.set_pid_gains(vesc_pos_gain_target.k_p,vesc_pos_gain_target.k_d);
		// vesc1.pid_update(vesc_pos_gain_target.pos);

		// Hard-coded values
		// P=0.05 and D= 0.001 doesn't work with 20A but marginally with 15A
		// vesc1.set_pid_gains(0.05,0.001);
		// vesc1.pid_update(180.0);
		// Serial.println(vesc1.read());

		executed_code |= 1;
  }
	// 500Hz loop
	if(elapsed_500HZ > UPDATE_500HZ) {
		elapsed_500HZ = 0;

		// Finally this works
		// Serial.println(vesc1.read());
		// encoder_printing();
		// encoder_printing();
		// send_vesc_target(vesc1, vesc_pos_gain_target);

		executed_code |= 1;
	}
	// 100Hz loop
	if(elapsed_100HZ > UPDATE_100HZ) {
		elapsed_100HZ = 0;

		// Serial.println(vesc1.read());
		if(PRINT_DEBUG) {
			// vesc1.print_debug();
			vesc2.print_debug();
		}

		// SERIAL POSITION
		// float pos = vesc_pos_gain_target.pos;
		// vesc1.write(pos);
		// impulse();
    // send_vesc_target(vesc1, vesc_pos_gain_target);

		executed_code |= 1;
	}
	return executed_code;
}

/**
 * ESTOP: Stops motor in emergency
 */
void ESTOP_STATE() {
  vesc1.write_current(0.0f);
	vesc2.write_current(0.0f);

	// Pause for 100ms
	long now = millis();
	while(millis() - now < 100) {}
}

/**
 * Called when Teensy boots
 */
void setup() {
  // TODO initialize serial4
	VESC1_SERIAL.begin(VESC_BAUDRATE);
	VESC1_SERIAL.clear();

	VESC2_SERIAL.begin(VESC_BAUDRATE);
	VESC2_SERIAL.clear();

	// Init Serial
	Serial.begin(COMPUTER_BAUDRATE);
  Serial.println("CAN Transmitter Initialized");
	Serial.setTimeout(2); // 2 ms timeout

  // Initialize "power on" led
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);



  // Initialize VESC controller objects
  vesc1.attach(VESC1_OFFSET,
               VESC1_DIRECTION,
               MAX_CURRENT);

	vesc2.attach(VESC2_OFFSET,
						 	 VESC2_DIRECTION,
						 	 MAX_CURRENT);

	// Wait for shit to get set up
  delay(1000);

	VESC1_SERIAL.clear();
	VESC2_SERIAL.clear();
}

long busy_loops = 0;
long idle_loops = 0;
/**
 * Called indefinitely
 */
void loop() {
		// Keeps track whether anything happened this loop
		int busy = 0;
		// Important: read any data sent by the VESC
		busy |= process_VESC_serial();

    // IMPORTANT: read any commands sent by the computer
    busy |= process_serial();

    // IMPORTANT: for some reason the code doesn't work without this function call
    // Probably has to do with a delay thing
    // print_shit();

    switch(controller_state) {
      case STAGING:
        STAGING_STATE();
        break;
      case RUNNING:
        busy |= RUNNING_STATE();
        break;
      case ESTOP:
        ESTOP_STATE();
        break;
      default:
        ESTOP_STATE();
        break;
    }
		busy_loops += busy;
		idle_loops += (1-busy);

		if(PRINT_CPU_USAGE) {
			print_processor_usage(busy_loops,idle_loops);
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
