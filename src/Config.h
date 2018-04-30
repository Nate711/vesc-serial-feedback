#ifndef CONFIG_H
#define CONFIG_H

// Baudrate for computer to teensy serial communication
#define COMPUTER_BAUDRATE 500000

// Set true if you want to print pid debugging information at 100hz
#define PRINT_DEBUG false
#define PRINT_CPU_USAGE false

/********** MOTOR CONTROL CONFIGURATION ********/

// Microseconds between receiving encoder readings from the VESC
#define VESC_ENCODER_PERIOD 500

// 320k, 250k, 200k, 160k, 125k, 100k all work
// 500k does not work, causes motor spasms
// >= 1000k does NOT work, no data received
// DELAYS FIXED: eliminated start byte type 3 (payload length message is
// 2 bytes, aka more than 256 bytes but thats way to fricken long
#define VESC_BAUDRATE 320000

// Unused when pos control is done on the VESC
// NOTES 12-21-17
// When running at 40A, P=0.05 and D=.001 work but any more P or D
// causes vibrations. Sending VESC-Teensy serial encoder readings at 2000hz
// reduces motor vibrations, probably because of better differentiation, but
// running the Teensy-side PID at 2000hz doesn't improve PID performance very
// much
#define MAX_CURRENT 0.0 // 30 amps seems the max

#define VESC_TIMEOUT 10 // ms

////// VESC 1 configuration //////
#define VESC1_OFFSET 77 // updated 2/22 for robity v2
#define VESC1_DIRECTION 1
#define VESC1_SERIAL Serial4

////// VESC 2 configuration //////
// TODO FIX OFFSET BUG. MOTOR INITTING TO WRONG ANGLE
#define VESC2_OFFSET 360+47 // updated 2/22 for robity v2
#define VESC2_DIRECTION -1
#define VESC2_SERIAL Serial1

// KittyHawk Demo
#define TOUCH_TEST false
#define GAIT_TEST false
#define HOLD_TEST false
#define JUMP_TEST false
#define HOP_TEST false
#define PROBE_TEST true

#endif
