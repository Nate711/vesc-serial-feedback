#ifndef CONFIG_H
#define CONFIG_H

// Set true if you want to print pid debugging information at 100hz
#define PRINT_DEBUG false
#define PRINT_CPU_USAGE true

// Microseconds between receiving encoder readings from the VESC
#define VESC_ENCODER_PERIOD 500

// Baudrate for computer to teensy serial communication
#define COMPUTER_BAUDRATE 500000

// 320k, 250k, 200k, 160k, 125k, 100k all work
// 500k does not work, causes motor spasms
// 1000k does NOT work, no data received
// VESC MUST BEGIN TRANSMITTING AFTER TEENSY
// When vesc goes back into running mode, the message handler is all messed up
// and is stuck in rx_state 0 trying to find the start byte
// Somehow the teensy is getting payload length 3 messages, possibly stay alive ???
// TODO: FIXED: eliminated start byte type 3 (payload length message is 2 bytes,
// aka more than 256 bytes but thats way to fricken long
// #define VESC_BAUDRATE 250000
#define VESC_BAUDRATE 320000

// Unused when pos control is done on the VESC
// NOTES 12-21-17
// When running at 40A, P=0.05 and D=.001 work but any more P or D
// causes vibrations. Sending VESC-Teensy serial encoder readings at 2000hz
// reduces motor vibrations, probably because of better differentiation, but
// running the Teensy-side PID at 2000hz doesn't improve PID performance very
// much
#define MAX_CURRENT 40.0 // 30 amps seems the max

#endif
