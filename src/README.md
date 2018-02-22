Hello World type project for controlling VESC motor controllers from a Teensy 3.5. Schematic for wiring the motor controllers to the Teensy coming soon.

Hardware Dependencies:
1. Serial connection from both VESCs to the Teensy. See http://vedder.se/wp-content/uploads/2015/01/PCB_Front.png for location of the uart (serial) bus.
2. VESC connected to motor and AS5048 encoder configured.

Software Dependencies:
1. Teensyduino
2. Nathan's VESC firmware that sends encoder data over serial at 2kHz

Configuration:
1. Open Config.h and check values. Set maximum current. Make sure to set vesc 1 and 2 offset, direction, and serial port numbers.
