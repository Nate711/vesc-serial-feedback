#ifndef VESC_TUTORIAL_PROJECT_H
#define VESC_TUTORIAL_PROJECT_H

#include "SerialVESC.h"
#include "DualSerialVESC.h"

// App specific functions
float sinusoid(float t, float amp, float freq, float phase_shift, float yshift);
void walking_gait_control(float t, float& theta_sp, float& gamma_sp);
void jumping_gait_control(float t, float& theta_sp, float& gamma_sp);


void update_pos_and_gain_target(float pos, float kp, float kd);
void send_vesc_target(VESC &vesc, struct vesc_pos_gain_command &comm);

void print_pos_gain_target();

void start_encoder_prints();
void stop_encoder_prints();
void encoder_printing();

struct vesc_pos_gain_command {
	float pos;
	float k_p;
	float k_d;
	float k_i;
};
void send_vesc_target(VESC &vesc, struct vesc_pos_gain_command &comm);

// State transition functions
void transition_to_running();
void transition_to_ESTOP();
void transition_to_STAGING();


// State functions
void STAGING_STATE();
int RUNNING_STATE();
void ESTOP_STATE();


// Shit functions
void print_shit();

// Serial and CAN functions
int process_serial();
int process_VESC_serial();

//PROBE
void probe();
void probing_control();
void initiate_probe();

#endif
