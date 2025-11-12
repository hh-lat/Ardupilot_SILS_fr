/*
 * rotor_dynamic.h
 *
 *  Created on:
 *      Author: Rajat P.
 */


#include "stdio.h"
#include "plant.h"
#include "Forces_and_moments_rotors.h"
#include "common_variable.h"

extern float w_omega[28],zeta_omega[28],rotor_speed_dot_dot[28],factor;
extern float rotor_speed_old[28],rotor_speed_dot[28];

extern float thrust_noise_old[28];
extern float thrust_noise[28];
extern void input_noise_in_thrust();
typedef struct
{
	float pwm_in[28];
	float motor_pwm_min;
	float motor_pwm_max;
	float rate_limit_throttle;
	float max_thrust_per_motor;
	float throttle_cmd[28];
	float throttle_cmd_old[28];
	float rotor_force_out[28];
	float rotor_xyz[28][3];
 	float rotor_moment[28][3];
 	float rotor_tilt[28][3];
 	float rotor_yaw_moment_b[28][3];

extern float rotor_r_direction[28];
}S_motor;

extern S_motor s_motor;

