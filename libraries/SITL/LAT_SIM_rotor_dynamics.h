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

extern void v_param_init_rotor();

extern void v_update_thrust_torque();


extern void rotor_speed_to_thrust();

extern void input_noise_in_thrust();


typedef struct
{
	float quad_thrust_max;
	float quad_thrust_min;
	float quad_thrust;

	float fwv_thrust_max;
	float fwv_thrust_min;
	float fwv_thrust;

	float quad_torque_max;
	float quad_torque_min;
	float quad_torque;

	float fwv_torque_max;
	float fwv_torque_min;
	float fwv_torque;

	float motor_thrust[quad_num_motors + fwv_motors];
	float motor_torque[quad_num_motors + fwv_motors];

	float pwm_min;
	float pwm_max;
	float thrust_min;
	float thrust_max;

	float ratio_end;
	float thrust_l[28];
	float thrust_nl[28];
	float thrust_out[28];


	float skewness_ratio;
	float skewness_power;

	uint16_t max_quad_thr_tor_pwm;
	uint16_t zero_quad_thr_tor_pwm;

	uint16_t max_fwv_thr_tor_pwm;
	uint16_t zero_fwv_thr_tor_pwm;

	uint8_t flg_use_scale_model;
}struct_motor;

extern struct_motor s_rotor;


