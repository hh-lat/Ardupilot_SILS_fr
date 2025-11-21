/*
 * rotor_dynamic.h
 *
 *  Created on:
 *      Author: Rajat P.
 */

#include "stdio.h"
#include "LAT_SIM_Runner.h"

extern void v_rotor_dynamics(float t_step_rot);


typedef struct 
{
	int num_motors;
	
}S_MOTOR_MANAGER;

extern  S_MOTOR_MANAGER s_motor_manager;


typedef struct
{
float pwm_in;
float pwm_out;
float thrust_out;
float torque_out;
float omega_tf;
float zeta_tf;
float omega;
float poles;
float max_thrust;
float min_thrust;
float max_torque;
float min_torque;
float max_omega;
float pwm_min;
float pwm_max;
float rate_limit_throttle;
float throttle_cmd;
float throttle_cmd_old;
float thrust_2_torque_factor;
float rotor_r_direction; // +1 for cw torque and -1 for ccw torque
float rotor_tilt[3]; // roll, pitch, yaw tilt angles of rotor
float rotor_xyz[3]; // x,y,z location of rotor in body frame
float rotor_force_out;
float rotor_force[3];
float rotor_moment[3];
float rotor_yaw_moment_m;
float rotor_yaw_moment_b[3];

}S_motor;

extern S_motor s_motor[28];
extern void v_throttle_to_thrust_torque(float t_step_rot);
extern void v_rotors_force_and_moments();
extern void v_motor_pwm_to_throttle(float t_step_rot);
extern void v_thrust_rotor2body();
extern void v_yaw_moment_motor_frame();
extern void v_yaw_moment_rotor2body();
extern void v_moment_rotor2body();
