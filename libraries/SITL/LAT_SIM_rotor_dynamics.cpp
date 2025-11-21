/*
 * rotor_dynamic.c
 *
 *  Created on:
 *      Author: Rajat P.
 */

#include "LAT_SIM_Runner.h"
#include "math.h"
#include "LAT_SIM_math_util.h"
#include "LAT_SIM_rotor_dynamics.h"


S_motor s_motor[28];
S_MOTOR_MANAGER s_motor_manager;

void v_rotor_dynamics(float t_step_rot)
{
	v_throttle_to_thrust_torque(t_step_rot); // updates s_motor.rotor_force_out[]

	v_rotors_force_and_moments(); // updates in body frame of uav
}

void v_motor_pwm_to_throttle(float t_step_rot)
{
	int i=0;
	float throttle_rate_cmd = 0.0f;

	for (i=0;i<(s_motor_manager.num_motors);i++)
	{
		// PWM to throttle conversion
		if (s_motor[i].pwm_in <= s_motor[i].pwm_min)
		{
			s_motor[i].throttle_cmd = 0.0f;
		}
		else if (s_motor[i].pwm_in >= s_motor[i].pwm_max)
		{
			s_motor[i].throttle_cmd = 1.0f;
		}
		else if ( (s_motor[i].pwm_in > s_motor[i].pwm_min) && (s_motor[i].pwm_in < s_motor[i].pwm_max) )
		{
			s_motor[i].throttle_cmd = (s_motor[i].pwm_in - s_motor[i].pwm_min)/(s_motor[i].pwm_max - s_motor[i].pwm_min);
		}

		// Rate limiting on throttle command
		throttle_rate_cmd = (s_motor[i].throttle_cmd - s_motor[i].throttle_cmd_old)/t_step_rot;

		if (throttle_rate_cmd > s_motor[i].rate_limit_throttle)
		{
			throttle_rate_cmd = s_motor[i].rate_limit_throttle;
		}
		else if (throttle_rate_cmd < -s_motor[i].rate_limit_throttle)
		{
			throttle_rate_cmd = -s_motor[i].rate_limit_throttle;
		}

		s_motor[i].throttle_cmd = s_motor[i].throttle_cmd_old + throttle_rate_cmd*t_step_rot;
	}

}


void v_throttle_to_thrust_torque(float t_step_rot)
{
	int i=0;

	switch (vehcle.plane_model)
	{
		case PLANE_ARDU_DEFAULT:
		{
		for (i=0;i<(s_motor_manager.num_motors);i++)
		{
			s_motor[i].thrust_out = s_motor[i].throttle_cmd * s_motor[i].max_thrust;
			s_motor[i].torque_out = (s_motor[i].thrust_out/s_motor[i].max_thrust) * s_motor[i].max_torque;
		}
		break;
		}

		case PLANE_EQX:
		{


		break;
		}
	}

}



void v_thrust_rotor2body()
{

	// this is conversion of [u,v,w] body frame vector to same vector representation in NED frame( in this case from motor frame to uav frame)
	/*w*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - v*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + cos_psi*cos_theta*u;
	 v*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - w*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + cos_theta*sin_psi*u;
	cos_phi*cos_theta*w - sin_theta*u + cos_theta*sin_phi*v ;       */


	// this is conversion of [0,0,w] body frame vector to same vector representation in NED frame(in this case from motor frame to uav frame)
	/* w*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta)
	-w*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta)
	                             cos_phi*cos_theta*w	*/
	int i = 0;

	for (i = 0; i < s_motor_manager.num_motors ; i++)
	{
		s_motor[i].rotor_force[0] =  s_motor[i].rotor_force_out * (sinf(s_motor[i].rotor_tilt[0]) * sinf(s_motor[i].rotor_tilt[0]) + cosf(s_motor[i].rotor_tilt[0]) * cosf(s_motor[i].rotor_tilt[2]) * sinf(s_motor[i].rotor_tilt[1]));
		s_motor[i].rotor_force[1] = -s_motor[i].rotor_force_out * (sinf(s_motor[i].rotor_tilt[1]) * cosf(s_motor[i].rotor_tilt[1]) - cosf(s_motor[i].rotor_tilt[0]) * sinf(s_motor[i].rotor_tilt[2]) * sinf(s_motor[i].rotor_tilt[1]));
		s_motor[i].rotor_force[2] =  s_motor[i].rotor_force_out * (cosf(s_motor[i].rotor_tilt[2]) * cosf(s_motor[i].rotor_tilt[2]));
	}
}


void v_yaw_moment_motor_frame()
{
	int i=0;

	for (i = 0; i < s_motor_manager.num_motors ; i++)
	{
		s_motor[i].rotor_yaw_moment_m = s_motor[i].thrust_2_torque_factor * s_motor[i].rotor_r_direction * fabsf(s_motor[i].rotor_force_out)  ;
	}
}


void v_yaw_moment_rotor2body()
{
	int i = 0;
	for (i = 0; i < s_motor_manager.num_motors ; i++)
	{
		s_motor[i].rotor_yaw_moment_b[0] =  s_motor[i].rotor_yaw_moment_m * (sinf(s_motor[i].rotor_tilt[0]) * sinf(s_motor[i].rotor_tilt[0]) + cosf(s_motor[i].rotor_tilt[0]) * cosf(s_motor[i].rotor_tilt[1])*sinf(s_motor[i].rotor_tilt[2]));
		s_motor[i].rotor_yaw_moment_b[1] = -s_motor[i].rotor_yaw_moment_m * (sinf(s_motor[i].rotor_tilt[1]) * cosf(s_motor[i].rotor_tilt[1]) - cosf(s_motor[i].rotor_tilt[0]) * sinf(s_motor[i].rotor_tilt[1])*sinf(s_motor[i].rotor_tilt[2]));
		s_motor[i].rotor_yaw_moment_b[2] =  s_motor[i].rotor_yaw_moment_m * (cosf(s_motor[i].rotor_tilt[2]) * cosf(s_motor[i].rotor_tilt[2]));
	}
}


void v_moment_rotor2body()
{
	int i = 0;

	for (i = 0; i < s_motor_manager.num_motors ; i++)
	{
		cross_product(s_motor[i].rotor_xyz,s_motor[i].rotor_force,s_motor[i].rotor_moment);
		s_motor[i].rotor_moment[0] = s_motor[i].rotor_moment[0] + s_motor[i].rotor_yaw_moment_b[0] ;
		s_motor[i].rotor_moment[1] = s_motor[i].rotor_moment[1] + s_motor[i].rotor_yaw_moment_b[1] ;
		s_motor[i].rotor_moment[2] = s_motor[i].rotor_moment[2] + s_motor[i].rotor_yaw_moment_b[2] ;
	}
}


void v_rotors_force_and_moments()
{
	int i = 0;

	v_thrust_rotor2body();
	v_yaw_moment_motor_frame();
	v_yaw_moment_rotor2body();
	v_moment_rotor2body();

	float all_rotors_force[3]={0.0};
	float all_rotors_moment[3]={0.0};

	for (i = 0; i < s_motor_manager.num_motors; i++)
	{
		all_rotors_force[0] = all_rotors_force[0] + s_motor[i].rotor_force[0];
		all_rotors_force[1] = all_rotors_force[1] + s_motor[i].rotor_force[1];
		all_rotors_force[2] = all_rotors_force[2] + s_motor[i].rotor_force[2];

		all_rotors_moment[0] =  all_rotors_moment[0] + s_motor[i].rotor_moment[0];
		all_rotors_moment[1] =  all_rotors_moment[1] + s_motor[i].rotor_moment[1];
		all_rotors_moment[2] =  all_rotors_moment[2] + s_motor[i].rotor_moment[2];
	}

	vehcle.all_rotors_force[0] = all_rotors_force[0];
	vehcle.all_rotors_force[1] = all_rotors_force[1];
	vehcle.all_rotors_force[2] = all_rotors_force[2];

	vehcle.all_rotors_moment[0] = all_rotors_moment[0];
	vehcle.all_rotors_moment[1] = all_rotors_moment[1];
	vehcle.all_rotors_moment[2] = all_rotors_moment[2];
}

