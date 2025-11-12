/*
 * rotor_dynamic.c
 *
 *  Created on:
 *      Author: Rajat P.
 */


#include "plant.h"
#include <math.h>
#include <math_util.h>
#include <common_variable.h>
#include "rotor_dynamics.h"
#include "battery_dynamics.h"
#include "Forces_and_moments_rotors.h"
#include "common_variable.h"


float thrust_noise_old[28]={0.0};
float thrust_noise[28]={0.0};

float aa = 0.0f, bb = 0.0f, cc = 0.0f;

S_motor s_motor;

void rotor_dynamics(float t_step_rot)
{
	v_throttle_to_thrust_torque(t_step_rot); // updates s_motor.rotor_force_out[]

	v_rotors_force_and_moments(); // updates in body frame of uav



	#ifdef Quad_H

		for (i=4;i<28;i++)
		{
			rotor_force_out[i]=0.0;
		}
	#endif


	#ifdef Quad_X

		for (i=4;i<28;i++)
		{
			rotor_force_out[i]=0.0;
		}
	#endif


	#ifdef Quad_+

		for (i=4;i<28;i++)
		{
			rotor_force_out[i]=0.0;
		}
	#endif


	#ifdef Coax_Quad_X

		for (i=8;i<28;i++)
		{
			rotor_force_out[i]=0.0;
		}
	#endif


	#ifdef Coax_Hexa_H_sym

		for (i=12;i<28;i++)
		{
			rotor_force_out[i]=0.0;
		}
	#endif
}

void v_motor_pwm_to_throttle(float t_step_rot)
{
	int i=0;
	float throttle_rate_cmd = 0.0f;

	for (i=0;i<(fwv_motors);i++)
	{
		// PWM to throttle conversion
		if (s_motor.pwm_in[i] <= s_motor.motor_pwm_min)
		{
			s_motor.throttle_cmd[i] = 0.0f;
		}
		else if (s_motor.pwm_in[i] >= s_motor.motor_pwm_max)
		{
			s_motor.throttle_cmd[i] = 1.0f;
		}
		else if ( (s_motor.pwm_in[i] > s_motor.motor_pwm_min) && (s_motor.pwm_in[i] < s_motor.motor_pwm_max) )
		{
			s_motor.throttle_cmd[i] = (s_motor.pwm_in[i] - s_motor.motor_pwm_min)/(s_motor.motor_pwm_max - s_motor.motor_pwm_min);
		}

		// Rate limiting on throttle command
		throttle_rate_cmd = (s_motor.throttle_cmd[i] - s_motor.throttle_cmd_old[i])/t_step_rot;

		if (throttle_rate_cmd > s_motor.rate_limit_throttle)
		{
			throttle_rate_cmd = s_motor.rate_limit_throttle;
		}
		else if (throttle_rate_cmd < -s_motor.rate_limit_throttle)
		{
			throttle_rate_cmd = -s_motor.rate_limit_throttle;
		}

		s_motor.throttle_cmd[i] = s_motor.throttle_cmd_old[i] + throttle_rate_cmd*t_step_rot;
	}

}


void v_throttle_to_thrust_torque(t_step_rot, int model)
{
	int i=0;

	if (model == 0)
	{
		for (i=0;i<(fwv_motors);i++)
		{
			s_motor.rotor_force_out[i] = s_motor.throttle_cmd[i] * s_motor.max_thrust_per_motor;
		}

		
	}
	else if (model == 1)
	{

	}

	input_noise_in_thrust();
}



void input_noise_in_thrust()
{
	int i=0;
	float TC_T_noise = 1.0f;
	float noise_percent = 0.01f/100.0f;

	int pp=1;
	// Noise/bias in Thrust
	for (i=0;i<(quad_num_motors + fwv_motors);i++)
	{
		thrust_noise[i] = 1.0*mass*9.81*noise_percent/4.0;//0.02 = 2 percent noise in thrust , 1.6 = T/w, /4 = 4 motors
		thrust_noise[i] =thrust_noise_old[i] + ( (t_step_rot/(t_step_rot+TC_T_noise))*(thrust_noise[i] - thrust_noise_old[i]) );
		thrust_noise_old[i] = thrust_noise[i];

		//srand ( time(NULL) );
		srand(i + time(NULL) );
		pp=rand()%10000;
		rotor_force_out[i]=rotor_force_out[i] + (powf(-1.0,pp)*thrust_noise[i]);
	}
}



void rotor_geometry_definition()
{
	int i =1;

	s_motor.rotor_xyz[2-i][0]=  x2;                         s_motor.rotor_xyz[3-i][0]=x2;
	s_motor.rotor_xyz[2-i][1]= -y2;                         s_motor.rotor_xyz[3-i][1]=y2;
	s_motor.rotor_xyz[2-i][2]= -z;                         s_motor.rotor_xyz[3-i][2]= -z;

	s_motor.rotor_xyz[1-i][0]=   x1;                       s_motor.rotor_xyz[4-i][0]= x1;
	s_motor.rotor_xyz[1-i][1]=  -y1;                       s_motor.rotor_xyz[4-i][1]= y1;
	s_motor.rotor_xyz[1-i][2]=   z;                        s_motor.rotor_xyz[4-i][2]= z;

	s_motor.rotor_xyz[8-i][0]= -x1;                        s_motor.rotor_xyz[5-i][0]= -x1;
	s_motor.rotor_xyz[8-i][1]= -y1;                        s_motor.rotor_xyz[5-i][1]=  y1;
	s_motor.rotor_xyz[8-i][2]= -z;                         s_motor.rotor_xyz[5-i][2]= -z;

	s_motor.rotor_xyz[7-i][0]= -x2;                        s_motor.rotor_xyz[6-i][0]= -x2;
	s_motor.rotor_xyz[7-i][1]= -y2;                        s_motor.rotor_xyz[6-i][1]=  y2;
	s_motor.rotor_xyz[7-i][2]=  z;                         s_motor.rotor_xyz[6-i][2]=  z;

	////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////////
	s_motor.rotor_tilt[2-i][0]= 0.0;                        s_motor.rotor_tilt[3-i][0]= 0.0;
	s_motor.rotor_tilt[2-i][1]= 0.0;                        s_motor.rotor_tilt[3-i][1]= 0.0;
	s_motor.rotor_tilt[2-i][2]= 0.0;                        s_motor.rotor_tilt[3-i][2]= 0.0;

	s_motor.rotor_tilt[1-i][0]= 0.0;                        s_motor.rotor_tilt[4-i][0]= 0.0;
	s_motor.rotor_tilt[1-i][1]= 0.0;                        s_motor.rotor_tilt[4-i][1]= 0.0;
	s_motor.rotor_tilt[1-i][2]= 0.0;                        s_motor.rotor_tilt[4-i][2]= 0.0;

	s_motor.rotor_tilt[8-i][0]= 0.0;                        s_motor.rotor_tilt[5-i][0]= 0.0;
	s_motor.rotor_tilt[8-i][1]= 0.0;                        s_motor.rotor_tilt[5-i][1]= 0.0;
	s_motor.rotor_tilt[8-i][2]= 0.0;                        s_motor.rotor_tilt[5-i][2]= 0.0;

	s_motor.rotor_tilt[7-i][0]= 0.0;                        s_motor.rotor_tilt[6-i][0]= 0.0;
	s_motor.rotor_tilt[7-i][1]= 0.0;                        s_motor.rotor_tilt[6-i][1]= 0.0;
	s_motor.rotor_tilt[7-i][2]= 0.0;                        s_motor.rotor_tilt[6-i][2]= 0.0;


	////////////////////////////////////////////////////////////////////////////////////////

	s_motor.rotor_r_direction[2-i]= 1.0;                	s_motor.rotor_r_direction[3-i]=-1.0;
	s_motor.rotor_r_direction[1-i]=-1.0;                	s_motor.rotor_r_direction[4-i]= 1.0;

	s_motor.rotor_r_direction[8-i]= 1.0;                	s_motor.rotor_r_direction[5-i]=-1.0;
	s_motor.rotor_r_direction[7-i]=-1.0;                	s_motor.rotor_r_direction[6-i]=1.0;

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

	for (i = 0; i <  fwv_motors; i++)
	{
		s_motor.rotor_force[i][0] =  s_motor.rotor_force_out[i] * (sinf(s_motor.rotor_tilt[i][0]) * sinf(s_motor.rotor_tilt[i][2]) + cosf(s_motor.rotor_tilt[i][0]) * cosf(s_motor.rotor_tilt[i][2]) * sinf(s_motor.rotor_tilt[i][1]));
		s_motor.rotor_force[i][1] = -s_motor.rotor_force_out[i] * (sinf(s_motor.rotor_tilt[i][0]) * cosf(s_motor.rotor_tilt[i][2]) - cosf(s_motor.rotor_tilt[i][0]) * sinf(s_motor.rotor_tilt[i][2]) * sinf(s_motor.rotor_tilt[i][1]));
		s_motor.rotor_force[i][2] =  s_motor.rotor_force_out[i] * (cosf(s_motor.rotor_tilt[i][0]) * cosf(s_motor.rotor_tilt[i][1]));
	}
}


void v_yaw_moment_motor_frame()
{
	int i=0;

	for (i = 0; i <  fwv_motors; i++)
	{
		s_motor.rotor_yaw_moment_m[i] = d_by_b_fwv * s_motor.rotor_r_direction[i] * fabsf(s_motor.rotor_force_out[i])  ;
	}
}


void v_yaw_moment_rotor2body()
{
	int i = 0;
	for (i = 0; i <  fwv_motors; i++)
	{
		s_motor.rotor_yaw_moment_b[i][0] =  s_motor.rotor_yaw_moment_m[i] * (sinf(s_motor.rotor_tilt[i][0]) * sinf(s_motor.rotor_tilt[i][2]) + cosf(s_motor.rotor_tilt[i][0]) * cosf(s_motor.rotor_tilt[i][2])*sinf(s_motor.rotor_tilt[i][1]));
		s_motor.rotor_yaw_moment_b[i][1] = -s_motor.rotor_yaw_moment_m[i] * (sinf(s_motor.rotor_tilt[i][0]) * cosf(s_motor.rotor_tilt[i][2]) - cosf(s_motor.rotor_tilt[i][0]) * sinf(s_motor.rotor_tilt[i][2])*sinf(s_motor.rotor_tilt[i][1]));
		s_motor.rotor_yaw_moment_b[i][2] =  s_motor.rotor_yaw_moment_m[i] * (cosf(s_motor.rotor_tilt[i][0]) * cosf(s_motor.rotor_tilt[i][1]));
	}
}


void v_moment_rotor2body()
{
	int i = 0;

	for (i = 0; i < fwv_motors; i++)
	{
		cross_product(s_motor.rotor_xyz[i],s_motor.rotor_force[i],s_motor.rotor_moment[i]);
		s_motor.rotor_moment[i][0] = s_motor.rotor_moment[i][0] + s_motor.rotor_yaw_moment_b[i][0] ;
		s_motor.rotor_moment[i][1] = s_motor.rotor_moment[i][1] + s_motor.rotor_yaw_moment_b[i][1] ;
		s_motor.rotor_moment[i][2] = s_motor.rotor_moment[i][2] + s_motor.rotor_yaw_moment_b[i][2] ;
	}
}


void v_rotors_force_and_moments()
{
	int i = 0;

	v_thrust_rotor2body();
	v_yaw_moment_motor_frame();
	v_yaw_moment_rotor2body();
	v_moment_rotor2body();

	all_rotors_force[0] = 0.0f;
	all_rotors_force[1] = 0.0f;
	all_rotors_force[2] = 0.0f;

	all_rotors_moment[0] = 0.0f;
	all_rotors_moment[1] = 0.0f;
	all_rotors_moment[2] = 0.0f;

	for (i = 0; i < fwv_motors; i++)
	{
		all_rotors_force[0] = all_rotors_force[0] + rotor_force[i][0];
		all_rotors_force[1] = all_rotors_force[1] + rotor_force[i][1];
		all_rotors_force[2] = all_rotors_force[2] + rotor_force[i][2];

		all_rotors_moment[0] =  all_rotors_moment[0] + rotor_moment[i][0];
		all_rotors_moment[1] =  all_rotors_moment[1] + rotor_moment[i][1];
		all_rotors_moment[2] =  all_rotors_moment[2] + rotor_moment[i][2];
	}

	vehicle.all_rotors_force[0] = all_rotors_force[0];
	vehicle.all_rotors_force[1] = all_rotors_force[1];
	vehicle.all_rotors_force[2] = all_rotors_force[2];

	vehicle.all_rotors_moment[0] = all_rotors_moment[0];
	vehicle.all_rotors_moment[1] = all_rotors_moment[1];
	vehicle.all_rotors_moment[2] = all_rotors_moment[2];
}

