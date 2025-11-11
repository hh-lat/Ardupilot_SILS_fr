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
# include "Forces_and_moments_rotors.h"
# include "common_variable.h"
# include "time.h"

float w_omega[28]={0.0f},zeta_omega[28]={0.0f},rotor_speed_dot_dot[28]={0.0f},factor=1.0f;
float rotor_speed_old[28]={0.0f},rotor_speed_dot[28]={0.0f};

float thrust_noise_old[28]={0.0};
float thrust_noise[28]={0.0};

float aa = 0.0f, bb = 0.0f, cc = 0.0f;

struct_motor s_rotor;


void v_param_init_rotor()
{
	/*max and zero thrust/torque corresponding PWM for quad motor(present using the cruise motor data)*/
	s_rotor.max_quad_thr_tor_pwm = 1900;
	s_rotor.zero_quad_thr_tor_pwm = 1150;

	/*max and zero thrust/torque corresponding PWM for cruise motor*/
	s_rotor.max_fwv_thr_tor_pwm = 1900;
	s_rotor.zero_fwv_thr_tor_pwm = 1150;

	s_rotor.quad_thrust_max = 2335.2f;	// grams
	s_rotor.quad_thrust_min = 0.0f;		// grams
	s_rotor.quad_thrust = 0.0f;			// grams

	s_rotor.fwv_thrust_max = 2335.2f;	// grams
	s_rotor.fwv_thrust_min = 0.0f;		// grams
	s_rotor.fwv_thrust = 0.0f;			// grams

	s_rotor.quad_torque_max = 0.0f;		// Nm
	s_rotor.quad_torque_min = 0.0f;		// Nm
	s_rotor.quad_torque = 0.0f;			// Nm

	s_rotor.fwv_torque_max = 0.0f;		// Nm
	s_rotor.fwv_torque_min = 0.0f;		// Nm
	s_rotor.fwv_torque = 0.0f;			// Nm

	memset(s_rotor.motor_thrust, 0.0f, sizeof(s_rotor.motor_thrust));	// grams
	memset(s_rotor.motor_torque, 0.0f, sizeof(s_rotor.motor_torque));	// Nm

	s_rotor.pwm_min = 1150;
	s_rotor.pwm_max = 1860;
	s_rotor.thrust_min = 0.0;


	if (quad_num_motors>0)
	{
		s_rotor.thrust_max = 2.0f*mass*g/quad_num_motors;
	}
	else
	{
		s_rotor.thrust_max = 0.0;
	}


	//s_rotor.thrust_l[28] = {0.0};
	//s_rotor.thrust_nl[28] = {0.0};
	//s_rotor.thrust_out[28] = {0.0};
	s_rotor.ratio_end =1.0;
	s_rotor.skewness_ratio = 0.5;
	s_rotor.skewness_power =2.0;

	s_rotor.flg_use_scale_model = 0; // for new scale model use 1, else use 0
}


void v_update_thrust_torque()
{
	float quad_thrust = 0.0f, quad_torque = 0.0f, norm_motor_thrust = 0.0f, norm_motor_torque = 0.0f;

	float norm_pwm[quad_num_motors + fwv_motors] = {0.0f};
	uint16_t rotor_pwm[quad_num_motors + fwv_motors] = {1000};

	int i = 0;
	/*taken from motor static test data*/
	for(i = 0; i < (quad_num_motors + fwv_motors); i++)
	{
		if (i < quad_num_motors)
		{
			rotor_pwm[i] = constrain_float(pwm_out_esc[i], s_rotor.zero_quad_thr_tor_pwm, s_rotor.max_quad_thr_tor_pwm);

			norm_pwm[i] = (float) (rotor_pwm[i] - s_rotor.zero_quad_thr_tor_pwm)/(s_rotor.max_quad_thr_tor_pwm - s_rotor.zero_quad_thr_tor_pwm);

			/*this is MN3515(AARAV cruise motor) motor static test curve, replace it with U5(AARAV quad motor) for exact quad motor characteristics*/
			norm_motor_thrust = -0.02257f + 0.601f*norm_pwm[i] + 0.4709f*powf(norm_pwm[i], 2);	// grams
			norm_motor_thrust = constrain_float(norm_motor_thrust, 0.0f, 1.0f);

			norm_motor_torque = 0.0f + 0.0f*norm_pwm[i] + 0.0f*powf(norm_pwm[i], 2);				// Nm
			norm_motor_torque = constrain_float(norm_motor_torque, 0.0f, 1.0f);

			s_rotor.motor_thrust[i] = (norm_motor_thrust*(s_rotor.quad_thrust_max - s_rotor.quad_thrust_min)) + s_rotor.quad_thrust_min;
			s_rotor.motor_thrust[i] = constrain_float(s_rotor.motor_thrust[i], s_rotor.quad_thrust_min, s_rotor.quad_thrust_max);

			s_rotor.motor_torque[i] = (norm_motor_torque*(s_rotor.quad_torque_max - s_rotor.fwv_torque_min)) + s_rotor.quad_torque_min;
			s_rotor.motor_torque[i] = constrain_float(s_rotor.motor_torque[i], s_rotor.quad_torque_min, s_rotor.quad_torque_max);

			quad_thrust = quad_thrust + s_rotor.motor_thrust[i];
			quad_torque = quad_torque + s_rotor.motor_torque[i];
		}
		else
		{
			rotor_pwm[i] = constrain_float(pwm_out_esc[i], s_rotor.zero_fwv_thr_tor_pwm, s_rotor.max_fwv_thr_tor_pwm);

			norm_pwm[i] = (float) (rotor_pwm[i] - s_rotor.zero_fwv_thr_tor_pwm)/(s_rotor.max_fwv_thr_tor_pwm - s_rotor.zero_fwv_thr_tor_pwm);

			/*MN3515(AARAV cruise motor) motor static test curve*/
			norm_motor_thrust = -0.02257f + 0.601f*norm_pwm[i] + 0.4709f*powf(norm_pwm[i], 2);	//grams
			norm_motor_thrust = constrain_float(norm_motor_thrust, 0.0f, 1.0f);

			norm_motor_torque = 0.0f + 0.0f*norm_pwm[i] + 0.0f*powf(norm_pwm[i], 2);				// Nm
			norm_motor_torque = constrain_float(norm_motor_torque, 0.0f, 1.0f);

			s_rotor.motor_thrust[i] = (norm_motor_thrust*(s_rotor.fwv_thrust_max - s_rotor.fwv_thrust_min)) + s_rotor.fwv_thrust_min;
			s_rotor.motor_torque[i] = (norm_motor_torque*(s_rotor.fwv_torque_max - s_rotor.fwv_torque_min)) + s_rotor.fwv_torque_min;

			s_rotor.fwv_thrust = constrain_float(s_rotor.motor_thrust[i], s_rotor.fwv_thrust_min, s_rotor.fwv_thrust_max);
			s_rotor.fwv_torque = constrain_float(s_rotor.motor_torque[i], s_rotor.fwv_torque_min, s_rotor.fwv_torque_max);
		}
	}

	s_rotor.quad_thrust = constrain_float(quad_thrust, quad_num_motors*s_rotor.quad_thrust_min, quad_num_motors*s_rotor.quad_thrust_max);
	s_rotor.quad_torque = constrain_float(quad_torque, quad_num_motors*s_rotor.quad_torque_min, quad_num_motors*s_rotor.quad_torque_max);
}


void rotor_dynamics(float t_step_rot)
{
	if (t_step_rot<0.0009)
	{
		t_step_rot = 0.001;
	}
	int i=0,j=0,k=0;
	float w =2.0*3.14*40.0;
	float zeta=0.5;

	float max_limit =  (587.0f-317.0f)/(10.0f/1000.0f);
	float min_limit =  -(587.0f-317.0f)/(10.0f/1000.0f);
	// RPM PWM mapping
	/* pwm1 =   ((0.0009489)*powf(rotor_speed[0],2))   +   (0.622 *rotor_speed[0]) + 1107.0;
       pwm2 =   ((0.0009489)*powf(rotor_speed[1],2))   +   (0.622 *rotor_speed[1]) + 1107.0;
       pwm3 =   ((0.0009489)*powf(rotor_speed[2],2))   +   (0.622 *rotor_speed[2]) + 1107.0;
       pwm4 =   ((0.0009489)*powf(rotor_speed[3],2))   +   (0.622 *rotor_speed[3]) + 1107.0;*/

#ifdef Quad_H
	bb = 0.622*1.0f;
	aa = 0.0009489*1.0f;
	cc = 1107.0f;
#endif

#ifdef Quadplane
	bb = 1.0f;
	aa = 1.0f;
	cc = 1.0f;
#endif

#ifdef Coax_Hexa_H_sym
	bb = 0.166556f;
	aa = 0.180722f;
	cc = 1192.9f;
#endif

	factor=1.2f; /*** 1.2 rad/sec/pwm from rpm fineness test ***/

	// PWM to rotor_speed coversion model
	s_rotor.flg_use_scale_model=1;// done for T1 testing, this maps pwm to rotor_speed using linear and non-linear functions with weightage
	//s_rotor.flg_use_scale_model=0;// this maps exact rotor_speed to pwm , generally if you tested on bench then you get data to use like this


	switch(s_rotor.flg_use_scale_model)
	{

	uint8_t iii=0;
	case 0:
	{
		for ( iii=0;iii<quad_num_motors + fwv_motors;iii++)
		{
			if (pwm_out_esc[iii]<=cc)
			{
				rotor_speed[iii]=0.0;
			}
			else
			{
				rotor_speed[iii] = (1.0f)*(-bb  + sqrtf((powf(bb,2)) - (4.0f*aa*(cc-pwm_out_esc[iii]))))/(2.0f*aa);
			}

			/*** RPM quantization  ***/
			rotor_speed[iii]=(floorf(rotor_speed[iii]/factor))*factor;
		}
		break;
	}

	case 1:
	{
		for ( iii=0;iii<quad_num_motors + fwv_motors;iii++)
		{
#ifdef Quadplane
			if ((iii>=quad_num_motors) && (iii<quad_num_motors + fwv_motors))
			{
				s_rotor.thrust_max = mass*g*0.5;
			}
			else
			{
				if (quad_num_motors>0)
				{
					s_rotor.thrust_max = 2.0f*mass*g/quad_num_motors;
				}
				else
				{
					s_rotor.thrust_max = 0.0;
				}
			}
#endif

			if (pwm_out_esc[iii] < (s_rotor.pwm_min + 20.0))
			{
				s_rotor.thrust_out[iii] = s_rotor.thrust_min;
			}
			else if (pwm_out_esc[iii] > (s_rotor.pwm_max - 20.0))
			{
				s_rotor.thrust_out[iii] = s_rotor.thrust_max;
			}
			else
			{
				s_rotor.thrust_l[iii] = ((s_rotor.thrust_max - s_rotor.thrust_min)/(s_rotor.pwm_max - s_rotor.pwm_min))*(pwm_out_esc[iii]-s_rotor.pwm_min);
				s_rotor.thrust_nl[iii] = (powf((pwm_out_esc[iii]-s_rotor.pwm_min),s_rotor.skewness_power))*(s_rotor.ratio_end*((s_rotor.thrust_max - s_rotor.thrust_min)/(powf((s_rotor.pwm_max - s_rotor.pwm_min),s_rotor.skewness_power))));
				s_rotor.thrust_out[iii] = (s_rotor.skewness_ratio)*s_rotor.thrust_l[iii] + (1.0-s_rotor.skewness_ratio)*s_rotor.thrust_nl[iii];
			}

			if (s_rotor.thrust_out[iii] < s_rotor.thrust_min)
			{
				s_rotor.thrust_out[iii] = s_rotor.thrust_min;
			}

			if (s_rotor.thrust_out[iii] > s_rotor.thrust_max)
			{
				s_rotor.thrust_out[iii] = s_rotor.thrust_max;
			}

			rotor_speed[iii] = sqrtf(s_rotor.thrust_out[iii]/b1);

			/*** RPM quantization  ***/
			rotor_speed[iii]=(floorf(rotor_speed[iii]/factor))*factor;
		}
		break;
	}
	}


	// Bandwidth simulation of rotor_speed
	for(i=0;i<quad_num_motors + fwv_motors;i++)
	{
		w_omega[i]=w;
		zeta_omega[i]=zeta;
		rotor_speed_dot_dot[i]=rotor_speed[i]*powf(w_omega[i],2) - 2.0f*zeta_omega[i]*w_omega[i]*rotor_speed_dot[i] - (powf(w_omega[i],2))*rotor_speed_old[i];
	}
	for(k=0;k<4;k++)
	{
		for(i=0;i<quad_num_motors + fwv_motors;i++)
		{	
			rotor_speed_dot[i] = rotor_speed_dot_dot[i]*(t_step_rot/4.0f) + rotor_speed_dot[i];

			if (rotor_speed_dot[i]<min_limit)
			{
				rotor_speed_dot[i]=min_limit;
			}

			if (rotor_speed_dot[i]>max_limit)
			{
				rotor_speed_dot[i]=max_limit;
			}
		}
		for(i=0;i<quad_num_motors + fwv_motors;i++)
		{
			rotor_speed[i] = rotor_speed_dot[i]*(t_step_rot/4.0f) + rotor_speed[i];
		}
	}

	for(i=0;i<quad_num_motors + fwv_motors;i++)
	{
		rotor_speed_old[i]=rotor_speed[i];
	}

	rotor_speed_to_thrust();

	//input_noise_in_thrust();


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



void rotor_speed_to_thrust()
{
	int i=0;

	for(i=0;i<quad_num_motors;i++)
	{
		rotor_force_out[i]= - b1*rotor_speed[i]*rotor_speed[i]; // - sign because motor z axis is down and thrust is up
		// but b*w^2 will always give +ve value, so making it -ve for making it consistent with motor frame z axis direction
	}

	for(i=quad_num_motors;i<quad_num_motors+fwv_motors;i++)
	{
		rotor_force_out[i]= - b1_fwv*rotor_speed[i]*rotor_speed[i]; // - sign because motor z axis is down and thrust is up
		// but b*w^2 will always give +ve value, so making it -ve for making it consistent with motor frame z axis direction
	}
}



void scale_thrust_for_plant_mass()
{

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

