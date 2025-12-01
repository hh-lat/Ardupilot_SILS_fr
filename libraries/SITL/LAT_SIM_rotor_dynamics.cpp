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

void v_rotor_esc_dynamics(float t_step_rot)
{
	v_motor_pwm_in_2_out();

	v_motor_pwm_to_throttle(t_step_rot);
}

void v_rotor_dynamics(float t_step_rot)
{
	v_throttle_to_thrust_torque(t_step_rot); // updates s_motor.rotor_force_out[]

	v_rotors_force_and_moments(); // updates in body frame of uav
}

void v_motor_pwm_in_2_out()
{
	for (int i=0;i<(s_motor_manager.num_motors);i++)
	{
		s_motor[i].pwm_out = s_motor[i].pwm_in;
	}
}


void v_motor_pwm_to_throttle(float t_step_rot)
{
	float throttle_rate_cmd = 0.0f;

	for (int i=0;i<(s_motor_manager.num_motors);i++)
	{
		// PWM to throttle conversion
		if (s_motor[i].pwm_out <= s_motor[i].pwm_min)
		{
			s_motor[i].throttle_cmd = 0.0f;
		}
		else if (s_motor[i].pwm_out >= s_motor[i].pwm_max)
		{
			s_motor[i].throttle_cmd = 1.0f;
		}
		else if ( (s_motor[i].pwm_out > s_motor[i].pwm_min) && (s_motor[i].pwm_out < s_motor[i].pwm_max) )
		{
			s_motor[i].throttle_cmd = (s_motor[i].pwm_out - s_motor[i].pwm_min)/(s_motor[i].pwm_max - s_motor[i].pwm_min);
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
		s_motor[i].throttle_cmd_old = s_motor[i].throttle_cmd;
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
			v_update_dynamic_thrust_parameters();
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

	for (int i = 0; i < s_motor_manager.num_motors ; i++)
	{
		float u = 0;
		float v = 0;
		float w = 0;
		u = s_motor[i].thrust_out;
		
		float sin_phi = sinf(s_motor[i].rotor_tilt[0]);
		float cos_phi = cosf(s_motor[i].rotor_tilt[0]);
		float sin_theta = sinf(s_motor[i].rotor_tilt[1]);
		float cos_theta = cosf(s_motor[i].rotor_tilt[1]);
		float sin_psi = sinf(s_motor[i].rotor_tilt[2]);
		float cos_psi = cosf(s_motor[i].rotor_tilt[2]);
		
		s_motor[i].rotor_force[0] =  u*cos_psi*cos_theta  + w * (sin_phi * sin_psi + cos_phi * cos_psi * sin_theta) - v * (cos_phi * sin_psi - cos_psi * sin_phi * sin_theta);
		s_motor[i].rotor_force[1] =  u*cos_theta*sin_psi  + v * (cos_phi * cos_psi + sin_phi * sin_psi * sin_theta) - w * (cos_psi * sin_phi - cos_phi * sin_psi * sin_theta);
		s_motor[i].rotor_force[2] = -u*sin_theta          + w * cos_phi*cos_theta  + cos_theta * sin_phi * v;
	}
}


void v_yaw_moment_motor_frame()
{
	for (int i = 0; i < s_motor_manager.num_motors ; i++)
	{
		s_motor[i].rotor_yaw_moment_m = s_motor[i].thrust_2_torque_factor * s_motor[i].rotor_r_direction * fabsf(s_motor[i].rotor_force_out)  ;
	}
}


void v_yaw_moment_rotor2body()
{
	for (int i = 0; i < s_motor_manager.num_motors ; i++)
	{
		
		float u = 0;
		float v = 0;
		float w = 0;
		u = s_motor[i].rotor_yaw_moment_m;
		
		float sin_phi = sinf(s_motor[i].rotor_tilt[0]);
		float cos_phi = cosf(s_motor[i].rotor_tilt[0]);
		float sin_theta = sinf(s_motor[i].rotor_tilt[1]);
		float cos_theta = cosf(s_motor[i].rotor_tilt[1]);
		float sin_psi = sinf(s_motor[i].rotor_tilt[2]);
		float cos_psi = cosf(s_motor[i].rotor_tilt[2]);
		
		s_motor[i].rotor_yaw_moment_b[0] =  u*cos_psi*cos_theta  + w * (sin_phi * sin_psi + cos_phi * cos_psi * sin_theta) - v * (cos_phi * sin_psi - cos_psi * sin_phi * sin_theta);
		s_motor[i].rotor_yaw_moment_b[1] =  u*cos_theta*sin_psi  + v * (cos_phi * cos_psi + sin_phi * sin_psi * sin_theta) - w * (cos_psi * sin_phi - cos_phi * sin_psi * sin_theta);
		s_motor[i].rotor_yaw_moment_b[2] = -u*sin_theta          + w * cos_phi*cos_theta  + cos_theta * sin_phi * v;
	}
}


void v_moment_rotor2body()
{
	for (int i = 0; i < s_motor_manager.num_motors ; i++)
	{
		cross_product(s_motor[i].rotor_xyz,s_motor[i].rotor_force,s_motor[i].rotor_moment);

		s_motor[i].rotor_moment[0] = s_motor[i].rotor_moment[0] + s_motor[i].rotor_yaw_moment_b[0] ;
		s_motor[i].rotor_moment[1] = s_motor[i].rotor_moment[1] + s_motor[i].rotor_yaw_moment_b[1] ;
		s_motor[i].rotor_moment[2] = s_motor[i].rotor_moment[2] + s_motor[i].rotor_yaw_moment_b[2] ;
	}
}


void v_rotors_force_and_moments()
{
	v_thrust_rotor2body();
	v_yaw_moment_motor_frame();
	v_yaw_moment_rotor2body();
	v_moment_rotor2body();

	float all_rotors_force[3]={0.0};
	float all_rotors_moment[3]={0.0};

	for (int i = 0; i < s_motor_manager.num_motors; i++)
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


void v_update_dynamic_thrust_parameters()
{
	v_update_rotors_rpm_from_throttle();
	v_update_rotors_advance_ratio(vehcle.tas);
	v_update_rotors_Cmu();
	v_update_rotors_thrust_coefficient();
	v_update_rotors_thrust_from_CT();
}


void v_update_rotors_rpm_from_throttle()
{
	for (int i=0;i<(s_motor_manager.num_motors);i++)
	{
		switch (vehcle.plane_model)
		{
			case PLANE_ARDU_DEFAULT:
			{
				s_motor[i].rpm = s_motor[i].rpm_min + s_motor[i].throttle_cmd * (s_motor[i].rpm_max - s_motor[i].rpm_min);
				if (s_motor[i].rpm < 0.0f)
				{
					s_motor[i].rpm = 0.0f;
				}
			break;
			}

			case PLANE_EQX:
			{
				s_motor[i].rpm = 39543*s_motor[i].throttle_cmd - 3792.7;
				if (s_motor[i].rpm < 0.0f)
				{
					s_motor[i].rpm = 0.0f;
				}
			break;
			}
		}
	}
}

void v_update_rotors_advance_ratio(float V_inf)
{
	for (int i=0;i<(s_motor_manager.num_motors);i++)
	{
		s_motor[i].J = V_inf/(((s_motor[i].rpm + 1e-6)/60.0)*s_motor[i].dia_prop);
		if (s_motor[i].J > 15.0)
		{
			s_motor[i].J = 15.0;
		}

		if (s_motor[i].J < 0.01)
		{
			s_motor[i].J = 0.01;
		}
	}
}

void v_update_rotors_Cmu()
{
	for (int i=0;i<(s_motor_manager.num_motors);i++)
	{
		switch (vehcle.plane_model)
		{
			case PLANE_ARDU_DEFAULT:
			{
				s_motor[i].Cmu = 0;
			break;
			}

			case PLANE_EQX:
			{
				s_motor[i].Cmu = 0.0966 / powf(s_motor[i].J,2.314);

				if (s_motor[i].Cmu > 50.0f)
				{
					s_motor[i].Cmu = 50.0f;
				}

			break;
			}
		}
	}
}

void v_update_rotors_thrust_coefficient()
{
	for (int i=0;i<(s_motor_manager.num_motors);i++)
	{
		switch (vehcle.plane_model)
		{
			case PLANE_ARDU_DEFAULT:
			{
				s_motor[i].CT = s_motor[i].CT_static;
				if (s_motor[i].CT < 0.0f)
				{
					s_motor[i].CT = 0.0f;
				}
			break;
			}

			case PLANE_EQX:
			{
				s_motor[i].CT = -0.0283*powf(s_motor[i].J,2) - 0.917*s_motor[i].J + 1.3647;

				if (s_motor[i].CT < 0.0f)
				{
					s_motor[i].CT = 0.0f;
				}
			break;
			}
		}
	}
}


void v_update_rotors_thrust_from_CT()
{
	for (int i=0;i<(s_motor_manager.num_motors);i++)
	{
		s_motor[i].thrust_out =  vehcle.rho * powf((s_motor[i].rpm/60.0),2) * powf(s_motor[i].dia_prop,4) * s_motor[i].CT;
		if (s_motor[i].thrust_out < 0.0f)
		{
			s_motor[i].thrust_out = 0.0f;
		}
		break;
	}
}


void v_update_vehcle_Cmu()
{
	float Cmu =0;
		switch (vehcle.plane_model)
		{
			case PLANE_ARDU_DEFAULT:
			{
				vehcle.Cmu = 0;
			break;
			}

			case PLANE_EQX:
			{
				for (int i=0;i<(s_motor_manager.num_motors);i++)
				{
					Cmu = Cmu + s_motor[i].Cmu;
				}
				vehcle.Cmu = Cmu / s_motor_manager.num_motors;
			break;
			}
		}

}