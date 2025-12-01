#include <math.h>
#include "LAT_SIM_math_util.h"

#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_Runner.h"


#define DIV 1e-1

S_SERVO s_servo[16];
CONTROL_SURFACE_TYPE csta[16] ={NOT_ASSIGNED};
S_SERVO_MANAGER s_servo_manager;


void v_set_servo_params(float pwm_min, float pwm_max, float angle_min, float angle_max,
		float omega, float zeta, float min_rate, float max_rate,
		float min_accel, float max_accel, CONTROL_SURFACE_TYPE type)
{
	s_servo[type].pwm_min = pwm_min;
	s_servo[type].pwm_max = pwm_max;

	s_servo[type].angle_min = angle_min;
	s_servo[type].angle_max = angle_max;

	s_servo[type].omega = omega;
	s_servo[type].zeta = zeta;

	s_servo[type].angular_rate_min = min_rate;
	s_servo[type].angular_rate_max = max_rate;

	s_servo[type].angular_accel_min = min_accel;
	s_servo[type].angular_accel_max = max_accel;
}

void v_pwm_out_servo_to_angles()
{
	uint8_t i = 0;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		if (s_servo[i].pwm_out <= s_servo[i].pwm_min)
		{
			s_servo[i].angle = s_servo[i].angle_min;
		}
		else if (s_servo[i].pwm_out >= s_servo[i].pwm_max)
		{
			s_servo[i].angle = s_servo[i].angle_max;
		}
		else if ( (s_servo[i].pwm_out > s_servo[i].pwm_min) && (s_servo[i].pwm_out < s_servo[i].pwm_max) )
		{
			s_servo[i].angle = s_servo[i].angle_min +
					((s_servo[i].angle_max - s_servo[i].angle_min)/(s_servo[i].pwm_max - s_servo[i].pwm_min))*(s_servo[i].pwm_out - s_servo[i].pwm_min);
		}
	}
}

void v_pwm_in_2_out_servo()
{
	uint8_t i = 0;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		if (s_servo[i].pwm_in <= s_servo[i].pwm_min)
		{
			s_servo[i].pwm_out = s_servo[i].pwm_min;
		}
		else if (s_servo[i].pwm_in >= s_servo[i].pwm_max)
		{
			s_servo[i].pwm_out = s_servo[i].pwm_max;
		}
		else if ( (s_servo[i].pwm_in > s_servo[i].pwm_min) && (s_servo[i].pwm_in < s_servo[i].pwm_max) )
		{
			s_servo[i].pwm_out = s_servo[i].pwm_in;
		}
	}
}


void v_servo_dynamics(float t_step_act)
{
	v_pwm_in_2_out_servo();
	v_pwm_out_servo_to_angles();
	v_apply_rate_limits_to_control_surfaces(t_step_act);
	v_fill_vehcle_angles();
}


void v_fill_vehcle_angles()
{
	vehcle.delta_a = s_servo[AILERON_COMMON].angle;
	vehcle.delta_e = s_servo[ELEVATOR_COMMON].angle;
	vehcle.delta_r = s_servo[RUDDER_COMMON].angle;

	vehcle.delta_aL = s_servo[AILERON_LEFT].angle;
	vehcle.delta_aR = s_servo[AILERON_RIGHT].angle;
	vehcle.delta_f = s_servo[FLAP].angle;
}



void v_apply_rate_limits_to_control_surfaces(float t_step_act)
{
	uint8_t i = 0;
	float angle_rate_cmd = 0.0f;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		angle_rate_cmd = (s_servo[i].angle - s_servo[i].angle_old)/t_step_act;

		if (angle_rate_cmd > s_servo[i].angular_rate_max)
		{
			angle_rate_cmd = s_servo[i].angular_rate_max;
		}
		else if (angle_rate_cmd < s_servo[i].angular_rate_min)
		{
			angle_rate_cmd = s_servo[i].angular_rate_min;
		}

		s_servo[i].angle = s_servo[i].angle_old + angle_rate_cmd*t_step_act;

		s_servo[i].angle_old = s_servo[i].angle;
	}
}
