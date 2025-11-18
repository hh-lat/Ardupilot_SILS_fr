#include <math.h>
#include "LAT_SIM_math_util.h"

#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_esc_dynamics.h"


#define DIV 1e-1

S_SERVO s_servo[16] ={0};
CONTROL_SURFACE_TYPE csta[16] ={0};
S_SERVO_MANAGER s_servo_manager;

void v_pwm_out_servo_to_angles()
{
	uint8_t i = 0;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		if (s_servo[csta[i]].pwm_out <= s_servo[csta[i]].pwm_min)
		{
			s_servo[csta[i]].angle = s_servo[csta[i]].angle_min;
		}
		else if (s_servo[csta[i]].pwm_out >= s_servo[csta[i]].pwm_max)
		{
			s_servo[csta[i]].angle = s_servo[csta[i]].angle_max;
		}
		else if ( (s_servo[csta[i]].pwm_out > s_servo[csta[i]].pwm_min) && (s_servo[csta[i]].pwm_out < s_servo[csta[i]].pwm_max) )
		{
			s_servo[csta[i]].angle = s_servo[csta[i]].angle_min +
					((s_servo[csta[i]].angle_max - s_servo[csta[i]].angle_min)/(s_servo[csta[i]].pwm_max - s_servo[csta[i]].pwm_min))*(s_servo[csta[i]].pwm_out - s_servo[csta[i]].pwm_min);
		}
	}
}



void v_servo_dynamics(float t_step_act)
{
	pwm_out_servo_to_angles();
	v_apply_rate_limits_to_control_surfaces(t_step_act);
}


void v_apply_rate_limits_to_control_surfaces(float t_step_act)
{
	uint8_t i = 0;
	float angle_rate_cmd = 0.0f;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		angle_rate_cmd = (s_servo[csta[i]].angle - s_servo[csta[i]].angle_old)/t_step_act;

		if (angle_rate_cmd > s_servo[csta[i]].angular_rate_max)
		{
			angle_rate_cmd = s_servo[csta[i]].angular_rate_max;
		}
		else if (angle_rate_cmd < s_servo[csta[i]].angular_rate_min)
		{
			angle_rate_cmd = s_servo[csta[i]].angular_rate_min;
		}

		s_servo[csta[i]].angle = s_servo[csta[i]].angle_old + angle_rate_cmd*t_step_act;

		s_servo[csta[i]].angle_old = s_servo[csta[i]].angle;
	}
}
