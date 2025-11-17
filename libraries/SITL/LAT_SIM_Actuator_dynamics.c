#include <math.h>
#include <LAT_SIM_math_util.h>

#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_Actuator_dynamics.h"

#define DIV 1e-1


Ctrl_srfc_actuator actuator[16] ={0};
AKAP_CONTROL_SURFACE_TYPE csta[16] ={0};

void v_actuator_dynamics_param_init()
{


}


void v_fill_pwm_in_csta()
{
	actuator[AILERON_LEFT].pwm  = pwm_in_servo[0];// aileron left
	actuator[AILERON_RIGHT].pwm  = pwm_in_servo[1]; // aileron right

	actuator[ELEVATOR_COMMON].pwm  = pwm_in_servo[2]; // elevator common
	actuator[RUDDER_COMMON].pwm  = pwm_in_servo[3]; // rudder_common
}



void pwm_out_servo_to_angles()
{
	uint8_t i = 0;

	for(i=0;i<num_actuator;i++)
	{
		if (actuator[csta[i]].pwm <= actuator[csta[i]].pwm_min)
		{
			actuator[csta[i]].angle = actuator[csta[i]].angle_min;
		}
		else if (actuator[csta[i]].pwm >= actuator[csta[i]].pwm_max)
		{
			actuator[csta[i]].angle = actuator[csta[i]].angle_max;
		}
		else if ( (actuator[csta[i]].pwm > actuator[csta[i]].pwm_min) && (actuator[csta[i]].pwm < actuator[csta[i]].pwm_max) )
		{
			actuator[csta[i]].angle = actuator[csta[i]].angle_min +
					((actuator[csta[i]].angle_max - actuator[csta[i]].angle_min)/(actuator[csta[i]].pwm_max - actuator[csta[i]].pwm_min))*(actuator[csta[i]].pwm - actuator[csta[i]].pwm_min);
		}
	}
}



void v_actuator_dynamics(float t_step_act)
{
	pwm_out_servo_to_angles();
	v_apply_rate_limits_to_control_surfaces(t_step_act);

}


void v_apply_rate_limits_to_control_surfaces(float t_step_act)
{
	uint8_t i = 0;
	float angle_rate_cmd = 0.0f;

	for(i=0;i<num_actuator;i++)
	{
		angle_rate_cmd = (actuator[csta[i]].angle - actuator[csta[i]].angle_old)/t_step_act;

		if (angle_rate_cmd > actuator[csta[i]].max_rate)
		{
			angle_rate_cmd = actuator[csta[i]].max_rate;
		}
		else if (angle_rate_cmd < actuator[csta[i]].min_rate)
		{
			angle_rate_cmd = actuator[csta[i]].min_rate;
		}

		actuator[csta[i]].angle = actuator[csta[i]].angle_old + angle_rate_cmd*t_step_act;

		actuator[csta[i]].angle_old = actuator[csta[i]].angle;
	}
}
