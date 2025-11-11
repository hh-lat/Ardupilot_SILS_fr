#include <math.h>
#include <math_util.h>
#include "plant.h"
#include <common_variable.h>
#include "Forces_and_moments_ctrl_srfce.h"
#include "Actuator_dynamics.h"

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



void Actuator_dynamics(float t_step_act)
{

	float zeta_elvtr = 0.7;
	float zeta_alrn  = 0.7;
	float zeta_rdr   = 0.7;
	float w_elvtr    = 80.0;
	float w_alrn     = 80.0;
	float w_rdr      = 80.0;
	static float ss2_array[3];
	uint8_t i = 0;
	uint8_t k = 0;

	/************************************************************************************************************/
	/*creating array for pwm input and output*/
	uint16_t pwm_in_servo[16] = {0};
	uint16_t pwm_out_servo[16] = {0};


	/*adding delay for the servo output through function*/
	//pure_transport_delay_int(pwm_out_servo, pwm_in_servo, bufferarray_pwm_servo, sizeof(pwm_in_servo)/sizeof(uint16_t),  delay_array_length_pwm_servo);

	v_fill_pwm_in_csta();
	/*Conversion function from pwm out servo to angle*/
	pwm_out_servo_to_angles();


	/*second order transfer function for angle to increase gradually for steps with rate limit*/
	for(i=0;i<num_actuator;i++)
	{
		actuator[csta[i]].ang_accel=actuator[csta[i]].angle*powf(actuator[csta[i]].omega,2) - 2.0f*actuator[csta[i]].zeta*actuator[csta[i]].omega*actuator[csta[i]].ang_vel - (powf(actuator[csta[i]].omega,2))*actuator[csta[i]].angle_old;
	}

	for(k=0;k<4;k++)
	{
		for(i=0;i<num_actuator;i++)
		{
			actuator[csta[i]].ang_vel = actuator[csta[i]].ang_accel*(t_step_act/4.0f) + actuator[csta[i]].ang_vel;//argha latest changes as per January 10 matlab code changes

			if (actuator[csta[i]].ang_vel<actuator[csta[i]].min_rate)
			{
				actuator[csta[i]].ang_vel=actuator[csta[i]].min_rate;
			}

			if (actuator[csta[i]].ang_vel>actuator[csta[i]].max_rate)
			{
				actuator[csta[i]].ang_vel=actuator[csta[i]].max_rate;
			}
		}
		for(i=0;i<num_actuator;i++)
		{
			actuator[csta[i]].angle = actuator[csta[i]].ang_vel*(t_step_act/4.0f) + actuator[csta[i]].angle;
		}
	}

	for(i=0;i<num_actuator;i++)
	{
		actuator[csta[i]].angle_old = actuator[csta[i]].angle;
	}
}

