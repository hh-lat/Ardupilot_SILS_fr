/*
 * esc_dynamics.c
 *  Created on:
 *      Author: Rajat P.
 */


#include "esc_dynamics.h"
#include "plant.h"
#include <math.h>
#include <math_util.h>
#include <common_variable.h>
#include "Forces_and_moments_rotors.h"


s_esc_dynamics s_esc;

void v_param_init_esc_dynamics()
{
	memset(pwm_out_esc, 1000, sizeof(pwm_out_esc));

	s_esc.pwm_out_esc_max = 2000;
	s_esc.pwm_out_esc_min = 1000;
}

void esc_dynamics()
{
	//pure_transport_delay_int(pwm_out_esc, pwm_in, bufferarray_pwm, sizeof(pwm_in)/sizeof(int),  delay_array_length_pwm);

	 memmove(pwm_out_esc,pwm_in,sizeof(pwm_in));
}
		

