/*
 * esc_dynamics.h
 *  Created on:
 *      Author: Rajat P.
 */


#include "stdio.h"
#include "stdint.h"


typedef struct
{
	uint16_t pwm_out_esc_max;
	uint16_t pwm_out_esc_min;
}s_esc_dynamics;
extern s_esc_dynamics s_esc;

extern uint16_t pwm_out_esc[28];
extern uint16_t pwm_in[28];
void v_param_init_esc_dynamics();

void esc_dynamics();


