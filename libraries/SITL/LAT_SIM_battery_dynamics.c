/*
 * battery_dynamic.c
 *
 *  Created on:
 *      Author: Rajat P.
 */


#include "LAT_SIM_Runner.h"
#include "LAT_SIM_math_util.h"
#include "LAT_SIM_common_variable.h"
#include "LAT_SIM_battery_dynamics.h"

s_battery  batt;
s_battery_dynamics s_batt;

void v_param_init_battery_dynamics()
{
	s_batt.max_current_pwm = 1900;
	s_batt.zero_current_pwm = 1200;

	memset(s_batt.motor_current, 0.0f, sizeof(s_batt.motor_current));	// Amphere

	s_batt.avionics_current = 0.2f;	// Amphere (consumes around 2A if payload is added)

	s_batt.current_max = 22.7f;			// Amphere
	s_batt.current_min = 0.0f;			// Amphere
	s_batt.current = 0.0f;				// Amphere
	s_batt.current_pass_time = 0.0f;	// seconds

	s_batt.capacity_max = 25342; 						// mAh
	s_batt.capacity_cutoff = 5388;						// mAh
	s_batt.capacity_consumed = 0.0f;					// mAh
	s_batt.capacity_rem = (float) s_batt.capacity_max; 	// mAh

	s_batt.state_of_charge = 100;	// percentage calc by (capacity remaining)/(total capacity)

	s_batt.voltage_max = 24.2769f; 							// volts
	s_batt.voltage_cutoff = 20.4824f; 						// volts
	s_batt.voltage_rest_rem = (float) s_batt.voltage_max;	// volts

	s_batt.no_of_cells = 6;
	s_batt.internal_resistance = 1.3f*(1e-3)*s_batt.no_of_cells;	// ohms

	s_batt.voltage_out = s_batt.voltage_rest_rem - (s_batt.current*s_batt.internal_resistance);	// battery voltage - current*internal_resistance
}


void v_update_battery_dynamics()
{
	static float prev_time;

	float motor_current = 0.0f, norm_motor_current = 0.0f;
	float norm_cap_rem = 1.0f, norm_batt_rem = 1.0f;

	float norm_pwm[quad_num_motors + fwv_motors] = {0.0f};
	uint16_t battery_pwm[quad_num_motors + fwv_motors] = {1000};

	int i = 0;
	/*current consumtion of battery based on motor PWM and battery voltage*/
	/*taken from motor static test data*/
	for(i = 0; i < (quad_num_motors + fwv_motors); i++)
	{
		battery_pwm[i] = constrain_float(pwm_out_esc[i], s_batt.zero_current_pwm, s_batt.max_current_pwm);

		norm_pwm[i] = (float) (battery_pwm[i] - s_batt.zero_current_pwm)/(s_batt.max_current_pwm - s_batt.zero_current_pwm);

		norm_motor_current = - 0.01034f - 0.01726*norm_pwm[i] + 1.0703f*powf(norm_pwm[i], 2); // Amphere
		norm_motor_current = constrain_float(norm_motor_current, 0.0f, 1.0f);

		s_batt.motor_current[i] = (norm_motor_current*(s_batt.current_max - s_batt.current_min)) + s_batt.current_min;
		s_batt.motor_current[i] = constrain_float(s_batt.motor_current[i], s_batt.current_min, s_batt.current_max);

		motor_current = motor_current + s_batt.motor_current[i];
	}

	s_batt.current = motor_current + s_batt.avionics_current;

	s_batt.current_pass_time = t - prev_time; // t in seconds

	prev_time = t;

	/*calculate consumed mAh to know mAh remaining*/
	s_batt.capacity_consumed = s_batt.current*(s_batt.current_pass_time/3600.0f)*1000.0f; // Amphere*hour*milli = mAh

	/*battery capacity remaining*/
	s_batt.capacity_rem = s_batt.capacity_rem - s_batt.capacity_consumed;
	s_batt.capacity_rem = constrain_float(s_batt.capacity_rem, 0.0f, s_batt.capacity_max);

	/*percentage of battery remaining*/
	s_batt.state_of_charge = (s_batt.capacity_rem/s_batt.capacity_max)*100;
	s_batt.state_of_charge = constrain_float(s_batt.state_of_charge, 0, 100);

	if (s_batt.capacity_rem >= s_batt.capacity_cutoff)
	{
		/*normalizing the capacity remaining*/
		norm_cap_rem = (float) (s_batt.capacity_rem - s_batt.capacity_cutoff)/(s_batt.capacity_max - s_batt.capacity_cutoff);
		norm_cap_rem = constrain_float(norm_cap_rem, 0.0f, 1.0f);

		/*normalized battery discharge curve for cruise [(voltage_measured + current_measured*internal_resistance) vs mAh_remaining)]*/
		/*taken from battery discharge test data*/
		norm_batt_rem = 0.01322f + 0.9409f*norm_cap_rem + 0.08534f*powf(norm_cap_rem, 2);
		norm_batt_rem = constrain_float(norm_batt_rem, 0.0f, 1.0f);

		/*battery voltage remaining*/
		s_batt.voltage_rest_rem = (norm_batt_rem*(s_batt.voltage_max - s_batt.voltage_cutoff)) + s_batt.voltage_cutoff;
		s_batt.voltage_rest_rem = constrain_float(s_batt.voltage_rest_rem, s_batt.voltage_cutoff, s_batt.voltage_max);
	}
	else
	{
		/*if the mAh left in the battery is very less. then the resting voltage vs mAh remaining curve changes*/
		norm_batt_rem = 0.0f + 0.0f*norm_cap_rem + 0.0f*powf(norm_cap_rem, 2);
		norm_batt_rem = constrain_float(norm_batt_rem, 0.0, 1.0);

		s_batt.voltage_rest_rem = 18.0f;
	}

	/*battery out = remaining battery - (current * internal resistance)*/
	s_batt.voltage_out = s_batt.voltage_rest_rem - (s_batt.current*s_batt.internal_resistance);
}


void battery_dynamics()
{
	v_update_battery_dynamics();


//   batt.inst_consumed_energy = batt.inst_consumed_power * batt.t_step;
//   batt.consumed_energy =  batt.consumed_energy + batt.inst_consumed_energy;
//   batt.rem_energy = batt.init_energy- batt.consumed_energy;
//
//
//   batt.init_energy = batt.init_capacity*batt.init_voltage*3600.0f;
}


void thrust_to_power()
{
	batt.inst_consumed_power = 0.0f;
	for (int i = 0; i < 28; i++)
	{
	     batt.individual_rotor_power = (rotor_force_out[i]*rotor_force_out[i])*31.17f + 64.97f*(rotor_force_out[i]) - 1.244;// for U5 motor with 16*5.4 prop and 22.2 constant voltage
	     batt.inst_consumed_power  =   batt.inst_consumed_power  + batt.individual_rotor_power;
	     batt.capacity = batt.capacity - batt.current*batt.t_step;
	}
}



