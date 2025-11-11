/*
 * battery_dynamics.h
 *
 *  Created on:
 *      Author: Rajat P.
 */


#include "stdio.h"
#include "plant.h"

#pragma pack(1)


typedef struct
{
	float init_voltage;
	float max_voltage;
	float min_voltage;

	float init_capacity;
	float init_soc;
	float init_resitance;
	float init_energy;

	float rem_energy;
	float rem_capacity;

	float power_consumed;

	float individual_rotor_power;

	float inst_consumed_power;
	float inst_consumed_energy;
	float consumed_energy;
	float voltage;
	float current;
	float capacity;
	float soc;
	float resistance;

	float t_step;
}s_battery;
extern s_battery batt;

typedef struct
{
	float motor_current[quad_num_motors + fwv_motors];
	float avionics_current;

	float current_max;
	float current_min;
	float current;
	float current_pass_time;

	float capacity_consumed;
	float capacity_rem;

	float voltage_max;
	float voltage_cutoff;
	float voltage_rest_rem;

	float internal_resistance;
	float voltage_out;

	uint16_t max_current_pwm;
	uint16_t zero_current_pwm;

	uint8_t no_of_cells;
	uint16_t capacity_max;
	uint16_t capacity_cutoff;
	uint8_t state_of_charge;
}s_battery_dynamics;
extern s_battery_dynamics s_batt;

void v_param_init_battery_dynamics();

void v_update_battery_dynamics();

void battery_dynamics();
