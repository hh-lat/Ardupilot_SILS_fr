/*
 * wind_model.h
 *
 *  Created on	: 07-Nov-2022
 *      Author	: Monesh P.
 */


#include "stdio.h"
#include "plant.h"


typedef struct
{
	float wind_ned_low[3];
	float wind_ned_high[3];

	float wind_ned_low_old[3];
	float wind_ned_high_old[3];

	float t_low_wind;
	float t_high_wind;

	float t_old;

	uint8_t wind_flag;

	uint8_t turbulent_wind_flag;
}strct_wind;

extern strct_wind s_wind;


extern void v_param_init_wind();

extern void v_filter(float vector[3], float vector_old[3], float dt, float freq);

extern void v_update_wind_ned();


