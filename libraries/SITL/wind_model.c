/*
 * wind_model.c
 *
 *  Created on	: 07-Nov-2022
 *      Author	: Monesh P.
 *  Description	: This file contains the Turbulent wind modeling.
 */


#include "wind_model.h"
#include "plant.h"
#include "math_util.h"
#include "stdint.h"
#include "stdlib.h"

strct_wind s_wind;


void v_param_init_wind()
{
	s_wind.wind_ned_low[0] = 0.0f;
	s_wind.wind_ned_low[1] = 0.0f;
	s_wind.wind_ned_low[2] = 0.0f;

	s_wind.wind_ned_high[0] = 0.0f;
	s_wind.wind_ned_high[1] = 0.0f;
	s_wind.wind_ned_high[2] = 0.0f;

	s_wind.wind_ned_low_old[0] = 0.0f;
	s_wind.wind_ned_low_old[1] = 0.0f;
	s_wind.wind_ned_low_old[2] = 0.0f;

	s_wind.wind_ned_high_old[0] = 0.0f;
	s_wind.wind_ned_high_old[1] = 0.0f;
	s_wind.wind_ned_high_old[2] = 0.0f;

	s_wind.t_low_wind = 0.0f;
	s_wind.t_high_wind = 0.0f;

	s_wind.wind_flag = 0;
	s_wind.turbulent_wind_flag = 0;
}


void v_filter(float vector[3], float vector_old[3], float dt, float freq)
{
	int i = 0;

	for(i=0;i<3;i++)
	{
		vector[i] = vector_old[i] + (dt/(dt+(1/(2*PI*freq))))*(vector[i] - vector_old[i]);
	}
}


void v_update_wind_ned()
{
	//wind off
	if (s_wind.wind_flag == 0)
	{
		vehicle.wind_ned[0] = 0.0f;
		vehicle.wind_ned[1] = 0.0f;
		vehicle.wind_ned[2] = 0.0f;
	}
	//wind on
	else
	{
		//constant wind
		if (s_wind.turbulent_wind_flag == 0)
		{
			vehicle.wind_ned[0] = 3.0f;
			vehicle.wind_ned[1] = 2.0f;
			vehicle.wind_ned[2] = 0.0f;
		}
		//turbulent wind
		else
		{
			if ((t - s_wind.t_low_wind) > 0.2f)
			{
				s_wind.t_low_wind = t;

				// 0.5m/s wind with filter frequency of 5Hz (low wind with high filter frequency)
				srand(time(0));
				//outputs random float value between 0 and 0.5m/s
				s_wind.wind_ned_low[0] = ((float)rand()/(float)RAND_MAX)*0.5f;
				s_wind.wind_ned_low[1] = ((float)rand()/(float)RAND_MAX)*0.5f;
				s_wind.wind_ned_low[2] = ((float)rand()/(float)RAND_MAX)*0.2f;

				//5Hz filter
//				v_filter(s_wind.wind_ned_low, s_wind.wind_ned_low_old, 0.001f, 5.0f);

				s_wind.wind_ned_low_old[0] = s_wind.wind_ned_low[0];
				s_wind.wind_ned_low_old[1] = s_wind.wind_ned_low[1];
				s_wind.wind_ned_low_old[2] = s_wind.wind_ned_low[2];
			}

			if ((t - s_wind.t_high_wind) > 3.0f)
			{
				s_wind.t_high_wind = t;

				// 3m/s wind with filter frequency of 0.333Hz (high wind with low filter frequency)
				srand(time(0));
				//outputs random float value between 0 and 3m/s
				s_wind.wind_ned_high[0] = ((float)rand()/(float)RAND_MAX)*3.0f;
				s_wind.wind_ned_high[1] = ((float)rand()/(float)RAND_MAX)*3.0f;
				s_wind.wind_ned_high[2] = ((float)rand()/(float)RAND_MAX)*1.0f;

				//0.333Hz filter
//				v_filter(s_wind.wind_ned_high, s_wind.wind_ned_high_old, 0.001f, 0.333f);

				s_wind.wind_ned_high_old[0] = s_wind.wind_ned_high[0];
				s_wind.wind_ned_high_old[1] = s_wind.wind_ned_high[1];
				s_wind.wind_ned_high_old[2] = s_wind.wind_ned_high[2];
			}


			vehicle.wind_ned[0] = s_wind.wind_ned_low[0] + s_wind.wind_ned_high[0];
			vehicle.wind_ned[1] = s_wind.wind_ned_low[1] + s_wind.wind_ned_high[1];
			vehicle.wind_ned[2] = s_wind.wind_ned_low[2] + s_wind.wind_ned_high[2];

			if ((t - s_wind.t_old) > 1.0f)
			{
				s_wind.t_old = t;
			}
		}
	}
}


