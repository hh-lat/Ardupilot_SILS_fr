#include <stdint.h>
#include <stdlib.h>
#include "plant.h"
#include "update_vehicle_state.h"
#include "math_util.h"
#include "wind_model.h"

float temp3X1_5[3] ={0.0};
float temp3X1_6[3] ={0.0};

void v_init_vehicle_states()
{
	vehicle.dof = DOF_ALL_MOTION;
}

void v_update_vehicle_states(float state[])
{
	vehicle.V_b_gnd[0] = state[0];
	vehicle.V_b_gnd[1] = state[1];
	vehicle.V_b_gnd[2] = state[2];

	body_to_NED(vehicle.V_b_gnd, vehicle.V_ned_gnd);

	v_update_wind_ned(); //updates wind velocities in NED frame

	temp3X1_5[0] = vehicle.V_ned_gnd[0] - vehicle.wind_ned[0];
	temp3X1_5[1] = vehicle.V_ned_gnd[1] - vehicle.wind_ned[1];
	temp3X1_5[2] = vehicle.V_ned_gnd[2] - vehicle.wind_ned[2];

	NED_to_body(temp3X1_5,temp3X1_6); //&vehicle.V_b_tas);

	vehicle.V_b_tas[0] = temp3X1_6[0];
	vehicle.V_b_tas[1] = temp3X1_6[1];
	vehicle.V_b_tas[2] = temp3X1_6[2];

	vehicle.tas = sqrtf(vehicle.V_b_tas[0]*vehicle.V_b_tas[0] + vehicle.V_b_tas[1]*vehicle.V_b_tas[1] + vehicle.V_b_tas[2]*vehicle.V_b_tas[2]);
	vehicle.gs  = sqrtf(vehicle.V_b_gnd[0]*vehicle.V_b_gnd[0] + vehicle.V_b_gnd[1]*vehicle.V_b_gnd[1] + vehicle.V_b_gnd[2]*vehicle.V_b_gnd[2]);

	vehicle.hrz_gnd_speed = sqrtf(vehicle.V_b_gnd[0]*vehicle.V_b_gnd[0] + vehicle.V_b_gnd[1]*vehicle.V_b_gnd[1] );
	vehicle.vert_gnd_vel  = vehicle.V_b_gnd[2];


	vehicle.phi    		= state[6];
	vehicle.theta  		= state[7];
	vehicle.psi    		= state[8];
	vehicle.pos_ned[0] 	= state[9];
	vehicle.pos_ned[1] 	= state[10];
	vehicle.pos_ned[2] 	= state[11];


	if ((vehicle.tas < 3.0) || (fabsf(vehicle.V_b_tas[0]) < 3.0))
	{
		vehicle.alpha = 0;
	}
	else
	{
		vehicle.alpha = atan2f(vehicle.V_b_tas[2], vehicle.V_b_tas[0]);
	}

	vehicle.alpha = constrain_float(vehicle.alpha,-20.0f/57.3f,20.0f/57.3f);


	if(vehicle.tas < 3.0)
	{
		vehicle.beta = 0;
	}
	else
	{
		vehicle.beta = asinf(vehicle.V_b_tas[1]/sqrtf(powf(vehicle.V_b_tas[0],2) + powf(vehicle.V_b_tas[1],2) + powf(vehicle.V_b_tas[2],2)));
	}

	vehicle.beta = constrain_float(vehicle.beta,-20.0/57.3f,20.0/57.3f);

	/*vehicle.gamma = vehicle.vert_gnd_vel/vehicle.hrz_gnd_speed*/

	vehicle.Q = 0.5f*vehicle.rho*vehicle.tas*vehicle.tas;
}


void v_fill_lla_to_vehicle_state(float latitude_point,float longitude_point, float Alt)
{
	vehicle.lat = latitude_point;
	vehicle.lon = longitude_point;
	vehicle.alt_msl = Alt; // should fill negative for height above MSL
	vehicle.alt_agl = vehicle.alt_msl - s_home_state.alt_msl;// up is negative
}



