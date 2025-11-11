
/* Rajat*/

#include "stdio.h"
#include "Ground_model.h"
#include "plant.h"
#include "rotor_dynamics.h"
struct_grnd_model s_grnd_model;

void v_ground_model_param_init()
{
	s_grnd_model.kfx = 1.0;
	s_grnd_model.kfy = 1.0;
	s_grnd_model.kfz = 1.0;
	s_grnd_model.kl  = 1.0;
	s_grnd_model.km  = 1.0;
	s_grnd_model.kn  = 1.0;

	s_grnd_model.init_takeoff_activate =0 ;
	s_grnd_model.plane_on_ground =1;
}


void v_grnd_model_run(float phi, float theta, float alt_agl)
{
	// alt_agl is integrated height, it always start with zero as ideal_plane_state and Plane state are initialised to zeros
	float sum_force_z=0;

	if ((alt_agl >= -0.0)) // 5mm, so if plane drops below 5mm mark above the ground, then we consider plane to be landed
	{
		s_grnd_model.plane_on_ground = 1;
	}
	else
	{
		s_grnd_model.plane_on_ground = 0;
	}


	// change logic for quadplane
	if (s_grnd_model.plane_on_ground == 1) 	// will defreeze integration of states if thrust is above weight
	{
		for (int i=0;i<quad_num_motors;i++)	// only add quad motors thrust, to check total_vertical > mass*g, if yes put plane_on_ground = 0
		{
			sum_force_z = sum_force_z + fabsf(rotor_force_out[i]);
		}

		if (sum_force_z >= (mass*g*cosf(phi)*cosf(theta)))
		{
			s_grnd_model.plane_on_ground = 0;
		}
	}

#ifdef Plane
	s_grnd_model.plane_on_ground = 0; // only use this line for pure plane starting from ground.
#endif
}




