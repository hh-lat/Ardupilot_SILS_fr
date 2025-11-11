#include "plant.h"
#include "Forces_and_moments_rotors.h"
#include "common_variable.h"
#include "math_util.h"
#include "math.h"


float all_rotors_force[3]  ={0.0f};
float all_rotors_moment[3] ={0.0f};
float rotor_yaw_moment_b[28][3] = {0.0f};
float rotor_yaw_moment_m[28] = {0.0f};
float rotor_force[28][3]  = {0.0f};
float rotor_moment[28][3] = {0.0f};


float rotor_xyz[28][3]    = {0.0f};

float rotor_tilt[28][3]   = {0.0f};

float rotor_r_direction[28]  = {0.0f};


void fn_thrust_rotor2body()
{

	// this is conversion of [u,v,w] body frame vector to same vector representation in NED frame( in this case from motor frame to uav frame)
	/*w*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - v*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + cos_psi*cos_theta*u;
	 v*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - w*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + cos_theta*sin_psi*u;
	cos_phi*cos_theta*w - sin_theta*u + cos_theta*sin_phi*v ;       */


	// this is conversion of [0,0,w] body frame vector to same vector representation in NED frame(in this case from motor frame to uav frame)
	/* w*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta)
	-w*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta)
	                             cos_phi*cos_theta*w	*/
	int i = 0;

	for (i = 0; i < quad_num_motors + fwv_motors; i++)
	{
		rotor_force[i][0] =  rotor_force_out[i] * (sinf(rotor_tilt[i][0]) * sinf(rotor_tilt[i][2]) + cosf(rotor_tilt[i][0]) * cosf(rotor_tilt[i][2]) * sinf(rotor_tilt[i][1]));
		rotor_force[i][1] = -rotor_force_out[i] * (sinf(rotor_tilt[i][0]) * cosf(rotor_tilt[i][2]) - cosf(rotor_tilt[i][0]) * sinf(rotor_tilt[i][2]) * sinf(rotor_tilt[i][1]));
		rotor_force[i][2] =  rotor_force_out[i] * (cosf(rotor_tilt[i][0]) * cosf(rotor_tilt[i][1]));
	}
}


void fn_yaw_moment_motor_frame()
{
	int i=0;

	for (i = 0; i < quad_num_motors; i++)
	{
		rotor_yaw_moment_m[i] = d_by_b * rotor_r_direction[i] * fabsf(rotor_force_out[i])  ;
	}

	for (i = quad_num_motors; i < quad_num_motors + fwv_motors; i++)
	{
		rotor_yaw_moment_m[i] = d_by_b_fwv * rotor_r_direction[i] * fabsf(rotor_force_out[i])  ;
	}
}


void fn_yaw_moment_rotor2body()
{
	int i = 0;
	for (i = 0; i < quad_num_motors + fwv_motors; i++)
	{
		rotor_yaw_moment_b[i][0] =  rotor_yaw_moment_m[i] * (sinf(rotor_tilt[i][0]) * sinf(rotor_tilt[i][2]) + cosf(rotor_tilt[i][0]) * cosf(rotor_tilt[i][2])*sinf(rotor_tilt[i][1]));
		rotor_yaw_moment_b[i][1] = -rotor_yaw_moment_m[i] * (sinf(rotor_tilt[i][0]) * cosf(rotor_tilt[i][2]) - cosf(rotor_tilt[i][0]) * sinf(rotor_tilt[i][2])*sinf(rotor_tilt[i][1]));
		rotor_yaw_moment_b[i][2] =  rotor_yaw_moment_m[i] * (cosf(rotor_tilt[i][0]) * cosf(rotor_tilt[i][1]));
	}
}


void fn_moment_rotor2body()
{
	int i = 0;

	for (i = 0; i < quad_num_motors + fwv_motors; i++)
	{
		cross_product(rotor_xyz[i],rotor_force[i],rotor_moment[i]);
		rotor_moment[i][0] = rotor_moment[i][0] + rotor_yaw_moment_b[i][0] ;
		rotor_moment[i][1] = rotor_moment[i][1] + rotor_yaw_moment_b[i][1] ;
		rotor_moment[i][2] = rotor_moment[i][2] + rotor_yaw_moment_b[i][2] ;
	}
}


void fn_rotors_force_and_moments()
{
	int i = 0;

	fn_thrust_rotor2body();
	fn_yaw_moment_motor_frame();
	fn_yaw_moment_rotor2body();
	fn_moment_rotor2body();

	all_rotors_force[0] = 0.0f;
	all_rotors_force[1] = 0.0f;
	all_rotors_force[2] = 0.0f;

	all_rotors_moment[0] = 0.0f;
	all_rotors_moment[1] = 0.0f;
	all_rotors_moment[2] = 0.0f;

	for (i = 0; i < quad_num_motors + fwv_motors; i++)
	{
		all_rotors_force[0] = all_rotors_force[0] + rotor_force[i][0];
		all_rotors_force[1] = all_rotors_force[1] + rotor_force[i][1];
		all_rotors_force[2] = all_rotors_force[2] + rotor_force[i][2];

		all_rotors_moment[0] =  all_rotors_moment[0] + rotor_moment[i][0];
		all_rotors_moment[1] =  all_rotors_moment[1] + rotor_moment[i][1];
		all_rotors_moment[2] =  all_rotors_moment[2] + rotor_moment[i][2];
	}

	vehicle.all_rotors_force[0] = all_rotors_force[0];
	vehicle.all_rotors_force[1] = all_rotors_force[1];
	vehicle.all_rotors_force[2] = all_rotors_force[2];

	vehicle.all_rotors_moment[0] = all_rotors_moment[0];
	vehicle.all_rotors_moment[1] = all_rotors_moment[1];
	vehicle.all_rotors_moment[2] = all_rotors_moment[2];
}

