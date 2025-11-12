#include <math.h>
#include <stdlib.h>

#include <aero.h>
#include <math_util.h>
#include <common_variable.h>
#include "LAT_SIM_Runner.h"
#include "Forces_and_moments_ctrl_srfce.h"
#include "Actuator_dynamics.h"

#define pi 3.14159265

void v_calculate_lift_force()
{
	vehicle.all_lift_force=0.0;

	switch (vehicle.aero_model_type)
	{
		case 0:
			for(int i=0;i<num_actuator;i++)
			{
				vehicle.all_lift_force = vehicle.all_lift_force + vehicle.Q*vehicle.s*actuator[csta[i]].cL*actuator[csta[i]].angle;
			}

			//linear lift model
			vehicle.all_lift_force = vehicle.all_lift_force + vehicle.Q*vehicle.s*(vehicle.CLo + vehicle.CL_alpha*vehicle.alpha) +
					(0.5*vehicle.rho*vehicle.tas)*vehicle.CL_q*vehicle.q*c/2.0f;

			// Stall lift model
			float sigma = 0.0f;
			float alpha_stall = 14.0f/57.3f;

			alpha_stall = vehicle.alpha_stall;

			sigma = (1 + powf(2.7182,-100.0*(vehicle.alpha-alpha_stall)) + powf(2.7182,100.0*(vehicle.alpha+alpha_stall)))/
					((1 + powf(2.7182,-100.0*(vehicle.alpha-alpha_stall)))*(1 + powf(2.7182,100.0*(vehicle.alpha+alpha_stall))));
			sigma = constrain_float(sigma, 0, 1);

			vehicle.all_lift_force = vehicle.all_lift_force + vehicle.Q*vehicle.s*((1-sigma)*(vehicle.CLo + vehicle.CL_alpha*vehicle.alpha) +
					sigma*(2.0f*(float)sign(vehicle.alpha)*sinf(vehicle.alpha)*sinf(vehicle.alpha)*cosf(vehicle.alpha)));
		break;


		case 1:

			float Lambda = 0;
			float CL =0;
			Lambda = vehicle.s_blown / vehicle.s;

			float Cmu_S=0,Cmu_Sp = 0;
			Cmu_S  = vehicle.Cmu * vehicle.c / vehicle.s;
			Cmu_Sp = vehicle.Cmu * vehicle.c / vehicle.blown_s;

			float theta_rad=0; // jet flap angle
			if (actuator[FLAP].angle >= 10.0/57.3)
			{
				theta_rad = (66/57.3 - abs(actuator[FLAP].angle - 20/57.3) + actuator[FLAP].angle);
			}
			else
			{
				theta_rad = (35/57.3 + actuator[FLAP].angle);
			}

			float F=0;
			F = (vehicle.AR + 2.0/pi*Cmu_S) / (vehicle.AR + 2.0 + 0.604*sqrtf(Cmu_S) + 0.876*Cmu_S);
			float dCl_dtheta =0,dCl_dalpha=0;
			dCl_dtheta = sqrtf(4.0*pi*Cmu_Sp*(1.0 + 0.151*sqrtf(Cmu_Sp) + 0.139*Cmu_Sp));
			dCl_dalpha = 2.0*pi*(1.0 + 0.151*sqrt(Cmu_Sp) + 0.219*Cmu_Sp);

			float nu=0;
			nu = (vehicle.s * dCl_dalpha + (vehicle.s  - vehicle.s_blown) * 2.0*pi) / (vehicle.s * dCl_dalpha);

			float CL_alpha_tot=0;
			CL_alpha_tot = ((vehicle.s * dCl_dalpha + (vehicle.s  - vehicle.s_blown) * 2.0*pi) / (vehicle.s))  - (vehicle.t_by_c * Cmu_S);

			CL = vehicle.CL_0
				+ F * (1 + vehicle.t_by_c) * (Lambda * theta_rad * dCl_dtheta + nu * vehicle.alpha * dCl_dalpha)
				- vehicle.t_by_c * Cmu_S * (theta_rad + vehicle.alpha)
				+ actuator[ELEVATOR].cL*actuator[ELEVATOR].angle;

			vehicle.all_lift_force = vehicle.all_lift_force + vehicle.Q*vehicle.s*CL;
			
		break;
	}


}

void v_calculate_drag_force()
{
	vehicle.all_drag_force=0.0;

	switch (vehicle.aero_model_type)
	{
		case 0:

			for(int i=0;i<num_actuator;i++)
			{
				vehicle.all_drag_force = vehicle.all_drag_force + vehicle.Q*s*actuator[csta[i]].cD*fabsf(actuator[csta[i]].angle);
			}
				vehicle.all_drag_force = vehicle.all_drag_force + vehicle.Q*s*(vehicle.CDo + vehicle.CD_alpha*fabsf(vehicle.alpha));

		break;

		case 1:

		break;

	}

}

void v_calculate_side_force()
{
	vehicle.all_side_force = 0.0;

	switch (vehicle.aero_model_type)
	{
		case 0:
			for(int i=0;i<num_actuator;i++)
			{
				vehicle.all_side_force = vehicle.all_side_force + vehicle.Q*s*actuator[csta[i]].cY*actuator[csta[i]].angle;
			}
				vehicle.all_side_force = vehicle.all_side_force + vehicle.Q*s*(vehicle.CYo + vehicle.CY_beta*vehicle.beta) +
					(0.5*vehicle.rho*vehicle.tas)*vehicle.CY_r*vehicle.r*b/2.0f + (0.5*vehicle.rho*vehicle.tas)*vehicle.CY_p*vehicle.p*b/2.0f;
		break;

		case 1:

		break;
	}
}

void v_calculate_aero_roll_moment()
{
	vehicle.all_aero_moment[0]=0.0;

	switch (vehicle.aero_model_type)
	{
		case 0:
			for(int i=0;i<num_actuator;i++)
			{
				vehicle.all_aero_moment[0] = vehicle.all_aero_moment[0]  + vehicle.Q*s*actuator[csta[i]].cl*actuator[csta[i]].angle;
			}
			vehicle.all_aero_moment[0] = vehicle.all_aero_moment[0] + vehicle.Q*s*(vehicle.Clo + vehicle.Cl_beta*vehicle.beta) +
					(0.5*vehicle.rho*vehicle.tas)*vehicle.Cl_r*vehicle.r*b/2.0f + (0.5*vehicle.rho*vehicle.tas)*vehicle.Cl_p*vehicle.p*b/2.0f;
		break;

		case 1:
			


		break;
	}
}

void v_calculate_aero_pitch_moment()
{
	vehicle.all_aero_moment[1]=0.0;
	switch (vehicle.aero_model_type)
	{
		case 0:
			for(int i=0;i<num_actuator;i++)
			{
				vehicle.all_aero_moment[1] = vehicle.all_aero_moment[1]  + vehicle.Q*s*actuator[csta[i]].cm*actuator[csta[i]].angle;
			}
				vehicle.all_aero_moment[1] = vehicle.all_aero_moment[1] + vehicle.Q*s*(vehicle.Cmo + vehicle.Cm_alpha*vehicle.alpha) +
				(0.5*vehicle.rho*vehicle.tas)*vehicle.Cm_q*vehicle.q*c/2.0f;
			break;

			case 1:


			break;
	}
}

void v_calculate_aero_yaw_moment()
{
	vehicle.all_aero_moment[2]=0.0;
	switch (vehicle.aero_model_type)
	{
		case 0:
			for(int i=0;i<num_actuator;i++)
			{
				vehicle.all_aero_moment[2] = vehicle.all_aero_moment[2]  + vehicle.Q*s*actuator[csta[i]].cn*actuator[csta[i]].angle;
			}

			vehicle.all_aero_moment[2] = vehicle.all_aero_moment[2] + vehicle.Q*s*(vehicle.Cno + vehicle.Cn_beta*vehicle.beta) +
					(0.5*vehicle.rho*vehicle.tas)*vehicle.Cn_r*vehicle.r*b/2.0f + (0.5*vehicle.rho*vehicle.tas)*vehicle.Cn_p*vehicle.p*b/2.0f ;
		break;

		case 1:

		break;
	}
}


/*cal forces and moments*/
void v_aero_force_and_moments()
{
	float temp3X1_1[3]={0.0};

	v_calculate_drag_force();
	temp3X1_1[0] = -vehicle.all_drag_force;
	temp3X1_1[1] = 0.0;
	temp3X1_1[2] = 0.0;
	windframe_to_body(temp3X1_1, vehicle.D_b); // need to do stability frame to body conversion here, refer randal Beard pg 16

	v_calculate_lift_force();
	temp3X1_1[0] = 0.0;
	temp3X1_1[1] = 0.0;
	temp3X1_1[2] = -vehicle.all_lift_force;
	windframe_to_body(temp3X1_1, vehicle.L_b);// need to do stability frame to body conversion here, refer randal Beard pg 16

	v_calculate_side_force();

	v_calculate_aero_roll_moment();
	v_calculate_aero_pitch_moment();
	v_calculate_aero_yaw_moment();


	vehicle.all_aero_force[0] = vehicle.D_b[0] + vehicle.L_b[0]   ;
	vehicle.all_aero_force[1] = vehicle.D_b[1] + vehicle.L_b[1] + vehicle.all_side_force;
	vehicle.all_aero_force[2] = vehicle.D_b[2] + vehicle.L_b[2]   ;
}
