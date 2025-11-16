#include <math.h>
#include <stdlib.h>

#include <aero.h>
#include <math_util.h>
#include <common_variable.h>
#include "LAT_SIM_Runner.h"
#include "Forces_and_moments_ctrl_srfce.h"
#include "Actuator_dynamics.h"

#define pi 3.14159265
#define R2D 57.2957795
#define D2R 0.0174532925

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
			Cmu_Sp = vehicle.Cmu * vehicle.c / vehicle.s_blown;

			float theta_rad=0; // jet flap angle
			if (vehicle.delta_f >= 10.0/57.3)
			{
				theta_rad = (66.0/57.3 - abs(vehicle.delta_f - 20/57.3) + vehicle.delta_f);
			}
			else
			{
				theta_rad = (35.0/57.3 + vehicle.delta_f);
			}

			float F=0;
			F = (vehicle.AR + 2.0/pi*Cmu_S) / (vehicle.AR + 2.0 + 0.604*sqrtf(Cmu_S) + 0.876*Cmu_S);
			float dCl_dtheta =0,dCl_dalpha=0;
			dCl_dtheta = sqrtf(4.0*pi*Cmu_Sp*(1.0 + 0.151*sqrtf(Cmu_Sp) + 0.139*Cmu_Sp));
			dCl_dalpha = 2.0*pi*(1.0 + 0.151*sqrt(Cmu_Sp) + 0.219*Cmu_Sp);

			float nu=0;
			nu = (vehicle.s * dCl_dalpha + (vehicle.s  - vehicle.s_blown) * 2.0*pi) / (vehicle.s * dCl_dalpha);

			float CL_alpha_tot=0;
			CL_alpha_tot = (vehicle.s * dCl_dalpha + (vehicle.s  - vehicle.s_blown) * 2.0*pi) / (vehicle.s);

			CL = vehicle.CL_0
				+ F * (1.0 + vehicle.t_by_c) * (Lambda * theta_rad * dCl_dtheta + nu * vehicle.alpha * dCl_dalpha)
				- vehicle.t_by_c * Cmu_S * (theta_rad + vehicle.alpha)
				+ vehicle.CL_dele*vehicle.delta_e;

			vehicle.CL = CL;
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
				vehicle.all_drag_force = vehicle.all_drag_force + vehicle.Q*vehicle.s*actuator[csta[i]].cD*fabsf(actuator[csta[i]].angle);
			}
				vehicle.all_drag_force = vehicle.all_drag_force + vehicle.Q*vehicle.s*(vehicle.CDo + vehicle.CD_alpha*fabsf(vehicle.alpha));

		break;

		case 1:
		float Cmu_S=0;
		Cmu_S  = vehicle.Cmu * vehicle.c / vehicle.s;
			vehicle.CD = vehicle.CD_0 - 0.9*Cmu_S + (2.5*vehicle.CL*vehicle.CL / (pi * vehicle.AR * vehicle.e + 2.0*Cmu_S))
			+ vehicle.CD_delta_f * vehicle.delta_f + vehicle.CD_delta_e*vehicle.delta_e + vehicle.CD_delta_e2 * vehicle.delta_e*vehicle.delta_e;
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
					(0.5*vehicle.rho*vehicle.tas)*vehicle.CY_r*vehicle.r*vehicle.b/2.0f + (0.5*vehicle.rho*vehicle.tas)*vehicle.CY_p*vehicle.p*b/2.0f;
		break;

		case 1:
		float CY=0;
		float CY_att=0,CY_flat=0;
		CY_att = vehicle.CY0 + vehicle.CY_beta*vehicle.beta + vehicle.CY_delta_aL*vehicle.delta_aL* + vehicle.CY_delta_aR*vehicle.delta_aR +
                     vehicle.CY_delta_r*vehicle.delta_r + vehicle.CY_delta_aL_Cmu*vehicle.Cmu*vehicle.delta_aL + vehicle.CY_delta_aR_Cmu*vehicle.Cmu*vehicle.delta_aR 
					 vehicle.CY_beta_Cmu*vehicle.Cmu*vehicle.beta;

		float CY_base =0, CY_other =0;
		CY_base = vehicle.CY_beta*vehicle.beta;
		CY_other = CY_att - CY_base;

		//stall onset 
		float beta_stall = 0,beta_eff = 0;
		beta_stall = 14.999727 + 0.000001*((vehicle.delta_aL + vehicle.delta_aR)/2.0 + vehicle.delta_f) + 0.000148*vehicle.Cmu + 0.000003*vehicle.alpha ;
		beta_stall = beta_stall / 57.3f;

		beta_eff = vehicle.beta - 0.259*vehicle.delta_r;
		beta_eff = beta_eff/57.3;
		float W=0,num=0,den=0;
		num = 1 + expf(-44.296*(beta_eff - beta_stall)) + expf(44.296*(beta_eff + beta_stall));
		den = (1 + expf(-44.296*(beta_eff - beta_stall))) * (1 + expf(44.296*(beta_eff + beta_stall)));
		if (den < 0.00000001)
		{ 
			den = 0.00000001
		}

		W = num/(den);

		float CY_float=0;
		CY_flat =  2.0*sign(beta_eff)*(sin(beta_eff)*sin(beta_eff))*cos(beta_eff);

		CY = CY_base * (1 - W) + 1.3517*CY_flat * W + CY_other;
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
		float A_L_base=0,A_R_base=0,R_att=0;
		float aL=0,aR=0;
			A_L_base = -0.001441*vehicle.delta_aL  - 0.000338*(vehicle.delta_aL*vehicle.Cmu);
			A_R_base =  0.001443*vehicle.delta_aR  + 0.000348*(vehicle.delta_aR*vehicle.Cmu);
			R_att    =  vehicle.Cl_beta * vehicle.beta + vehicle.Cl_delta_r*vehicle.delta_r;

			vehicle.Cl = R_att + A_L_base + A_R_base; 
			vehicle.Cl = -vehicle.Cl;
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
			    float alpha_stall;
				float alpha_eff;
				float W=0,num=0,den=0;

				alpha_stall = 12.0 + 10.0*vehicle.Cmu;
				alpha_eff = vehicle.alpha*(180.0/pi) + 67.323356*vehicle.delta_f - 33.462394*vehicle.delta_aL - 34.046741*vehicle.delta_aR + 0.185774*vehicle.Cmu;

				alpha_stall = alpha_stall*(pi/180.0f);
				alpha_eff = alpha_eff*(pi/180.0f);
				
				num = 1.0 + expf(-8.001719*(alpha_eff - alpha_stall)) + expf(8.001719**(alpha_eff - alpha_stall));
				den = (1.0 + expf(-8.001719*(alpha_eff - alpha_stall))) * (1.0 + expf(8.001719*(alpha_eff - alpha_stall)));
				if (den < 0.00000001)
				{ 
					den = 0.00000001
				}

				W = num/(den);

				float Cm_att=0,Cm_base=0,Cm_other=0;
				Cm_base = vehicle.Cmo + vehicle.Cm_alpha*vehicle.alpha + vehicle.Cm_delta_aL*vehicle.delta_aL
						 + vehicle.Cm_delta_aR*vehicle.delta_aR;

				Cm_att = vehicle.Cmo + vehicle.Cm_alpha*vehicle.alpha + vehicle.Cm_delta_e*vehicle.delta_e
						 + vehicle.Cm_delta_f*vehicle.delta_f
						 + vehicle.Cm_Cmu*vehicle.Cmu + vehicle.Cm_alpha_Cmu*vehicle.alpha*vehicle.Cmu
						 + vehicle.Cm_beta2*vehicle.beta*vehicle.beta 
						 + vehicle.Cm_beta2_Cmu*vehicle.beta*vehicle.beta*vehicle.Cmu ;

				Cm_other = Cm_att - Cm_base;

				float Cm_flat=0;
				Cm_flat = -2.0*sign(alpha_eff)*(sin(alpha_eff)*sin(alpha_eff))*cos(alpha_eff);

				Vehicle.Cm = Cm_base * (1.0 - W) + (-0.052035)*Cm_flat * W + Cm_other;

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
		    float Cn_att=0, Cn_base=0, Cn_other=0;
			float beta_stall =0, beta_eff=0;
			Cn_att = vehicle.Cno + vehicle.Cn_beta*vehicle.beta + vehicle.Cn_delta_aL*vehicle.delta_aL + vehicle.Cn_delta_aR*vehicle.delta_aR +
					 vehicle.Cn_delta_r*vehicle.delta_r + vehicle.Cn_delta_aL_Cmu*vehicle.Cmu*vehicle.delta_aL + vehicle.Cn_delta_aR_Cmu*vehicle.Cmu*vehicle.delta_aR

			Cn_base = vehicle.Cn_beta*vehicle.beta;
			Cn_other = Cn_att - Cn_base;

			beta_stall = 14.999958 + 0.000001*(vehicle.delta_r) - 0.000001*((vehicle.delta_aL + vehicle.delta_aR)/2.0 + vehicle.delta_f ) + 0.000132*vehicle.Cmu; 
			beta_stall = beta_stall / 57.3f;			

			beta_eff = vehicle.beta - 0.050785*vehicle.delta_r;
			beta_eff = beta_eff/57.3;
			float W=0,num=0,den=0;
			num = 1 + expf(-8.001719*(beta_eff - beta_stall)) + expf(8.001719*(beta_eff + beta_stall));
			den = (1 + expf(-8.001719*(beta_eff - beta_stall))) * (1 + expf(8.001719*(beta_eff + beta_stall)));
			if (den < 0.00000001)
			{ 
				den = 0.00000001
			}
			W = num/(den);
			float Cn_flat=0;
			Cn_flat = -2.0*sign(beta_eff)*(sin(beta_eff)*sin(beta_eff))*cos(beta_eff);	

			vehicle.Cn = Cn_base * (1 - W) + (-0.096281)*Cn_flat * W + Cn_other;	

			vehicle.Cn = -vehicle.Cn;
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
