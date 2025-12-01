#include <math.h>
#include <stdlib.h>

#include "LAT_SIM_math_util.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_Conversions_Frame_rotations.h"


void v_calculate_lift_force()
{
	vehcle.all_lift_force=0.0;

	if (vehcle.tas < vehcle.aero_zero_speed)
	{
		vehcle.CL = 0.0;
		vehcle.all_lift_force = 0.0;
		return;
	}

	switch (vehcle.plane_model)
	{
		case PLANE_ARDU_DEFAULT:
		{
			//linear lift model
			vehcle.CL = vehcle.CL_0 + vehcle.CL_alpha*vehcle.alpha + vehcle.CL_delta_e*vehcle.delta_e +
					vehcle.CL_q*vehcle.q*vehcle.c/(2.0f*vehcle.tas);

			vehcle.all_lift_force = vehcle.Q*vehcle.s*(vehcle.CL);

			// Stall lift model
			float sigma = 0.0f;
			float alpha_stall = 0.0f;
			float M=0;

			M = vehcle.lift_stall_M;

			alpha_stall = vehcle.alpha_stall;

			float alpha =0;
			alpha = vehcle.alpha;

			float max_alpha_delta = 0.8f;
			if (alpha-alpha_stall > max_alpha_delta) {
				alpha = alpha_stall + max_alpha_delta;
			} else if (alpha_stall-alpha > max_alpha_delta) {
				alpha = alpha_stall - max_alpha_delta;
			}

			sigma = (1.0 + powf(2.7182,-M*(alpha-alpha_stall)) + powf(2.7182,M*(alpha+alpha_stall)))/
					((1.0 + powf(2.7182,-M*(alpha-alpha_stall)))*(1.0 + powf(2.7182,M*(alpha+alpha_stall))));
			sigma = constrain_float1(sigma, 0, 1);

			vehcle.all_lift_force = vehcle.Q*vehcle.s*( (1-sigma)*(vehcle.CL) +
					sigma*(2.0f*sign_1(vehcle.alpha)*sinf(vehcle.alpha)*sinf(vehcle.alpha)*cosf(vehcle.alpha)));
		break;
		}


		case PLANE_EQX:
		{
			float Lambda = 0;
			float CL =0;
			Lambda = vehcle.s_blown / vehcle.s;

			float Cmu_S=0,Cmu_Sp = 0;
			Cmu_S  = vehcle.Cmu * vehcle.c / vehcle.s;
			Cmu_Sp = vehcle.Cmu * vehcle.c / vehcle.s_blown;

			float theta_rad=0; // jet flap angle
			if (vehcle.delta_f >= 10.0*D2R)
			{
				theta_rad = (66.0*D2R - fabsf(vehcle.delta_f - 20.0*D2R) + vehcle.delta_f);
			}
			else
			{
				theta_rad = (35.0*D2R + vehcle.delta_f);
			}

			float F=0;
			F = (vehcle.AR + 2.0/pi*Cmu_S) / (vehcle.AR + 2.0 + 0.604*sqrtf(Cmu_S) + 0.876*Cmu_S);
			float dCl_dtheta =0,dCl_dalpha=0;
			dCl_dtheta = sqrtf(4.0*pi*Cmu_Sp*(1.0 + 0.151*sqrtf(Cmu_Sp) + 0.139*Cmu_Sp));
			dCl_dalpha = 2.0*pi*(1.0 + 0.151*sqrt(Cmu_Sp) + 0.219*Cmu_Sp);

			float nu=0;
			nu = (vehcle.s_blown * dCl_dalpha + (vehcle.s  - vehcle.s_blown) * 2.0*pi) / (vehcle.s * dCl_dalpha);

			// used in Clp, Cnp derivation
			vehcle.CL_alpha_tot = (vehcle.s_blown * dCl_dalpha + (vehcle.s  - vehcle.s_blown) * 2.0*pi) / (vehcle.s);

			CL = vehcle.CL_0
				+ F * (1.0 + vehcle.t_by_c) * (Lambda * theta_rad * dCl_dtheta + nu * vehcle.alpha * dCl_dalpha)
				- vehcle.t_by_c * Cmu_S * (theta_rad + vehcle.alpha)
				+ vehcle.CL_delta_e*vehcle.delta_e;


			vehcle.CL = CL  + vehcle.CL_q * (vehcle.q * vehcle.c / (2.0f * vehcle.tas));
			vehcle.all_lift_force = vehcle.Q*vehcle.s*vehcle.CL;
			
		break;
		}
	}


}

void v_calculate_drag_force()
{
	vehcle.all_drag_force=0.0;

	if (vehcle.tas < vehcle.aero_zero_speed)
	{
		vehcle.CD = 0.0;
		vehcle.all_drag_force = 0.0;
		return;
	}

	switch (vehcle.plane_model)
	{
		case PLANE_ARDU_DEFAULT:
		{
			if (0)
			{
				//linear drag model
				vehcle.CD = vehcle.CD_0 + vehcle.CD_alpha*fabsf(vehcle.alpha)
					+ vehcle.CD_delta_e*fabsf(vehcle.delta_e) + vehcle.CD_delta_f*vehcle.delta_f +
					vehcle.CD_q*vehcle.q*vehcle.c/(2.0f*vehcle.tas);

				vehcle.all_drag_force = vehcle.Q*vehcle.s*(vehcle.CD);
			}
			else
			{
				vehcle.CD = vehcle.CD_0 + ((vehcle.CL*vehcle.CL)/(pi * vehcle.AR * vehcle.e)) ;
				//drag due to lift
				vehcle.all_drag_force =  vehcle.Q*vehcle.s*(vehcle.CD);
			}

		break;
		}

		case PLANE_EQX:
		{
			float Cmu_S=0;
			Cmu_S  = vehcle.Cmu * vehcle.c / vehcle.s;
				vehcle.CD = vehcle.CD_0 - 0.9*Cmu_S + (2.5*vehcle.CL*vehcle.CL / (pi * vehcle.AR * vehcle.e + 2.0*Cmu_S))
				+ vehcle.CD_delta_f * vehcle.delta_f + vehcle.CD_delta_e*vehcle.delta_e + vehcle.CD_delta_e2 * vehcle.delta_e*vehcle.delta_e;
			
			vehcle.all_drag_force = vehcle.Q*vehcle.s*vehcle.CD;
			break;
		}

	}

}

void v_calculate_side_force()
{
	vehcle.all_side_force = 0.0;

	if (vehcle.tas < vehcle.aero_zero_speed)
	{
		vehcle.CY = 0.0;
		vehcle.all_side_force = 0.0;
		return;
	}

	switch (vehcle.plane_model)
	{
		case PLANE_ARDU_DEFAULT:
		{
			vehcle.CY = vehcle.CY_0 + vehcle.CY_beta*vehcle.beta + vehcle.CY_delta_r*vehcle.delta_r + vehcle.CY_delta_a*vehcle.delta_a +
					 vehcle.CY_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
					 vehcle.CY_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);

			vehcle.all_side_force = vehcle.Q*vehcle.s*(vehcle.CY);
		break;
		}

		case PLANE_EQX:
		{
		float CY=0;
		float CY_att=0,CY_flat=0;
		CY_att = vehcle.CY_0 + vehcle.CY_beta*vehcle.beta + vehcle.CY_delta_aL*vehcle.delta_aL + vehcle.CY_delta_aR*vehcle.delta_aR +
                     vehcle.CY_delta_r*vehcle.delta_r + vehcle.CY_delta_aL_Cmu*vehcle.Cmu*vehcle.delta_aL + vehcle.CY_delta_aR_Cmu*vehcle.Cmu*vehcle.delta_aR +
					 vehcle.CY_beta_Cmu*vehcle.Cmu*vehcle.beta;

		float CY_base =0, CY_other =0;
		CY_base = vehcle.CY_beta*vehcle.beta;
		CY_other = CY_att - CY_base;

		//stall onset 
		float beta_stall = 0,beta_eff = 0;
		beta_stall = 14.999727 + 0.000001*R2D*((vehcle.delta_aL + vehcle.delta_aR)/2.0 + vehcle.delta_f) + 0.000148*vehcle.Cmu + 0.000003*R2D*vehcle.alpha ;
		beta_stall = beta_stall *D2R;

		beta_stall = constrain_float1(beta_stall, -40.0f*D2R, 40.0f*D2R);

		beta_eff = vehcle.beta - D2R*(0.259116*R2D*vehcle.delta_r);
		beta_eff = constrain_float1(beta_eff, -60.0f*D2R, 60.0f*D2R);

		float W=0,num=0,den=0;
		float exp_arg1 = -44.296724*(beta_eff - beta_stall);
		float exp_arg2 = 44.296724*(beta_eff + beta_stall);
		exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
		exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
		num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
		den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

		W = num/(den);
		W = constrain_float1(W, 0, 1);

		CY_flat =  2.0*sign_1(beta_eff)*(sin(beta_eff)*sin(beta_eff))*cos(beta_eff);

		CY = CY_base * (1.0 - W) + W*1.351782*CY_flat + CY_other;

		CY = -CY;

		vehcle.CY = CY + vehcle.CY_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
					 vehcle.CY_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);

		vehcle.all_side_force = vehcle.Q*vehcle.s*vehcle.CY;
		break;
		}
	}
}

void v_calculate_aero_roll_moment()
{
	vehcle.all_aero_moment[0]=0.0;

	if (vehcle.tas < vehcle.aero_zero_speed)
	{
		vehcle.Cl = 0.0;
		vehcle.all_aero_moment[0] = 0.0;
		return;
	}

	switch (vehcle.plane_model)
	{
		case PLANE_ARDU_DEFAULT:
		{
			vehcle.Cl = vehcle.Cl_0 + vehcle.Cl_beta*vehcle.beta + vehcle.Cl_delta_r*vehcle.delta_r + vehcle.Cl_delta_a*vehcle.delta_a +
					 vehcle.Cl_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
					 vehcle.Cl_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);
			vehcle.all_aero_moment[0] = vehcle.Q*vehcle.s*vehcle.b*(vehcle.Cl);
		break;
		}

		case PLANE_EQX:
		{
		float A_L_base=0,A_R_base=0,R_att=0;
			A_L_base = vehcle.Cl_delta_aL*vehcle.delta_aL  + vehcle.Cl_delta_aL_Cmu*(vehcle.delta_aL*vehcle.Cmu);
			A_R_base = vehcle.Cl_delta_aR*vehcle.delta_aR  + vehcle.Cl_delta_aR_Cmu*(vehcle.delta_aR*vehcle.Cmu);
			R_att    = vehcle.Cl_0 + vehcle.Cl_beta * vehcle.beta + vehcle.Cl_delta_r*vehcle.delta_r;

			vehcle.Cl = R_att + A_L_base + A_R_base; 
			vehcle.Cl = -vehcle.Cl;

			vehcle.Cl_p = -vehcle.CL_alpha_tot / 6.0f; 

			vehcle.Cl_r = (1.0/3.0)*vehcle.CL + 0.0399;
			
			vehcle.Cl = vehcle.Cl + vehcle.Cl_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
							vehcle.Cl_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);

			vehcle.all_aero_moment[0] = vehcle.Q*vehcle.s*vehcle.b*vehcle.Cl;
		break;
		}
	}
}

void v_calculate_aero_pitch_moment()
{
	vehcle.all_aero_moment[1]=0.0;

	if (vehcle.tas < vehcle.aero_zero_speed)
	{
		vehcle.Cm = 0.0;
		vehcle.all_aero_moment[1] = 0.0;
		return;
	}

	switch (vehcle.plane_model)
	{
		case PLANE_ARDU_DEFAULT:
		{
			vehcle.Cm = vehcle.Cm_0 + vehcle.Cm_alpha*vehcle.alpha + vehcle.Cm_delta_e*vehcle.delta_e +
					 vehcle.Cm_q*vehcle.q*vehcle.c/(2.0f*vehcle.tas);
			vehcle.all_aero_moment[1] = vehcle.Q*vehcle.s*vehcle.c*(vehcle.Cm);
			break;
		}

		case PLANE_EQX:
		{
			    float alpha_stall=0;
				float alpha_eff=0;
				float W=0,num=0,den=0;

				alpha_stall = 12.0 + 10.0*vehcle.Cmu;
				alpha_eff = vehcle.alpha*R2D + 67.323356*vehcle.delta_f*R2D - 33.462394*vehcle.delta_aL*R2D - 34.046741*vehcle.delta_aR*R2D + 0.185774*vehcle.Cmu;

				alpha_stall = alpha_stall*D2R;
				alpha_stall = constrain_float1(alpha_stall, 5.0f*D2R, 45.0f*D2R);

				alpha_eff = alpha_eff*D2R;
				alpha_eff = constrain_float1(alpha_eff, -60.0f*D2R, 60.0f*D2R);
				
				float exp_arg1 = -8.001719*(alpha_eff - alpha_stall);
				float exp_arg2 = 8.001719*(alpha_eff + alpha_stall);
				exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
				exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
				num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
				den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

				W = num/(den);

				W = constrain_float1(W, 0, 1);

				float Cm_att=0,Cm_base=0,Cm_other=0;
				Cm_base = vehcle.Cm_0 + vehcle.Cm_alpha*vehcle.alpha + vehcle.Cm_delta_aL*vehcle.delta_aL
						 + vehcle.Cm_delta_aR*vehcle.delta_aR;

				Cm_att = vehcle.Cm_0 + vehcle.Cm_alpha*vehcle.alpha + vehcle.Cm_delta_e*vehcle.delta_e
						 + vehcle.Cm_delta_f*vehcle.delta_f + vehcle.Cm_delta_aL*vehcle.delta_aL + vehcle.Cm_delta_aR*vehcle.delta_aR
						 + vehcle.Cm_Cmu*vehcle.Cmu + vehcle.Cm_alpha_Cmu*vehcle.alpha*vehcle.Cmu
						 + vehcle.Cm_beta2*vehcle.beta*vehcle.beta 
						 + vehcle.Cm_beta2_Cmu*vehcle.beta*vehcle.beta*vehcle.Cmu ;

				Cm_other = Cm_att - Cm_base;

				float Cm_flat=0;
				Cm_flat = 2.0*sign_1(alpha_eff)*(sin(alpha_eff)*sin(alpha_eff))*cos(alpha_eff);

				vehcle.Cm = Cm_base * (1.0 - W) + (-0.052035)*Cm_flat * W + Cm_other;
				vehcle.Cm = vehcle.Cm + vehcle.Cm_q*vehcle.q*vehcle.c/(2.0f*vehcle.tas) ;

				vehcle.all_aero_moment[1] = vehcle.Q*vehcle.s*vehcle.c*vehcle.Cm;

			break;
		}
	}
}

void v_calculate_aero_yaw_moment()
{
	vehcle.all_aero_moment[2]=0.0;

	if (vehcle.tas < vehcle.aero_zero_speed)
	{
		vehcle.Cn = 0.0;
		vehcle.all_aero_moment[2] = 0.0;
		return;
	}

	switch (vehcle.plane_model)
	{
		case PLANE_ARDU_DEFAULT:
		{
			vehcle.Cn = vehcle.Cn_0 + vehcle.Cn_beta*vehcle.beta + vehcle.Cn_delta_r*vehcle.delta_r + vehcle.Cn_delta_a*vehcle.delta_a +
					 vehcle.Cn_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
					 vehcle.Cn_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);
			vehcle.all_aero_moment[2] = vehcle.Q*vehcle.s*vehcle.b*(vehcle.Cn);
			break;
		}

		case PLANE_EQX:
		{
		    float Cn_att=0, Cn_base=0, Cn_other=0;
			float beta_stall =0, beta_eff=0;
			Cn_att = vehcle.Cn_0 + vehcle.Cn_beta*vehcle.beta + vehcle.Cn_delta_aL*vehcle.delta_aL + vehcle.Cn_delta_aR*vehcle.delta_aR +
					 vehcle.Cn_delta_r*vehcle.delta_r + vehcle.Cn_delta_aL_Cmu*vehcle.Cmu*vehcle.delta_aL + vehcle.Cn_delta_aR_Cmu*vehcle.Cmu*vehcle.delta_aR
                     + vehcle.Cn_beta_Cmu*vehcle.Cmu*vehcle.beta;

			Cn_base = vehcle.Cn_beta*vehcle.beta;
			Cn_other = Cn_att - Cn_base;

			beta_stall = 14.999958 + 0.000001*R2D*(vehcle.delta_r) - 0.000001*R2D*((vehcle.delta_aL + vehcle.delta_aR)/2.0 + vehcle.delta_f ) + 0.000132*vehcle.Cmu; 
			beta_stall = beta_stall*D2R;		
			beta_stall = constrain_float1(beta_stall, -40.0f*D2R, 40.0f*D2R);

			beta_eff = vehcle.beta - D2R*(0.050785*R2D*vehcle.delta_r);
			beta_eff = constrain_float1(beta_eff, -60.0f*D2R, 60.0f*D2R);

			float W=0,num=0,den=0;
			float exp_arg1 = -8.001719*(beta_eff - beta_stall);
			float exp_arg2 = 8.001719*(beta_eff + beta_stall);
			exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
			exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
			num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
			den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

			W = num/(den);

			W = constrain_float1(W, 0, 1);

			float Cn_flat=0;
			Cn_flat = 2.0*sign_1(beta_eff)*(sin(beta_eff)*sin(beta_eff))*cos(beta_eff);	

			vehcle.Cn = Cn_base * (1.0 - W) + (-0.096281)*Cn_flat * W + Cn_other;	
			vehcle.Cn = -vehcle.Cn;

			vehcle.Cn_p = (-1.0/6.0)*(1.0-0.5*vehcle.CL_alpha_tot/(pi*0.7*vehcle.AR))*vehcle.CL;
			vehcle.Cn_r = (-1.0/3.0)*vehcle.CD - 0.7979;
			
			vehcle.Cn = vehcle.Cn + vehcle.Cn_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
							vehcle.Cn_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);
							
			vehcle.all_aero_moment[2] = vehcle.Q*vehcle.s*vehcle.b*vehcle.Cn;
		break;
		}
	}
}


/*cal forces and moments*/
void v_aero_force_and_moments()
{
	float temp3X1_1[3]={0.0};

	if (vehcle.tas>11.1)
	{
		vehcle.aero_zero_speed = 5.0;
	}

	v_calculate_lift_force();
	temp3X1_1[0] = 0.0;
	temp3X1_1[1] = 0.0;
	temp3X1_1[2] = -vehcle.all_lift_force;
	windframe_to_body(temp3X1_1, vehcle.L_b);// need to do stability frame to body conversion here, refer randal Beard pg 16

	v_calculate_drag_force();
	temp3X1_1[0] = -vehcle.all_drag_force;
	temp3X1_1[1] = 0.0;
	temp3X1_1[2] = 0.0;

	windframe_to_body(temp3X1_1, vehcle.D_b); // need to do stability frame to body conversion here, refer randal Beard pg 16

	v_calculate_side_force();

	v_calculate_aero_roll_moment();
	v_calculate_aero_pitch_moment();
	v_calculate_aero_yaw_moment();
}

