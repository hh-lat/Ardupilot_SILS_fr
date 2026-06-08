#include <math.h>
#include <stdlib.h>

#include "LAT_SIM_math_util.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_Conversions_Frame_rotations.h"


// uSTOL v1: wing aerodynamic-center x/c vs flap config (18 or 32) and blowing coeff
static float ustol_aero_center(float flap_config, float Cmyu)
{
	const float Cmyu_max = 9.21f;
	// linearly interpolate the config-18 / config-32 fits (flap_config passes through both)
	float frac  = (flap_config - 18.0f) / (32.0f - 18.0f);
	float c0    =  0.2668f + frac*( 0.4043f - 0.2668f);
	float bb    = -0.0833f + frac*(-0.2612f - (-0.0833f));
	float c_lin =  0.01399f + frac*(0.02112f - 0.01399f);
	float Cmyu_clip = constrain_float1(Cmyu, 0.0f, Cmyu_max);
	return c0 + bb*sqrtf(Cmyu_clip) + c_lin*Cmyu_clip;
}

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

		case PLANE_EQX_V1_NEW_MODEL:
		{
			// Stall lift model
			float alpha_stall = 0.0f;

			float Lambda = 0;
			float CL =0;
			Lambda = vehcle.s_blown / vehcle.s;

			float Cmu_S=0,Cmu_Sp = 0;
			Cmu_S  = vehcle.Cmu * vehcle.c / vehcle.s;
			Cmu_Sp = vehcle.Cmu * vehcle.c / vehcle.s_blown;

			float theta_rad=0; // jet flap angle
			if (vehcle.delta_f <= 25.0*D2R)
			{
				theta_rad = 46.0*D2R - fabsf(vehcle.delta_f - 20.0*D2R);
			}
			else
			{
				theta_rad = 24.0*D2R;
			}

			float F=0;
			F = (vehcle.AR + 2.0/pi*Cmu_S) / (vehcle.AR + 2.0 + 0.604*sqrtf(Cmu_S) + 0.876*Cmu_S);
			float dCl_dtheta =0,dCl_dalpha=0;
			dCl_dtheta = sqrtf(4.0*pi*Cmu_Sp*(1.0 + 0.151*sqrtf(Cmu_Sp) + 0.139*Cmu_Sp));
			dCl_dalpha = vehcle.CL_alpha*(1.0 + 0.151*sqrt(Cmu_Sp) + 0.219*Cmu_Sp)/1.15;

			float nu=0;
			nu = (vehcle.s_blown * dCl_dalpha + (vehcle.s  - vehcle.s_blown) * vehcle.CL_alpha) / (vehcle.s * dCl_dalpha);

			CL = vehcle.CL_0
				+ F * (1.0 + vehcle.t_by_c) * (Lambda * theta_rad * dCl_dtheta + nu * vehcle.alpha * dCl_dalpha)
				- vehcle.t_by_c * Cmu_S * (theta_rad + vehcle.alpha)
				+ vehcle.CL_delta_e*vehcle.delta_e;

			CL = CL + 1.2*vehcle.Cmu*vehcle.beta*vehcle.beta;

			// used in Clp, Cnp derivation
			vehcle.CL_alpha_tot = F*(1.0 + vehcle.t_by_c)*nu*dCl_dalpha - vehcle.t_by_c*Cmu_S;

			alpha_stall = vehcle.alpha_stall;
			alpha_stall = constrain_float1(alpha_stall, 5.0f*D2R, 45.0f*D2R);

			float exp_arg1 = -30.0*(vehcle.alpha - alpha_stall);
			float exp_arg2 = 30.0*(vehcle.alpha + alpha_stall);
			exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
			exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
			float num,den,W=0;
			num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
			den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

			W = num/(den);

			W = constrain_float1(W, 0, 1);

			float CL_fp=0;

			CL_fp = 2.0f*sign_1(vehcle.alpha)*sinf(vehcle.alpha)*sinf(vehcle.alpha)*cosf(vehcle.alpha);

			CL = (1.0f - W)*CL + W*CL_fp; 

			CL = CL + vehcle.CL_q*(vehcle.q * vehcle.c / (2.0f * vehcle.tas));

			vehcle.CL = CL;
			vehcle.all_lift_force = vehcle.Q*vehcle.s*vehcle.CL;
			break;
		}

		case PLANE_USTOL_V1:
		{
			float Cmu_star = vehcle.Cmu;     
			float alpha    = vehcle.alpha;
			float delta_f  = vehcle.delta_f;
			float delta_e  = vehcle.delta_e;
			float V        = vehcle.tas;       

			float flap_config = 18.0f + 0.7f*(delta_f*R2D);   // linear: df=0->18, df=20deg->32

			// Wing Conponent
			float Cmyu    = Cmu_star * vehcle.wing.lambda_b;  

			float cla     = 2.0f*pi*(1.0f + 0.151f*sqrtf(Cmu_star) + 0.219f*Cmu_star);                 
			float clt     = sqrtf(4.0f*pi*Cmu_star*(1.0f + 0.151f*sqrtf(Cmu_star) + 0.139f*Cmu_star));
			float cl_delf = 2.0f*pi*vehcle.controls.tau_f;

			float G       = (vehcle.AR + 0.637f*Cmyu) /
			                 (vehcle.AR + 2.0f + 0.604f*sqrtf(Cmyu) + 0.876f*Cmyu);

			float nu_camber = vehcle.wing.k_fit;
			float nu_alpha  = (vehcle.wing.lambda_b + (1.0f - vehcle.wing.lambda_b)*2.0f*pi/cla) * vehcle.wing.k_fit;
			float nu_tau    = vehcle.wing.lambda_b;
			float nu_delf   = vehcle.wing.S_f / vehcle.s * vehcle.controls.Kb_f * vehcle.wing.k_fit;

			float tau = (flap_config * 7.0f/9.0f + 18 *2.0f/9.0f) * D2R;

			float CL_w = G*(1.0f + vehcle.t_by_c)*( nu_camber*vehcle.wing.cl0_camber
			             + nu_tau*clt*tau
			             + nu_alpha*cla*alpha
			             + nu_delf*cl_delf*delta_f)
			           - vehcle.t_by_c*(tau + alpha)*Cmyu;

			// wing CL_alpha (analytic — CL_wing is linear in alpha), used by CLq
			float CL_alpha_w = G*(1.0f + vehcle.t_by_c)*nu_alpha*cla - vehcle.t_by_c*Cmyu;

			// Tail Component
			float eta_t = vehcle.tail.eta_t_0 - vehcle.tail.eta_t_1*Cmu_star;
			float deps  = (alpha < 0.0f) ? vehcle.tail.deps_neg : vehcle.tail.deps_pos;
			float CL_t  = eta_t * vehcle.tail.S_ht_S * vehcle.tail.a_t *
			              (-vehcle.tail.eps0 + (1.0f - deps)*alpha + vehcle.controls.tau_e*delta_e);

			// Fuselage Component
			float CL_f = (0.030384f + 0.004932f*sqrtf(Cmu_star))
			           + (0.196106f + 0.073289f*sqrtf(Cmu_star) - 0.016049f*Cmu_star)*alpha;

			// Dynamic Coefficient component
			float x_ac_w = ustol_aero_center(flap_config, Cmu_star);
			float CLq = 2.0f*CL_alpha_w*(x_ac_w - vehcle.cg.x_cg_c)
			          + 2.0f*eta_t*vehcle.tail.a_t*vehcle.tail.V_H;

			// ---- Post-stall blend (wing & rest -> flat-plate surrogates; Beard) ----
			float Cmu_b = 0.52f*Cmu_star;
			float f20   = (flap_config - 18.0f) / 14.0f;   // linear flap fraction (0 at 18, 1 at 32)

			float Mw  = vehcle.cl_stall.wM0 + vehcle.cl_stall.wM1*Cmu_b;
			if (Mw < 0.5f) Mw = 0.5f;
			float a0w = (vehcle.cl_stall.wa00 + vehcle.cl_stall.wa0mu*Cmu_b + vehcle.cl_stall.wa0f*f20)*pi/180.0f;
			float ew1 = constrain_float1(-Mw*(alpha - a0w), -88.0f, 88.0f);
			float ew2 = constrain_float1( Mw*(alpha + a0w), -88.0f, 88.0f);
			float Ww  = (1.0f + expf(ew1) + expf(ew2)) / ((1.0f + expf(ew1))*(1.0f + expf(ew2)));
			Ww = constrain_float1(Ww, 0.0f, 1.0f);

			float a0r = (vehcle.cl_stall.ra0 + vehcle.cl_stall.rkcu*Cmu_b)*pi/180.0f;
			float er1 = constrain_float1(-vehcle.cl_stall.rM*(alpha - a0r), -88.0f, 88.0f);
			float er2 = constrain_float1( vehcle.cl_stall.rM*(alpha + a0r), -88.0f, 88.0f);
			float Wr  = (1.0f + expf(er1) + expf(er2)) / ((1.0f + expf(er1))*(1.0f + expf(er2)));
			Wr = constrain_float1(Wr, 0.0f, 1.0f);

			float CL_wing_post = (1.0f - Ww)*CL_w + Ww*vehcle.cl_stall.wkflat*sinf(2.0f*alpha);
			float CL_rest_post = (1.0f - Wr)*(CL_t + CL_f)
			                   + Wr*vehcle.cl_stall.rkflat*(2.0f*sign_1(alpha)*sinf(alpha)*sinf(alpha)*cosf(alpha));

			float CL = CL_wing_post + CL_rest_post + CLq*(vehcle.q * vehcle.c / (2.0f*V));

			vehcle.CL_w = CL_w;  
			vehcle.CL_t = CL_t;
			vehcle.CL = CL;
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

		case PLANE_EQX_V1_NEW_MODEL:
		{
			float Cmu_S=0;
			float CD=0;
			Cmu_S  = vehcle.Cmu * vehcle.c / vehcle.s;

			CD = vehcle.CD_0 - 0.003*Cmu_S + (2.8*vehcle.CL*vehcle.CL / (pi * vehcle.AR * vehcle.e + 2.0*Cmu_S))
				+ vehcle.CD_delta_f * vehcle.delta_f + vehcle.CD_delta_e*vehcle.delta_e + vehcle.CD_delta_e2 * vehcle.delta_e*vehcle.delta_e;

			float alpha_stall=0;
			alpha_stall = vehcle.alpha_stall;
			alpha_stall = constrain_float1(alpha_stall, 5.0f*D2R, 45.0f*D2R);

			float exp_arg1 = -30.0*(vehcle.alpha - alpha_stall);
			float exp_arg2 = 30.0*(vehcle.alpha + alpha_stall);
			exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
			exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
			float num,den,W=0;
			num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
			den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

			W = num/(den);

			W = constrain_float1(W, 0, 1);

			float CD_fp=0;

			CD_fp = 2.0f*(sinf(vehcle.alpha)*sinf(vehcle.alpha)*sinf(vehcle.alpha));

			CD = ((1.0f - W)*CD + W*CD_fp);

			vehcle.CD = CD;

			vehcle.all_drag_force = vehcle.Q*vehcle.s*vehcle.CD;
			break;
		}

		case PLANE_USTOL_V1:
		{
			// CD = (1-W)*CD_baseline + W*CD_flat + CD_rest + CD_rate.
			// All drag terms use the 0.52-rescaled Cmu; CL_w/CL_t come from the lift calc.
			float alpha = vehcle.alpha;
			float beta  = vehcle.beta;
			float Cmu   = 0.52f*vehcle.Cmu;
			float CL_w  = vehcle.CL_w;
			float CL_t  = vehcle.CL_t;
			float de = vehcle.delta_e, df = vehcle.delta_f, da = vehcle.delta_a, dr = vehcle.delta_r;

			// attached baseline polar
			float CD_baseline = vehcle.fuse.CD0
			                  + vehcle.wing.k_w*CL_w*CL_w / (pi*vehcle.AR + 2.0f*Cmu)
			                  + vehcle.fuse.CD_a2*alpha*alpha
			                  + vehcle.fuse.CD_a2_Cmyu*alpha*alpha*Cmu;

			// fully-stalled flat-plate surrogate
			float CD_flat = vehcle.stall.K_flat*(2.0f*sinf(alpha)*sinf(alpha));

			// Beard stall-blend weight
			float a0 = (vehcle.stall.a0_const + vehcle.stall.a0_Cmyu*Cmu)*pi/180.0f;
			float e1 = constrain_float1(-vehcle.stall.k*(alpha - a0), -88.0f, 88.0f);
			float e2 = constrain_float1( vehcle.stall.k*(alpha + a0), -88.0f, 88.0f);
			float W = (1.0f + expf(e1) + expf(e2)) / ((1.0f + expf(e1))*(1.0f + expf(e2)));
			W = constrain_float1(W, 0.0f, 1.0f);

			// always-present terms (sideslip, blowing, controls, tail induced)
			float CD_rest = vehcle.fuse.CD_b2*beta*beta
			              + vehcle.wing.r*Cmu
			              + vehcle.controls.CD_df2*df*df
			              + vehcle.controls.CD_de2*de*de + vehcle.controls.CD_de*de
			              + vehcle.controls.CD_da2*da*da + vehcle.controls.CD_da*da
			              + vehcle.controls.CD_dr2*dr*dr
			              + vehcle.tail.CD_ht0
			              + vehcle.tail.k_ht*CL_t*CL_t / (pi*vehcle.tail.AR_ht);

			// rate (blowing cross) terms — rates in rad/s
			float CD_rate = vehcle.fuse.CDp_cu*Cmu*vehcle.p
			              + vehcle.fuse.CDq_cu*Cmu*vehcle.q
			              + vehcle.fuse.CDr_cu*Cmu*vehcle.r;

			vehcle.CD = (1.0f - W)*CD_baseline + W*CD_flat + CD_rest + CD_rate;
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

		case PLANE_EQX_V1_NEW_MODEL:
		{
			float CY=0;
			float CY_att=0,CY_flat=0;
			CY_att = vehcle.CY_0 + vehcle.CY_beta*vehcle.beta + vehcle.CY_delta_aL*vehcle.delta_aL + vehcle.CY_delta_aR*vehcle.delta_aR +
						vehcle.CY_delta_r*vehcle.delta_r  + vehcle.CY_beta_Cmu*vehcle.Cmu*vehcle.beta;

			float CY_base =0, CY_other =0;
			CY_base = vehcle.CY_beta*vehcle.beta;
			CY_other = CY_att - CY_base;

			//stall onset 
			float beta_stall = 0,beta_eff = 0;
			beta_stall = 17.0 + 5.0*vehcle.Cmu ;
			beta_stall = beta_stall *D2R;

			beta_stall = constrain_float1(beta_stall, -40.0f*D2R, 40.0f*D2R);

			beta_eff = vehcle.beta - (0.588463*vehcle.delta_r);
			beta_eff = constrain_float1(beta_eff, -60.0f*D2R, 60.0f*D2R);

			float W=0,num=0,den=0;
			float exp_arg1 = -37.27*(beta_eff - beta_stall);
			float exp_arg2 =  37.27*(beta_eff + beta_stall);
			exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
			exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
			num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
			den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

			W = num/(den);
			W = constrain_float1(W, 0, 1);

			CY_flat =  2.0*sign_1(beta_eff)*(sin(beta_eff)*sin(beta_eff))*cos(beta_eff);

			CY = CY_base * (1.0 - W) + W*(-1.1277)*CY_flat + CY_other;

			vehcle.CY = CY + vehcle.CY_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
						vehcle.CY_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);

			vehcle.all_side_force = vehcle.Q*vehcle.s*vehcle.CY;
			break;
		}

		case PLANE_USTOL_V1:
		{
			// CY: attached linear + Beard post-stall blend + rate terms. Fits in DEGREES.
			float Cmu = 0.52f*vehcle.Cmu;
			float beta_d = vehcle.beta*R2D;
			float daL_d  = vehcle.delta_aL*R2D;
			float daR_d  = vehcle.delta_aR*R2D;
			float dr_d   = vehcle.delta_r*R2D;
			float df_d   = vehcle.delta_f*R2D;
			float alpha_d= vehcle.alpha*R2D;

			float CY_att = vehcle.lateral.theta0 + vehcle.lateral.theta_b*beta_d
			             + vehcle.lateral.theta_aL*daL_d + vehcle.lateral.theta_aR*daR_d
			             + vehcle.lateral.theta_r*dr_d + vehcle.lateral.theta_bcu*(beta_d*Cmu);
			float CY_base  = vehcle.lateral.theta_b*beta_d;
			float CY_other = CY_att - CY_base;

			// stall onset [deg] & post-stall effective sideslip [rad]
			float beta_stall = vehcle.lateral.beta0
			                 + vehcle.lateral.kr*dr_d
			                 + vehcle.lateral.kaf*(0.5f*(daL_d + daR_d) + df_d)
			                 + vehcle.lateral.kcu*Cmu
			                 + vehcle.lateral.kalpha*alpha_d;
			float beta_eff = (beta_d + vehcle.lateral.kps_r*dr_d)*pi/180.0f;

			float b0 = beta_stall*pi/180.0f;
			float e1 = constrain_float1(-vehcle.lateral.M*(beta_eff - b0), -88.0f, 88.0f);
			float e2 = constrain_float1( vehcle.lateral.M*(beta_eff + b0), -88.0f, 88.0f);
			float W = (1.0f + expf(e1) + expf(e2)) / ((1.0f + expf(e1))*(1.0f + expf(e2)));
			W = constrain_float1(W, 0.0f, 1.0f);

			float CY_flat = 2.0f*sign_1(beta_eff)*sinf(beta_eff)*sinf(beta_eff)*cosf(beta_eff);

			// rate terms: p_hat = p*b/2V, r_hat = r*c/2V  (CY uses chord for r_hat)
			float V = vehcle.tas;
			float p_hat = vehcle.p*vehcle.b/(2.0f*V);
			float r_hat = vehcle.r*vehcle.c/(2.0f*V);
			float CY_rate = vehcle.lateral.CYp*p_hat + vehcle.lateral.CYr*r_hat
			              + vehcle.lateral.CYp_cu*(p_hat*Cmu) + vehcle.lateral.CYr_cu*(r_hat*Cmu)
			              + vehcle.lateral.CYp2_cu*(p_hat*p_hat*Cmu) + vehcle.lateral.CYr2_cu*(r_hat*r_hat*Cmu);

			vehcle.CY = CY_base*(1.0f - W) + W*(vehcle.lateral.kv*CY_flat) + CY_other + CY_rate;
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

		case PLANE_EQX_V1_NEW_MODEL:
		{
			float A_L_base=0,A_R_base=0,R_att=0;
			A_L_base = vehcle.Cl_delta_aL*vehcle.delta_aL  + vehcle.Cl_delta_aL_Cmu*(vehcle.delta_aL*vehcle.Cmu);
			A_R_base = vehcle.Cl_delta_aR*vehcle.delta_aR  + vehcle.Cl_delta_aR_Cmu*(vehcle.delta_aR*vehcle.Cmu);
			R_att    = vehcle.Cl_0 + vehcle.Cl_beta * vehcle.beta + vehcle.Cl_delta_r*vehcle.delta_r;

			vehcle.Cl = R_att + A_L_base + A_R_base; 

			vehcle.Cl_p = -vehcle.CL/ 6.0f; 
			
			vehcle.Cl = vehcle.Cl + vehcle.Cl_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
							vehcle.Cl_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);

			vehcle.all_aero_moment[0] = vehcle.Q*vehcle.s*vehcle.b*vehcle.Cl;
			break;
		}

		case PLANE_USTOL_V1:
		{
			// Cml: attached (no stall blend) + rate terms. Fits in DEGREES.
			float Cmu = 0.52f*vehcle.Cmu;
			float beta_d = vehcle.beta*R2D;
			float daL_d  = vehcle.delta_aL*R2D;
			float daR_d  = vehcle.delta_aR*R2D;
			float dr_d   = vehcle.delta_r*R2D;

			float A_L = vehcle.roll.theta_aL*daL_d + vehcle.roll.theta_aLcu*(daL_d*Cmu);
			float A_R = vehcle.roll.theta_aR*daR_d + vehcle.roll.theta_aRcu*(daR_d*Cmu);
			float R_att = vehcle.roll.theta0 + vehcle.roll.theta_b*beta_d
			            + vehcle.roll.theta_r*dr_d + vehcle.roll.theta_b_cu*(beta_d*Cmu);

			// rate terms: p_hat = p*b/2V, r_hat = r*b/2V
			float V = vehcle.tas;
			float p_hat = vehcle.p*vehcle.b/(2.0f*V);
			float r_hat = vehcle.r*vehcle.b/(2.0f*V);
			float Cml_rate = vehcle.roll.Clp*p_hat + vehcle.roll.Clr*r_hat
			               + vehcle.roll.Clp_cu*(p_hat*Cmu) + vehcle.roll.Clr_cu*(r_hat*Cmu);

			vehcle.Cl = A_L + A_R + R_att + Cml_rate;
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

		case PLANE_EQX_V1_NEW_MODEL:
		{
			float alpha_stall=0;
			float alpha_eff=0;
			float W=0,num=0,den=0;


			alpha_stall = 23.999772 + (-0.239994)*R2D*vehcle.delta_e + (-0.10001)*R2D*vehcle.delta_f + (-0.10001)*vehcle.delta_aL*R2D  +  (-0.10001)*vehcle.delta_aR*R2D + (0.000730)*vehcle.Cmu ;
			alpha_eff = vehcle.alpha*R2D + 0.188339*vehcle.delta_e*R2D + 0.028563*vehcle.delta_f*R2D - 0.291419*vehcle.delta_aL*R2D - 0.658*vehcle.delta_aR*R2D;

			alpha_stall = alpha_stall*D2R;
			alpha_stall = constrain_float1(alpha_stall, 5.0f*D2R, 45.0f*D2R);

			alpha_eff = alpha_eff*D2R;
			alpha_eff = constrain_float1(alpha_eff, -60.0f*D2R, 60.0f*D2R);
			
			float exp_arg1 = -25.0*(alpha_eff - alpha_stall);
			float exp_arg2 =  25.0*(alpha_eff + alpha_stall);
			exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
			exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
			num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
			den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

			W = num/(den);

			W = constrain_float1(W, 0, 1);

			float Cm_att=0,Cm_base=0,Cm_other=0;
			Cm_base = vehcle.Cm_0 + vehcle.Cm_alpha*vehcle.alpha +  vehcle.Cm_Cmu*vehcle.Cmu + vehcle.Cm_alpha_Cmu*vehcle.alpha*vehcle.Cmu;

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

		case PLANE_USTOL_V1:
		{
			// Cm about design CG = Cm0_ac_wing + CL_w*|x_cg_c - x_ac_w| - CL_t*x_cg_tac_abs.
			// Uses RAW Cmu* (not 0.52-rescaled). CL_w/CL_t come from the lift calc.
			// NOTE: the classdef Cm_total has NO Cmq pitch-damping term — none added here.
			float Cmu_star = vehcle.Cmu;
			float flap_config = 18.0f + 0.7f*(vehcle.delta_f*R2D);   // linear: df=0->18, df=20deg->32

			// Cm about wing aero center — linearly interpolate config-18 / config-32 fits
			float Cmyu_clip = constrain_float1(Cmu_star, 0.0f, 9.21f);
			float frac = (flap_config - 18.0f) / 14.0f;
			float a_p = -0.0803f + frac*( 0.0170f - (-0.0803f));
			float b_p = -0.103f  + frac*(-0.2442f - (-0.103f));
			float c_p = -0.083f  + frac*(-0.0459f - (-0.083f));
			float Cm0_ac_wing = a_p + b_p*sqrtf(Cmyu_clip) + c_p*Cmyu_clip;

			float x_ac_w = ustol_aero_center(flap_config, Cmu_star);

			vehcle.Cm = Cm0_ac_wing
			          + vehcle.CL_w*fabsf(vehcle.cg.x_cg_c - x_ac_w)
			          - vehcle.CL_t*vehcle.cg.x_cg_tac_abs;
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

		case PLANE_EQX_V1_NEW_MODEL:
		{
			float Cn_att=0, Cn_base=0, Cn_other=0;
			float beta_stall =0, beta_eff=0;
			Cn_att = vehcle.Cn_0 + vehcle.Cn_beta*vehcle.beta + vehcle.Cn_delta_aL*vehcle.delta_aL + vehcle.Cn_delta_aR*vehcle.delta_aR +
					 vehcle.Cn_delta_r*vehcle.delta_r + vehcle.Cn_delta_aL_Cmu*vehcle.Cmu*vehcle.delta_aL + vehcle.Cn_delta_aR_Cmu*vehcle.Cmu*vehcle.delta_aR
                     + vehcle.Cn_beta_Cmu*vehcle.Cmu*vehcle.beta;

			Cn_base = vehcle.Cn_beta*vehcle.beta;
			Cn_other = Cn_att - Cn_base;

			beta_stall = 15.0*D2R;	
			beta_stall = constrain_float1(beta_stall, -40.0f*D2R, 40.0f*D2R);

			beta_eff = vehcle.beta - (0.499573*vehcle.delta_r);
			beta_eff = constrain_float1(beta_eff, -60.0f*D2R, 60.0f*D2R);

			float W=0,num=0,den=0;
			float exp_arg1 = -29.896104*(beta_eff - beta_stall);
			float exp_arg2 =  29.896104*(beta_eff + beta_stall);
			exp_arg1 = constrain_float1(exp_arg1, -88.0f, 88.0f);
			exp_arg2 = constrain_float1(exp_arg2, -88.0f, 88.0f);
			num = 1.0 + expf(exp_arg1) + expf(exp_arg2);
			den = (1.0 + expf(exp_arg1)) * (1.0 + expf(exp_arg2));

			W = num/(den);

			W = constrain_float1(W, 0, 1);

			float Cn_flat=0;
			Cn_flat = 2.0*sign_1(beta_eff)*(sin(beta_eff)*sin(beta_eff))*cos(beta_eff);	

			vehcle.Cn = Cn_base * (1.0 - W) + (0.123950)*Cn_flat * W + Cn_other;	
			
			vehcle.Cn = vehcle.Cn + vehcle.Cn_p*vehcle.p*vehcle.b/(2.0f*vehcle.tas) +
							vehcle.Cn_r*vehcle.r*vehcle.b/(2.0f*vehcle.tas);
							
			vehcle.all_aero_moment[2] = vehcle.Q*vehcle.s*vehcle.b*vehcle.Cn;

			break;
		}

		case PLANE_USTOL_V1:
		{
			// Cn: attached linear + Beard post-stall blend + rate terms. Fits in DEGREES.
			// NOTE: yaw-rate terms use RAW p, r (per classdef), not non-dimensionalized.
			float Cmu = 0.52f*vehcle.Cmu;
			float beta_d = vehcle.beta*R2D;
			float daL_d  = vehcle.delta_aL*R2D;
			float daR_d  = vehcle.delta_aR*R2D;
			float dr_d   = vehcle.delta_r*R2D;
			float df_d   = vehcle.delta_f*R2D;
			float alpha_d= vehcle.alpha*R2D;

			float Cn_att = vehcle.yaw.theta0 + vehcle.yaw.theta_b*beta_d
			             + vehcle.yaw.theta_aL*daL_d + vehcle.yaw.theta_aR*daR_d
			             + vehcle.yaw.theta_r*dr_d + vehcle.yaw.theta_bcu*(beta_d*Cmu)
			             + vehcle.yaw.theta_aLcu*(daL_d*Cmu) + vehcle.yaw.theta_aRcu*(daR_d*Cmu);
			float Cn_base  = vehcle.yaw.theta_b*beta_d;
			float Cn_other = Cn_att - Cn_base;

			float beta_stall = vehcle.yaw.beta0 + vehcle.yaw.kr*dr_d
			                 + vehcle.yaw.kaf*(0.5f*(daL_d + daR_d) + df_d)
			                 + vehcle.yaw.kcu*Cmu + vehcle.yaw.kalpha*alpha_d;
			float beta_eff = (beta_d + vehcle.yaw.kps_r*dr_d)*pi/180.0f;

			float b0 = beta_stall*pi/180.0f;
			float e1 = constrain_float1(-vehcle.yaw.M*(beta_eff - b0), -88.0f, 88.0f);
			float e2 = constrain_float1( vehcle.yaw.M*(beta_eff + b0), -88.0f, 88.0f);
			float W = (1.0f + expf(e1) + expf(e2)) / ((1.0f + expf(e1))*(1.0f + expf(e2)));
			W = constrain_float1(W, 0.0f, 1.0f);

			float Cn_flat = 2.0f*sign_1(beta_eff)*sinf(beta_eff)*sinf(beta_eff)*cosf(beta_eff);

			float Cn_rate = (vehcle.yaw.Cnp + vehcle.yaw.Cnp_cu*Cmu)*vehcle.p
			              + (vehcle.yaw.Cnr + vehcle.yaw.Cnr_cu*Cmu)*vehcle.r;

			vehcle.Cn = Cn_base*(1.0f - W) + W*(vehcle.yaw.kv*Cn_flat) + Cn_other + Cn_rate;
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

	//test_aero_model
	// vehcle.alpha = D2R*8;
	// vehcle.beta  = D2R*3;
	// vehcle.q	 = 0.0;
	// vehcle.p	 = 0.0;
	// vehcle.r	 = 0.0;
	// vehcle.delta_e = D2R*(-3);
	// vehcle.delta_a = D2R*(0);
	// vehcle.delta_aL = D2R*(5);
	// vehcle.delta_aR = D2R*(-5);
	// vehcle.delta_r = D2R*(3);
	// vehcle.delta_f = D2R*10;
	// vehcle.Cmu    = 0.7;
	// vehcle.tas = 30;
	// vehcle.p = 0.1;
	// vehcle.q = 0.2;
	// vehcle.r = 0.15;

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

	vehcle.all_aero_force[0] = vehcle.L_b[0] + vehcle.D_b[0] ;
	vehcle.all_aero_force[1] = vehcle.L_b[1] + vehcle.D_b[1] + vehcle.all_side_force;
	vehcle.all_aero_force[2] = vehcle.L_b[2] + vehcle.D_b[2] ;
}

