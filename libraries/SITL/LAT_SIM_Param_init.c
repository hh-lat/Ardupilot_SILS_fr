
#include "LAT_SIM_Runner.h"

void fn_uav_states_init(int SITL_mode,float* latitude_point,float* longitude_point,float* Alt,float *V_bd_ins ,float* V_ned_ins,float* ax_bd_ins,float* Body_rate_bf_ins,float* attitude)
{

	switch(SITL_mode)
	{
	case 0:

#ifdef XPLANE_IN_LOOP
		/*Delhi Airport loacation (using for Xplane visualization)*/
		s_home_state.lat 	 = 28.465137902821322;
		s_home_state.longt 	 = 77.23188084021669;
		s_home_state.alt_msl = -236.72;
#else
		/*VTOL location*/
		s_home_state.lat 	 = 18.962030586366f;
		s_home_state.longt 	 = 73.1649582087994f;
		s_home_state.alt_msl = -15.0f;

		/*Tembhare location*/
//		s_home_state.lat 	 = 19.0348086391794f;
//		s_home_state.longt 	 = 73.4996514581144f;
//		s_home_state.alt_msl = -166.0f;

#ifdef Plane
		s_home_state.alt_msl = -200.0f;		//start with a initial altitude of 200m(alt > VTOL & Tembhare), if using plane configuration.
#endif

#endif

		s_home_state.alt_agl = 0.0;		// up is negative

		*latitude_point  = s_home_state.lat;
		*longitude_point = s_home_state.longt;
		*Alt =  s_home_state.alt_msl;	// -ve is up

#ifdef Plane
		*V_bd_ins  = 18.0;            		*(V_bd_ins + 1)  = 0.0;              	*(V_bd_ins + 2)  = 0.0;
		*V_ned_ins = 18.0;                	*(V_ned_ins + 1) = 0.0;                 *(V_ned_ins + 2) = 0.0;
#else
		*V_bd_ins  = 0.0;            		*(V_bd_ins + 1)  = 0.0;              	*(V_bd_ins + 2)  = 0.0;
		*V_ned_ins = 0.0;                	*(V_ned_ins + 1) = 0.0;                 *(V_ned_ins + 2) = 0.0;
#endif

		*ax_bd_ins = 0;                     *(ax_bd_ins+1) = 0;                     *(ax_bd_ins+2) = 0;
		*Body_rate_bf_ins = 0.0/57.3;       *(Body_rate_bf_ins+1) = 0.0/57.3;       *(Body_rate_bf_ins+2) = 0.0/57.3;
		*attitude = 0.0/57.3;               *(attitude+1)  = 0.0/57.3;            	*(attitude+2)  = 0.0/57.3;	// VTOL: 0 (North), Tembhare: 180 (South)

		break;

	case 1:
		//	v_tas = xp_recv_data[i][3]*0.514f;
		//	v_in = xp_recv_data[i][1]*0.514f;
		//	v_true_gnd = xp_recv_data[i][4]*0.514f;
		//	v_in_eq = xp_recv_data[i][2]*0.514f;

		*Body_rate_bf_ins 		= xp_states.p; //p
		*(Body_rate_bf_ins+1) 	= xp_states.q; //q
		*(Body_rate_bf_ins+2) 	= xp_states.r; //r


		*attitude      =	xp_states.phi; //phi
		*(attitude+1)  =	xp_states.theta; //theta
		*(attitude+2)  =	xp_states.psi; //psi
		//*(attitude+2)  =	xp_states.mag_psi;//mag_psi

		//aoa = xp_recv_data[i][1]/57.2958f;
		//ssa = xp_recv_data[i][2]/57.2958f;

		s_home_state.lat = xp_states.lat;
		s_home_state.longt = xp_states.longt;
		s_home_state.alt_msl =xp_states.alt_msl;
		s_home_state.alt_agl =xp_states.alt_agl;

		*latitude_point  = s_home_state.lat; //latitude
		*longitude_point = s_home_state.longt;  //longitude
		*Alt =s_home_state.alt_msl; //alt_msl
		// *Alt = s_home_state.alt_agl; //alt_agl


		//xp_recv_data[i][1]; //pos_ned 0
		//xp_recv_data[i][2]; //pos_ned 1
		//xp_recv_data[i][3]; //pos_ned 2

		*V_ned_ins         = xp_states.v_ned[0]; //v_ned 0
		*(V_ned_ins + 1)   = xp_states.v_ned[1]; //v_ned 1
		*(V_ned_ins + 2)   = xp_states.v_ned[2]; //v_ned 2

		break;
	}
}



