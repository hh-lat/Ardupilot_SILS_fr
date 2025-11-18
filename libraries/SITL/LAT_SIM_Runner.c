
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include "LAT_SIM_rk4.h"
#include "LAT_SIM_Runner.h"


VEHICLE_STATES vehicle;

strct_home_states s_home_state;

void v_lat_fdm_init()
{
    v_rotor_geometry_define(); // defines geometry
    v_plane_param_define();
    vehicle.dof = DOF_ALL_MOTION;
    vehicle.aero_model_type = 0; // 0 for default simple model, 1 for equinox model
	s_servo_manager.num_servos = 10; // number of servo channels
}


void v_lat_fdm_run()
{
    static int init = 0;
    float t_step = 0.001;
    static float t=0;
    float plane_state[12]={0};

    v_actuator_dynamics(t_step); // converts servopwm to angles and applies rate limits
   	v_motor_pwm_to_throttle(t_step); //update throttle from pwm with rate limiting on throttle

    v_rk4(plane_state, t, t_step);
}


void v_plane_param_define()
{
    v_plane_param_define_equinox();
}

void v_plane_param_define_equinox()
{
    vehicle.mass = 65.0 ;
	vehicle.g = 9.81; 
    vehicle.Ixx = 14.658 ;
    vehicle.Iyy = 16.944 ;
    vehicle.Izz = 27.412 ;
    vehicle.Ixz = 3.985 ;
	vehicle.Iyz = 0.0 ;
	vehicle.Ixy = 0.0 ;
    vehicle.s = 0.9;
	vehicle.s_blown = 0.216;
    vehicle.b = 3.0;
    vehicle.c = 0.3;
	vehicle.e =1;
	vehicle.AR =10;
	vehicle.t_by_c = 0.15;
    vehicle.rho = 1.15;
    vehicle.mlgL_x = 0;
    vehicle.mlgL_y = 0;
    vehicle.mlgL_z = 0;
    vehicle.mlgR_x = 0;
    vehicle.mlgR_y = 0;
    vehicle.mlgR_z = 0;
    vehicle.nlg_x = 0;
    vehicle.nlg_y = 0;
    vehicle.nlg_z = 0;
	vehicle.alpha_stall = 14.0*D2R; // stall angle in rad

	vehicle.CL_0 = 0.29;
	vehicle.CL_delta_e =1.175;
	vehicle.CL_alpha = 5.0; // default
	vehicle.CL_q =0; // default

	vehicle.CD_0 = 0.2; 
	vehicle.CD_delta_e = -0.355;
	vehicle.CD_delta_f =0.1;
	vehicle.CD_delta_e2 = 0.5;
	vehicle.CD_alpha = 0.3;// default

	vehicle.CY_0 = 0;
	vehicle.CY_beta = 0.011931*R2D;
	vehicle.CY_delta_r =0.001499*R2D;
	vehicle.CY_delta_aL_Cmu =-0.000199*R2D;
	vehicle.CY_delta_aR_Cmu = 0.000196*R2D;
	vehicle.CY_delta_aL = 0.000391*R2D;
	vehicle.CY_delta_aR = -0.000403*R2D;
	vehicle.CY_beta_Cmu = 0.012240*R2D;

	vehicle.Cl_0 = 0;
	vehicle.Cl_beta = 0.000137*R2D;
	vehicle.Cl_delta_r =0.000303*R2D;
	vehicle.Cl_delta_aL_Cmu =-0.000338*R2D;
	vehicle.Cl_delta_aR_Cmu = 0.000348*R2D;
	vehicle.Cl_delta_aL = -0.001441*R2D;
	vehicle.Cl_delta_aR =  0.001443*R2D;

	vehicle.Cm_0 = 0.241903;
	vehicle.Cm_alpha = -0.082990*R2D;
	vehicle.Cm_delta_e =-0.096531*R2D;
	vehicle.Cm_delta_aL = 0.004447*R2D;
	vehicle.Cm_delta_aR =  0.004414*R2D;
	vehicle.Cm_Cmu = 0.481194;
	vehicle.Cm_alpha_Cmu = 0.042538*R2D;
	vehicle.Cm_delta_f = 0.006377*R2D;
	vehicle.Cm_beta2 = -0.001099*R2D*R2D;
	vehicle.Cm_beta2_Cmu = 0.001855*R2D;

	vehicle.Cn_0 = 0;
	vehicle.Cn_beta = -0.000660*R2D;
	vehicle.Cn_delta_r = -0.000679*R2D;
	vehicle.Cn_delta_aL_Cmu = 0.000624*R2D;
	vehicle.Cn_delta_aR_Cmu = -0.000626*R2D;
	vehicle.Cn_delta_aL = -0.000118*R2D;
	vehicle.Cn_delta_aR =  0.000119*R2D;
	VEHICLE.Cn_beta_Cmu = -0.000829*R2D;
}



void v_plane_param_define_ardupilot_default()
{
	vehicle.mass = 2.0 ; // kg
	vehicle.Ixx = 0.0342 ;
	vehicle.Iyy = 0.0454 ;
	vehicle.Izz = 0.0977 ;
	vehicle.Ixz = 0.0002 ;
	vehicle.s = 0.2589;
	vehicle.b = 1.4224;
	vehicle.c = 0.3302;
	vehicle.rho = 1.225;
	vehicle.mlgL_x = -0.16;
	vehicle.mlgL_y = -0.08;
	vehicle.mlgL_z = 0;
	vehicle.mlgR_x = -0.16;
	vehicle.mlgR_y = 0.08;
	vehicle.mlgR_z = 0;
	vehicle.nlg_x = 0.85;
	vehicle.nlg_y = 0;
	vehicle.nlg_z = 0;
	
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

	vehicle.alpha = constrain_float1(vehicle.alpha,-20.0f/57.3f,20.0f/57.3f);


	if(vehicle.tas < 3.0)
	{
		vehicle.beta = 0;
	}
	else
	{
		vehicle.beta = asinf(vehicle.V_b_tas[1]/sqrtf(powf(vehicle.V_b_tas[0],2) + powf(vehicle.V_b_tas[1],2) + powf(vehicle.V_b_tas[2],2)));
	}

	vehicle.beta = constrain_float1(vehicle.beta,-20.0/57.3f,20.0/57.3f);

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
