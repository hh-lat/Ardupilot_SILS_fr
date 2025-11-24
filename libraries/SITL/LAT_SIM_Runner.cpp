
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include "LAT_SIM_rk4.h"
#include "LAT_SIM_Runner.h"
#include <AP_Math/AP_Math.h>
#include "SIM_Aircraft.h"

vehcle_STATES vehcle;

strct_home_states s_home_state;

// Pointer to the Aircraft instance (set from SIM_Plane)
static SITL::Aircraft* g_aircraft_instance = nullptr;

void v_set_aircraft_instance(SITL::Aircraft* aircraft) {
    g_aircraft_instance = aircraft;
}

void v_lat_fdm_init()
{	
	vehcle.plane_model = PLANE_ARDU_DEFAULT; // 0 for default simple model, 1 for equinox model
    vehcle.dof = DOF_ALL_MOTION;
	vehcle.plane_on_ground = 1;
    v_plane_param_define();
}

void v_update_vehcle_state_from_ardu_ekf()
{
	// Get Euler angles (phi, theta, psi) from DCM, ardupilot dcm- body to NED
	if (g_aircraft_instance != nullptr) 
	{
		// phi, theta , psi via dcm
		const Matrix3f &dcm_matrix = g_aircraft_instance->get_dcm();

		float phi = atan2f(dcm_matrix.b.z, dcm_matrix.c.z);      // roll
		float theta = -asinf(dcm_matrix.a.z);                     // pitch
		float psi = atan2f(dcm_matrix.a.y, dcm_matrix.a.x);      // yaw
		
		vehcle.phi = phi;
		vehcle.theta = theta;
		vehcle.psi = psi;

		// aoa & beta via air velcity components
		const Vector3f &velocity_air_bf = g_aircraft_instance->get_velocity_air_bf();
		
		float angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
		float beta = atan2f(velocity_air_bf.y, velocity_air_bf.x);
		
		// Store in vehicle state
		vehcle.alpha = angle_of_attack;
		vehcle.beta = beta;
		
		// Also fill V_b_tas for other calculations
		vehcle.V_b_tas[0] = velocity_air_bf.x;
		vehcle.V_b_tas[1] = velocity_air_bf.y;
		vehcle.V_b_tas[2] = velocity_air_bf.z;

		vehcle.tas = sqrtf( vehcle.V_b_tas[0]*vehcle.V_b_tas[0]  + vehcle.V_b_tas[1]*vehcle.V_b_tas[1] + 
							vehcle.V_b_tas[2]*vehcle.V_b_tas[2] );

		vehcle.Q = 0.5*vehcle.rho*vehcle.tas*vehcle.tas;
	}
}

void v_lat_fdm_run(const struct sitl_input &input)
{
    float t_step = 0.001;
    static float t=0;
    float plane_state[12]={0};

	v_update_vehcle_state_from_ardu_ekf();
	plane_state[6] = vehcle.phi;
	plane_state[7] = vehcle.theta;
	plane_state[8] = vehcle.psi;

    v_ardu_input_to_lat_input(input); // fills servo_channels from ardupilot to control surface and motor pwms
    v_servo_dynamics(t_step); // converts servopwm to angles and applies rate limits
   	v_motor_pwm_to_throttle(t_step); //update throttle from pwm with rate limiting on throttle
    v_rk4(plane_state, t, t_step);

	vehcle.p = plane_state[3];
	vehcle.q = plane_state[4];
	vehcle.r = plane_state[5];
}


void v_plane_param_define()
{

	switch (vehcle.plane_model)
	{
		case PLANE_EQX:
		{
		     v_plane_param_define_equinox();
			 break;
		}
		default:
		{
			 v_plane_param_define_equinox();
			 break;
		}
	}
}

void v_plane_param_define_equinox()
{
	s_servo_manager.num_servos = 10; //  put max servo enum that is used
	s_motor_manager.num_motors = 8;

   //v_set_servo_params(pwm_min, pwm_max, angle_pwm_min, angle_pwm_max,omega, zeta, min_rate, max_rate, min_accel, max_accel,  type)
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, AILERON_COMMON);
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, ELEVATOR_COMMON);
	v_set_servo_params(1100,1900, 30*D2R,-30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, RUDDER_COMMON);

	v_set_servo_params(1100,1900, 30*D2R,-30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, AILERON_LEFT);
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, AILERON_RIGHT);

	v_set_servo_params(1100,1900,-40*D2R,40*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, FLAP);

	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, NOSE_LG_SERVO);
	
    vehcle.mass = 65.0 ;
	vehcle.g = 9.81; 
    vehcle.Ixx = 14.658 ;
    vehcle.Iyy = 16.944 ;
    vehcle.Izz = 27.412 ;
    vehcle.Ixz = 3.985 ;
	vehcle.Iyz = 0.0 ;
	vehcle.Ixy = 0.0 ;
    vehcle.s = 0.9;
	vehcle.s_blown = 0.216;
    vehcle.b = 3.0;
    vehcle.c = 0.3;
	vehcle.e =1;
	vehcle.AR =vehcle.b*vehcle.b/vehcle.s;
	vehcle.t_by_c = 0.15;
    vehcle.rho = 1.15;
    vehcle.mlgL_x = 0;
    vehcle.mlgL_y = 0;
    vehcle.mlgL_z = 0;
    vehcle.mlgR_x = 0;
    vehcle.mlgR_y = 0;
    vehcle.mlgR_z = 0;
    vehcle.nlg_x = 0;
    vehcle.nlg_y = 0;
    vehcle.nlg_z = 0;
	vehcle.alpha_stall = 14.0*D2R; // stall angle in rad

	vehcle.CL_0 = 0.29;
	vehcle.CL_delta_e =1.175;
	vehcle.CL_alpha = 5.0; // default
	vehcle.CL_q =0; // default

	vehcle.CD_0 = 0.2; 
	vehcle.CD_delta_e = -0.355;
	vehcle.CD_delta_f =0.1;
	vehcle.CD_delta_e2 = 0.5;
	vehcle.CD_alpha = 0.3;// default

	vehcle.CY_0 = 0;
	vehcle.CY_beta = 0.011931*R2D;
	vehcle.CY_delta_r =0.001499*R2D;
	vehcle.CY_delta_aL_Cmu =-0.000199*R2D;
	vehcle.CY_delta_aR_Cmu = 0.000196*R2D;
	vehcle.CY_delta_aL = 0.000391*R2D;
	vehcle.CY_delta_aR = -0.000403*R2D;
	vehcle.CY_beta_Cmu = 0.012240*R2D;

	vehcle.Cl_0 = 0;
	vehcle.Cl_beta = 0.000137*R2D;
	vehcle.Cl_delta_r =0.000303*R2D;
	vehcle.Cl_delta_aL_Cmu =-0.000338*R2D;
	vehcle.Cl_delta_aR_Cmu = 0.000348*R2D;
	vehcle.Cl_delta_aL = -0.001441*R2D;
	vehcle.Cl_delta_aR =  0.001443*R2D;

	vehcle.Cm_0 = 0.241903;
	vehcle.Cm_alpha = -0.082990*R2D;
	vehcle.Cm_delta_e =-0.096531*R2D;
	vehcle.Cm_delta_aL = 0.004447*R2D;
	vehcle.Cm_delta_aR =  0.004414*R2D;
	vehcle.Cm_Cmu = 0.481194;
	vehcle.Cm_alpha_Cmu = 0.042538*R2D;
	vehcle.Cm_delta_f = 0.006377*R2D;
	vehcle.Cm_beta2 = -0.001099*R2D*R2D;
	vehcle.Cm_beta2_Cmu = 0.001855*R2D;

	vehcle.Cn_0 = 0;
	vehcle.Cn_beta = -0.000660*R2D;
	vehcle.Cn_delta_r = -0.000679*R2D;
	vehcle.Cn_delta_aL_Cmu = 0.000624*R2D;
	vehcle.Cn_delta_aR_Cmu = -0.000626*R2D;
	vehcle.Cn_delta_aL = -0.000118*R2D;
	vehcle.Cn_delta_aR =  0.000119*R2D;
	vehcle.Cn_beta_Cmu = -0.000829*R2D;

	s_motor[0].rotor_xyz[0] = 0;                     s_motor[1].rotor_xyz[0] = 0;
	s_motor[0].rotor_xyz[1] = 0;                     s_motor[1].rotor_xyz[1] = 0;
	s_motor[0].rotor_xyz[2] = 0;                     s_motor[1].rotor_xyz[2] = 0;
	
	s_motor[2].rotor_xyz[0] = 0;                     s_motor[3].rotor_xyz[0] = 0;
	s_motor[2].rotor_xyz[1] = 0;                     s_motor[3].rotor_xyz[1] = 0;
	s_motor[2].rotor_xyz[2] = 0;                     s_motor[3].rotor_xyz[2] = 0;
	
	s_motor[4].rotor_xyz[0] = 0;                     s_motor[5].rotor_xyz[0] = 0;
	s_motor[4].rotor_xyz[1] = 0;                     s_motor[5].rotor_xyz[1] = 0;
	s_motor[4].rotor_xyz[2] = 0;                     s_motor[5].rotor_xyz[2] = 0;
	
	s_motor[6].rotor_xyz[0] = 0;                     s_motor[7].rotor_xyz[0] = 0;
	s_motor[6].rotor_xyz[1] = 0;                     s_motor[7].rotor_xyz[1] = 0;
	s_motor[6].rotor_xyz[2] = 0;                     s_motor[7].rotor_xyz[2] = 0;

	s_motor[0].rotor_tilt[0] = 0.0;                     s_motor[1].rotor_tilt[0] = 0.0;
	s_motor[0].rotor_tilt[1] = 0.0;                     s_motor[1].rotor_tilt[1] = 0.0;
	s_motor[0].rotor_tilt[2] = 0.0;                     s_motor[1].rotor_tilt[2] = 0.0;

	s_motor[2].rotor_tilt[0] = 0.0;                     s_motor[3].rotor_tilt[0] = 0.0;
	s_motor[2].rotor_tilt[1] = 0.0;                     s_motor[3].rotor_tilt[1] = 0.0;
	s_motor[2].rotor_tilt[2] = 0.0;                     s_motor[3].rotor_tilt[2] = 0.0;

	s_motor[4].rotor_tilt[0] = 0.0;                     s_motor[5].rotor_tilt[0] = 0.0;
	s_motor[4].rotor_tilt[1] = 0.0;                     s_motor[5].rotor_tilt[1] = 0.0;
	s_motor[4].rotor_tilt[2] = 0.0;                     s_motor[5].rotor_tilt[2] = 0.0;

	s_motor[6].rotor_tilt[0] = 0.0;                     s_motor[7].rotor_tilt[0] = 0.0;
	s_motor[6].rotor_tilt[1] = 0.0;                     s_motor[7].rotor_tilt[1] = 0.0;
	s_motor[6].rotor_tilt[2] = 0.0;                     s_motor[7].rotor_tilt[2] = 0.0;

	///////////////////////////////////////////////////////////////////////////////////////
	s_motor[0].rotor_r_direction = 1.0;              s_motor[1].rotor_r_direction = -1.0;
	s_motor[2].rotor_r_direction = -1.0;             s_motor[3].rotor_r_direction = 1.0;

	s_motor[4].rotor_r_direction = 1.0;              s_motor[5].rotor_r_direction = -1.0;
	s_motor[6].rotor_r_direction = -1.0;             s_motor[7].rotor_r_direction = 1.0;
}



void v_plane_param_define_ardupilot_default()
{

	//v_set_servo_params(pwm_min, pwm_max, angle_pwm_min, angle_pwm_max,omega, zeta, min_rate, max_rate, min_accel, max_accel,  type)
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, AILERON_COMMON);
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, ELEVATOR_COMMON);
	v_set_servo_params(1100,1900, 30*D2R,-30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, RUDDER_COMMON);

	v_set_servo_params(1100,1900, 30*D2R,-30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, AILERON_LEFT);
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, AILERON_RIGHT);

	v_set_servo_params(1100,1900,-40*D2R,40*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, FLAP);
		
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -360*D2R, 360*D2R, -720*D2R, 720*D2R, NOSE_LG_SERVO);


	// Ensure coffecient signs are compatible with aerospace sign convention for elevator, aileron, rudder
	vehcle.s = 0.45;
	vehcle.b = 1.88;
	vehcle.c = 0.24;
	vehcle.rho = 1.225;
	vehcle.alpha_stall = 0.4712; // 27 degrees
	vehcle.e = 0.9;
	vehcle.g=9.81;
	vehcle.lift_stall_M = 50.0;
	vehcle.AR = (vehcle.b*vehcle.b)/vehcle.s;

	vehcle.mass = 2.0 ; // kg
	vehcle.Ixx = 1.0 ;
	vehcle.Iyy = 1.0 ;
	vehcle.Izz = 1.0 ;
	vehcle.Ixz = 0.0 ;

	vehcle.mlgL_x = -0.16;
	vehcle.mlgL_y = -0.08;
	vehcle.mlgL_z = 0;
	vehcle.mlgR_x = -0.16;
	vehcle.mlgR_y = 0.08;
	vehcle.mlgR_z = 0;
	vehcle.nlg_x = 0.85;
	vehcle.nlg_y = 0;
	vehcle.nlg_z = 0;
	
	vehcle.CL_0 = 0.56;
	vehcle.CL_delta_e = 0;
	vehcle.CL_alpha = 6.9;
	vehcle.CL_q = 0;
	vehcle.CD_0 = 0.1;
	vehcle.CD_delta_e = 0;
	vehcle.CD_delta_f = 0;
	vehcle.CD_delta_e2 = 0;
	vehcle.CD_alpha = 0.3;
	vehcle.CY_0 = 0;
	vehcle.CY_beta = -0.98;
	vehcle.CY_p = 0;
	vehcle.CY_r = 0;
	vehcle.CY_delta_r = 0.2;
	vehcle.CY_delta_a = 0;
	vehcle.CY_delta_aL_Cmu = 0;
	vehcle.CY_delta_aR_Cmu = 0;
	vehcle.CY_delta_aL = 0;
	vehcle.CY_delta_aR = 0;
	vehcle.CY_beta_Cmu = 0;
	vehcle.Cl_0 = 0;
	vehcle.Cl_beta = -0.12;
	vehcle.Cl_delta_r = 0.037;
	vehcle.Cl_delta_aL_Cmu = 0;
	vehcle.Cl_delta_aR_Cmu = 0;
	vehcle.Cl_delta_aL = 0;
	vehcle.Cl_delta_a = 0.25;
	vehcle.Cl_delta_aR = 0;
	vehcle.Cl_p = -1.0;
	vehcle.Cl_r = 0.14;
	vehcle.Cm_0 = 0.045;
	vehcle.Cm_alpha = -0.7;
	vehcle.Cm_q = -20;
	vehcle.Cm_delta_e = -1.0;
	vehcle.Cm_delta_aL = 0;
	vehcle.Cm_delta_aR = 0;
	vehcle.Cm_Cmu = 0;
	vehcle.Cm_alpha_Cmu = 0;
	vehcle.Cm_delta_f = 0;
	vehcle.Cm_beta2 = 0;
	vehcle.Cm_beta2_Cmu = 0;
	vehcle.Cn_0 = 0;
	vehcle.Cn_beta = 0.25;
	vehcle.Cn_p = 0.022;
	vehcle.Cn_r = -1;
	vehcle.Cn_delta_a = 0;
	vehcle.Cn_delta_r = -0.1;
	vehcle.Cn_delta_aL_Cmu = 0;
	vehcle.Cn_delta_aR_Cmu = 0;
	vehcle.Cn_delta_aL = 0;
	vehcle.Cn_delta_aR = 0;
	vehcle.Cn_beta_Cmu = 0;	
}


void v_update_vehcle_states(float state[])
{
	float temp3X1_5[3]={0};
	float temp3X1_6[3]={0};

	vehcle.V_b_gnd[0] = state[0];
	vehcle.V_b_gnd[1] = state[1];
	vehcle.V_b_gnd[2] = state[2];

	body_to_NED(vehcle.V_b_gnd, vehcle.V_ned_gnd);

	temp3X1_5[0] = vehcle.V_ned_gnd[0] - vehcle.wind_ned[0];
	temp3X1_5[1] = vehcle.V_ned_gnd[1] - vehcle.wind_ned[1];
	temp3X1_5[2] = vehcle.V_ned_gnd[2] - vehcle.wind_ned[2];

	NED_to_body(temp3X1_5,temp3X1_6); //&vehcle.V_b_tas);

	vehcle.V_b_tas[0] = temp3X1_6[0];
	vehcle.V_b_tas[1] = temp3X1_6[1];
	vehcle.V_b_tas[2] = temp3X1_6[2];

	vehcle.tas = sqrtf(vehcle.V_b_tas[0]*vehcle.V_b_tas[0] + vehcle.V_b_tas[1]*vehcle.V_b_tas[1] + vehcle.V_b_tas[2]*vehcle.V_b_tas[2]);
	vehcle.gs  = sqrtf(vehcle.V_b_gnd[0]*vehcle.V_b_gnd[0] + vehcle.V_b_gnd[1]*vehcle.V_b_gnd[1] + vehcle.V_b_gnd[2]*vehcle.V_b_gnd[2]);

	vehcle.hrz_gnd_speed = sqrtf(vehcle.V_b_gnd[0]*vehcle.V_b_gnd[0] + vehcle.V_b_gnd[1]*vehcle.V_b_gnd[1] );
	vehcle.vert_gnd_vel  = vehcle.V_b_gnd[2];


	vehcle.phi    		= state[6];
	vehcle.theta  		= state[7];
	vehcle.psi    		= state[8];
	vehcle.pos_ned[0] 	= state[9];
	vehcle.pos_ned[1] 	= state[10];
	vehcle.pos_ned[2] 	= state[11];


	if ((vehcle.tas < 3.0) || (fabsf(vehcle.V_b_tas[0]) < 3.0))
	{
		vehcle.alpha = 0;
	}
	else
	{
		vehcle.alpha = atan2f(vehcle.V_b_tas[2], vehcle.V_b_tas[0]);
	}

	vehcle.alpha = constrain_float1(vehcle.alpha,-20.0f/57.3f,20.0f/57.3f);


	if(vehcle.tas < 3.0)
	{
		vehcle.beta = 0;
	}
	else
	{
		vehcle.beta = asinf(vehcle.V_b_tas[1]/sqrtf(powf(vehcle.V_b_tas[0],2) + powf(vehcle.V_b_tas[1],2) + powf(vehcle.V_b_tas[2],2)));
	}

	vehcle.beta = constrain_float1(vehcle.beta,-20.0/57.3f,20.0/57.3f);

	/*vehcle.gamma = vehcle.vert_gnd_vel/vehcle.hrz_gnd_speed*/

	vehcle.Q = 0.5f*vehcle.rho*vehcle.tas*vehcle.tas;
}


void v_fill_lla_to_vehcle_state(float latitude_point,float longitude_point, float Alt)
{
	vehcle.lat = latitude_point;
	vehcle.lon = longitude_point;
	vehcle.alt_msl = Alt; // should fill negative for height above MSL
	vehcle.alt_agl = vehcle.alt_msl - s_home_state.alt_msl;// up is negative
}
