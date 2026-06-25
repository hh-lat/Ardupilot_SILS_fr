
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include "LAT_SIM_rk4.h"
#include "LAT_SIM_Runner.h"
#include <AP_Math/AP_Math.h>
#include "SIM_Aircraft.h"
#include "Output.h"
#include "LAT_SIM_MonteCarlo.h"

vehcle_STATES vehcle;

strct_home_states s_home_state;

// Pointer to the Aircraft instance (set from SIM_Plane)
static SITL::Aircraft* g_aircraft_instance = nullptr;

void v_set_aircraft_instance(SITL::Aircraft* aircraft) {
    g_aircraft_instance = aircraft;
}

void v_lat_fdm_init()
{	
	vehcle.plane_model =PLANE_USTOL_V1;   // PLANE_ARDU_DEFAULT;//PLANE_EQX_V1_NEW_MODEL;//PLANE_EQX;////PLANE_EQX//PLANE_ARDU_DEFAULT // 0 for default simple model, 1 for equinox model
    vehcle.dof = DOF_ALL_MOTION;//DOF_ALL_MOTION;          // DOF_LONGITUDINAL_ONLY;
	vehcle.plane_on_ground = 1;
    v_plane_param_define();
	v_output_log_init();                  // open CSV log file for this run
}

void v_update_vehcle_state_from_ardu_ekf()
{
	// Get Euler angles (phi, theta, psi) from DCM, ardupilot dcm- body to NED
	if (g_aircraft_instance != nullptr) 
	{
		// phi, theta , psi via dcm
		// Matrix3f &dcm_matrix = g_aircraft_instance->get_dcm();

		// float phi = atan2f(dcm_matrix.b.z, dcm_matrix.c.z);      // roll
		// float theta = -asinf(dcm_matrix.a.z);                     // pitch
		// float psi = atan2f(dcm_matrix.a.y, dcm_matrix.a.x);      // yaw
		
		// vehcle.phi = phi;
		// vehcle.theta = theta;
		// vehcle.psi = psi;

		// aoa & beta via air velcity components
		//const Vector3f &velocity_air_bf = g_aircraft_instance->get_velocity_air_bf();
		vehcle.tas = sqrtf( vehcle.V_b_tas[0]*vehcle.V_b_tas[0]  + vehcle.V_b_tas[1]*vehcle.V_b_tas[1] + 
							vehcle.V_b_tas[2]*vehcle.V_b_tas[2] );


		float angle_of_attack = 0.0f;
		float beta = 0.0f;

		if (vehcle.plane_moving_state == STATIONARY)
		{
			angle_of_attack = 0;
			beta = 0;
		}
		else if (vehcle.plane_moving_state == CT_RUNWAY_MOVING)
		{
			angle_of_attack = 0;
			beta = 0;
		}
		else if (vehcle.plane_moving_state == CT_RUNWAY_ROTATING)
		{
			//angle_of_attack = D2R*8;//vehcle.theta;
			angle_of_attack = vehcle.theta;
			beta = 0;
		}
		else if (vehcle.plane_moving_state == IN_AIR)
		{
			angle_of_attack = atan2f(vehcle.V_b_tas[2] , vehcle.V_b_tas[0] );
			beta = atan2f(vehcle.V_b_tas[1] , vehcle.V_b_tas[0] );
		}

		if ((vehcle.tas < 3.0) || (fabsf(vehcle.V_b_tas[0]) < 3.0))
		{
			angle_of_attack  = 0;
		}

		angle_of_attack = constrain_float1(angle_of_attack, -70.0f*D2R, 70.0f*D2R);

		beta = constrain_float1(beta, -50.0f*D2R, 50.0f*D2R);
		
		// Store in vehicle state
		vehcle.alpha = angle_of_attack;
		vehcle.beta = beta;
		
		// // Also fill V_b_tas for other calculations
		// vehcle.V_b_tas[0] = velocity_air_bf.x;
		// vehcle.V_b_tas[1] = velocity_air_bf.y;
		// vehcle.V_b_tas[2] = velocity_air_bf.z;

	
		vehcle.Q = 0.5*vehcle.rho*vehcle.tas*vehcle.tas;
	}
}

void v_lat_fdm_run(const struct sitl_input &input)
{
    static float t=0;
    float plane_state[12]={0};

	if  (vehcle.step_dt >0.05)
	{
		vehcle.step_dt =0.05; //max 20 Hz
	}

	if (vehcle.step_dt <0.000001)
	{
		vehcle.step_dt =0.000001; 
	}

	v_update_vehcle_state_from_ardu_ekf();
	plane_state[0] = vehcle.V_b_tas[0];
	plane_state[1] = vehcle.V_b_tas[1];
	plane_state[2] = vehcle.V_b_tas[2];
	plane_state[3] = vehcle.p;
	plane_state[4] = vehcle.q;
	plane_state[5] = vehcle.r;
	plane_state[6] = vehcle.phi;
	plane_state[7] = vehcle.theta;
	plane_state[8] = vehcle.psi;


    v_ardu_input_to_lat_input(input); // fills servo_channels from ardupilot to control surface and motor pwms
    v_servo_dynamics(vehcle.step_dt); // converts servopwm to angles and applies rate limits
	v_rotor_esc_dynamics(vehcle.step_dt); // converts motor pwm to throttle with rate limiting

    // if (vehcle.plane_moving_state == STATIONARY)
	// {


	// }
	// else if ((vehcle.plane_on_ground == 0) && (vehcle.plane_moving_state != STATIONARY))
	// {
	// 	for (int i=0;i<s_motor_manager.num_motors;i++)
	// 	{
	// 		s_motor[i].throttle_cmd = 1700;
	// 	}

	// 	vehcle.delta_f = 20.0f*D2R; // set throttle to 20% when in air
	// 	vehcle.delta_e = 0.75;//25.0f*D2R;
	// 	vehcle.delta_aL = 0;
	// 	vehcle.delta_aR = 0;
	// 	vehcle.delta_r = 0;
	// 	vehcle.delta_a = 0;
	// }
	// else if ((vehcle.plane_on_ground == 1) && (vehcle.plane_moving_state == STATIONARY))
	// {
	// 	for (int i=0;i<s_motor_manager.num_motors;i++)
	// 	{
	// 		s_motor[i].throttle_cmd = 1000;
	// 	}

	// 	vehcle.delta_f = 20.0f*D2R; // set throttle to 20% when in air
	// 	vehcle.delta_e = 25.0f*D2R;
	// 	vehcle.delta_aL = 0;
	// 	vehcle.delta_aR = 0;
	// 	vehcle.delta_r = 0;
	// 	vehcle.delta_a = 0;	
	// }
	// else if ( (vehcle.plane_on_ground == 1) && (vehcle.plane_moving_state != STATIONARY) )
	// {
	// 	for (int i=0;i<s_motor_manager.num_motors;i++)
	// 	{
	// 		s_motor[i].throttle_cmd = 1800;
	// 	}

	// 	vehcle.delta_f = 20.0f*D2R; // set throttle to 20% when in air
	// 	vehcle.delta_e = -0.1566;//-20.0f*D2R;
	// 	vehcle.delta_aL = 0;
	// 	vehcle.delta_aR = 0;
	// 	vehcle.delta_r = 0;
	// 	vehcle.delta_a = 0;	
	// }	

	t += vehcle.step_dt;

	v_rk4(plane_state, t, vehcle.step_dt);
	
	static float print_timer = 0;
	print_timer += vehcle.step_dt;
	static float print_timer1 = 0;
	print_timer1 += vehcle.step_dt;

	// if (print_timer >= 0.1f) 
	// {
	// 	print_timer = 0;
	// 	printf("plane_moving_state: %d, Rotor-force(N): %.2f, delta_e: %.2f, delta_aL: %.2f, delta_aR: %.2f, delta_r: %.2f, delta_f: %.2f, delta_a: %.2f, MLG_NR: %.2f, FLG_NR: %.2f, Accel_ned[0]: %.2f, Accel_ned[1]: %.2f, Accel_ned[2]: %.2f, alt_agl: %.2f\n", 
	// 	vehcle.plane_moving_state, vehcle.all_rotors_force[0],vehcle.delta_e*R2D, vehcle.delta_aL*R2D, vehcle.delta_aR*R2D, vehcle.delta_r*R2D, vehcle.delta_f*R2D, vehcle.delta_a*R2D, vehcle.MLG_NR, vehcle.FLG_NR, vehcle.Accel_ned[0], vehcle.Accel_ned[1], vehcle.Accel_ned[2], vehcle.alt_agl);

	// 	print_timer1 = 0;
	// }
if (print_timer >= 0.1f) 
{
    print_timer = 0;

    // Write all vehicle states to CSV log file
    v_output_log_write(t);

    // Also keep a minimal console print for quick monitoring
    printf("Time: %.2f, state: %d, MLG_NR: %.1f, FLG_NR: %.2f\n", 
           t,
           vehcle.plane_moving_state,
           vehcle.MLG_NR,
           vehcle.FLG_NR);

    print_timer1 = 0;
}


    // For Ardupilot
	switch (vehcle.plane_moving_state)
	{
		case STATIONARY:
		{
			vehcle.Accel_b[0]= 0;// ardupilot needs accel without normal reaction and weight//(vehcle.total_force_bd[0] -vehcle.mg_b[0])/vehcle.mass;
			vehcle.Accel_b[1]= 0;//(vehcle.total_force_bd[1] -vehcle.mg_b[1])/vehcle.mass;
			vehcle.Accel_b[2]= 0;//(vehcle.total_force_bd[2] -vehcle.mg_b[2])/vehcle.mass;


			vehcle.Accel_ned[0] = 0;
			vehcle.Accel_ned[1] = 0;
			vehcle.Accel_ned[2] = 0;
			vehcle.p_dot =0;
			vehcle.q_dot =0;
			vehcle.r_dot =0;
			break;
		}

		case CT_RUNWAY_MOVING:
		case CT_RUNWAY_ROTATING:
		{
			vehcle.Accel_b[0]= (vehcle.total_force_bd[0])/vehcle.mass;
			vehcle.Accel_b[1]= 0;
			vehcle.Accel_b[2]= (vehcle.total_force_bd[2])/vehcle.mass;

			body_to_NED(vehcle.Accel_b,  vehcle.Accel_ned);

			//vehcle.Accel_ned[1]=0;
			vehcle.Accel_ned[2]=0;

			NED_to_body(vehcle.Accel_ned, vehcle.Accel_b);

			vehcle.p_dot = 0;

			if (fabsf(vehcle.delta_r) > 5.0/57.3)
			{
				vehcle.r_dot= 0.5*vehcle.rho*powf(vehcle.tas-vehcle.aero_zero_speed,2)*(vehcle.delta_r*vehcle.ground_yaw_gain);
			}
			else
			{
				vehcle.r_dot= 0;
			}

			vehcle.r_dot=0;
			break;	
		}

		case IN_AIR:
		{
			vehcle.Accel_b[0]= (vehcle.total_force_bd[0])/vehcle.mass;
			vehcle.Accel_b[1]= (vehcle.total_force_bd[1])/vehcle.mass;
			vehcle.Accel_b[2]= (vehcle.total_force_bd[2])/vehcle.mass;

			body_to_NED(vehcle.Accel_b,  vehcle.Accel_ned);

			break;	
		}
	}
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
		case PLANE_EQX_V1_NEW_MODEL:
		{
			 v_plane_param_define_eqx_v1_new_model();
			 break;
		}
		case PLANE_USTOL_V1:
		{
			 v_plane_param_define_ustol_v1();
			 break;
		}
		default:
		{
			 v_plane_param_define_ardupilot_default();
			 break;
		}
	}

	// Apply Monte Carlo overrides (if override file exists).
	// If no override file is present this is a silent no-op;
	// nominal parameters above remain untouched.
	v_apply_monte_carlo_overrides();
}

void v_plane_param_define_eqx_v1_new_model()
{
	s_servo_manager.num_servos = 16; //  put max servo enum that is used
	s_motor_manager.num_motors = 8;

   //v_set_servo_params(pwm_min, pwm_max, angle_pwm_min, angle_pwm_max,omega, zeta, min_rate, max_rate, min_accel, max_accel,  type)
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, AILERON_COMMON);
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, ELEVATOR_COMMON);

	// CFD assumed right rudder as +ve, so when ardupilot demands 1900 to turn right, rudder  should be moved right, hence +ve angle at 1900
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, RUDDER_COMMON);
	v_set_servo_params(1100,1900, 20*D2R,-20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, AILERON_LEFT);
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, AILERON_RIGHT);

	v_set_servo_params(1100,1900,0,40*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, FLAP);

	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, NOSE_LG_SERVO);
	
	vehcle.ground_yaw_gain = 0.5; 
	vehcle.cg_x = 1030.0/1000.0; // from nose center, put positve number
	vehcle.cg_z = 43.0/1000.0;// from nose center, 43 mm above  nose center
	vehcle.MLG_x = fabsf((1319.2/1000.0) - vehcle.cg_x); // put all positive
	vehcle.MLG_z = fabsf(319.6/1000.0 + vehcle.cg_z); // put all positive
	vehcle.FLG_x = fabsf((339.2/1000.0) - vehcle.cg_x);
	vehcle.FLG_z = vehcle.MLG_z;
    vehcle.delta_r_deadzone = 3.0*D2R; // 5 degree deadzone for rudder on ground
	vehcle.theta_tolerance_for_ground = 0.0*D2R; // pitch angle below which plane is shifted to runway moving state
	vehcle.altitude_tolerance_for_ground = -0.1; // altitude below which plane is considered on ground

	vehcle.mg_b[0]=0;
	vehcle.mg_b[1]=0;
	vehcle.mg_b[2]=0;

	vehcle.aero_zero_speed = 5.0; // m/s
    vehcle.mass = 65.0;
	vehcle.g = 9.81; 
	vehcle.lift_stall_M = 50.0;
    vehcle.Ixx = 14.658;
    vehcle.Iyy = 16.944;
    vehcle.Izz = 27.412;
    vehcle.Ixz = 0*3.985 ;
	vehcle.Iyz = 0.0 ;
	vehcle.Ixy = 0.0 ;
    vehcle.s = 0.9;
	vehcle.s_blown = 0.216;
    vehcle.b = 3.0;
    vehcle.c = 0.3;
	vehcle.e =0.7;
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
	vehcle.alpha_stall = 25.0*D2R; // stall angle in rad

	vehcle.CL_0 = 0.18;
	vehcle.CL_delta_e = 0.69;
	vehcle.CL_alpha = 5.718; // default
	vehcle.CL_q =12.5;//3.0; 

	vehcle.CD_0 = 0.0763; 
	vehcle.CD_delta_e = 0.012;
	vehcle.CD_delta_f =0.23;
	vehcle.CD_delta_e2 = 0.004;
	vehcle.CD_alpha = 0.3;// default

	vehcle.CY_0 = 0;
	vehcle.CY_beta = -0.013686*R2D;
	vehcle.CY_delta_r =0.001042*R2D;
	vehcle.CY_delta_aL_Cmu = 0*R2D;
	vehcle.CY_delta_aR_Cmu = 0*R2D;
	vehcle.CY_delta_aL = -0.000212*R2D;
	vehcle.CY_delta_aR =  0.000186*R2D;
	vehcle.CY_beta_Cmu = 0.000597*R2D;
	vehcle.CY_p = -0.078266;
	vehcle.CY_r =  0.20338;

	vehcle.Cl_0 = -0.000117;
	vehcle.Cl_beta = -0.000003*R2D;
	vehcle.Cl_delta_r =0.000145*R2D;
	vehcle.Cl_delta_aL_Cmu =0.001281*R2D;
	vehcle.Cl_delta_aR_Cmu = -0.001267*R2D;
	vehcle.Cl_delta_aL = 0.001088*R2D;
	vehcle.Cl_delta_aR =  -0.001097*R2D;
	vehcle.Cl_p = 0;
	vehcle.Cl_r = 0.08;

	vehcle.Cm_0 = 0.249992;
	vehcle.Cm_alpha = -0.073394*R2D;
	vehcle.Cm_delta_e =-0.057653*R2D;
	vehcle.Cm_delta_aL = -0.000813*R2D;
	vehcle.Cm_delta_aR = -0.000659*R2D;
	vehcle.Cm_Cmu = 0.199495;
	vehcle.Cm_alpha_Cmu = 0.020895*R2D;
	vehcle.Cm_delta_f = 0.002852*R2D;
	vehcle.Cm_beta2 = -0.001134*R2D*R2D;
	vehcle.Cm_beta2_Cmu = -0.000034*R2D*R2D;
	vehcle.Cm_q = -77.0;

	vehcle.Cn_0 = 0;
	vehcle.Cn_beta = 0.001227*R2D;
	vehcle.Cn_delta_r = -0.000494*R2D;
	vehcle.Cn_delta_aL_Cmu = -0.000193*R2D;
	vehcle.Cn_delta_aR_Cmu =  0.000194*R2D;
	vehcle.Cn_delta_aL = -0.000029*R2D;
	vehcle.Cn_delta_aR =  0.000035*R2D;
	vehcle.Cn_beta_Cmu = 0.000786*R2D;
	vehcle.Cn_p = -0.01;
	vehcle.Cn_r = -0.05;
	
	for (int i=0;i<s_motor_manager.num_motors;i++)
	{
		s_motor[i].pwm_min = 1100;
		s_motor[i].pwm_max = 1900;
		s_motor[i].omega_tf = 30.0; // rad/s
		s_motor[i].zeta_tf = 0.7;
		s_motor[i].rpm_max = 36000.0;
		s_motor[i].rpm_min = 0.0;
		s_motor[i].dia_prop = 0.12;
		s_motor[i].max_thrust = 47.0*9.81/8.0; // 47 kg total thrust divided by number of motors	
		s_motor[i].min_thrust = 0.0;
		s_motor[i].thrust_2_torque_factor =0; // JP Hobby EDFs kind of not produces any torque
		s_motor[i].CT_static = s_motor[0].max_thrust / (vehcle.rho*powf(s_motor[0].dia_prop,4)*powf(s_motor[0].rpm_max/60.0f,2));;
		s_motor[i].rate_limit_throttle = (1.0/0.01); // (1.0/0.1)=full throttle change in 0.1 sec
	}

	float y1= 0.274;
	float y2= 0.598;
	float y3 = 0.922;
	float y4 = 1.246;

	float z = 0.027;//0948 //cg to edf, cg is above edf

	float x = 0;
	x= vehcle.cg_x - 757.2/1000.0; // edf are 30 mm behind cg

	s_motor[0].rotor_xyz[0] = x;                     s_motor[1].rotor_xyz[0] = x;
	s_motor[0].rotor_xyz[1] =-y4;                    s_motor[1].rotor_xyz[1] = -y3;
	s_motor[0].rotor_xyz[2] = z;                     s_motor[1].rotor_xyz[2] = z;
	
	s_motor[2].rotor_xyz[0] = x;                     s_motor[3].rotor_xyz[0] = x;
	s_motor[2].rotor_xyz[1] = -y2;                   s_motor[3].rotor_xyz[1] = -y1;
	s_motor[2].rotor_xyz[2] = z;                     s_motor[3].rotor_xyz[2] = z;

	s_motor[4].rotor_xyz[0] = x;                     s_motor[5].rotor_xyz[0] = x;
	s_motor[4].rotor_xyz[1] = y1;                   s_motor[5].rotor_xyz[1] = y2;
	s_motor[4].rotor_xyz[2] = z;                     s_motor[5].rotor_xyz[2] = z;	

	s_motor[6].rotor_xyz[0] = x;                     s_motor[7].rotor_xyz[0] = x;
	s_motor[6].rotor_xyz[1] = y3;                   s_motor[7].rotor_xyz[1] = y4;
	s_motor[6].rotor_xyz[2] = z;                     s_motor[7].rotor_xyz[2] = z;

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

	vehcle.alpha = constrain_float1(vehcle.alpha,-20.0f/57.3f,70.0f/57.3f);


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




void v_plane_param_define_equinox()
{
	s_servo_manager.num_servos = 16; //  put max servo enum that is used
	s_motor_manager.num_motors = 8;

   //v_set_servo_params(pwm_min, pwm_max, angle_pwm_min, angle_pwm_max,omega, zeta, min_rate, max_rate, min_accel, max_accel,  type)
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, AILERON_COMMON);
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, ELEVATOR_COMMON);

	// CFD assumed right rudder as +ve, so when ardupilot demands 1900 to turn right, rudder  should be moved right, hence +ve angle at 1900
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, RUDDER_COMMON);

	v_set_servo_params(1100,1900, 20*D2R,-20*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, AILERON_LEFT);
	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, AILERON_RIGHT);

	v_set_servo_params(1100,1900,0,40*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, FLAP);

	v_set_servo_params(1100,1900,-20*D2R,20*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, NOSE_LG_SERVO);
	

	vehcle.ground_yaw_gain = 0.5; 
	vehcle.cg_x = 1070.0/1000.0; // from nose center, put positve number
	vehcle.cg_z = 43.0/1000.0;// from nose center, 43 mm above  nose center
	vehcle.MLG_x = fabsf( (1319.2/1000.0) - vehcle.cg_x); // put all positive
	vehcle.MLG_z = fabsf(319.6/1000.0 + vehcle.cg_z); // put all positive
	vehcle.FLG_x = fabsf((339.2/1000.0) - vehcle.cg_x);
	vehcle.FLG_z = vehcle.MLG_z;
    vehcle.delta_r_deadzone = 3.0*D2R; // 5 degree deadzone for rudder on ground
	vehcle.theta_tolerance_for_ground = 0.0*D2R; // pitch angle below which plane is shifted to runway moving state
	vehcle.altitude_tolerance_for_ground = -0.1; // altitude below which plane is considered on ground

	vehcle.mg_b[0]=0;
	vehcle.mg_b[1]=0;
	vehcle.mg_b[2]=0;

	vehcle.aero_zero_speed = 5.0; // m/s
    vehcle.mass = 65.0 ;
	vehcle.g = 9.81; 
	vehcle.lift_stall_M = 50.0;
    vehcle.Ixx = 14.658 ;
    vehcle.Iyy = 16.944 ;
    vehcle.Izz = 27.412 ;
    vehcle.Ixz = 0*3.985 ;
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
	vehcle.alpha_stall = 20.0*D2R; // stall angle in rad

	vehcle.CL_0 = 0.29;
	vehcle.CL_delta_e =1.175;
	vehcle.CL_alpha = 5.0; // default
	vehcle.CL_q =22;//3.0; 

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
	vehcle.CY_p = -0.0299;
	vehcle.CY_r =  0.7979;

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
	vehcle.Cm_beta2_Cmu = 0.001855*R2D*R2D;
	vehcle.Cm_q = -116.5576;

	vehcle.Cn_0 = 0;
	vehcle.Cn_beta = -0.000660*R2D;
	vehcle.Cn_delta_r = -0.000679*R2D;
	vehcle.Cn_delta_aL_Cmu = 0.000624*R2D;
	vehcle.Cn_delta_aR_Cmu = -0.000626*R2D;
	vehcle.Cn_delta_aL = -0.000118*R2D;
	vehcle.Cn_delta_aR =  0.000119*R2D;
	vehcle.Cn_beta_Cmu = -0.000829*R2D;

	for (int i=0;i<s_motor_manager.num_motors;i++)
	{
		s_motor[i].pwm_min = 1100;
		s_motor[i].pwm_max = 1900;
		s_motor[i].omega_tf = 30.0; // rad/s
		s_motor[i].zeta_tf = 0.7;
		s_motor[i].rpm_max = 36000.0;
		s_motor[i].rpm_min = 0.0;
		s_motor[i].dia_prop = 0.12;
		s_motor[i].max_thrust = (vehcle.mass*vehcle.g/s_motor_manager.num_motors)*0.8;	
		s_motor[i].min_thrust = 0.0;
		s_motor[i].thrust_2_torque_factor =0; // JP Hobby EDFs kind of not produces any torque
		s_motor[i].CT_static = s_motor[0].max_thrust / (vehcle.rho*powf(s_motor[0].dia_prop,4)*powf(s_motor[0].rpm_max/60.0f,2));;
		s_motor[i].rate_limit_throttle = (1.0/0.01); // full throttle change in 0.1 sec
	}

	float y1= 0.274;
	float y2= 0.598;
	float y3 = 0.922;
	float y4 = 1.246;

	float z = 0.0948; //cg to edf, cg is above edf

	float x = 0;
	x= vehcle.cg_x - 757.2/1000.0; // edf are 30 mm behind cg

	s_motor[0].rotor_xyz[0] = x;                     s_motor[1].rotor_xyz[0] = x;
	s_motor[0].rotor_xyz[1] =-y4;                    s_motor[1].rotor_xyz[1] = -y3;
	s_motor[0].rotor_xyz[2] = z;                     s_motor[1].rotor_xyz[2] = z;
	
	s_motor[2].rotor_xyz[0] = x;                     s_motor[3].rotor_xyz[0] = x;
	s_motor[2].rotor_xyz[1] = -y2;                   s_motor[3].rotor_xyz[1] = -y1;
	s_motor[2].rotor_xyz[2] = z;                     s_motor[3].rotor_xyz[2] = z;

	s_motor[4].rotor_xyz[0] = x;                     s_motor[5].rotor_xyz[0] = x;
	s_motor[4].rotor_xyz[1] = y1;                   s_motor[5].rotor_xyz[1] = y2;
	s_motor[4].rotor_xyz[2] = z;                     s_motor[5].rotor_xyz[2] = z;	

	s_motor[6].rotor_xyz[0] = x;                     s_motor[7].rotor_xyz[0] = x;
	s_motor[6].rotor_xyz[1] = y3;                   s_motor[7].rotor_xyz[1] = y4;
	s_motor[6].rotor_xyz[2] = z;                     s_motor[7].rotor_xyz[2] = z;

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



void v_plane_param_define_ustol_v1()
{
	// Actuators
	s_servo_manager.num_servos = 16;
	s_motor_manager.num_motors = 18; 

	//v_set_servo_params(pwm_min, pwm_max, angle_pwm_min, angle_pwm_max, omega, zeta, min_rate, max_rate, min_accel, max_accel, type)
	v_set_servo_params(1100,1900,-20*D2R, 20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, AILERON_COMMON);
	v_set_servo_params(1100,1900,-10*D2R, 10*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, ELEVATOR_COMMON);
	v_set_servo_params(1100,1900,-20*D2R, 20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, RUDDER_COMMON);
	v_set_servo_params(1100,1900, 20*D2R,-20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, AILERON_LEFT);
	v_set_servo_params(1100,1900,-20*D2R, 20*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, AILERON_RIGHT);
	v_set_servo_params(1100,1900, 0,      32*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, FLAP);
	v_set_servo_params(1100,1900,-15*D2R, 15*D2R, 10.0, 0.7, -200*D2R, 200*D2R, -720*D2R, 720*D2R, NOSE_LG_SERVO);

	// Shared physical params (read by the common EOM / force assembly)
	vehcle.mass = 17.0;
	vehcle.g    = 9.81;
	vehcle.Ixx  = 11.41;
	vehcle.Iyy  = 5.23;
	vehcle.Izz  = 16.497;
	vehcle.Ixz  = -0.57;
	vehcle.Ixy  = 0.0;
	vehcle.Iyz  = 0.0;

	vehcle.s       = 1.681;   
	vehcle.b       = 4.1;    
	vehcle.c       = 0.41;   
	vehcle.s_blown = 0.8856;  
	vehcle.t_by_c  = 0.15;    
	vehcle.AR      = (vehcle.b*vehcle.b)/vehcle.s;  
	vehcle.e       = 0.7;     // placeholder (uSTOL CD uses wing.k_w, not e)

	vehcle.rho             = 1.15;    // ISA SL (atmos.rho); runtime atmosphere model may update
	vehcle.sound_speed     = 340.294;  // ISA SL (atmos.a) — needed for prop Mtip
	vehcle.aero_zero_speed = 3.0;
	vehcle.alpha_stall     = 18.459054*D2R;  // not used by uSTOL lift; kept non-garbage
	vehcle.lift_stall_M    = 50.0;

	// Ground / CG geometry (dimensional)
	vehcle.ground_yaw_gain = 0.5;
	vehcle.cg_x = 0.8613;   // CG aft of nose (x_cg_from_nose = 861.34 mm)        [m]
	vehcle.cg_z = 0.0537;   // CG above nose centre (z_cg_from_nose = 53.68 mm)   [m]
	vehcle.MLG_x = 0.040;   // CG->main-gear longitudinal: main LG 0.743 - CG 0.703 (40 mm aft of CG)  [m]
	vehcle.MLG_z = 0.25;    // CG->gear vertical = CG height above ground (z_cg_from_ground = 250 mm)
	vehcle.FLG_x = 0.534;   // CG->nose-gear longitudinal: CG 0.703 - nose LG 0.169 (534 mm fwd of CG)  [m]
	vehcle.FLG_z = 0.25;    // CG->gear vertical = CG height above ground
	vehcle.delta_r_deadzone = 3.0*D2R;
	vehcle.theta_tolerance_for_ground = 0.0*D2R;
	vehcle.theta_max_ground = 11.0*D2R;   // tail-strike rotation limit (entire config) [rad]
	vehcle.altitude_tolerance_for_ground = -0.1;
	vehcle.mg_b[0] = 0; vehcle.mg_b[1] = 0; vehcle.mg_b[2] = 0;

	// Motors (18 EDFs)  --- TODO
	for (int i=0; i<s_motor_manager.num_motors; i++) {
		s_motor[i].pwm_min = 1100;
		s_motor[i].pwm_max = 1900;
		s_motor[i].omega_tf = 30.0;     // rad/s
		s_motor[i].zeta_tf  = 0.7;
		s_motor[i].rpm_max  = 12000.0;  // n_max [rpm]
		s_motor[i].rpm_min  = 0.0;
		s_motor[i].dia_prop = 0.120;    // Dia [m]
		s_motor[i].max_thrust = 0.5*9.81;   // TODO per-EDF thrust clamp
		s_motor[i].min_thrust = 0.0;
		s_motor[i].thrust_2_torque_factor = 0;
		s_motor[i].CT_static = s_motor[i].max_thrust /
			(vehcle.rho*powf(s_motor[i].dia_prop,4)*powf(s_motor[i].rpm_max/60.0f,2));
		s_motor[i].rate_limit_throttle = (1.0/0.01);
		// 18-EDF spanwise layout: 9 per side, innermost 585 mm from centreline, 125 mm pitch.
		// rotor_xyz = EDF position relative to the CG, body axes (x fwd +, y right +, z down +).
		// Thrust is purely axial (rotor_tilt = 0), so M = r x F = (0, z*T, -y*T): only the y
		// offset (yaw via differential thrust) and z offset (pitch) create moments; the fore/aft
		// x offset is moment-neutral, set to the wing-LE station (CG 703 - wing LE 500) for completeness.
		int edf_side_idx = i % 9;                            // 0..8 outboard step (motors 0-8 left, 9-17 right)
		float edf_y = 0.585f + 0.125f*edf_side_idx;          // |spanwise offset| from centreline [m]
		s_motor[i].rotor_xyz[0] = 0.203f;                    // 203 mm forward of CG [m]
		s_motor[i].rotor_xyz[1] = (i < 9) ? -edf_y : edf_y;  // motors 0-8 left wing (y<0), 9-17 right (y>0)
		s_motor[i].rotor_xyz[2] = 0.03336f;                  // thrust line 33.36 mm below CG (down +) [m]
		s_motor[i].rotor_tilt[0] = 0.0; s_motor[i].rotor_tilt[1] = 0.0; s_motor[i].rotor_tilt[2] = 0.0;
		s_motor[i].rotor_r_direction = (i % 2 == 0) ? 1.0 : -1.0;
	}

	// Wing
	vehcle.wing.lambda_b   = 0.503*0.8;
	vehcle.wing.S_f        = 0.20664;
	vehcle.wing.S_a        = 0.05904;
	vehcle.wing.k_fit      = 0.535;
	vehcle.wing.cl0_camber = 0.519;
	vehcle.wing.k_w        = 2.632757;
	vehcle.wing.r          = -0.005473;

	// Tail
	vehcle.tail.a_t      = 4.27;
	vehcle.tail.eta_t_0  = 1.0271;
	vehcle.tail.eta_t_1  = 0.010824;
	vehcle.tail.eps0     = 0.030729;
	vehcle.tail.deps_pos = -0.018088;
	vehcle.tail.deps_neg = 0.283491;
	vehcle.tail.k_ht     = 3.13292;
	vehcle.tail.CD_ht0   = 0.004050;
	vehcle.tail.S_ht_S   = 0.23805;
	vehcle.tail.V_H      = 0.95;
	vehcle.tail.AR_ht    = 5.0;

	// Fuselage
	vehcle.fuse.CD_a2      = 1.956124;
	vehcle.fuse.CD_b2      = 0.453343;
	vehcle.fuse.CD_a2_Cmyu = 0.010846;
	vehcle.fuse.CD0        = 0.045249;
	vehcle.fuse.CDq	   = -0.373360;

	// Controls
	vehcle.controls.tau_f  = 0.297565;
	vehcle.controls.tau_a  = 0.086943;
	vehcle.controls.Kb_f   = 0.506727;
	vehcle.controls.Kb_a   = 0.131584;
	vehcle.controls.tau_e  = 0.625;
	vehcle.controls.de_eff_pos_break_deg = 10.0f;   // +del_e beyond this loses authority (TE-down)
	vehcle.controls.de_eff_pos_factor    = 0.25f;   // marginal effectiveness beyond +break
	vehcle.controls.de_eff_neg_break_deg = 6.0f;    // -del_e beyond this loses authority (TE-up)
	vehcle.controls.de_eff_neg_factor    = 0.5f;    // marginal effectiveness beyond -break
	vehcle.controls.CD_df2 = 0.217554;
	vehcle.controls.CD_da2 = 0.030418;
	vehcle.controls.CD_da  = 0.013566;
	vehcle.controls.CD_de2 = 0.164864;
	vehcle.controls.CD_de  = 0.010177;
	vehcle.controls.CD_dr2 = 0.142042;

	vehcle.controls.del_e_max = 10;   vehcle.controls.del_e_min = -10;
	vehcle.controls.del_a_max = 20;   vehcle.controls.del_a_min = -20;
	vehcle.controls.del_r_max = 15;   vehcle.controls.del_r_min = -15;
	vehcle.controls.del_thr_max = 0.9; vehcle.controls.del_thr_min = 0.05;
	vehcle.controls.alpha_max = 10;   vehcle.controls.alpha_min = -10;
	vehcle.controls.beta_max  = 15;   vehcle.controls.beta_min  = -15;

	// Stall (drag blend)
	vehcle.stall.K_flat   = 5.057530;
	vehcle.stall.a0_const = 18.459054;
	vehcle.stall.a0_Cmyu  = -0.125111;
	vehcle.stall.k        = 25.6810;

	// Post-stall lift blend
	vehcle.cl_stall.wM0    = 21.8813;
	vehcle.cl_stall.wM1    = -2.4776;
	vehcle.cl_stall.wa00   = 19.9997;
	vehcle.cl_stall.wa0mu  = 1.4955;
	vehcle.cl_stall.wa0f   = -5.2527;
	vehcle.cl_stall.wkflat = 4.0284;
	vehcle.cl_stall.rM     = 14.3286;
	vehcle.cl_stall.ra0    = 13.2261;
	vehcle.cl_stall.rkcu   = 0.4765;
	vehcle.cl_stall.rkflat = 1.3514;

	// Lateral side force 
	vehcle.lateral.theta0    = 0.000424;
	vehcle.lateral.theta_b   = -0.014394;
	vehcle.lateral.theta_aL  = -0.000145;
	vehcle.lateral.theta_aR  = 0.000141;
	vehcle.lateral.theta_r   = 0.004425;
	vehcle.lateral.theta_bcu = -0.006646;
	vehcle.lateral.beta0     = 12.00000;
	vehcle.lateral.kr        = 0.0;
	vehcle.lateral.kaf       = 0.0;
	vehcle.lateral.kcu       = 0.0;
	vehcle.lateral.kalpha    = 0.0;
	vehcle.lateral.kv        = -1.370707;
	vehcle.lateral.kps_r     = 0.606335;
	vehcle.lateral.M         = 59.998281;
	vehcle.lateral.CYp     = 0.156542;
	vehcle.lateral.CYr     = 0.364915;

	// Rolling moment
	vehcle.roll.theta0     = -0.000102;
	vehcle.roll.theta_aL   = 0.000638;
	vehcle.roll.theta_aR   = -0.000677;
	vehcle.roll.theta_aLcu = 0.000108;
	vehcle.roll.theta_aRcu = -0.000094;
	vehcle.roll.theta_b    = -0.000381;
	vehcle.roll.theta_b_cu = 0.000069;
	vehcle.roll.theta_r    = 0.000497;
	vehcle.roll.Clp    = -0.642739;
	vehcle.roll.Clr    =  0.398063;

	// Pitching momet
	vehcle.pitch.Cmq = -29.186868;

	// Yawing moment 
	vehcle.yaw.theta0     = 0.000281;
	vehcle.yaw.theta_b    = 0.003317;
	vehcle.yaw.theta_aL   = -0.000016;
	vehcle.yaw.theta_aR   = 0.000001;
	vehcle.yaw.theta_r    = -0.001842;
	vehcle.yaw.theta_bcu  = -0.000090;
	vehcle.yaw.theta_aLcu = -0.000029;
	vehcle.yaw.theta_aRcu = 0.000027;
	vehcle.yaw.Cnp        = -0.139344;
	vehcle.yaw.Cnr        = -0.162811;
	vehcle.yaw.beta0      = 11.999974;
	vehcle.yaw.kr         = 0.0;
	vehcle.yaw.kaf        = 0.0;
	vehcle.yaw.kcu        = 0.000009;
	vehcle.yaw.kalpha     = 0.0;
	vehcle.yaw.kv         = 0.290114;
	vehcle.yaw.kps_r      = 0.655515;
	vehcle.yaw.M          = 59.998281;

	// Propulsion
	vehcle.prop.cmyu_J2  = 0.53211;
	vehcle.prop.cmyu_0   = 0.03669;
	vehcle.prop.CT_1     = 0.691683;
	vehcle.prop.CT_J     = -0.734466;
	vehcle.prop.CT_JM    = 1.64705;
	vehcle.prop.J_min    = 1e-3;
	vehcle.prop.J_max    = 5;
	vehcle.prop.Cmyu_max = 9.21;

	// CG (non-dimensional)
	vehcle.cg.x_cg_c       = 0.4951;        // (x_cg 703 - wing LE 500)/c = 203/410, aft of wing LE
	vehcle.cg.x_cg_tac_abs = 1.5/vehcle.c;  // |x_cg 703 - x_ac_ht 2203|/c = 1500/410 (tail arm 1.5 m)
	vehcle.cg.z_cg         = 0.25;          // CG height above A/C base (z_cg_from_ground = 250 mm) [m]
}



void v_plane_param_define_ardupilot_default()
{
	s_servo_manager.num_servos = 16; //  put max servo enum that is used
	s_motor_manager.num_motors = 8;

	//v_set_servo_params(pwm_min, pwm_max, angle_pwm_min, angle_pwm_max,omega, zeta, min_rate, max_rate, min_accel, max_accel,  type)
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, AILERON_COMMON);
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, ELEVATOR_COMMON);
	v_set_servo_params(1100,1900, 30*D2R,-30*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, RUDDER_COMMON);

	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, AILERON_LEFT);
	v_set_servo_params(1100,1900, 30*D2R,-30*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, AILERON_RIGHT);

	v_set_servo_params(1100,1900,-40*D2R,40*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, FLAP);
		
	v_set_servo_params(1100,1900,-30*D2R,30*D2R, 10.0, 0.7, -1000*D2R, 1000*D2R, -720*D2R, 720*D2R, NOSE_LG_SERVO);

	vehcle.ground_yaw_gain = 0.5; 
	vehcle.cg_x = 1070.0/1000.0; // from nose center, put positve number
	vehcle.cg_z = 43.0/1000.0;// from nose center, 43 mm above  nose center
	vehcle.MLG_x = fabsf( (1319.2/1000.0) - vehcle.cg_x); // put all positive
	vehcle.MLG_z = fabsf(319.6/1000.0 + vehcle.cg_z); // put all positive
	vehcle.FLG_x = fabsf((339.2/1000.0) - vehcle.cg_x);
	vehcle.FLG_z = vehcle.MLG_z;
    vehcle.delta_r_deadzone = 3.0*D2R; // 5 degree deadzone for rudder on ground
	vehcle.theta_tolerance_for_ground = 0.0*D2R; // pitch angle below which plane is shifted to runway moving state
	vehcle.altitude_tolerance_for_ground = -0.1; // altitude below which plane is considered on ground

	vehcle.mg_b[0]=0;
	vehcle.mg_b[1]=0;
	vehcle.mg_b[2]=0;
	// Ensure coffecient signs are compatible with aerospace sign convention for elevator, aileron, rudder
	vehcle.aero_zero_speed = 5.0; // m/s
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
	vehcle.CD_alpha = 0*0.3;
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
	vehcle.Cm_q = -60.0;
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

	for (int i=0;i<s_motor_manager.num_motors;i++)
	{
		s_motor[i].pwm_min = 1100;
		s_motor[i].pwm_max = 1900;
		s_motor[i].omega_tf = 30.0; // rad/s
		s_motor[i].zeta_tf = 0.7;
		s_motor[i].rpm_max = 36000.0;
		s_motor[i].rpm_min = 0.0;
		s_motor[i].dia_prop = 0.12;
		s_motor[i].max_thrust = (vehcle.mass*vehcle.g/s_motor_manager.num_motors)*0.8;	
		s_motor[i].min_thrust = 0.0;
		s_motor[i].thrust_2_torque_factor =0; // JP Hobby EDFs kind of not produces any torque
		s_motor[i].CT_static = s_motor[0].max_thrust / (vehcle.rho*powf(s_motor[0].dia_prop,4)*powf(s_motor[0].rpm_max/60.0f,2));;
		s_motor[i].rate_limit_throttle = (1.0/0.01); // full throttle change in 0.1 sec
	}

	float y1= 0.274*0;
	float y2= 0.598*0;
	float y3 = 0.922*0;
	float y4 = 1.246*0;

	float z = 0;//0.0948; //cg to edf, cg is above edf

	float x = 0;
	x= 0*(vehcle.cg_x - 757.2/1000.0); // edf are 30 mm behind cg

	s_motor[0].rotor_xyz[0] = x;                     s_motor[1].rotor_xyz[0] = x;
	s_motor[0].rotor_xyz[1] =-y4;                    s_motor[1].rotor_xyz[1] = -y3;
	s_motor[0].rotor_xyz[2] = z;                     s_motor[1].rotor_xyz[2] = z;
	
	s_motor[2].rotor_xyz[0] = x;                     s_motor[3].rotor_xyz[0] = x;
	s_motor[2].rotor_xyz[1] = -y2;                   s_motor[3].rotor_xyz[1] = -y1;
	s_motor[2].rotor_xyz[2] = z;                     s_motor[3].rotor_xyz[2] = z;

	s_motor[4].rotor_xyz[0] = x;                     s_motor[5].rotor_xyz[0] = x;
	s_motor[4].rotor_xyz[1] = y1;                   s_motor[5].rotor_xyz[1] = y2;
	s_motor[4].rotor_xyz[2] = z;                     s_motor[5].rotor_xyz[2] = z;	

	s_motor[6].rotor_xyz[0] = x;                     s_motor[7].rotor_xyz[0] = x;
	s_motor[6].rotor_xyz[1] = y3;                   s_motor[7].rotor_xyz[1] = y4;
	s_motor[6].rotor_xyz[2] = z;                     s_motor[7].rotor_xyz[2] = z;

	
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


