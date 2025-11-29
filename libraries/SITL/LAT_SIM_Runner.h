
#ifndef LAT_SIM_RUNNER_H_
#define LAT_SIM_RUNNER_H_

#include <stdio.h>
#include "math.h"
#include "LAT_SIM_math_util.h"
#include "SITL_Input.h"
#include "stdint.h"
#include "string.h"
#include "LAT_SIM_Conversions_Frame_rotations.h"

// Forward declaration
namespace SITL {
    class Aircraft;
}


#define PI 3.141592653589793

#define LAT_EQUINOX
#define fwv_motors		 	 1
#define num_actuator		 4

#define pi 3.14159265
#define R2D 57.2957795
#define D2R 0.0174532925

extern void pseudo_INS(float,float[],float[],float[],float*,float*,float*,float[],float[],float[],float[],float[],float);
void rk4(float y[],float t,float h,float[]);
void derivative(float [],float ,float[],float[]);
extern float Angle_Ranges(float);

extern float delay_array_T[];
extern float delay_array_T1[];
extern float Delay_input_fn_T(float);
extern int delay_array_length;

extern int delay_array_length_pwm_servo ;   // delay_array_length =  (int)(Plant_freq*delay_in_seconds);
extern uint16_t bufferarray_pwm_servo[2000][num_actuator] ;

extern int delay_array_length_pwm;
extern uint16_t bufferarray_pwm[][fwv_motors];

typedef enum
{
	STATIONARY=0,
	CT_RUNWAY_MOVING=1,
	CT_RUNWAY_ROTATING=2,
	IN_AIR=3,	
}enum_plane_moving_state;

typedef struct
{
	float lat;
	float longt;
	float alt_msl;
	float alt_agl;
}strct_home_states;

typedef enum
{
	PLANE_ARDU_DEFAULT = 0,
	PLANE_EQX = 1,
}struct_enum_plane_model;

typedef enum
{
	DOF_ALL_MOTION = 0,
	DOF_ROLL = 1,
	DOF_PITCH = 2,
	DOF_YAW = 3,
	DOF_ROLL_PITCH = 4,
	DOF_ROLL_YAW = 5,
	DOF_PITCH_YAW  = 6,
	DOF_ROLL_PITCH_YAW = 7,
	DOF_HORIZONTAL_MOTION = 8,
	DOF_VERTICAL_MOTION = 9,
}struct_enum_dof;


#pragma pack(1)
typedef struct
{
	float V_b_gnd[3] = {0, 0, 0};
	float V_ned_gnd[3] = {0, 0, 0};
	float V_ned_tas[3] = {0, 0, 0};
	float V_b_tas[3] = {0, 0, 0};

	float Accel_b[3] = {0, 0, 0};
	float Accel_ned[3] = {0, 0, 0};

	float Accel_b_gs[3] = {0, 0, 0};
	float Accel_ned_gs[3] = {0, 0, 0};

	float lat;
	float lon;
	float alt_msl;
	float alt_agl;

	float pos_ned[3]={0,0,0};


	float phi;
	float theta;
	float psi;

	float course;

	float alpha;
	float beta;
	float gamma;

	float wind_ned[3] = {0.0, 0.0, 0.0};

	float CL,CD,CY,Cl,Cm,Cn;
	float p, q, r;

    float Q;
	float tas;
	float gs;

	float hrz_gnd_speed;
	float vert_gnd_vel;

	float omega;
	float zeta;

	float sound_speed;

	float pressure;

	float rho;


	float all_lift_force;
	float all_side_force;
	float all_drag_force;

	float mg_b[3] = {0, 0, 0};
	float L_b[3] = {0, 0, 0};
	float D_b[3] = {0, 0, 0};

	float all_aero_force[3] = {0, 0, 0}, all_aero_moment[3] = {0, 0, 0}, all_payload_moment[3] = {0, 0, 0};
	float all_rotors_force[3] = {0, 0, 0}, all_rotors_moment[3] = {0, 0, 0}, all_payload_force[3] = {0, 0, 0};

	float alpha_stall;

    float delta_f;
    float delta_e;
    float delta_a;
    float delta_r;
    float delta_aL;
    float delta_aR;
    float AR;
    float e;
    float Cmu;
    float s;
    float s_blown;
    float t_by_c;
    float Ixx;
    float Iyy;
    float Izz;
    float Ixz;
	float Ixy;
	float Iyz;
    float c;
    float b;
    float mass;

    float CL_0;
    float CL_delta_e;
	float CL_delta_a;
	float CL_flap;
	float CL_delta_r;
	float CL_alpha;
	float CL_q;


    float CD_0;
    float CD_delta_e;
    float CD_delta_f;
	float CD_delta_r;
	float CD_delta_a;
    float CD_delta_e2;
	float CD_alpha;
	float CD_q;

    float CY_0;
    float CY_beta;
    float CY_delta_r;
	float CY_delta_a;
    float CY_delta_aL_Cmu;
    float CY_delta_aR_Cmu;
    float CY_delta_aL;
    float CY_delta_aR;
    float CY_beta_Cmu;
	float CY_r;
	float CY_p;


    float Cl_0;
    float Cl_beta;
    float Cl_delta_r;
    float Cl_delta_aL_Cmu;
    float Cl_delta_aR_Cmu;
	float Cl_delta_a;
    float Cl_delta_aL;
    float Cl_delta_aR;
	float Cl_p;
	float Cl_r;


    float Cm_0;
    float Cm_alpha;
    float Cm_delta_e;
    float Cm_delta_aL;
    float Cm_delta_aR;
    float Cm_Cmu;
	float Cm_delta_a;
    float Cm_alpha_Cmu;
    float Cm_delta_f;
    float Cm_beta2;
    float Cm_beta2_Cmu;
	float Cm_q;
	

    float Cn_0;
    float Cn_beta;
    float Cn_delta_r;
	float Cn_delta_a;
    float Cn_delta_aL_Cmu;
    float Cn_delta_aR_Cmu;
    float Cn_delta_aL;
    float Cn_delta_aR;
    float Cn_beta_Cmu;
	float Cn_p;
	float Cn_r;

	float mlgL_x;
	float mlgL_y;
	float mlgL_z;
	float mlgR_x;
	float mlgR_y;
	float mlgR_z;
	float nlg_x;
	float nlg_y;
	float nlg_z;

	float cg_x;
	float cg_y;
	float cg_z;
	float ground_yaw_gain;
	float MLG_x;
	float MLG_z;
	float FLG_x;
	float FLG_z;
	float MLG_NR;
	float FLG_NR;
	float delta_r_deadzone;

	float total_force_bd[3] = {0, 0, 0};
	float total_moment_bd[3] = {0, 0, 0};

	float g;
	float lift_stall_M;

	int plane_on_ground;
	int flag_sensor_input_delay;

	float aero_zero_speed;
	float CL_alpha_tot;
	float theta_tolerance_for_ground;
	float altitude_tolerance_for_ground;
	float step_dt;
	float p_dot,q_dot,r_dot;

	struct_enum_plane_model plane_model;
	enum_plane_moving_state plane_moving_state;
	
	struct_enum_dof dof;
}vehcle_STATES;

extern vehcle_STATES vehcle;

extern strct_home_states s_home_state;

extern void v_lat_fdm_init();
extern void v_lat_fdm_run(const struct sitl_input &input);
extern void v_set_aircraft_instance(SITL::Aircraft* aircraft);
extern void v_init_vehcle_states();
extern void v_update_vehcle_states(float state[]);
extern void v_fill_lla_to_vehcle_state(float, float , float );
extern void v_plane_param_define();
extern void v_plane_param_define_equinox();
extern void v_plane_param_define_ardupilot_default();
extern void v_update_vehcle_state_from_ardu_ekf();

#endif