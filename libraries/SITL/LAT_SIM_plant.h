#ifndef PLANT_H
#define PLANT_H
#include <stdio.h>
#include "math.h"
#include "math_util.h"
#include "common_variable.h"
#include "stdint.h"


#define NUM_STATEVARS 15
extern float current_time;
#define PI 3.141592653589793

#define Quadplane
#ifdef Quadplane
//	#define Plane
#endif
//#define Quad_H
//#define Quad_X
//#define Quad_+
//#define Coax_Quad_X
//#define Coax_Hexa_H_sym
//#define Octa

#ifdef Quadplane
#define quad_num_motors		 4
#define fwv_motors		 	 1
#define num_actuator		 4
#endif

#ifdef Quad_H
#define quad_num_motors		 4
#define fwv_motors		 	 0
#define num_actuator		 0
#endif

//#define XPLANE_IN_LOOP /******************Xplane for SITL mode***********/
//#define XPLANE_IN_LOOP_VISUALISATION /*********** only for xplane visualisation************/
//#define OBC_IN_LOOP_BLOCK	// NRT mode OBC in loop
//#define PRINT_DEBUG1		// Developmental testing, Not for test team/production, remove before gating
//#ifdef PRINT_DEBUG1
//	 #define PRINT_DEBUG2
//#endif


extern void ins_derivative(float y[],float,float[]);
extern void rk4_ins(float[],float,float);
extern void pseudo_INS(float,float[],float[],float[],float*,float*,float*,float[],float[],float[],float[],float[],float);
void rk4(float y[],float t,float h,float[]);
void derivative(float [],float ,float[],float[]);
extern float Angle_Ranges(float);
//============================================================================================//

//extern void rotor_dynamics(float,float,float,float,float,float*,float*,float*,float*,float*);
void rotor_dynamics(float);
void Actuator_dynamics(float);
void init_plant();
//===========test function============================================//
extern void test_ins_state_update_every_hundred_cycle(float [],float []);
void rk4(float y[],float t,float h,float[]);
extern FILE* fp_omega;
extern FILE* fp_delta;
extern FILE* fp_derivative;
extern FILE* fp_rot_dyn;
extern float t, t_step_plant, t_step_act, t_step_rot, t_step_imu;

extern float delay_array_T[];
extern float delay_array_T1[];
extern float Delay_input_fn_T(float);
extern int delay_array_length;

extern int delayed_ins_use;

extern uint16_t pwm_in[quad_num_motors + fwv_motors];
extern uint16_t pwm_out_esc[quad_num_motors + fwv_motors];
extern uint16_t mil_rpm[quad_num_motors + fwv_motors];

extern uint16_t gnc_rpm_in[quad_num_motors + fwv_motors];
extern float rotor_force_out[quad_num_motors + fwv_motors];
extern float rotor_speed[quad_num_motors + fwv_motors];


extern uint16_t pwm_in_servo[num_actuator];
extern uint16_t pwm_out_servo[num_actuator];


extern int delay_array_length_pwm_servo ;   // delay_array_length =  (int)(Plant_freq*delay_in_seconds);
extern uint16_t bufferarray_pwm_servo[2000][num_actuator] ;

extern int delay_array_length_pwm;
extern uint16_t bufferarray_pwm[][quad_num_motors + fwv_motors];


extern void v_fill_data_out_for_socket(float ,float[],float[] ,float[],float[],float[]);


void pure_transport_delay_int(uint16_t out[],uint16_t new[],uint16_t bufferarray[][quad_num_motors + fwv_motors + num_actuator],int len, int delay_array_length);
extern void pure_transport_delay_int_servo(float[],uint16_t[],float[][5],int, int);
extern void pure_transport_delay_float(float[],float[][quad_num_motors + fwv_motors + num_actuator],int, int);

extern void esc_dynamics();
extern void battery_dynamics();
extern void Configure_rotor_geometry();

extern int flg_sitl_mode;

void fn_uav_states_init(int,float* latitude_point,float* longitude_point,float* Alt,float *V_bd_ins ,float* V_ned_ins,float* ax_bd_ins,float* Body_rate_bf_ins,float* attitude);


typedef struct
{
	float lat;
	float longt;
	float alt_msl;
	float alt_agl;
}strct_home_states;

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

typedef struct
{
	float V_b_gnd[3];
	float V_ned_gnd[3];
	float V_ned_tas[3];
	float V_b_tas[3];

	float Accel_b[3];
	float Accel_ned[3];

	float Accel_b_gs[3];
	float Accel_ned_gs[3];


	float lat;
	float lon;
	float alt_msl;
	float alt_agl;

	float pos_ned[3];


	float phi;
	float theta;
	float psi;

	float course;

	float alpha;
	float beta;
	float gamma;

	float delta_e;
	float delta_r;
	float delta_a;

	float wind_ned[3];

	float CLo, CL_alpha, CL_q;
	float CDo, CD_alpha;
	float CYo, CY_beta, CY_p, CY_r;
	float Clo, Cl_beta, Cl_p, Cl_r;
	float Cmo, Cm_alpha, Cm_q;
	float Cno, Cn_beta, Cn_p, Cn_r;
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
	float L_b[3],D_b[3],mg_b[3];

	float all_aero_force[3], all_aero_moment[3], all_payload_moment[3];
	float all_rotors_force[3], all_rotors_moment[3],all_payload_force[3];

	struct_enum_dof dof;
}VEHICLE_STATES;

extern VEHICLE_STATES vehicle;

extern strct_home_states s_home_state;

extern void v_param_init_aarav();
extern void v_param_init_Abhay_12R10();
extern void v_param_init_Talon_quad();
extern void v_param_init_UT1();
#endif
