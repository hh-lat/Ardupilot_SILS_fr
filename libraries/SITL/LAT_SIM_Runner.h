
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
	PLANE_EQX_V1_NEW_MODEL = 2,
	PLANE_USTOL_V1 = 3,
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
	DOF_LONGITUDINAL_ONLY = 10,
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
	float CL_w;   // uSTOL: wing lift component (set in lift, reused by drag & pitch)
	float CL_t;   // uSTOL: tail lift component
	float theta_tolerance_for_ground;
	float altitude_tolerance_for_ground;
	float step_dt;
	float p_dot,q_dot,r_dot;

	struct_enum_plane_model plane_model;
	enum_plane_moving_state plane_moving_state;

	struct_enum_dof dof;

	// =====================================================================
	//  uSTOL v1 aerodynamic parameters (PLANE_USTOL_V1)
	//  DECLARATIONS ONLY — values are assigned in v_plane_param_define_ustol_v1().
	//  Grouped to mirror the MATLAB geom_ac / geom_ad component structs, and
	//  declared as direct (un-wrapped) members so the switch case can set them
	//  as vehcle.wing.k_fit = ... etc.
	//  Shared physical params (mass, Ixx.., s, b, c, AR, t_by_c, rho) are NOT
	//  duplicated here — they reuse the existing vehcle fields above, because
	//  the common EOM / force-assembly code reads those by name.
	// =====================================================================

	// Wing (uSTOL-only geometry + blown-lift / induced-drag fits)
	struct {
		float lambda_b;     // blown fraction (0.503*0.8) [-]
		float S_f;          // flap area    [m^2]
		float S_a;          // aileron area [m^2]
		float k_fit;        // Spence amplitude scaling [-]
		float cl0_camber;   // zero-alpha camber lift (2-D) [-]
		float k_w;          // induced drag factor 1/(pi*e) [-]
		float r;            // blown-wing momentum drag coeff [-]
	} wing;

	// Horizontal tail
	struct {
		float a_t;          // 3-D lift-curve slope [1/rad]
		float eta_t_0;      // tail efficiency offset [-]
		float eta_t_1;      // tail efficiency Cmyu slope [-]
		float eps0;         // zero-AoA downwash [rad]
		float deps_pos;     // d(eps)/d(alpha), alpha >= 0 [-]
		float deps_neg;     // d(eps)/d(alpha), alpha <  0 [-]
		float k_ht;         // HT induced drag factor [-]
		float CD_ht0;       // HT zero-lift drag [-]
		float S_ht_S;       // S_ht / S [-]
		float V_H;          // tail volume coefficient [-]
		float AR_ht;        // tail aspect ratio [-]
	} tail;

	// Fuselage + baseline parasite drag
	struct {
		float CD_a2;        // alpha^2 coeff [rad^-2]
		float CD_b2;        // beta^2 coeff  [rad^-2]
		float CD_a2_Cmyu;   // alpha^2*Cmyu cross term [rad^-2]
		float CD0;          // zero-lift parasite drag [-]
		float CDp_cu;       // d(CD)/d(p*Cmyu) [per rate]
		float CDq_cu;       // d(CD)/d(q*Cmyu) [per rate]
		float CDr_cu;       // d(CD)/d(r*Cmyu) [per rate]
	} fuse;

	// Control-surface effectiveness, control drag, and deflection limits
	struct {
		float tau_f;        // 2-D Fowler-flap effectiveness [-]
		float tau_a;        // 2-D aileron effectiveness [-]
		float Kb_f;         // Fowler-flap span effectiveness [-]
		float Kb_a;         // aileron span effectiveness [-]
		float tau_e;        // 3-D elevator effectiveness [-]
		float CD_df2;       // delta_f^2 [rad^-2]
		float CD_da2;       // delta_a^2 [rad^-2]
		float CD_da;        // delta_a (linear) [rad^-1]
		float CD_de2;       // delta_e^2 [rad^-2]
		float CD_de;        // delta_e (linear) [rad^-1]
		float CD_dr2;       // delta_r^2 [rad^-2]
		float del_e_max, del_e_min;     // elevator limits [deg]
		float del_a_max, del_a_min;     // aileron limits  [deg]
		float del_r_max, del_r_min;     // rudder limits   [deg]
		float del_thr_max, del_thr_min; // throttle limits [-]
		float alpha_max, alpha_min;     // AoA limits      [deg]
		float beta_max, beta_min;       // sideslip limits [deg]
	} controls;

	// Post-stall drag (flat-plate surrogate + stall blend)
	struct {
		float K_flat;       // flat-plate drag scaling [-]
		float a0_const;     // stall-angle offset [deg]
		float a0_Cmyu;      // stall-angle Cmyu sensitivity [deg]
		float k;            // blend sharpness [1/rad]
	} stall;

	// Post-stall LIFT blend (wing + rest -> flat-plate surrogates; Beard)
	struct {
		float wM0, wM1;          // wing blend sharpness: M = wM0 + wM1*Cmu [1/rad]
		float wa00, wa0mu, wa0f; // wing onset: (wa00 + wa0mu*Cmu + wa0f*[flap=32]) [deg]
		float wkflat;            // wing flat-plate (sin 2a) scale [-]
		float rM;                // rest blend sharpness [1/rad]
		float ra0, rkcu;         // rest onset: (ra0 + rkcu*Cmu) [deg]
		float rkflat;            // rest flat-plate scale [-]
	} cl_stall;

	// Lateral side force (CY; fit in degrees)
	struct {
		float theta0, theta_b, theta_aL, theta_aR, theta_r, theta_bcu;
		float beta0, kr, kaf, kcu, kalpha, kv, kps_r, M;
		float CYp, CYr, CYp_cu, CYr_cu, CYp2_cu, CYr2_cu;
	} lateral;

	// Rolling moment (Cml; fit in degrees)
	struct {
		float theta0, theta_aL, theta_aR, theta_aLcu, theta_aRcu;
		float theta_b, theta_b_cu, theta_r;
		float Clp, Clr, Clp_cu, Clr_cu;
	} roll;

	// Yawing moment (Cn; fit in degrees)
	struct {
		float theta0, theta_b, theta_aL, theta_aR, theta_r, theta_bcu;
		float theta_aLcu, theta_aRcu, Cnp, Cnp_cu, Cnr, Cnr_cu;
		float beta0, kr, kaf, kcu, kalpha, kv, kps_r, M;
	} yaw;

	// Propulsion (blowing momentum coeff & thrust coeff fits)
	struct {
		float cmyu_J2;      // 1/J^2 coeff [-]
		float cmyu_0;       // constant offset [-]
		float CT_1;         // CT constant [-]
		float CT_J;         // CT J term [-]
		float CT_JM;        // CT J*Mtip term [-]
		float J_min;        // advance-ratio lower clamp [-]
		float J_max;        // advance-ratio upper clamp [-]
		float Cmyu_max;     // blowing-coeff upper clamp [-]
	} prop;

	// CG / balance (NON-dimensional — distinct from the dimensional cg_x/cg_y/cg_z above)
	struct {
		float x_cg_c;       // x_cg / c, from wing LE, aft + [-]
		float x_cg_tac_abs; // |x_cg - x_ac_t| / c [-]
		float z_cg;         // CG height above A/C base, up + [m]
	} cg;

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
extern void v_plane_param_define_eqx_v1_new_model();
extern void v_plane_param_define_ustol_v1();
extern void v_plane_param_define_ardupilot_default();
extern void v_update_vehcle_state_from_ardu_ekf();

#endif
