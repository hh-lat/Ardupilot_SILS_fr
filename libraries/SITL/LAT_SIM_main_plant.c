//#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <netinet/in.h>
#include <linux/types.h>
#include <unistd.h>
#include <math.h>

#ifdef DEBUG
#include <fenv.h>
#endif

#include "plant.h"
#include "aero.h"
#include "common_variable.h"
#include "math_util.h"
#include "Forces_and_moments_rotors.h"
#include "pseudo_ins.h"
#include "rt.h"
#include "log/srl_log_common.h"
#include "measure.h"
#include "srl_io.h"
#include "iface_plant.h"
#include "xplane_sil.h"
#include "Ground_model.h"
#include "rotor_dynamics.h"
#include "Actuator_dynamics.h"
#include "data_out_socket.h"

struct sched_attr attr;

#define LOG_BYTES sizeof(log_struct)

#define SIZE_CRC 2
#define SIZE_HDR 2

#pragma pack(1)

static int sched_setattr(pid_t pid,const struct sched_attr *attr,unsigned int flags)
{
	return syscall(__NR_sched_setattr, pid, attr, flags);
}

static int sched_getattr (pid_t pid, struct sched_attr *attr,unsigned int size, unsigned int flags)
{
	return syscall (__NR_sched_getattr, pid, attr, size, flags);
}



int key , shm_id;
VEHICLE_STATES vehicle;



char string_1[] = "XPLANE_MODEL_RUNNING";
char string_2[] = "XPLANE_VISUALISATION(RK4)";
char string_3[] = "RK4_ONLY";

volatile float t_debug1 = 0, t_debug2 = 0;
volatile unsigned int g_ui_dbg_var_1 = 0, g_ui_dbg_var_2 = 0;
float time_gnc = 0.0f;

int rx_bytes_p0 = 0, rx_bytes_temp_p0 = 0,tx_bytes = 0, tx_bytes_temp = 0, rx_loops_done_p0 = 0;
int rx_bytes_p1 = 0, rx_bytes_temp_p1 = 0,rx_loops_done_p1 = 0;
int ptr = 0;

struct Sturct_Gnc_Out s_gnc_out;

struct param_send_1000Hz s_send_param_1000hz;
struct param_send_200Hz s_send_param_200hz;
struct param_send_100Hz s_send_param_100hz;

int rx_bytes_count_temp = 0 ,rx_bytes_count = 0, receiving_loop_done = 0;
volatile unsigned char rx_data_temp_buffer[sizeof(s_gnc_out)], rx_data_buffer[sizeof(s_gnc_out)] , rx_data_final_buffer[sizeof(s_gnc_out)];
_Bool rx_data_incomplete = 0;



volatile unsigned char rx_data_p0[sizeof(s_gnc_out)], rx_data_p0_buf[sizeof(s_gnc_out)], rx_data_p0_prev[sizeof(s_gnc_out)], rx_data_p0_buf_temp[sizeof(s_gnc_out)];
volatile unsigned char rx_data_p1[200], rx_data_p1_buf[200], rx_data_p1_prev[200], rx_data_p1_buf_temp[280];
unsigned char incomplete_p0 = 0;

struct struct_plant_dat s_plant_tx_data;
unsigned char tx_data_buf[sizeof(s_plant_tx_data)-2], tx_data_init[sizeof(s_plant_tx_data)-2];

unsigned char framesync = 0;
int ctr_obc_data_none = 0, ctr_obc_data_hdr_correct = 0;
unsigned char *plant_log_addr, *plant_log_addr2;
unsigned char *shm_current_addr, *shm_current_addr2;
unsigned char tx_crc[SIZE_CRC], rx_crc[SIZE_CRC];
unsigned char tx_hdr[SIZE_HDR] = {0x70,0x52}, rx_hdr[SIZE_HDR];

struct ins_data ins_state_plant_out;
strct_home_states s_home_state;

unsigned int ins_data_miss = 0;


int flg_sitl_mode;

int received_gnc_count = 0 ;

float t_step_plant = 0.0f;

int N = 1;
float t_step_act = 0.0f;
float t_step_rot = 0.0f;
float t_step_ins = 0.0f;
FILE *fp_plant   = NULL;
struct shm_plant  *plant_plot_logptr = NULL;

uint16_t pwm_in[quad_num_motors + fwv_motors] = {0};
uint16_t pwm_old[quad_num_motors + fwv_motors] = {0};
uint16_t mil_rpm[quad_num_motors + fwv_motors] = {0};
uint16_t gnc_rpm_in[quad_num_motors + fwv_motors] = {0};
uint16_t pwm_out_esc[quad_num_motors + fwv_motors] = {0};
extern float rotor_force_out[quad_num_motors + fwv_motors] = {0.0f};
float rotor_speed[quad_num_motors + fwv_motors] = {0.0f};

uint16_t pwm_in_servo[num_actuator]={0},pwm_out_servo[num_actuator]={0};


float delay_array_T[2000] = {0.0f};
float delay_array_T1[2000] = {0.0f};
int delay_array_length    = 1;
int delayed_ins_use  = 0;

int delay_array_length_pwm = (int)(1000.0 * 0.06f) ;  // delay_array_length =  (int)(Plant_freq*delay_in_seconds);
uint16_t bufferarray_pwm[2000][quad_num_motors + fwv_motors] = {0};
int delay_array_length_pwm_servo = (int)(1000.0 * 0.06f) ;  // delay_array_length =  (int)(Plant_freq*delay_in_seconds);
uint16_t bufferarray_pwm_servo[2000][num_actuator] = {0};

float Plane_state[NUM_STATEVARS];
float ideal_plane_state[NUM_STATEVARS];
float Ins_states[9];

uint16_t fletcher16_checksum(const uint8_t *data, size_t len)
{
	uint32_t c0 = 0, c1 = 0;
	unsigned int i;
	for (i = 0; i < len; ++i)
	{
		c0 = c0 + *data++;
		c1 = c1 + c0;
	}
	c0 = c0 % 255;
	c1 = c1 % 255;
	return (c1 << 8 | c0);
}


void init_common_variable()
{

#ifdef Coax_Hexa_H_sym
#endif

#ifdef 	Quad_H
	v_param_init_fwv();
#endif

#ifdef Quadplane
	v_param_init_fwv();
#endif

#ifdef Octa
#endif


	// ESC Model high level params
	delay_array_length_pwm = (int)(1000.0*0.01f) ; // delay_array_length =  (int)(Plant_freq*delay_in_seconds)

	// Servo pwm model high level params
	delay_array_length_pwm_servo = (int)(1000.0*0.01f);// delay_array_length =  (int)(Plant_freq*delay_in_seconds)

	// Sensor Model high level params
	delayed_ins_use    = 0; // used in old pseudo ins code, need to delete it
	flag_sensor_input_delay   = 0 ;
	flag_sensor_input_inacc   = 0 ;
	flag_sensor_input_hfnoise = 0 ;
	delay_array_length_pseuso_ins = (int)(1000.0*0.1f) ;  // delay_array_length =  (int)(Plant_freq*delay_in_seconds)


	// Sensor Model low level params
	fn_sensor_initialise_pseudo_ins();

	b1_inv = 1.0f/b1;
	d_by_b = d1*b1_inv;

#ifdef Quadplane
	b1_fwv_inv = 1.0/b1_fwv;
	d_by_b_fwv = d1_fwv*b1_fwv_inv;
#endif

	// UAV Inertias
	c0 = Ix * Iz -(Ixz * Ixz);
	c1 = Ixz * (Ix - Iy + Iz)/c0;
	c2 = (Iz * (Iz - Iy) + (Ixz * Ixz))/c0;
	c3 = Iz/c0;
	c4 = Ixz/c0;
	//c5=(Iz-Ix)/c0;
	c5 = (Iz - Ix)/Iy;// changed on 24 April 2021
	c6 = Ixz/Iy;
	c7 = ((Ix - Iy) * Ix +(Ixz * Ixz))/c0;
	c8 = Ix/c0;
	c3_inv = 1.0/c3;
	c8_inv = 1.0/c8;

	// sets DOF of vehicle
	v_init_vehicle_states();

	// Ground param init
	v_ground_model_param_init();

	// dynamic models init
	v_param_init_esc_dynamics();
	v_param_init_battery_dynamics();
	v_param_init_rotor();

	// Wind param init
	v_param_init_wind();

	// Simulation runs in real time for (t1-t) seconds
	t = 0.0;
	t1 = 10.0f * 24.0f * 60.0f * 60.0f; // 10 days
}

void init_plant()
{
	Configure_rotor_geometry();

	flg_sitl_mode = 0; // OBC in loop
#ifdef XPLANE_IN_LOOP
	flg_sitl_mode =1; // Xplane
#endif

	t_step_plant = 0.001;
	t_step_rot = 0.001;
	t_step_act = 0.001;
	t_step_ins = 0.001;

#ifdef XPLANE_IN_LOOP
	fn_xplane_udp_open();
	fn_xplane_init();
#endif

#ifdef XPLANE_IN_LOOP_VISUALISATION
	fn_xplane_udp_open();
#endif
}

void fn_sensor_initialise_pseudo_ins()
{
	sensor_accuracy.phi   = 2.0f/57.3f; // in rad
	sensor_accuracy.theta = 2.0f/57.3f;
	sensor_accuracy.psi   = 2.0f/57.3f;
	sensor_accuracy.p     = 3.2f/57.3f; // in rad/sec
	sensor_accuracy.q     = 3.2f/57.3f;
	sensor_accuracy.r     = 3.2f/57.3f;
	sensor_accuracy.u     = 2.2f;  // in m/sec
	sensor_accuracy.v     = 0.2f;
	sensor_accuracy.w     = 0.2f;
	sensor_accuracy.lat   =  0.0000001f; // in degrees
	sensor_accuracy.longt = 0.0000001f;
	sensor_accuracy.z     = 2.1f;       // in m
	sensor_accuracy.ax    = 0.05f; // in m/sec^2
	sensor_accuracy.ay    = 0.05f;
	sensor_accuracy.az    = 0.05f;



	sensor_inacc_lpff.phi   = (1.0/(2.0*PI*10.0));   // in secs
	sensor_inacc_lpff.theta = (1.0/(2.0*PI*10.0));
	sensor_inacc_lpff.psi   = (1.0/(2.0*PI*10.0));
	sensor_inacc_lpff.p     = (1.0/(2.0*PI*20.0));
	sensor_inacc_lpff.q     = (1.0/(2.0*PI*20.0));
	sensor_inacc_lpff.r     = (1.0/(2.0*PI*20.0));
	sensor_inacc_lpff.u     = (1.0/(2.0*PI*20.0));
	sensor_inacc_lpff.v     = (1.0/(2.0*PI*5.0));
	sensor_inacc_lpff.w     = (1.0/(2.0*PI*5.0));
	sensor_inacc_lpff.lat   = (1.0/(2.0*PI*1.0));
	sensor_inacc_lpff.longt = (1.0/(2.0*PI*1.0));
	sensor_inacc_lpff.z     = (1.0/(2.0*PI*1.0));
	sensor_inacc_lpff.ax    = (1.0/(2.0*PI*20.0));
	sensor_inacc_lpff.ay    = (1.0/(2.0*PI*20.0));
	sensor_inacc_lpff.az    = (1.0/(2.0*PI*20.0));


	sensor_noise_hf_amp.phi 	= 0.5/57.3f;
	sensor_noise_hf_amp.theta 	= 0.5/57.3f;
	sensor_noise_hf_amp.psi 	= 0.5/57.3f;
	sensor_noise_hf_amp.p 		= 5.1/57.3f;
	sensor_noise_hf_amp.q 		= 5.1/57.3f;
	sensor_noise_hf_amp.r 		= 5.1/57.3f;
	sensor_noise_hf_amp.u		= 0.2;
	sensor_noise_hf_amp.v 		= 0.05;
	sensor_noise_hf_amp.w 		= 0.05;
	sensor_noise_hf_amp.lat 	= 0.0000000001f;
	sensor_noise_hf_amp.longt 	= 0.0000000001f;
	sensor_noise_hf_amp.z 		= 0.05;
	sensor_noise_hf_amp.ax 		= 0.05;
	sensor_noise_hf_amp.ay 		= 0.05;
	sensor_noise_hf_amp.az 		= 0.05;



	sensor_hf_noise_lpff.phi   = (1.0f/(2.0f*PI*200.0));   // in secs
	sensor_hf_noise_lpff.theta = (1.0/(2.0*PI*200.0));
	sensor_hf_noise_lpff.psi   = (1.0/(2.0*PI*200.0));
	sensor_hf_noise_lpff.p     = (1.0/(2.0*PI*400.0));
	sensor_hf_noise_lpff.q     = (1.0/(2.0*PI*400.0));
	sensor_hf_noise_lpff.r     = (1.0/(2.0*PI*400.0));
	sensor_hf_noise_lpff.u     = (1.0/(2.0*PI*100.0));
	sensor_hf_noise_lpff.v     = (1.0/(2.0*PI*100.0));
	sensor_hf_noise_lpff.w     = (1.0/(2.0*PI*100.0));
	sensor_hf_noise_lpff.lat   = (1.0/(2.0*PI*10.0));
	sensor_hf_noise_lpff.longt = (1.0/(2.0*PI*10.0));
	sensor_hf_noise_lpff.z     = (1.0/(2.0*PI*20.0));
	sensor_hf_noise_lpff.ax    = (1.0/(2.0*PI*50.0));
	sensor_hf_noise_lpff.ay    = (1.0/(2.0*PI*50.0));
	sensor_hf_noise_lpff.az    = (1.0/(2.0*PI*50.0));
}

void v_Ins_State_Plant_Out_Fill(float latitude_point, float longitude_point,float Alt,float V_bd_ins[] ,float V_ned_ins[],float ax_bd_ins[],float Body_rate_bf_ins[],float attitude[])
{
	ins_state_plant_out.latitude     = latitude_point;// send to gnc in deg
	ins_state_plant_out.longitude    = longitude_point;
	ins_state_plant_out.Alt          = Alt;

	ins_state_plant_out.V_bd_ins[0]  = V_bd_ins[0];
	ins_state_plant_out.V_bd_ins[1]  = V_bd_ins[1];
	ins_state_plant_out.V_bd_ins[2]  = V_bd_ins[2];

	ins_state_plant_out.V_ned_ins[0] = V_ned_ins[0];
	ins_state_plant_out.V_ned_ins[1] = V_ned_ins[1];
	ins_state_plant_out.V_ned_ins[2] = V_ned_ins[2];

	ins_state_plant_out.ax_bd_ins[0] = ax_bd_ins[0];
	ins_state_plant_out.ax_bd_ins[1] = ax_bd_ins[1];
	ins_state_plant_out.ax_bd_ins[2] = ax_bd_ins[2];

	ins_state_plant_out.Body_rate_bf_ins[0] = Body_rate_bf_ins[0];
	ins_state_plant_out.Body_rate_bf_ins[1] = Body_rate_bf_ins[1];
	ins_state_plant_out.Body_rate_bf_ins[2] = Body_rate_bf_ins[2];

	ins_state_plant_out.attitude[0] = attitude[0];// + (powf(-1.0,rand())*(0.00/57.3));
	ins_state_plant_out.attitude[1] = attitude[1];// + (powf(-1.0,rand())*(0.00/57.3));
	ins_state_plant_out.attitude[2] = attitude[2];// + (powf(-1.0,rand())*(0.00/57.3));
}

void v_PWM_RPM_Data_Fill()
{
	int j_memove = 0;
	int i_memove = 0;

	for ( i_memove = 0; i_memove < (quad_num_motors + fwv_motors); i_memove++)
	{
		pwm_in[i_memove] = *((uint16_t*)(&rx_data_final_buffer[62 + j_memove]));

#ifdef MOTOR_IN_LOOP
		gnc_rpm_in[i_memove] = *((uint16_t*)(&rx_data_final_buffer[(sizeof(s_gnc_out)-10) + j_memove]));
#endif

		if(800< pwm_in[i_memove] < 2200)
		{
			pwm_old[i_memove] = pwm_in[i_memove];
		}
		else
		{
			pwm_in[i_memove] = pwm_old[i_memove];
		}

		j_memove = j_memove + 2;
	}

	for ( i_memove = 0; i_memove <num_actuator; i_memove++)
	{
		pwm_in_servo[i_memove] = *((uint16_t*)( &rx_data_final_buffer[62 + j_memove]));

		j_memove = j_memove + 2;
	}
}


void Log_To_Sharedmemory(float ideal_plane_state[])
{
	if(shm_current_addr < (plant_log_addr + MAX_BUFF - 100)) // 100 just given, it should be size of below logged frame size// oil_dat.c
	{
		memcpy(shm_current_addr, &t, 4); 					/* 1.plant time --- this is plant related variable only*/
		shm_current_addr += 4;

		memcpy(shm_current_addr, &time_gnc, 4);				/* 2. TIME from gnc --- from gnc receiving */
		shm_current_addr += 4;

		memcpy(shm_current_addr, &g_ui_dbg_var_2, 4);		/* 3. TIME of INS data arrival */ /*DISARM variable from GCS */
		shm_current_addr += 4;								/* This is to check the INS data is received correctly or not*/

		memcpy(shm_current_addr, &t_debug2, 4);       		/* 4. Time of plant execution */
		shm_current_addr += 4;

		memcpy(shm_current_addr, &ins_data_miss, 4); 		/* 5.ins_data_miss variable form gnc *//* ARMING received from the  autopilot*/
		shm_current_addr += 4;

		memcpy(shm_current_addr, ideal_plane_state, 12); 	/* 6 --> u  , 7 --> v, 8 --> w */
		shm_current_addr += 12;

		float p_57 = ideal_plane_state[3] * 57.2958;
		memcpy(shm_current_addr, &p_57, 4); 				/*  9 ----> p */
		shm_current_addr += 4;

		float q_57 =  ideal_plane_state[4] * 57.2958;
		memcpy(shm_current_addr, &q_57, 4); 				/* 10 -----> q */
		shm_current_addr += 4;

		float r_57 = ideal_plane_state[5] * 57.2958;
		memcpy(shm_current_addr, &r_57, 4);					/* 11 -----> r */
		shm_current_addr += 4;

		float phi_57 = ideal_plane_state[6] * 57.2958;
		memcpy(shm_current_addr, &phi_57, 4);				/* 12 -----> phi */
		shm_current_addr += 4;

		float theta_57 = ideal_plane_state[7] * 57.2958;
		memcpy(shm_current_addr, &theta_57, 4);				/* 13 -----> theta */
		shm_current_addr += 4;

		float psi_57 = ideal_plane_state[8] * 57.2958;
		memcpy(shm_current_addr, &psi_57, 4);				/* 14 -----> psi */
		shm_current_addr += 4;

		memcpy(shm_current_addr, ideal_plane_state + 9 , 12);   /* 15 ---> x ,16 --->y ,17 --->z  */
		shm_current_addr += 12;

		memcpy(shm_current_addr, &rx_data_final_buffer[14], 48); 		/* 18 --> udem, 19 --->vdem, 20 --->wdem */
		shm_current_addr += 48;								/* 21 --->pdem, 22 --->qdem, 23 ---rdem*/
		/* 24 --->phi_dem, 25 --->theta_dem , 26 --->psi_dem*/
		/* 27 --->x_dem, 28 ---> y_dem, 29 --->z_dem*/

		if ((quad_num_motors + fwv_motors + num_actuator) > 12) // max 12 pwm values we can accomodate fro logging
		{
			uint16_t pwm_in_temp[12] ={0};
			memcpy(shm_current_addr, pwm_in_temp, 24);        /* 30-41 ---> 12 pwm_in values for 12 motors*/
			shm_current_addr += 24;
		}
		else
		{
			memcpy(shm_current_addr, pwm_in, 2*(quad_num_motors+fwv_motors));        /* 30-41 ---> 12 pwm_in values for 12 motors*/
			shm_current_addr += 2*(quad_num_motors+fwv_motors);

			memcpy(shm_current_addr, pwm_in_servo, 2*(num_actuator));        /* 30-41 ---> 12 pwm_in values for 12 motors*/
			shm_current_addr += 2*(num_actuator);

			if (quad_num_motors+fwv_motors+num_actuator!=12)
			{
				uint16_t pwm_in_temp2[12 - (quad_num_motors+fwv_motors+num_actuator)] ={0};
				memcpy(shm_current_addr, pwm_in_temp2, 2*(12 - (quad_num_motors+fwv_motors+num_actuator)));        /* 30-41 ---> 12 pwm_in values for 12 motors*/
				shm_current_addr += 2*(12 - (quad_num_motors+fwv_motors+num_actuator));
			}
		}


		float aileron_left = actuator[AILERON_LEFT].angle *57.3;
		memcpy(shm_current_addr, &aileron_left , 4); 	/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;

		float aileron_right = actuator[AILERON_RIGHT].angle *57.3;
		memcpy(shm_current_addr, &aileron_right , 4); 	/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;

		float elevator_common = actuator[ELEVATOR_COMMON].angle *57.3;
		memcpy(shm_current_addr, &elevator_common , 4); 	/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;

		float rudder_common = actuator[RUDDER_COMMON].angle *57.3;
		memcpy(shm_current_addr, &rudder_common , 4); 	/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;

		// commented on Aug 30 2023 for filling xplane states in place of rotor, uncomment once test is done
		//memcpy(shm_current_addr, rotor_force_out, 32); /* 42-53 ---> 12 actur_out values for 12 Motors*/
		//shm_current_addr += 32;

		// comment below once tested, Aug 30 2023
		memcpy(shm_current_addr,&xp_states.v_tas, 4);		/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;		//46

		memcpy(shm_current_addr,&xp_states.v_in, 4);		/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;		//47

		memcpy(shm_current_addr,&xp_states.v_true_gnd, 4);	/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;		//48

		memcpy(shm_current_addr,&xp_states.v_in_eq, 4);		/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;		//49

		memcpy(shm_current_addr,xp_states.v_ned, 12);		/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 12;		//50 //51//52

		float hrz_spd=0;
		hrz_spd=sqrt(xp_states.v_ned[0]*xp_states.v_ned[0] + xp_states.v_ned[1]*xp_states.v_ned[1]) ;

		memcpy(shm_current_addr,&hrz_spd, 4);				/* 42-53 ---> 12 actur_out values for 12 Motors*/
		shm_current_addr += 4;		//53

		memcpy(shm_current_addr, mil_rpm, 8);				/* 54-57 ---> 4 rpm feedback values of 4 motors form gnc */
		shm_current_addr += 8;
	}
	else
	{
		printf("shm full, unable to log...%f  ",t);
#ifdef PRINT_DEBUG1
		printf("shm full, unable to log...%f  ",t);
#endif
	}

	//	rx_bytes_temp_p1 = SIO_read(PORT3,rx_data_p1_buf_temp,LOG_BYTES);
	//
	//	if((rx_bytes_temp_p1 > 0) && (shm_current_addr2 < (plant_log_addr2 + MAX_BUFF2 - LOG_BYTES)))
	//	{
	//		memcpy(shm_current_addr2, rx_data_p1_buf_temp, rx_bytes_temp_p1); // telemetry.dat
	//		shm_current_addr2 += rx_bytes_temp_p1;
	//	}
	//	else
	//	{
	//		//printf("%f no telemetry ",t);
	//	}
}

_Bool Find_Header_Sync(unsigned short *i_sync,int loop_no)
{
	_Bool headers_found = 0;

	while(*i_sync < (rx_bytes_count - 1))    /* while loop run until the header found or data completed */
	{
		if((rx_data_buffer[*i_sync] == 0x70U) && (rx_data_buffer[(*i_sync)+1] == 0x52U))	/* Condition to find the Header bytes */
		{
			headers_found = 1;								/* Header bytes found*/
			printf("t : %f ...loop :%d found the headers \n", t,loop_no);
			break;
		}
		else						/* when header bytes not found it will increase the i_sync count */
		{
			(*i_sync)++;
		}
	}
	return headers_found;
}

int Check_data_Checksum()
{
	int checksum_correct = 0U;

	s_gnc_out.gnc_out_Checksum = fletcher16_checksum(&(rx_data_buffer[2]),sizeof(s_gnc_out)-4);
	if(s_gnc_out.gnc_out_Checksum == *(uint16_t *)(&rx_data_buffer[sizeof(s_gnc_out)-2]))
	{
		checksum_correct = 1;
		memcpy(rx_data_final_buffer,rx_data_buffer ,sizeof(s_gnc_out));	/* If the checksum is correct then copy the data from the to the rx_data_final_buffer */
	}
	else
	{
		checksum_correct = 0;
		printf(" Check sum is wrong... Going for another loop data \n ");
	}

	return checksum_correct;
}

void Init_Loop_variables()
{
	if(!incomplete_p0)    						/* incomplete_p0 = 1 means received some bytes and some are remained */
	{
		rx_bytes_p0 = 0 ;
	}

	rx_bytes_temp_p0 = 0;						/* Temporary received bytes count variable giving as zero */
	rx_loops_done_p0 = 0;						/* Receiving loops done is equating to zero at starting point */
}

int UART_PORT_Check()
{
	int ret = 0;

	ret = SIO_open(PORT0, BAUD921600, DATABIT_8, STOPBIT_1, PAR_NO); // GNC
	if (ret < 0)
	{
		printf("Failed to open read port:%d\n", ret);
		return 0;
	}
	else if (ret > 0)
	{
		printf("Opened serial port successfully\n");
	}

	//	ret = SIO_open(PORT3,BAUD921600,DATABIT_8,STOPBIT_1,PAR_NO);//Telemetry
	//	if(ret < 0) {
	//		printf("Failed to open read port:%d\n",ret);
	//		return 0;
	//	}
	//	else if(ret > 0) printf("Opened serial port successfully\n");

	return 1;
}

void v_Ideal_Plane_State_Fill(float V_bd_ins[],float Body_rate_bf_ins[],float pos[])
{

	ideal_plane_state[0] = V_bd_ins[0];
	ideal_plane_state[1] = V_bd_ins[1];
	ideal_plane_state[2] = V_bd_ins[2];

	ideal_plane_state[3] = Body_rate_bf_ins[0];
	ideal_plane_state[4] = Body_rate_bf_ins[1];
	ideal_plane_state[5] = Body_rate_bf_ins[2];

	ideal_plane_state[6] = ins_state_plant_out.attitude[0] ;
	ideal_plane_state[7] = ins_state_plant_out.attitude[1] ;
	ideal_plane_state[8] = ins_state_plant_out.attitude[2] ;

	ideal_plane_state[9]  = pos[0];
	ideal_plane_state[10] = pos[1];
	ideal_plane_state[11] = pos[2];
}

void v_INS_States_Fill(float V_ned_ins[], float attitude[])
{
	Ins_states[0] 	= V_ned_ins[0];
	Ins_states[1] 	= V_ned_ins[1];
	Ins_states[2] 	= V_ned_ins[2];
	Ins_states[3] 	= attitude[0];
	Ins_states[4]	= attitude[1];
	Ins_states[5] 	= attitude[2];
}

void  Print_Transmitting_Data()
{
	memcpy(tx_data_init,&s_plant_tx_data,sizeof(s_plant_tx_data));

	for(int i = 0; i < sizeof(s_plant_tx_data); i++)
		printf("%x ", tx_data_init[i]);
	printf("\n");
}

void Fill_Plant_Data_for_Transmition()
{
	/*  Filling the 1000Hz structure to send the data to the gnc */
	memcpy(&s_send_param_1000hz, tx_hdr, 2);

	s_send_param_1000hz.msg_length	= sizeof(s_send_param_1000hz) - 2;
	s_send_param_1000hz.msg_ID		= 0xA1;

	memcpy(&s_send_param_1000hz.time, &t, 4);
	memcpy(&s_send_param_1000hz.accel_bd_ins, &ins_state_plant_out.ax_bd_ins, 24);

	s_send_param_1000hz.tx_gps_fix = '1';
	s_send_param_1000hz.data_valid = 1;
	s_send_param_1000hz.checksum = fletcher16_checksum(&(s_send_param_1000hz.msg_length), sizeof(s_send_param_1000hz)-4);

	/*  Filling the 200Hz structure to send the data to the gnc */
	memcpy(&s_send_param_200hz, tx_hdr, 2);
	s_send_param_200hz.msg_length	= sizeof(s_send_param_200hz) - 2;
	s_send_param_200hz.msg_ID		= 0xA2;

	memcpy(&s_send_param_200hz.time, &t, 4);
	memcpy(&s_send_param_200hz.vel_bd_ins, &ins_state_plant_out.V_bd_ins, 12);
	memcpy(&s_send_param_200hz.attitude, &ins_state_plant_out.attitude, 12);

	s_send_param_200hz.checksum = fletcher16_checksum(&(s_send_param_200hz.msg_length), sizeof(s_send_param_200hz)-4);

	/*  Filling the 100Hz structure to send the data to the gnc */
	memcpy(&s_send_param_100hz, tx_hdr, 2);
	s_send_param_100hz.msg_length	= sizeof(s_send_param_100hz) - 2;
	s_send_param_100hz.msg_ID		= 0xA3;

	memcpy(&s_send_param_100hz.time, &t, 4);
	memcpy(&s_send_param_100hz.latitude, &ins_state_plant_out.latitude, 12);
	memcpy(&s_send_param_100hz.V_ned_ins, &ins_state_plant_out.V_ned_ins, 12);

#ifdef XPLANE_IN_LOOP
	memcpy(&s_send_param_100hz.speed_tas, &xp_states.v_tas, 4);
#else
	memcpy(&s_send_param_100hz.speed_tas, &ins_state_plant_out.V_bd_ins[0], 4);
#endif

#ifdef XPLANE_IN_LOOP
	memcpy(&s_send_param_100hz.speed_gs, &xp_states.v_true_gnd, 4);
#else
	float gs_speed = sqrt(ins_state_plant_out.V_ned_ins[1]*ins_state_plant_out.V_ned_ins[1] + ins_state_plant_out.V_ned_ins[0]*ins_state_plant_out.V_ned_ins[0] );
	memcpy(&s_send_param_100hz.speed_gs, &gs_speed, 4);
#endif

	s_send_param_100hz.checksum = fletcher16_checksum(&(s_send_param_100hz.msg_length), sizeof(s_send_param_100hz)-4);
}

int Thread_Scheduling(int plant_hz)
{
	int ret = 0;
	attr.size = sizeof(attr);
	attr.sched_policy   = SCHED_DEADLINE;
	attr.sched_runtime  = 5 * 1000 * plant_hz;/* this is in nano seconds*/ /*this defines execution time*/
	attr.sched_deadline = 6 * 1000 * plant_hz;/* this is in nano seconds*/  /*this defines deadline time it will wait until this value from start and then go to next thread or restart this thread again*/
	attr.sched_period   = 10 * 1000 * plant_hz; /* this is in nano seconds , so for 1000 hz, value will be = 10 ^6 */

	ret = sched_setattr(0,&attr,0);
	if(ret < 0)
	{
		perror("sched error\n");
		return 0;
	}
	return 1;
}

void Send_data_to_GNC(int loop_number)
{
	tx_bytes = 0;
	int send_200Hz = 5, send_100Hz = 10;

	if ((loop_number % send_100Hz) == 0)
	{
		while(tx_bytes < sizeof(s_send_param_100hz))
		{

			tx_bytes_temp = SIO_write(PORT0, &s_send_param_100hz, sizeof(s_send_param_100hz));

			if(tx_bytes_temp > 0)
				tx_bytes += tx_bytes_temp;

#ifdef PRINT_DEBUG2
			printf("total sent bytes: %d ",tx_bytes);
#endif
			if (tx_bytes_temp < sizeof(s_send_param_100hz))
			{
				printf("check code here.\n");
			}

			printf("tx_bytes %d\n",tx_bytes);
		}
	}

	tx_bytes = 0;
	if ((loop_number % send_200Hz) == 0)
	{
		while(tx_bytes < sizeof(s_send_param_200hz))
		{

			tx_bytes_temp = SIO_write(PORT0, &s_send_param_200hz, sizeof(s_send_param_200hz));

			if(tx_bytes_temp > 0)
				tx_bytes += tx_bytes_temp;

#ifdef PRINT_DEBUG2
			printf("total sent bytes: %d ",tx_bytes);
#endif
			if (tx_bytes_temp < sizeof(s_send_param_200hz))
			{
				printf("check code here.\n");
			}

			printf("tx_bytes %d\n",tx_bytes);
		}
	}

	if ((loop_number % 1) == 0)
	{
		tx_bytes = 0;
		while(tx_bytes < sizeof(s_send_param_1000hz))
		{

			tx_bytes_temp = SIO_write(PORT0, &s_send_param_1000hz, sizeof(s_send_param_1000hz));

			if(tx_bytes_temp > 0)
				tx_bytes += tx_bytes_temp;

#ifdef PRINT_DEBUG2
			printf("total sent bytes: %d ",tx_bytes);
#endif
			if (tx_bytes_temp < sizeof(s_send_param_1000hz))
			{
				printf("check code here.\n");
			}

			printf("tx_bytes %d\n",tx_bytes);
		}
	}
}


void Thread_Run(struct timespec * startTime, struct timespec *endTime)
{
	int plant_hz       = 100; //  for 100Hz put 1000,for 1000 Hz put 100
	t_debug1 = timeElapsed1();
#ifndef OBC_IN_LOOP_BLOCK
	sched_yield();
#endif
	t_debug2 = timeElapsed2();

	clock_gettime(CLOCK_MONOTONIC ,startTime);
	endTime->tv_sec  = startTime->tv_sec;
	endTime->tv_nsec = startTime->tv_nsec + (1000 * plant_hz);

	while(endTime->tv_nsec > 1000000000)
	{
		endTime->tv_sec++;
		endTime->tv_nsec -= 1000000000;
	}
}

/*
void v_plant_reboot()
{
	framesync = 0;
	t = 0;
	rx_loops_done_p0 = 0;
	loop_hz = 0;

	plant_log_addr = (char*)shmat(shm_id, NULL, 0);
	if(plant_log_addr < 0)
	{
		perror("Error in attaching shm log1.Press a key to continue, CTRL+C to quit ");
		getchar();
	}
	else
	{
		shm_current_addr = (unsigned char*)plant_log_addr;
		printf("successfully attached shm for log1.\n ");
	}
	printf("initializing shm buffer to zeroes...\n");
	memset((void*)plant_log_addr, 0, MAX_BUFF);

	//												SIO_write(PORT0,&reset, sizeof(reset));

	array_initd(Plane_state,15);
	array_initd(ideal_plane_state,15);
	array_initd(Ins_states,9);
	array_initd(pwm_in,sizeof(pwm_in));
	array_initd(pwm_out_esc,sizeof(pwm_out_esc));

	array_initd(all_rotors_force,sizeof(all_rotors_force));
	array_initd(all_rotors_moment,sizeof(all_rotors_moment));
	array_initd(rotor_yaw_moment_b,sizeof(rotor_yaw_moment_b));
	array_initd(rotor_yaw_moment_m,sizeof(rotor_yaw_moment_m));
	array_initd(rotor_force,sizeof(rotor_force));
	array_initd(rotor_moment,sizeof(rotor_moment));

	array_initd(rotor_speed_dot_dot,sizeof(rotor_speed_dot_dot));
	array_initd(rotor_speed_old,sizeof(rotor_speed_old));
	array_initd(rotor_speed_dot,sizeof(rotor_speed_dot));
	array_initd(thrust_noise_old,sizeof(thrust_noise_old));
	array_initd(thrust_noise,sizeof(thrust_noise));

	array_initd(acc_real_plant,sizeof(acc_real_plant));



	init_plant();

	fn_uav_states_init(flg_sitl_mode,&latitude_point,&longitude_point,&Alt,&V_bd_ins ,&V_ned_ins,&ax_bd_ins,&Body_rate_bf_ins,&attitude);//OBC IN LOOP AKA SITL

	lat_long_to_xy( latitude_point, longitude_point, Alt,ideal_plane_state + 9);

	v_Ins_State_Plant_Out_Fill(latitude_point,longitude_point,Alt,V_bd_ins ,V_ned_ins,ax_bd_ins,Body_rate_bf_ins,attitude);

	v_Ideal_Plane_State_Fill(V_bd_ins,Body_rate_bf_ins,pos);

	v_INS_States_Fill(V_ned_ins,attitude);

	Fill_Plant_Data_for_Transmition();

	Print_Transmitting_Data();
}*/
void* main_rt()
{
	int loop_hz        =   0;
	int plant_hz       = 100;    //  for 100Hz put 1000,for 1000 Hz put 100
	int send_hz_divide =  10;		// for 100Hz put 1 , for 1000Hz put 10
	int Armed = 0;
	_Bool ARMED  = 0;
	int loop = 0;
	int port_check;
	int Schedling_Success = 0;
	// for shared memory based logging
	int rx_bytes_temp_p0 = 0, tx_bytes = 0;
	int ret = 0;
	int i = 0, j = 0, k = 0;
	int truevalue = 1;

	float prev_t1 = 0, t1_us = 0, period_time_us = 0, exec_time_us = 0, t2_us = 0;

	//key = ftok("./",'a');
	struct timespec startTime    = {0,0},   endTime	={0,0};
	struct timespec startTime_io = {0,0}, endTime_io={0,0};
	unsigned short i_sync = 0;

	//    float Plane_state[NUM_STATEVARS];
	//    float ideal_plane_state[NUM_STATEVARS];
	//    float Ins_states[9];

	float V_ins_b[3][1];
	float R_v_b[3][3];

	float pos[3] = {0.0f};//X=0.0,Y=0.0,Z=0.0;
	float temp3X3_1[3][3], temp3X3_2[3][3], temp3X1_1[3][1], V_v[3][1], ppp[3][1];
	float T   = 0.0f;
	float rot = 0.0f;
	float latitude_point = 0.0f, longitude_point = 0.0f, Alt = 0.0f;
	float V_bd_ins[3] = {0.0f}, V_ned_ins[3] = {0.0f}, ax_bd_ins[3] = {0.0f}, Body_rate_bf_ins[3] = {0.0f}, attitude[3] = {0.0f};
	float ss2_dot_alrn = 0.0f, ss2_dot_elvtr = 0.0f, ss2_dot_rdr = 0.0f;
	float dummy_1, dummy_2;
	float delta_dem[]   = {0.0f}; 
	float delta_array[] = {0.0f}; 

	float acc_real_plant[3] = {0.0f};
	unsigned int  t_exec = 0;

	enable_rt();	
	char reset[80] = {0};
	reset[0] = 'R';

	init_common_variable();

	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
	{
		
		perror("mlockall failed");
		exit(-2);
	}

	//Array initialization
	array_initd(Plane_state,15);
	array_initd(ideal_plane_state,15);
	array_initd(Ins_states,9);
	array_initd(rx_data_final_buffer,8);
	array_initd(rx_data_p1,50);

	//array_initd(R_v_b,9);
	array_initd(V_ins_b,3);

	init_plant();

	port_check = UART_PORT_Check();
	if(port_check)
	{
//		SIO_write(PORT0,&reset, sizeof(reset));		/*commented rajat*/
	}
	else
	{
		return 0;
	}

	// UAV states initialiser based on SITL or XPLane_SITL
#ifndef XPLANE_IN_LOOP
	fn_uav_states_init(flg_sitl_mode,&latitude_point,&longitude_point,&Alt,&V_bd_ins ,&V_ned_ins,&ax_bd_ins,&Body_rate_bf_ins,&attitude);//OBC IN LOOP AKA SITL
	lat_long_to_xy( latitude_point, longitude_point, Alt,ideal_plane_state + 9);
#endif


#ifdef XPLANE_IN_LOOP
	fn_uav_states_init(flg_sitl_mode,&latitude_point,&longitude_point,&Alt,&V_bd_ins ,&V_ned_ins,&ax_bd_ins,&Body_rate_bf_ins,&attitude);//XPLANE SITL
#endif

	v_Ins_State_Plant_Out_Fill(latitude_point,longitude_point,Alt,V_bd_ins ,V_ned_ins,ax_bd_ins,Body_rate_bf_ins,attitude);

	v_Ideal_Plane_State_Fill(V_bd_ins,Body_rate_bf_ins,pos);

	v_INS_States_Fill(V_ned_ins,attitude);

	Fill_Plant_Data_for_Transmition();

	Print_Transmitting_Data();

	struct timespec currentTime;

	float ht_planestate = 0;
#ifndef OBC_IN_LOOP_BLOCK
	fcntl(mx_fd[PORT0],F_SETFL,fcntl(mx_fd[PORT0],F_GETFL,0)|O_NONBLOCK);

	Schedling_Success = Thread_Scheduling(plant_hz);//  plant_hz = for 100Hz put 1000,for 1000 Hz put 100
	if(Schedling_Success == 0)
	{
		return 0;
	}
#endif

	Send_data_to_GNC(loop_hz);// loop_hz is incrementing number ++

	Thread_Run(&startTime, &endTime);

	int missing_loops = 0;
#define New_logic
	/* writing the new logic to receive ....Its just modification to the last one */
#ifdef New_logic
	while(t < t1)   // t1 = 86400 seconds, = 24 hours	/* where 't' is the plant running time , it will start after the frameSync done and 't1' is the max time to run the plant */
	{
		if((loop % N) == 0)   	 						/* loop count will start once the frame sync is done */
		{
			Init_Loop_variables();

#ifndef OBC_IN_LOOP_BLOCK
			while(1)
#else
			while(receiving_loop_done == 0)
#endif
			{
				rx_bytes_count_temp = SIO_read(PORT0, rx_data_temp_buffer,sizeof(s_gnc_out) - rx_bytes_count);   /*Receiving the data from  the GNC*/
				if(rx_bytes_count_temp < 0)				/* Checking whether the bytes received or not */
				{
					printf("Bytes not received exit from Data receiving while loop \n");
					if(ARMED == 1)
					{
						missing_loops++;
						if(missing_loops >= 100)
						{
							ARMED = 0;
							framesync = 0;
							printf("plant is stopped due to not receiving any data from the autopilot... \n");
						}
					}
					break;								/* Exit to the while loop... Because the data not received*/
				}
				else
				{
					printf("t : %f ...loop :%d and %d bytes Received from GNC \n",t,loop, rx_bytes_count_temp);

					memcpy(rx_data_buffer + rx_bytes_count , rx_data_temp_buffer ,rx_bytes_count_temp);
					rx_bytes_count += rx_bytes_count_temp;
					memset(rx_data_temp_buffer,0,sizeof(rx_data_temp_buffer)); 			/* reset the receiving buffer to zero */

					if(rx_bytes_count > 1)			/* checking the data in the buffer more than 1 byte or not,to found the two headers */
					{
						i_sync = 0;						/* i_sync is to find the headers in the Received Buffer */

						_Bool header_sync_found = Find_Header_Sync(&i_sync ,loop);

						if(!header_sync_found)					/* when Header bytes not found */
						{
							rx_data_incomplete = 0;
							rx_bytes_count = 0;
							printf("t : %f ...loop :%d ...header un_sync to zeroth position , sync bytes not found \n",t, loop);
						}
						else							/* Somewhere header bytes found */
						{
							if(i_sync != 0)				/* Header byte position is not sync with zeroth position */
							{
								memcpy(rx_data_buffer, rx_data_buffer + i_sync, rx_bytes_count - i_sync);	/* copying the data from the byte header found to till the end of buffer */
								rx_bytes_count  -= i_sync;
								rx_data_incomplete = 1;						/* Headers found but some data is missing */
								printf("t : %f ...loop :%d ...Header un_sync to zeroth position, sync bytes found \n", t,loop);
							}
							else
							{
								if(rx_bytes_count ==  sizeof(s_gnc_out))	/* If Header found is at zeroth position */
								{

									Check_data_Checksum();

									receiving_loop_done++;						/* increasing the loops_done variable to indicate we received the data */
									received_gnc_count++;					/* received_gnc_count variable increasing indicate how many times we received the data from the GNC */

									rx_bytes_count = 0;
									rx_data_incomplete = 0;

									memset(rx_data_buffer,0,sizeof(s_gnc_out));
									printf("t : %f ...loop :%d ... No. of times received from GNC is %d   \n", t,loop,received_gnc_count);

									missing_loops = 0;
									break;
								}
								else									/* headers found sync with zero position but size of data is different */
								{
									printf("t : %f ...loop :%d ...incomplete frame \n",t,loop);
									rx_data_incomplete = 1;
								}
							}
						}
					}
					else
					{
						printf("t : %f ...loop :%d ...single byte received \n",t,loop);
						rx_data_incomplete = 0;
					}
				}
			}

			if(ARMED == 1)							/* Checking already ARMED or not ... */
			{
				if(*((unsigned int*)(&(rx_data_final_buffer[6]))) == 1)	/* Checking whether the AP_REBOOT request is received or not...*/
				{
					printf("t : %f ...loop :%d ...AP_Reboot is called......\n",t,loop);
					ARMED = 0;
//					v_plant_reboot(&loop_hz , &acc_real_plant);
					printf("t : %f ...loop :%d ...plant_Reboot is done.......\n",t,loop);
				}
			}
			else if(!ARMED)							/* If not ARMED .... it will enter into else case */
			{
				if(!framesync)						/* Checking... frame_sync is done or not */
				{

#ifdef XPLANE_IN_LOOP
					printf("t : %f ...loop :%d...%s...frame sync is not done yet.\n", t, loop,string_1);
#else
#ifdef XPLANE_IN_LOOP_VISUALISATION
					printf("t : %f ...loop :%d...%s...frame sync is not done yet.\n", t, loop,string_2);
#else
					printf("t : %f ...loop :%d...%s...frame sync is not done yet.\n", t, loop,string_3);
#endif

#endif
					if (receiving_loop_done > 0 /* || received_gnc_count > 0*/) /* Checking that, the data is received or not properly...*/
					{
						framesync = 1;
						printf("t : %f ...loop :%d ...frame sync is done ...\n", t, loop);
					}
				}
				else if (framesync) /* if frame_sync is done checking for ARM is received or not */
				{
#ifdef XPLANE_IN_LOOP
					printf("t : %f ...loop :%d...%s...ARMING is not done yet.\n", t, loop,string_1);
#else
#ifdef XPLANE_IN_LOOP_VISUALISATION
					printf("t : %f ...loop :%d...%s...ARMING is not done yet.\n", t, loop,string_2);
#else
					printf("t : %f ...loop :%d...%s...ARMING is not done yet.\n", t, loop,string_3);
#endif

#endif
					if (*((unsigned int *)(&(rx_data_final_buffer[10]))) == 1) /* checking the ARM variable... */
					{
						ARMED = 1;					/* ARMING here... */
						printf("t : %f ...loop :%d ...ARMING is received....ARMED .....\n",t,loop);
					}
				}

				tx_bytes      = 0;
				tx_bytes_temp = 0;

				memcpy(&s_plant_tx_data, tx_data_init, sizeof(s_plant_tx_data));
#ifndef OBC_IN_LOOP_BLOCK
				Thread_Run(&startTime, &endTime);
#endif
//				if ((loop_hz % send_hz_divide) == 0)
				{
					Send_data_to_GNC(loop_hz);
				}

				loop_hz = loop_hz + 1;

				continue;
			}
#else
			while(t < t1)
			{
				//         printf("%f ",t);
				if((loop % N) == 0)    /*  N Value is 1 here , and I don't think this condition have any use here*/
				{
					if(!incomplete_p0)
						rx_bytes_p0  = 0;

					rx_bytes_temp_p0 = 0;
					rx_loops_done_p0 = 0;
#ifndef OBC_IN_LOOP_BLOCK
			while(1)
#else
			while(rx_loops_done_p0 == 0)
#endif
			{
				rx_bytes_temp_p0 = SIO_read(PORT0,rx_data_p0_buf_temp,sizeof(s_gnc_out) - rx_bytes_p0);

				if(!framesync)
				{
					//#ifdef PRINT_DEBUG1
					printf("t %f loop %d : %d bytes recvd, rx_data_p0_buf_temp[0][1] = %x %x ",t,loop,rx_bytes_temp_p0,rx_data_p0_buf_temp[0],rx_data_p0_buf_temp[1]);
					//#endif
				}
				if(rx_bytes_temp_p0 <= 0)
				{
					printf(" bytes not received exit from while       ");
					break;
				}
				else
				{
//					printf("t %f loop %d : %d bytes recvd, rx_data_p0_buf_temp[0][1] = %x %x ",t,loop,rx_bytes_temp_p0,rx_data_p0_buf_temp[0],rx_data_p0_buf_temp[1]);
					memcpy(rx_data_p0_buf + rx_bytes_p0, rx_data_p0_buf_temp, rx_bytes_temp_p0);
					rx_bytes_p0 += rx_bytes_temp_p0;
					memset(rx_data_p0_buf_temp,0,rx_bytes_temp_p0);
					if(rx_bytes_p0 > 1)
					{
						i_sync = 0;
//						sync_found=0;
						while(i_sync < (rx_bytes_p0 - 1))
						{
							if(rx_data_p0_buf[i_sync] == 0x70)
							{
								if(rx_data_p0_buf[i_sync+1] == 0x52)
								{
									break;
//									sync_found=1;
								}
								else
								{
									i_sync++;
								}
							}
							else
							{
								i_sync++;
							}
						}
						if((rx_bytes_p0 - i_sync) < 2)
						{
							incomplete_p0 = 0;
							rx_bytes_p0   = 0;
#ifdef PRINT_DEBUG1                            
							printf("t %f loop %d:header unsync: sync bytes not found ",t,loop);
#endif                            
						}
						else
						{
							if(i_sync != 0)
							{
								// slide
								memcpy(rx_data_p0_buf, rx_data_p0_buf + i_sync, rx_bytes_p0 - i_sync);
								rx_bytes_p0  -= i_sync;
								incomplete_p0 = 1;
#ifdef PRINT_DEBUG1                                
								printf("t %f loop %d:header unsync : sync bytes found ",t,loop);
#endif                                
							}
							else
							{
								if(rx_bytes_p0 == sizeof(s_gnc_out))
								{
									rx_loops_done_p0++;
									received_gnc_count++;
									s_gnc_out.gnc_out_Checksum = fletcher16_checksum(&(rx_data_p0_buf[2]),sizeof(s_gnc_out)-4);
									if(s_gnc_out.gnc_out_Checksum == *(uint16_t *)(&rx_data_p0_buf[sizeof(s_gnc_out)-2]))
									{
										memset(rx_data_p0,0,sizeof(rx_data_p0));
										memcpy(rx_data_p0, rx_data_p0_buf, sizeof(s_gnc_out));
									}
									if(Armed == 1)
									{
										*((unsigned int*)(&(rx_data_p0[10]))) = 1;
//										}
//									else
//									{
										if(*((unsigned int*)(&(rx_data_p0[6]))) == 1)
										{
											framesync = 0;
											Armed = 0 ;
											t = 0;
											rx_loops_done_p0 = 0;
											loop_hz = 0;

											plant_log_addr = (char*)shmat(shm_id, NULL, 0);
											if(plant_log_addr < 0)
											{
												perror("Error in attaching shm log1.Press a key to continue, CTRL+C to quit ");
												getchar();
											}
											else
											{
												shm_current_addr = (unsigned char*)plant_log_addr;
												printf("successfully attached shm for log1.\n ");
											}
											printf("initializing shm buffer to zeroes...\n");
											memset((void*)plant_log_addr, 0, MAX_BUFF);

											//												SIO_write(PORT0,&reset, sizeof(reset));

											array_initd(Plane_state,15);
											array_initd(ideal_plane_state,15);
											array_initd(Ins_states,9);
											array_initd(pwm_in,sizeof(pwm_in));
											array_initd(pwm_out_esc,sizeof(pwm_out_esc));

											array_initd(all_rotors_force,sizeof(all_rotors_force));
											array_initd(all_rotors_moment,sizeof(all_rotors_moment));
											array_initd(rotor_yaw_moment_b,sizeof(rotor_yaw_moment_b));
											array_initd(rotor_yaw_moment_m,sizeof(rotor_yaw_moment_m));
											array_initd(rotor_force,sizeof(rotor_force));
											array_initd(rotor_moment,sizeof(rotor_moment));

											array_initd(rotor_speed_dot_dot,sizeof(rotor_speed_dot_dot));
											array_initd(rotor_speed_old,sizeof(rotor_speed_old));
											array_initd(rotor_speed_dot,sizeof(rotor_speed_dot));
											array_initd(thrust_noise_old,sizeof(thrust_noise_old));
											array_initd(thrust_noise,sizeof(thrust_noise));

											array_initd(acc_real_plant,sizeof(acc_real_plant));



											init_plant();

											fn_uav_states_init(flg_sitl_mode,&latitude_point,&longitude_point,&Alt,&V_bd_ins ,&V_ned_ins,&ax_bd_ins,&Body_rate_bf_ins,&attitude);//OBC IN LOOP AKA SITL

											lat_long_to_xy( latitude_point, longitude_point, Alt,ideal_plane_state + 9);

											v_Ins_State_Plant_Out_Fill(latitude_point,longitude_point,Alt,V_bd_ins ,V_ned_ins,ax_bd_ins,Body_rate_bf_ins,attitude);

											v_Ideal_Plane_State_Fill(V_bd_ins,Body_rate_bf_ins,pos);

											v_INS_States_Fill(V_ned_ins,attitude);

											Fill_Plant_Data_for_Transmition();

											Print_Transmitting_Data();

//											pthread_exit(1);
										}
									}
									rx_bytes_p0   = 0;
									incomplete_p0 = 0;
									memset(rx_data_p0_buf,0,sizeof(s_gnc_out));
									printf("No. of times received from gnc  %d   ", received_gnc_count);
									break;
								}
								else
								{
									//										printf("t %f loop %d:incomplete frame ",t,loop);
#ifdef PRINT_DEBUG1                                    
									printf("t %f loop %d:incomplete frame ",t,loop);
#endif                                        
									incomplete_p0 = 1;
								}
							}
						}
					}
					else
					{
#ifdef PRINT_DEBUG1                        
						printf("t %f loop %d:single byte recvd ",t,loop);
#endif                        
						incomplete_p0 = 1;
					}
				}
			}

			if(framesync)
			{
				Armed = 1 ;
#ifdef PRINT_DEBUG1                
				printf("t %f loop %d : %d loops ",t,loop,rx_loops_done_p0);
				printf("%x %x ",rx_data_p0[0],rx_data_p0[1]);
#endif                
				if(rx_loops_done_p0 > 0)
				{
				
				}
				else
				{
				
				}
			}
			else
			{
				//#ifdef PRINT_DEBUG1
				printf("t %f loop %d:framesync not done yet.\n",t,loop);
				//#endif
				if((rx_loops_done_p0 > 0)&& *((unsigned int*)(&(rx_data_p0[10]))) == 1)
				{
					framesync = 1;
					printf("%d loops, frame sync done. ",rx_loops_done_p0);
#ifdef PRINT_DEBUG1                    
					printf("%d loops, frame sync done. ",rx_loops_done_p0);
#endif                    
				}
				else
				{
					tx_bytes      = 0;
					tx_bytes_temp = 0;

					memcpy(&s_plant_tx_data, tx_data_init, sizeof(s_plant_tx_data));
#ifndef OBC_IN_LOOP_BLOCK

					Thread_Run(&startTime, &endTime);
#endif
					if ((loop_hz % send_hz_divide) == 0)
					{
						Send_data_to_GNC(loop_hz);
					}

					loop_hz = loop_hz + 1;
#ifdef PRINT_DEBUG1                    
					printf("\n");
#endif                    
					continue;
				}
			}
#endif
			v_PWM_RPM_Data_Fill();

//         	g_ui_dbg_var_1 = *((unsigned int*)(&(rx_data_p0[2])));
			time_gnc = *((float*)(&(rx_data_p0[2])));
			g_ui_dbg_var_2 = *((unsigned int*)(&(rx_data_p0[6])));
			ins_data_miss = *((unsigned int*)(&(rx_data_p0[10])));

			ideal_plane_state[12] = delta_array[0];
			ideal_plane_state[13] = delta_array[1];
			ideal_plane_state[14] = delta_array[2];
//          battery_dynamics();
			battery_dynamics(); /*added this on 11th Feb 2023 to check the battery model*/
			v_update_thrust_torque(); /*added this on 11th Feb 2023 to check the battery model*/
#ifndef MOTOR_IN_LOOP
			esc_dynamics();
#ifndef XPLANE_IN_LOOP
			/*	comment ROTOR and Actuator dynamics if we enable x-plane UDP simulation */
#endif
			Actuator_dynamics(t_step_act);
			rotor_dynamics(t_step_rot); // BLDC motor delay dynamics
#else
			motor_in_loop_thrust_genrtr();
#endif


#ifdef XPLANE_IN_LOOP
			if ((loop%10) == 0)
			{
				fn_recv_frm_xplane_udp();

				if(xp_states.xp_recv_valid ==1) // will be always 1 , can be removed
				{
					fn_xp_fill_xp_states();
					fn_xp_fill_ideal_pls(ideal_plane_state,&latitude_point , &longitude_point , &Alt);
					lat_long_to_xy( latitude_point, longitude_point, Alt,ideal_plane_state + 9);

				}
			}
#endif


#ifndef XPLANE_IN_LOOP
			//comment rk4 if we enable xplane recv data
			rk4(ideal_plane_state, t ,t_step_plant, acc_real_plant);  // calling 6 DOF
#endif


			ideal_plane_state[6] = Angle_Ranges(ideal_plane_state[6] * 57.2958)/57.2958;// limiting phi <0 -- -180, 0 -- 180>
			ideal_plane_state[7] = Angle_Ranges(ideal_plane_state[7] * 57.2958)/57.2958;// limiting theta <0 - -180, 0 -- 180>
			ideal_plane_state[8] = Angle_Ranges(ideal_plane_state[8] * 57.2958)/57.2958;// limiting psi <0 -- -180, 0 -- 180>

			if((loop % 10) == 0)
			{
				test_ins_state_update_every_hundred_cycle(ideal_plane_state, Ins_states);
			}
			t_step_ins = t_step_plant;

			v_update_vehicle_states(ideal_plane_state);


			pseudo_INS(t, ideal_plane_state, Ins_states, acc_real_plant, &latitude_point, &longitude_point, &Alt, V_bd_ins, V_ned_ins, ax_bd_ins, Body_rate_bf_ins, attitude, t_step_ins);
#ifdef XPLANE_IN_LOOP
			fn_send_to_xplane_udp();
#endif
#ifdef XPLANE_IN_LOOP_VISUALISATION
			fn_send_to_xplane_rk4_udp();
#endif
			
			v_Ins_State_Plant_Out_Fill(latitude_point, longitude_point, Alt, V_bd_ins, V_ned_ins, ax_bd_ins, Body_rate_bf_ins, attitude);
			
			v_fill_lla_to_vehicle_state(latitude_point, longitude_point, Alt);

			Fill_Plant_Data_for_Transmition();

			Log_To_Sharedmemory(ideal_plane_state);

			// v_send_data_to_client_via_tcp_float(ideal_plane_state);
			v_fill_data_out_for_socket(t, ideal_plane_state, V_bd_ins, ax_bd_ins, Body_rate_bf_ins, attitude);
			if (loop % 50 == 0)
			{
				// v_data_out_socket_udp();
			}
		}
#ifdef PRINT_DEBUG1
		printf("\n");
#endif
		loop++;
		int count = 0;
		count = count + 1;

		Thread_Run(&startTime, &endTime);
		// if(ret !=0)
		// printf("incomplete sleep\n");
		if(framesync)
		{
//			if ((loop_hz % send_hz_divide) == 0)
			{
				Send_data_to_GNC(loop_hz);
			}
			loop_hz = loop_hz + 1;
		}
		else
		{
			printf("no framesync\n");
		}

#ifndef OBC_IN_LOOP_BLOCK        

		// Todo: what is this approximation, is it the reason plant time differs from GNC duing long run
		// we assume real time !!!

		float t_debug_actual = t_debug2;
		if((t_debug2 < 850.0) || ( t_debug2 > 1150.0))
		{
			t_debug2 = 1000.0;
		}
		t_step_plant = t_debug2/1e6;
		t_step_rot   = t_debug2/1e6;
		t_step_act   = t_debug2/1e6;
#endif                
		t = t + t_step_plant;

		printf("%f %f... actual : %f \n",t,t_step_plant, t_debug_actual);
	}
	//	fn_xplane_udp_close();
}

int main()
{
	// data_out_socket_tcp_ip();
	// v_data_out_socket_udp; // uncomment if you want to send data out of plant via udp for plotting purpose

	pthread_t thread[100]; /* U can create multiple threads in the future, right now we are using only one thread */
	int thread_return = 0;
	int * thread_status = 0;
	int thread_call_num = 0;
	//    int key , shm_id;
	int key2, shm_id2;
	int ret = 0;
	key = ftok("./",'d');	/*The generated key is used to create and accessing the shared memory and oil_out.dat */
	stack_prefault(); 
	//shm_id = shmget(key,sizeof(struct shm_plant_struct)*NUMCYCLES,IPC_CREAT | 0666);
	//	do
	//	{
	shm_id = shmget(key, MAX_BUFF, 0666);
	if(shm_id < 0)
	{
		printf("creating shm\n" );
		shm_id = shmget(key, MAX_BUFF, IPC_CREAT | 0666);
		if(shm_id < 0)
		{
			perror("Error in shmget ");
			return 0;
		}
	}
	else
	{
		printf("shm id present, so reusing \n");
	}

	plant_log_addr = (char*)shmat(shm_id, NULL, 0);
	if(plant_log_addr < 0)
	{
		perror("Error in attaching shm log1.Press a key to continue, CTRL+C to quit ");
		getchar();
	}
	else
	{
		shm_current_addr = (unsigned char*)plant_log_addr;
		printf("successfully attached shm for log1.\n ");
	}
	printf("initializing shm buffer to zeroes...\n");
	memset((void*)plant_log_addr, 0, MAX_BUFF);

	key2 = ftok("./",'c');
	shm_id2 = shmget(key2, MAX_BUFF2, 0666);
	if(shm_id2 < 0)
	{
		printf("creating shm2\n" );
		shm_id2 = shmget(key2, MAX_BUFF2, IPC_CREAT | 0666);
		if(shm_id2<0)
		{
			perror("Error in shmget2 ");
			return 0;
		}
	}
	else
	{
		printf("shm id 2 present, so reusing \n");
	}
	plant_log_addr2 = (char*)shmat(shm_id2, NULL, 0);
	if(plant_log_addr2 < 0)
	{
		perror("Error in attaching shm log2.Press a key to continue, CTRL+C to quit ");
		getchar();
	}
	else
	{
		shm_current_addr2 = (unsigned char*)plant_log_addr2;
		printf("successfully attached shm for log2 \n ");
	}
	/*
    if(plant_log_addr < 0) 
    {
        if(plant_log_addr2 < 0) 
        {
            printf("shm log areas unable to be attached. Pree any key to continue. 1 shm log is for log of OBC in loop. Another is for debug log\n");
            printf("Press Ctrl+C to STOP. Any key to CONITNUE. Unless other log modes are enabled, you dont want to continue... isnt it? \n");
            getchar();
            return 0;
        }
        else
        {
            printf("\n");
        }
    }
    else
    {
        if(plant_log_addr2 < 0) 
        {
            printf("shm log areas unable to be attached. Pree any key to continue. 1 shm log is for log of OBC in loop. Another is for debug log\n");
            printf("Press Ctrl+C to STOP. Any key to CONITNUE. Unless other log modes are enabled, you dont want to continue... isnt it? \n");
            getchar();
            return 0;
        }
        else
        {
        }    	
    }
	 */
	printf("initializing shm buffer to zeroes...\n");
	memset((void*)plant_log_addr2, 0, MAX_BUFF2);

	printf("shm created, creating main_rt thread...\n");

	//	do
	//	{

	thread_return = pthread_create(&thread[thread_call_num], NULL, main_rt, NULL);
	thread_return = pthread_join(thread[thread_call_num], &thread_status);
	//    	thread_return = pthread_kill(thread,0);
	//    	thread_return = pthread_detach(thread);
	//		thread_call_num++;
	//	}while(thread_status > 0);

	ret = 0;
	//ret=shmdt(plant_log_addr);
	if(ret < 0)
	{
		perror("Error detaching shm");
	}
	else
		printf("detached shm\n");

	//ret=shmdt(plant_log_addr2);
	if(ret < 0)
	{
		perror("Error detaching shm2");
	}
	else
		printf("detached shm2\n");

	/*
    ret=shm_unlink(plant_log_addr);
    if(ret < 0)
    {
    perror("Error unlinking shm");
    }
	 */
	//printf("unlinked shm\n");
	// This shm id has been generated above with IPC_CREATE flag, so that we can destroy this as it is no further needed
	//if(shmctl(shm_id,IPC_RMID,NULL) < 0){perror("Error in shmctl ");return 0;}

	return 0;
}


void v_fill_data_out_for_socket(float t,float ideal_plane_state[15],float V_bd_ins[3] ,float ax_bd_ins[3],float Body_rate_bf_ins[3],float attitude[3])
{

	data_out_socket_arr[0] = t;  /* 1.plant time --- this is plant related variable only*/

	data_out_socket_arr[1] = time_gnc;  /* 2. TIME from gnc --- from gnc receiving */

	data_out_socket_arr[2] = V_bd_ins[0] * 57.2958;//u
	data_out_socket_arr[3] =  *((float*)(&rx_data_final_buffer[14]));
	data_out_socket_arr[4] = V_bd_ins[1] * 57.2958;//v
	data_out_socket_arr[5] =  *((float*)(&rx_data_final_buffer[18]));
	data_out_socket_arr[6] = V_bd_ins[2] * 57.2958;//w
	data_out_socket_arr[7] =  *((float*)(&rx_data_final_buffer[22]));

	data_out_socket_arr[8] =  Body_rate_bf_ins[0] * 57.2958;//p
	data_out_socket_arr[9] =   *((float*)(&rx_data_final_buffer[26]));
	data_out_socket_arr[10] = Body_rate_bf_ins[1] * 57.2958;//q
	data_out_socket_arr[11] =  *((float*)(&rx_data_final_buffer[30]));
	data_out_socket_arr[12] = Body_rate_bf_ins[2] * 57.2958;//r
	data_out_socket_arr[13] =  *((float*)(&rx_data_final_buffer[34]));

	data_out_socket_arr[14] = attitude[0] * 57.2958;//phi
	data_out_socket_arr[15] = *((float*)(&rx_data_final_buffer[38]));
	data_out_socket_arr[16] = attitude[1] * 57.2958;//theta
	data_out_socket_arr[17] = *((float*)(&rx_data_final_buffer[42]));
	data_out_socket_arr[18] = attitude[2] * 57.2958;//psi
	data_out_socket_arr[19] = *((float*)(&rx_data_final_buffer[46]));

	data_out_socket_arr[20] = ideal_plane_state[9] * 57.2958;//x
	data_out_socket_arr[21] = *((float*)(&rx_data_final_buffer[50]));
	data_out_socket_arr[22] = ideal_plane_state[10] * 57.2958;//y
	data_out_socket_arr[23] = *((float*)(&rx_data_final_buffer[54]));
	data_out_socket_arr[24] = ideal_plane_state[11] * 57.2958;//z
	data_out_socket_arr[25] = *((float*)(&rx_data_final_buffer[58]));

	data_out_socket_arr[26] = ax_bd_ins[0];
	data_out_socket_arr[27] = ax_bd_ins[1];
	data_out_socket_arr[28] = ax_bd_ins[2];
}



