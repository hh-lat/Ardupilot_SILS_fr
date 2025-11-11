#include "stdio.h"
#include "stdlib.h"

//extern void throttle_cntrl();

extern void fn_xplane_udp_open();

extern void fn_send_to_xplane_rk4_udp();

extern void fn_xplane_udp_close();

extern void fn_send_to_xplane_udp();

extern void fn_recv_frm_xplane_udp();

extern void fn_xp_fill_ideal_pls(float*,float*  , float*  , float* );

extern void fn_xp_fill_xp_states();


extern float xp_send_data[1][9];
extern float xp_recv_data[6][9],xp_recv_data_curr[6][9],xp_recv_data_prvs[6][9];

#pragma(1)
typedef struct
{
	int xp_in_rows; //read data rows
	int xp_out_rows; // send data rows
	int xp_in_cols;
	int xp_recv_valid; //to fill
	int xp_valid_data;
	int fst_fill; //flag to know first fill
	int valid_inc;//flag to know each data is valid
	float v_ned[3];
	float v_bd[3];
	float lat;
	float longt;
	float alt_msl;
	float alt_agl;
	float pos_ned[3];
	float phi;
	float theta;
	float psi;
	float mag_psi;
	float p;
	float q;
	float r;
	float aoa;
	float ssa;
	float v_in;
	float v_tas;
	float v_true_gnd;
	float v_in_eq;
}str_xp_states;


extern str_xp_states xp_states;



