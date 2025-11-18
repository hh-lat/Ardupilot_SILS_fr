#include "LAT_SIM_Runner.h"
#include "LAT_SIM_math_util.h"
#include "LAT_SIM_pseudo_ins.h"
#include "time.h"

#define NUM_STATEVARS_INS 9

float p_o,q_o,r_o;

//==================INS_variable=========================================//
float AX,AY,AZ;//22_3_19//  used in pseudo_INS.c and INS_derivative
//=====================================================================//



float pseudo_ins_states[28]={0.0};
float bufferarray_pseudo_ins[2000][28]={0.0};
int delay_array_length_pseuso_ins=(int)(1000.0*0.005) ;  // delay_array_length =  (int)(Plant_freq*delay_in_seconds);
int buffer_filled_first_time=0;

int  flag_sensor_input_delay=0, flag_sensor_input_inacc=0, flag_sensor_input_hfnoise=0;


float noise_part[15]={0.0};
float noise_sign[15]={1};
int noise_index=0;

sensor_accuracy_struct sensor_accuracy , low_freq_inacc_component, high_freq_noise_component , low_freq_inacc_component_old, high_freq_noise_component_old;
sensor_accuracy_struct sensor_inacc_lpff , sensor_hf_noise_lpff , sensor_noise_hf_amp;

// Assuming flat earth Model
void pseudo_INS(float t,float ideal_plane_state[],float Ins_states[],float acc_real_plant[],float *latitude_point,float *longitude_point,float *Alt,float V_bd_ins[],float V_ned_ins[],float ax_bd_ins[],float Body_rate_bf_ins[],float attitude[],float t_step_ins)
{
	if (t_step_ins<0.0009)
	{
		t_step_ins = 0.001;
	}
	float p = 0.0f, q = 0.0f, r = 0.0f;
	float phi=0.0f, theta = 0.0f, psi = 0.0f;
	static float p_o_dot, q_o_dot, r_o_dot;
	static float ax_o_dot, ay_o_dot, az_o_dot, ax_o, ay_o, az_o;
	
	float ax_o_dot_dot = 0.0f, ay_o_dot_dot = 0.0f, az_o_dot_dot = 0.0f;
	float p_o_dot_dot = 0.0f, q_o_dot_dot = 0.0f, r_o_dot_dot = 0.0f;
	float temp3X3_1[3][3], temp3X1_2[3][1], temp3X3_2[3][3], R_v_v1[3][3], R_v1_v2[3][3], R_v2_b[3][3], R_v_b[3][3], temp3X1_1[3][1];
	float Acc_in[3][1];

	float zeta_p = 0.5, zeta_ax = 0.5;
	float zeta_q = 0.5, zeta_ay = 0.5;
	float zeta_r = 0.5, zeta_az = 0.5;
	int i = 0;

	float w_p = 2.0 * 3.14 * 190.0, w_ax = 2.0 * 3.14 * 190.0;
	float w_q = 2.0 * 3.14 * 190.0, w_ay = 2.0 * 3.14 * 190.0;
	float w_r = 2.0 * 3.14 * 190.0, w_az = 2.0 * 3.14 * 190.0;

	float R = 6374049; //% Radius of earth at local ground level
	float X = 0.0f, Y = 0.0f, Z = 0.0f;
	float lat = 0.0f, longt = 0.0f;
	float Z_ned_frame=0;

	float R_ned_ecef[3][3], R_ecef_ned[3][3];
	float Point_from_ned_to_ecef_just_rotation_no_translation[3][1], ned_frame_origin_wrt_Ecef[3][1];
	float point_wrt_ecef[3][1], Point_from_ned_to_ecef[3][1], point_wrt_ned[3][1];
	
	array_initd((float*)temp3X3_1, 9);
	array_initd((float*)temp3X3_2, 9);
	array_initd((float*)temp3X1_1, 3);
	array_initd((float*)temp3X1_2, 3);
	array_initd((float*)R_v_v1, 9);
    array_initd((float*)R_v1_v2, 9);
    array_initd((float*)R_v2_b, 9);
    array_initd((float*)R_v_b, 9);
 
	array_initd((float*)R_ned_ecef, 9);
	array_initd((float*)R_ecef_ned, 9);
	array_initd((float*)Point_from_ned_to_ecef_just_rotation_no_translation, 3);
	array_initd((float*)ned_frame_origin_wrt_Ecef, 3);
	array_initd((float*)Point_from_ned_to_ecef, 3);
	array_initd((float*)point_wrt_ecef, 3);
	array_initd((float*)Acc_in, 3);

	p = ideal_plane_state[3];
	q = ideal_plane_state[4];
	r = ideal_plane_state[5];

    if ( delayed_ins_use  == 1)
    {
    	phi   = Ins_states[3];
    	theta = Ins_states[4];
    	psi   = Ins_states[5];

    	R_v_v1[0][0]= cosf(psi);
    	R_v_v1[0][1]= sinf(psi);
    	R_v_v1[0][2]= 0.0;
    	R_v_v1[1][0]= (-sinf(psi));
    	R_v_v1[1][1]= cosf(psi);
    	R_v_v1[1][2]= 0.0;
    	R_v_v1[2][0]= 0.0;
    	R_v_v1[2][1]= 0.0;
    	R_v_v1[2][2]= 1.0;

    	R_v1_v2[0][0]= cosf(theta);
    	R_v1_v2[0][1]= 0.0;
    	R_v1_v2[0][2]= (-sinf(theta));
    	R_v1_v2[1][0]= 0.0;
    	R_v1_v2[1][1]= 1.0;
    	R_v1_v2[1][2]= 0.0;
    	R_v1_v2[2][0]= sinf(theta);
    	R_v1_v2[2][1]= 0.0;
    	R_v1_v2[2][2]= cosf(theta);

    	R_v2_b[0][0]=1.0;
    	R_v2_b[0][1]=0.0;
    	R_v2_b[0][2]=0.0;
    	R_v2_b[1][0]=0.0;
    	R_v2_b[1][1]=cosf(phi);
    	R_v2_b[1][2]=sinf(phi);
    	R_v2_b[2][0]=0.0;
    	R_v2_b[2][1]= (-sinf(phi));
    	R_v2_b[2][2]=cosf(phi);

    	MatrixMultiply_old(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
    	MatrixMultiply_old(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);

	p_o_dot_dot = p*w_p*w_p - 2*zeta_p*w_p*p_o_dot - p_o*w_p*w_p;  
	q_o_dot_dot = q*w_q*w_q - 2*zeta_q*w_q*q_o_dot - q_o*w_q*w_q;  
	r_o_dot_dot = r*w_r*w_r - 2*zeta_r*w_r*r_o_dot - r_o*w_r*w_r; 
	
	for(i=0;i<4;i++)
	{
		p_o_dot = p_o_dot_dot*(t_step_ins/4) + p_o_dot;
		q_o_dot = q_o_dot_dot*(t_step_ins/4) + q_o_dot;
		r_o_dot = r_o_dot_dot*(t_step_ins/4) + r_o_dot;
	}
	
	for(i=0;i<4;i++)
	{
		p_o = p_o_dot*(t_step_ins/4) + p_o;
		q_o = q_o_dot*(t_step_ins/4) + q_o;
		r_o = r_o_dot*(t_step_ins/4) + r_o;
	}
	
	ax_o_dot_dot = acc_real_plant[0]*w_ax*w_ax - 2*zeta_ax*w_ax*ax_o_dot - ax_o*w_ax*w_ax;  
	ay_o_dot_dot = acc_real_plant[1]*w_ay*w_ay - 2*zeta_ay*w_ay*ay_o_dot - ay_o*w_ay*w_ay;  
	az_o_dot_dot = acc_real_plant[2]*w_az*w_az - 2*zeta_az*w_az*az_o_dot - az_o*w_az*w_az;
	
	for(i=0;i<4;i++)
	{
		ax_o_dot = ax_o_dot_dot*(t_step_ins/4) + ax_o_dot;
		ay_o_dot = ay_o_dot_dot*(t_step_ins/4) + ay_o_dot;
		az_o_dot = az_o_dot_dot*(t_step_ins/4) + az_o_dot;
	}
	for(i=0;i<4;i++)
	{
		ax_o = ax_o_dot*(t_step_ins/4) + ax_o;
		ay_o = ay_o_dot*(t_step_ins/4) + ay_o;
		az_o = az_o_dot*(t_step_ins/4) + az_o;
	}
	
	transposedmxnAToB(3,3,R_v_b,temp3X3_1);
	temp3X1_1[0][0]=ax_o;
	temp3X1_1[1][0]=ay_o;
	temp3X1_1[2][0]=az_o;
	
	MatrixMultiply_old(temp3X3_1,3,3,temp3X1_1,3,1,Acc_in);// True acceleration expressed in NED frame wrt NED frame

	AX=Acc_in[0][0];
	AY=Acc_in[1][0];
	AZ=Acc_in[2][0]; // these accelerations go to INS_derivative to evaluate Velocities in NED frame
	

	rk4_ins(Ins_states,t,t_step_ins);

	V_ned_ins[0] = Ins_states[0]; //% body velocity in ned frame , unit will out in m/sec
	V_ned_ins[1] = Ins_states[1];
	V_ned_ins[2] = Ins_states[2];

	Ins_states[3] = Angle_Ranges(Ins_states[3]*57.2958)/57.2958;// limiting phi <0 -- -180, 0 -- 180>
	Ins_states[4] = Angle_Ranges(Ins_states[4]*57.2958)/57.2958;// limiting theta <0 - -180, 0 -- 180>
	Ins_states[5] = Angle_Ranges(Ins_states[5]*57.2958)/57.2958;// limiting psi <0 -- -180, 0 -- 180>

	X=Ins_states[6];  //  % Coordinates of point wrt to NED frame
	Y=Ins_states[7];
	Z=Ins_states[8];

	Body_rate_bf_ins[0]=p_o;
	Body_rate_bf_ins[1]=q_o;
	Body_rate_bf_ins[2]=r_o;



	*Alt=s_home_state.alt_msl+Ins_states[8];


	 // srand(time(0));
	 phi   = Ins_states[3] ;//+ 0.0/57.3 ;//+ (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3)
		//srand(time(0));
    theta = Ins_states[4] ;//+ 0.0/57.3 ;//+ (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3);//(powf(-1.0,rand())*(0.2/57.3));
		//srand(time(0));
	 psi   = Ins_states[5] ;//+ 0.0/57.3 ;//+ (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3);//(powf(-1.0,rand())*(1.0/57.3));

	R_v_v1[0][0]= cosf(psi);
	R_v_v1[0][1]= sinf(psi);
	R_v_v1[0][2]= 0.0;
	R_v_v1[1][0]= (-sinf(psi));
	R_v_v1[1][1]= cosf(psi);
	R_v_v1[1][2]= 0.0;
	R_v_v1[2][0]= 0.0;
	R_v_v1[2][1]= 0.0;
	R_v_v1[2][2]= 1.0;

	R_v1_v2[0][0]= cosf(theta);
	R_v1_v2[0][1]= 0.0;
	R_v1_v2[0][2]= (-sinf(theta));
	R_v1_v2[1][0]= 0.0;
	R_v1_v2[1][1]= 1.0;
	R_v1_v2[1][2]= 0.0;
	R_v1_v2[2][0]= sinf(theta);
	R_v1_v2[2][1]= 0.0;
	R_v1_v2[2][2]= cosf(theta);

	R_v2_b[0][0]=1.0;
	R_v2_b[0][1]=0.0;
	R_v2_b[0][2]=0.0;
	R_v2_b[1][0]=0.0;
	R_v2_b[1][1]=cosf(phi);
	R_v2_b[1][2]=sinf(phi);
	R_v2_b[2][0]=0.0;
	R_v2_b[2][1]= (-sinf(phi));
	R_v2_b[2][2]=cosf(phi);

	MatrixMultiply_old(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
	MatrixMultiply_old(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);

	temp3X1_1[0][0]=V_ned_ins[0];
	temp3X1_1[1][0]=V_ned_ins[1];
	temp3X1_1[2][0]=V_ned_ins[2];

	MatrixMultiply_old(R_v_b,3,3,temp3X1_1,3,1,V_bd_ins);//% body velocity in body frame , unit will out in m/sec

	temp3X1_1[0][0]=Acc_in[0][0];
	temp3X1_1[1][0]=Acc_in[1][0];
	temp3X1_1[2][0]=Acc_in[2][0];// True Acceleration in NED frame

	MatrixMultiply_old(R_v_b,3,3,temp3X1_1,3,1,temp3X1_2);
	ax_bd_ins[0]=temp3X1_2[0][0];  // True Acceleration expressed in body frame and measured in NED frame in m/sec^2(i.e no coriolis force and centrifugal term here)   ,, accelerometers measure - Basically A=f/M), where f includes all forces including gravity
	ax_bd_ins[1]=temp3X1_2[1][0];  // True acceleration means here is that if plane is free falling , acceleration will be 9.81, if plane is stable then it is zero
	ax_bd_ins[2]=temp3X1_2[2][0];



	   // srand(time(0));
	phi   = Ins_states[3] ; //+ 1.0/57.3 + (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3)
		//srand(time(0));
	theta = Ins_states[4] ;//+ 1.0/57.3 + (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3);//(powf(-1.0,rand())*(0.2/57.3));
		//srand(time(0));
	psi   = Ins_states[5] ;//+ 1.0/57.3 + (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3);//(powf(-1.0,rand())*(1.0/57.3));

	attitude[0]=phi; //% Euler angles between ned and body frame, unit will go out in rad
	attitude[1]=theta; //% Euler angles between ned and body frame, unit will go out in rad
	attitude[2]=psi; //% Euler angles between ned and body frame, unit will go out in rad
	//////////////////////////////////////////////////////////////////
    }
    else
    {
    	phi= ideal_plane_state[6];
    	theta= ideal_plane_state[7];
    	psi= ideal_plane_state[8];

    	R_v_v1[0][0]= cosf(psi);
    	R_v_v1[0][1]= sinf(psi);
    	R_v_v1[0][2]= 0.0;
    	R_v_v1[1][0]= (-sinf(psi));
    	R_v_v1[1][1]= cosf(psi);
    	R_v_v1[1][2]= 0.0;
    	R_v_v1[2][0]= 0.0;
    	R_v_v1[2][1]= 0.0;
    	R_v_v1[2][2]= 1.0;

    	R_v1_v2[0][0]= cosf(theta);
    	R_v1_v2[0][1]= 0.0;
    	R_v1_v2[0][2]= (-sinf(theta));
    	R_v1_v2[1][0]= 0.0;
    	R_v1_v2[1][1]= 1.0;
    	R_v1_v2[1][2]= 0.0;
    	R_v1_v2[2][0]= sinf(theta);
    	R_v1_v2[2][1]= 0.0;
    	R_v1_v2[2][2]= cosf(theta);

    	R_v2_b[0][0]=1.0;
    	R_v2_b[0][1]=0.0;
    	R_v2_b[0][2]=0.0;
    	R_v2_b[1][0]=0.0;
    	R_v2_b[1][1]=cosf(phi);
    	R_v2_b[1][2]=sinf(phi);
    	R_v2_b[2][0]=0.0;
    	R_v2_b[2][1]= (-sinf(phi));
    	R_v2_b[2][2]=cosf(phi);

    	MatrixMultiply_old(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
    	MatrixMultiply_old(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);

    V_bd_ins[0] =ideal_plane_state[0];
    V_bd_ins[1] =ideal_plane_state[1];
    V_bd_ins[2] =ideal_plane_state[2];


	transposedmxnAToB(3,3,R_v_b,temp3X3_1);
	temp3X1_1[0][0]=V_bd_ins[0];
	temp3X1_1[1][0]=V_bd_ins[1];
	temp3X1_1[2][0]=V_bd_ins[2];

	MatrixMultiply_old(temp3X3_1,3,3,temp3X1_1,3,1,V_ned_ins);// Ned frame velocities

    ax_bd_ins[0]	=  acc_real_plant[0];// True Acceleration expressed in body frame and measured in NED frame in m/sec^2(i.e no coriolis force and centrifugal term here)   ,, accelerometers measure - Basically A=f/M), where f includes all forces including gravity
    ax_bd_ins[1]	=  acc_real_plant[1];
    ax_bd_ins[2]	=  acc_real_plant[2];


    ideal_plane_state[6] = Angle_Ranges(ideal_plane_state[6]*57.2958)/57.2958;// limiting phi <0 -- -180, 0 -- 180>
    ideal_plane_state[7] = Angle_Ranges(ideal_plane_state[7]*57.2958)/57.2958;// limiting theta <0 - -180, 0 -- 180>
    ideal_plane_state[8] = Angle_Ranges(ideal_plane_state[8]*57.2958)/57.2958;// limiting psi <0 -- -180, 0 -- 180>


	X = ideal_plane_state[9];  //  % Coordinates of point wrt to NED frame
	Y = ideal_plane_state[10];
	Z = ideal_plane_state[11];

#ifndef XPLANE_IN_LOOP
// if Xplane running as plant is defined, then Alt is already filled with msl altitude
	*Alt = s_home_state.alt_msl + ideal_plane_state[11];
#endif

	Body_rate_bf_ins[0]=p;
	Body_rate_bf_ins[1]=q;
	Body_rate_bf_ins[2]=r;

	/*// Todo: commented on 26 June 2023 as it will hinder with selective dof running of plant
	   // srand(time(0));
	phi   = ideal_plane_state[6]  + 0.1/57.3 ;// (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3)
		//srand(time(0));
	theta = ideal_plane_state[7]  + 0.1/57.3 ;//(powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3);//(powf(-1.0,rand())*(0.2/57.3));
		//srand(time(0));
	psi   = ideal_plane_state[8]  + 1.0/57.3 ;//+ (powf(-1.0,rand())*(0.1/57.3));//(0.01/57.3);//(powf(-1.0,rand())*(1.0/57.3));
*/

	attitude[0]=phi; //% Euler angles between ned and body frame, unit will go out in rad
	attitude[1]=theta; //% Euler angles between ned and body frame, unit will go out in rad
	attitude[2]=psi; //% Euler angles between ned and body frame, unit will go out in rad

    }


	lat   =  s_home_state.lat/57.2958f;
    longt =  s_home_state.longt/57.2958f;
    Z_ned_frame = s_home_state.alt_msl;


	//R_ned_ecef = [-sinf(lat) 0 cosf(lat);0 1 0;-cosf(lat) 0 -sinf(lat)]*[cosf(long) sinf(long) 0;-sinf(long) cosf(long) 0;0 0 1];
	temp3X3_1[0][0]=-sinf(lat);
	temp3X3_1[0][1]=0;
	temp3X3_1[0][2]=cosf(lat);
	temp3X3_1[1][0]=0;
	temp3X3_1[1][1]=1;
	temp3X3_1[1][2]=0;
	temp3X3_1[2][0]=-cosf(lat);
	temp3X3_1[2][1]=0;
	temp3X3_1[2][2]=-sinf(lat);
	
	temp3X3_2[0][0]=cosf(longt);
	temp3X3_2[0][1]=sinf(longt);
	temp3X3_2[0][2]=0;
	temp3X3_2[1][0]=-sinf(longt);
	temp3X3_2[1][1]=cosf(longt);
	temp3X3_2[1][2]=0;
	temp3X3_2[2][0]=0;
	temp3X3_2[2][1]=0;
	temp3X3_2[2][2]=1;
	
	MatrixMultiply_old(temp3X3_1,3,3,temp3X3_2,3,3,R_ned_ecef);
	transposedmxnAToB(3,3,R_ned_ecef,R_ecef_ned);
	
	temp3X1_1[0][0]=X;
	temp3X1_1[1][0]=Y;
	temp3X1_1[2][0]=Z;
	
	MatrixMultiply_old(R_ecef_ned,3,3,temp3X1_1,3,1,Point_from_ned_to_ecef_just_rotation_no_translation);
	
	ned_frame_origin_wrt_Ecef[0][0]=(R-Z_ned_frame)*cosf(lat)*cosf(longt);
	ned_frame_origin_wrt_Ecef[1][0]=(R-Z_ned_frame)*cosf(lat)*sinf(longt);
	ned_frame_origin_wrt_Ecef[2][0]=(R-Z_ned_frame)*sinf(lat);
	
	Point_from_ned_to_ecef[0][0] = ned_frame_origin_wrt_Ecef[0][0] + Point_from_ned_to_ecef_just_rotation_no_translation[0][0];
	Point_from_ned_to_ecef[1][0] = ned_frame_origin_wrt_Ecef[1][0] + Point_from_ned_to_ecef_just_rotation_no_translation[1][0];
	Point_from_ned_to_ecef[2][0] = ned_frame_origin_wrt_Ecef[2][0] + Point_from_ned_to_ecef_just_rotation_no_translation[2][0];
	

#ifndef XPLANE_IN_LOOP
	*longitude_point = atan2f(Point_from_ned_to_ecef[1][0],Point_from_ned_to_ecef[0][0]);
	*latitude_point  = asinf(Point_from_ned_to_ecef[2][0]/(R-*Alt));
	*latitude_point  = *latitude_point*57.2957802;//%converting to degrees
	*longitude_point = *longitude_point*57.2957802;
#endif

	
	//*latitude_point=(int)(*latitude_point)*100+((*latitude_point)-(int)(*latitude_point))*60 ;
	//*longitude_point=(int)(*longitude_point)*100+((*longitude_point)-(int)(*longitude_point))*60 ;


	pseudo_ins_states[0] = attitude[0];
	pseudo_ins_states[1] = attitude[1];
	pseudo_ins_states[2] = attitude[2];
	pseudo_ins_states[3] = Body_rate_bf_ins[0];
	pseudo_ins_states[4] = Body_rate_bf_ins[1];
	pseudo_ins_states[5] = Body_rate_bf_ins[2];
	pseudo_ins_states[6] = *latitude_point;
	pseudo_ins_states[7] = *longitude_point;
	pseudo_ins_states[8] = *Alt;
	pseudo_ins_states[9] = V_ned_ins[0];

	pseudo_ins_states[10] = V_ned_ins[1];
	pseudo_ins_states[11] = V_ned_ins[2];
	pseudo_ins_states[12] = V_bd_ins[0];
	pseudo_ins_states[13] = V_bd_ins[1];
	pseudo_ins_states[14] = V_bd_ins[2];
	pseudo_ins_states[15] = ax_bd_ins[0];
	pseudo_ins_states[16] = ax_bd_ins[1];
	pseudo_ins_states[17] = ax_bd_ins[2];


///////////////////////////////// DELAY IN SENSOR MODELLING //////////////////////////////////////////////////////////////////////////////////////////

	if (flag_sensor_input_delay ==1)
	{
        fn_sensor_delay_input();
	}

	buffer_filled_first_time= buffer_filled_first_time+1;// this ensures that initial states of plant are sent without any delay, thus buffer array is allowed to fill till delayed length and then delay is started

	if (buffer_filled_first_time>delay_array_length_pseuso_ins)
	{

	buffer_filled_first_time=2000;

	attitude[0]=pseudo_ins_states[0] ;
	attitude[1]=pseudo_ins_states[1] ;
	attitude[2]=pseudo_ins_states[2] ;
	Body_rate_bf_ins[0]=pseudo_ins_states[3];
	Body_rate_bf_ins[1]=pseudo_ins_states[4];
	Body_rate_bf_ins[2]=pseudo_ins_states[5];
	*latitude_point=pseudo_ins_states[6];
	*longitude_point=pseudo_ins_states[7];
	*Alt=pseudo_ins_states[8];
	V_ned_ins[0]=pseudo_ins_states[9];
	V_ned_ins[1]=pseudo_ins_states[10];
	V_ned_ins[2]=pseudo_ins_states[11];
	V_bd_ins[0]=pseudo_ins_states[12];
	V_bd_ins[1]=pseudo_ins_states[13];
	V_bd_ins[2]=pseudo_ins_states[14];
	ax_bd_ins[0]=pseudo_ins_states[15];
	ax_bd_ins[1]=pseudo_ins_states[16];
	ax_bd_ins[2]=pseudo_ins_states[17];

	}

if (flag_sensor_input_inacc ==1)
{
	fn_low_frequency_sensor_inacc_modeler(t_step_ins);
}


if (flag_sensor_input_hfnoise==1)
{
	fn_high_frequency_sensor_noise_modeler(t_step_ins);
}

 //////////////////////////////////////////// SENSOR INACCURACY AND NOISE INDUCED READINGS //////////////////////////////////////////////

    	attitude[0] = attitude[0] + low_freq_inacc_component.phi  + high_freq_noise_component.phi ;
    	attitude[1] = attitude[1] + low_freq_inacc_component.theta  + high_freq_noise_component.theta ;
    	attitude[2] = attitude[2] + low_freq_inacc_component.psi  + high_freq_noise_component.psi ;

    	Body_rate_bf_ins[0] = Body_rate_bf_ins[0] + low_freq_inacc_component.p  + high_freq_noise_component.p ;
    	Body_rate_bf_ins[1] = Body_rate_bf_ins[1] + low_freq_inacc_component.q  + high_freq_noise_component.q ;
    	Body_rate_bf_ins[2] = Body_rate_bf_ins[2] + low_freq_inacc_component.r  + high_freq_noise_component.r ;

       	*latitude_point  = *latitude_point  + low_freq_inacc_component.lat  + high_freq_noise_component.lat ;
        *longitude_point = *longitude_point + low_freq_inacc_component.longt  + high_freq_noise_component.longt ;
        *Alt = *Alt  + low_freq_inacc_component.z  + high_freq_noise_component.z ;

    	V_ned_ins[0] = V_ned_ins[0] + low_freq_inacc_component.u  + high_freq_noise_component.u ;
    	V_ned_ins[1] = V_ned_ins[1] + low_freq_inacc_component.v  + high_freq_noise_component.v ;
    	V_ned_ins[2] = V_ned_ins[2] + low_freq_inacc_component.w  + high_freq_noise_component.w ;

    	V_bd_ins[0] = V_bd_ins[0] + low_freq_inacc_component.u  + high_freq_noise_component.u ;
    	V_bd_ins[1] = V_bd_ins[1] + low_freq_inacc_component.v  + high_freq_noise_component.v ;
    	V_bd_ins[2] = V_bd_ins[2] + low_freq_inacc_component.w  + high_freq_noise_component.w ;

    	ax_bd_ins[0]= ax_bd_ins[0] + low_freq_inacc_component.ax  + high_freq_noise_component.ax ;
    	ax_bd_ins[1]= ax_bd_ins[1] + low_freq_inacc_component.ay  + high_freq_noise_component.ay ;
    	ax_bd_ins[2]= ax_bd_ins[2] + low_freq_inacc_component.az  + high_freq_noise_component.az ;
//
}

void ins_derivative(float y[],float t,float dydt[])
{
	float phi = 0.0, theta = 0.0, psi = 0.0;  //ins_derivative
	phi   = y[3];						  //ins_derivative
	theta = y[4];					//ins_derivative
	psi   = y[5];						//ins_derivative
	//% V_X_s=Ins_states(7);
	//% V_Y_s=Ins_states(8);
	//% V_Z_s=Ins_states(9);
	
	dydt[0] = AX;				//ins_derivative
	dydt[1] = AY;				//ins_derivative
	dydt[2] = AZ;				//ins_derivative
	dydt[3] = (p_o + q_o * sinf(phi) * tanf(theta) + r_o * cosf(phi) * tanf(theta));//%phi_dot //ins_derivative
	dydt[4] = q_o * cosf(phi) - r_o * sinf(phi);//%theta_dot							   //ins_derivative
	dydt[5] = (q_o * sinf(phi) * sec(theta) + r_o * cosf(phi) * sec(theta));//%psi_dot		//ins_derivative
	dydt[6] = y[0];//%inertial velocity X  //ins_derivative
	dydt[7] = y[1];//%inertial velocity Y	 //ins_derivative
	dydt[8] = y[2];//%inertial velocity Z  //ins_derivative
}

void rk4_ins(float y[],float t,float h)
{
	float hh = 0.0f,h6 = 0.0f,th = 0.0f;
	int i = 0;
	float dydt[NUM_STATEVARS_INS],dyt[NUM_STATEVARS_INS],dym[NUM_STATEVARS_INS]; // differential value of y wrt t
	float yt[NUM_STATEVARS_INS];

	array_initd(yt, NUM_STATEVARS_INS);
	array_initd(dydt, NUM_STATEVARS_INS);
	array_initd(dyt, NUM_STATEVARS_INS);
	array_initd(dym, NUM_STATEVARS_INS);
	
	hh = 0.5*h;
	h6 = h/6.0;
	
	ins_derivative(y, t, dydt);
	for(i = 0; i < NUM_STATEVARS_INS; i++)
	{
		yt[i] = y[i]+ (hh * dydt[i]);
	}

	th = t+hh;
	ins_derivative(yt, th, dyt);
	for(i = 0; i < NUM_STATEVARS_INS; i++)
	{
		yt[i] = y[i] + (hh * dyt[i]);
	}

	ins_derivative(yt, th, dym);
	for(i = 0; i < NUM_STATEVARS_INS; i++)
	{
		yt[i] = y[i] + h * dym[i];
	}
	for(i = 0; i < NUM_STATEVARS_INS; i++)
	{
		dym[i] = dym[i] + dyt[i];
	}
	ins_derivative(yt, t + h, dyt);
	for(i = 0; i < NUM_STATEVARS_INS; i++)
	{
		y[i] = y[i] + h6 * (dydt[i] + dyt[i] + (2.0 * dym[i]));
	}

}

float Angle_Ranges(float p)
{
	if (p>360)
	p =fmodf(p,360);

	if (p>180)
	p = -180 + fmodf(p,180);

	if (p<-360)
	   p =-fmodf(abs(p),360);

	if (p<-180)
	p= 180 - fmodf(abs(p),180);




/*	if (p>15.0)
	p = 15.0 ;//+ fmodf(p,5);

	if (p<-15.0)
	p = -15.0 ;//+ fmodf(p,5);*/

	return p;
}



void fn_sensor_delay_input()
{
    pure_transport_delay_float( pseudo_ins_states, bufferarray_pseudo_ins, sizeof(pseudo_ins_states)/sizeof(float),  delay_array_length_pseuso_ins);
}



void fn_high_frequency_sensor_noise_modeler(float t_step_ins)
{
    noise_index=0;
    while (noise_index<15)
	{
   	     srand(noise_index + time(NULL) );
   	     noise_part[noise_index]=rand()%10; // will make random values comes between 0 and 10
   	     noise_sign[noise_index]=powf(-1,fabsf(noise_part[noise_index]));
   	     noise_part[noise_index] = fabsf(noise_part[noise_index])/10.0f;

   	   if (noise_part[noise_index] >1.0)
   	   {
   		   noise_part[noise_index]=1.0;
	   }

   	     noise_index=noise_index+1;
	}

       // high frequency noise input
    high_freq_noise_component.phi   = sensor_noise_hf_amp.phi*noise_part[14]*noise_sign[6];  // randomising the indexes to make it more random
    high_freq_noise_component.theta = sensor_noise_hf_amp.theta*noise_part[13]*noise_sign[7];
    high_freq_noise_component.psi   = sensor_noise_hf_amp.psi*noise_part[12]*noise_sign[8];
    high_freq_noise_component.p     = sensor_noise_hf_amp.p*noise_part[11]*noise_sign[9];
    high_freq_noise_component.q     = sensor_noise_hf_amp.q*noise_part[10]*noise_sign[10];
    high_freq_noise_component.r     = sensor_noise_hf_amp.r*noise_part[9]*noise_sign[11];
    high_freq_noise_component.u     = sensor_noise_hf_amp.u*noise_part[8]*noise_sign[12];
    high_freq_noise_component.v     = sensor_noise_hf_amp.v*noise_part[7]*noise_sign[13];
    high_freq_noise_component.w     = sensor_noise_hf_amp.w*noise_part[6]*noise_sign[14];
    high_freq_noise_component.lat   = sensor_noise_hf_amp.lat*noise_part[5]*noise_sign[0];
    high_freq_noise_component.longt = sensor_noise_hf_amp.longt*noise_part[4]*noise_sign[1];
    high_freq_noise_component.z     = sensor_noise_hf_amp.z*noise_part[3]*noise_sign[2];
    high_freq_noise_component.ax    = sensor_noise_hf_amp.ax*noise_part[2]*noise_sign[3];
    high_freq_noise_component.ay    = sensor_noise_hf_amp.ay*noise_part[1]*noise_sign[4];
    high_freq_noise_component.az    = sensor_noise_hf_amp.az*noise_part[0]*noise_sign[5];


        high_freq_noise_component.phi   = high_freq_noise_component_old.phi   + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.phi))  *( high_freq_noise_component.phi  -  high_freq_noise_component_old.phi) );
        high_freq_noise_component.theta = high_freq_noise_component_old.theta + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.theta))*( high_freq_noise_component.theta-  high_freq_noise_component_old.theta) );
        high_freq_noise_component.psi   = high_freq_noise_component_old.psi   + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.psi))  *( high_freq_noise_component.psi  -  high_freq_noise_component_old.psi) );
        high_freq_noise_component.p     = high_freq_noise_component_old.p     + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.p))    *( high_freq_noise_component.p    -  high_freq_noise_component_old.p) );
        high_freq_noise_component.q     = high_freq_noise_component_old.q     + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.q))    *( high_freq_noise_component.q    -  high_freq_noise_component_old.q) );
        high_freq_noise_component.r     = high_freq_noise_component_old.r     + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.r))    *( high_freq_noise_component.r    -  high_freq_noise_component_old.r) );
        high_freq_noise_component.u     = high_freq_noise_component_old.u     + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.u))    *( high_freq_noise_component.u    -  high_freq_noise_component_old.u) );
        high_freq_noise_component.v     = high_freq_noise_component_old.v     + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.v))    *( high_freq_noise_component.v    -  high_freq_noise_component_old.v) );
        high_freq_noise_component.w     = high_freq_noise_component_old.w     + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.w))    *( high_freq_noise_component.w    -  high_freq_noise_component_old.w) );
        high_freq_noise_component.lat   = high_freq_noise_component_old.lat   + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.lat))  *( high_freq_noise_component.lat  -  high_freq_noise_component_old.lat) );
        high_freq_noise_component.longt = high_freq_noise_component_old.longt + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.longt))*( high_freq_noise_component.longt-  high_freq_noise_component_old.longt) );
        high_freq_noise_component.z     = high_freq_noise_component_old.z     + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.z))    *( high_freq_noise_component.z    -  high_freq_noise_component_old.z) );
        high_freq_noise_component.ax    = high_freq_noise_component_old.ax    + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.ax))   *( high_freq_noise_component.ax   -  high_freq_noise_component_old.ax) );
        high_freq_noise_component.ay    = high_freq_noise_component_old.ay    + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.ay))   *( high_freq_noise_component.ay   -  high_freq_noise_component_old.ay) );
        high_freq_noise_component.az    = high_freq_noise_component_old.az    + ( (t_step_ins/(t_step_ins + sensor_hf_noise_lpff.az))   *( high_freq_noise_component.az   -  high_freq_noise_component_old.az) );



        high_freq_noise_component_old.phi   =  high_freq_noise_component.phi   ;
        high_freq_noise_component_old.theta =  high_freq_noise_component.theta ;
        high_freq_noise_component_old.psi   =  high_freq_noise_component.psi   ;
        high_freq_noise_component_old.p     =  high_freq_noise_component.p  ;
        high_freq_noise_component_old.q     =  high_freq_noise_component.q  ;
        high_freq_noise_component_old.r     =  high_freq_noise_component.r ;
        high_freq_noise_component_old.u     =  high_freq_noise_component.u;
        high_freq_noise_component_old.v     =  high_freq_noise_component.v ;
        high_freq_noise_component_old.w     =  high_freq_noise_component.w;
        high_freq_noise_component_old.lat   =  high_freq_noise_component.lat ;
        high_freq_noise_component_old.longt =  high_freq_noise_component.longt  ;
        high_freq_noise_component_old.z     =  high_freq_noise_component.z   ;
        high_freq_noise_component_old.ax    =  high_freq_noise_component.ax ;
        high_freq_noise_component_old.ay    =  high_freq_noise_component.ay  ;
        high_freq_noise_component_old.az    =  high_freq_noise_component.az ;





}



void fn_low_frequency_sensor_inacc_modeler(float t_step_ins)
{
    noise_index=0;
    while (noise_index<15)
	{
   	     srand(noise_index + time(NULL) );
   	     noise_part[noise_index]=rand()%10; // will make random values comes between 0 and 10
   	     noise_sign[noise_index]=powf(-1,fabsf(noise_part[noise_index]));
   	     noise_part[noise_index] = fabsf(noise_part[noise_index])/10.0f;

   	   if (noise_part[noise_index] >1.0)
   	   {
   		   noise_part[noise_index]=1.0;
	   }

   	     noise_index=noise_index+1;
	}
        // low frequency sensor inaccuracies input
        low_freq_inacc_component.phi   = sensor_accuracy.phi*noise_part[0]*noise_sign[1];
        low_freq_inacc_component.theta = sensor_accuracy.theta*noise_part[2]*noise_sign[3];
        low_freq_inacc_component.psi   = sensor_accuracy.psi*noise_part[4]*noise_sign[5];
        low_freq_inacc_component.p     = sensor_accuracy.p*noise_part[6]*noise_sign[7];
        low_freq_inacc_component.q     = sensor_accuracy.q*noise_part[8]*noise_sign[9];
        low_freq_inacc_component.r     = sensor_accuracy.r*noise_part[10]*noise_sign[11];
        low_freq_inacc_component.u     = sensor_accuracy.u*noise_part[12]*noise_sign[13];
        low_freq_inacc_component.v     = sensor_accuracy.v*noise_part[14]*noise_sign[0];
        low_freq_inacc_component.w     = sensor_accuracy.w*noise_part[1]*noise_sign[2];
        low_freq_inacc_component.lat   = sensor_accuracy.lat*noise_part[3]*noise_sign[4];
        low_freq_inacc_component.longt = sensor_accuracy.longt*noise_part[5]*noise_sign[6];
        low_freq_inacc_component.z     = sensor_accuracy.z*noise_part[7]*noise_sign[8];
        low_freq_inacc_component.ax    = sensor_accuracy.ax*noise_part[9]*noise_sign[10];
        low_freq_inacc_component.ay    = sensor_accuracy.ay*noise_part[11]*noise_sign[12];
        low_freq_inacc_component.az    = sensor_accuracy.az*noise_part[13]*noise_sign[14];


        low_freq_inacc_component.phi   = low_freq_inacc_component_old.phi   + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.phi))  *( low_freq_inacc_component.phi  -  low_freq_inacc_component_old.phi) );
        low_freq_inacc_component.theta = low_freq_inacc_component_old.theta + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.theta))*( low_freq_inacc_component.theta-  low_freq_inacc_component_old.theta) );
        low_freq_inacc_component.psi   = low_freq_inacc_component_old.psi   + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.psi))  *( low_freq_inacc_component.psi  -  low_freq_inacc_component_old.psi) );
        low_freq_inacc_component.p     = low_freq_inacc_component_old.p     + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.p))    *( low_freq_inacc_component.p    -  low_freq_inacc_component_old.p) );
        low_freq_inacc_component.q     = low_freq_inacc_component_old.q     + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.q))    *( low_freq_inacc_component.q    -  low_freq_inacc_component_old.q) );
        low_freq_inacc_component.r     = low_freq_inacc_component_old.r     + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.r))    *( low_freq_inacc_component.r    -  low_freq_inacc_component_old.r) );
        low_freq_inacc_component.u     = low_freq_inacc_component_old.u     + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.u))    *( low_freq_inacc_component.u    -  low_freq_inacc_component_old.u) );
        low_freq_inacc_component.v     = low_freq_inacc_component_old.v     + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.v))    *( low_freq_inacc_component.v    -  low_freq_inacc_component_old.v) );
        low_freq_inacc_component.w     = low_freq_inacc_component_old.w     + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.w))    *( low_freq_inacc_component.w    -  low_freq_inacc_component_old.w) );
        low_freq_inacc_component.lat   = low_freq_inacc_component_old.lat   + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.lat))  *( low_freq_inacc_component.lat  -  low_freq_inacc_component_old.lat) );
        low_freq_inacc_component.longt = low_freq_inacc_component_old.longt + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.longt))*( low_freq_inacc_component.longt-  low_freq_inacc_component_old.longt) );
        low_freq_inacc_component.z     = low_freq_inacc_component_old.z     + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.z))    *( low_freq_inacc_component.z    -  low_freq_inacc_component_old.z) );
        low_freq_inacc_component.ax    = low_freq_inacc_component_old.ax    + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.ax))   *( low_freq_inacc_component.ax   -  low_freq_inacc_component_old.ax) );
        low_freq_inacc_component.ay    = low_freq_inacc_component_old.ay    + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.ay))   *( low_freq_inacc_component.ay   -  low_freq_inacc_component_old.ay) );
        low_freq_inacc_component.az    = low_freq_inacc_component_old.az    + ( (t_step_ins/(t_step_ins + sensor_inacc_lpff.az))   *( low_freq_inacc_component.az   -  low_freq_inacc_component_old.az) );




        low_freq_inacc_component_old.phi   =  low_freq_inacc_component.phi   ;
        low_freq_inacc_component_old.theta =  low_freq_inacc_component.theta ;
        low_freq_inacc_component_old.psi   =  low_freq_inacc_component.psi   ;
        low_freq_inacc_component_old.p     =  low_freq_inacc_component.p     ;
        low_freq_inacc_component_old.q     =  low_freq_inacc_component.q     ;
        low_freq_inacc_component_old.r     =  low_freq_inacc_component.r     ;
        low_freq_inacc_component_old.u     =  low_freq_inacc_component.u     ;
        low_freq_inacc_component_old.v     =  low_freq_inacc_component.v     ;
        low_freq_inacc_component_old.w     =  low_freq_inacc_component.w     ;
        low_freq_inacc_component_old.lat   =  low_freq_inacc_component.lat   ;
        low_freq_inacc_component_old.longt =  low_freq_inacc_component.longt ;
        low_freq_inacc_component_old.z     =  low_freq_inacc_component.z     ;
        low_freq_inacc_component_old.ax    =  low_freq_inacc_component.ax    ;
        low_freq_inacc_component_old.ay    =  low_freq_inacc_component.ay    ;
        low_freq_inacc_component_old.az    =  low_freq_inacc_component.az    ;


}

void lat_long_to_xy(float latitude_point,float longitude_point,float Alt,float* pos) /*flat earth*/
{

	static float R_ned_ecef[3][3],ned_frame_origin_wrt_Ecef[3][1];
	float R = 6374049.0;						/* Radius of earth at local ground level */
	float temp3X3_1[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
	float temp3X3_2[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
	float temp3X1_1[3][1] = {{0.0},{0.0},{0.0}};
	float point_wrt_ecef[3][1] = {{0.0},{0.0},{0.0}};
	float point_wrt_ned[3][1]  = {{0.0},{0.0},{0.0}};
    float lat_ned=0, long_ned=0, Z_ned_frame=0;



		lat_ned     = s_home_state.lat/57.2957802; 	/* latitude of Ned origin */
		long_ned    = s_home_state.longt/57.2957802; /* longitude of Ned origin */
		Z_ned_frame = s_home_state.alt_msl ;

		temp3X3_1[0][0] = -sinf(lat_ned);
		temp3X3_1[0][1] = 0.0;
		temp3X3_1[0][2] = cosf(lat_ned);
		temp3X3_1[1][0] = 0.0;
		temp3X3_1[1][1] = 1.0;
		temp3X3_1[1][2] = 0.0;
		temp3X3_1[2][0] = -cosf(lat_ned);
		temp3X3_1[2][1] = 0.0;
		temp3X3_1[2][2] = -sinf(lat_ned);

		temp3X3_2[0][0] = cosf(long_ned);
		temp3X3_2[0][1] = sinf(long_ned);
		temp3X3_2[0][2] = 0.0;
		temp3X3_2[1][0] = -sinf(long_ned);
		temp3X3_2[1][1] = cosf(long_ned);
		temp3X3_2[1][2] = 0.0;
		temp3X3_2[2][0] = 0.0;
		temp3X3_2[2][1] = 0.0;
		temp3X3_2[2][2] = 1.0;

		MatrixMultiply_old(temp3X3_1,3,3,temp3X3_2,3,3,R_ned_ecef);

		ned_frame_origin_wrt_Ecef[0][0] = (R-Z_ned_frame)*cosf(lat_ned)*cosf(long_ned);
		ned_frame_origin_wrt_Ecef[1][0] = (R-Z_ned_frame)*cosf(lat_ned)*sinf(long_ned);
		ned_frame_origin_wrt_Ecef[2][0] = (R-Z_ned_frame)*sinf(lat_ned);

	latitude_point  = latitude_point/57.2957802; /* latitude of point in radian */
	longitude_point = longitude_point/57.2957802; /* longitude of point in radian */

//	latitude_point=latitude_point/57.29577;  /* latitude of point in radian */
//	longitude_point=longitude_point/57.29577; /* longitude of point in radian */

	point_wrt_ecef[0][0] = (R-Alt)*cosf(latitude_point)*cosf(longitude_point);
	point_wrt_ecef[1][0] = (R-Alt)*cosf(latitude_point)*sinf(longitude_point);
	point_wrt_ecef[2][0] = (R-Alt)*sinf(latitude_point);

	temp3X1_1[0][0] = point_wrt_ecef[0][0]-ned_frame_origin_wrt_Ecef[0][0];
	temp3X1_1[1][0] = point_wrt_ecef[1][0]-ned_frame_origin_wrt_Ecef[1][0];
	temp3X1_1[2][0] = point_wrt_ecef[2][0]-ned_frame_origin_wrt_Ecef[2][0];

	MatrixMultiply_old(R_ned_ecef,3,3,temp3X1_1,3,1,point_wrt_ned);


	*pos = point_wrt_ned[0][0];
	*(pos+1) = point_wrt_ned[1][0];
	*(pos+2) = Alt-Z_ned_frame;

//	ax_bd_ins[0]=ax_bd_ins[0]*g;
//	ax_bd_ins[1]=ax_bd_ins[1]*g;
//	ax_bd_ins[2]=ax_bd_ins[2]*g;

}


	
