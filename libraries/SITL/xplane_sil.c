/*
 * xplane_sil.c
 * Author: Rakesh Vangaveti
 * Created On:26-08-2021
 * Description: this file having functions to recieve the data from the xplane,
 * and to send the data to xplane, UDP configuaration functions
 */


#include "xplane_connect.h"
#include "xplane_sil.h"
#include "rotor_dynamics.h"
#include "Actuator_dynamics.h"


// Number of motors (engines) to control added by Yovan 20-07-2023
// #define NUM_MOTORS 4
// #define XPLANE_PORT 49002



str_xp_states xp_states;

XPCSocket sock_send;
XPCSocket sock_recv;


float xp_snd_data[1][9] = { 0 };
// float throttleRatios[NUM_MOTORS] = {0.0}; // Throttle ratios for each motor (ranging from 0.0 to 1.0) added by Yovan 20-07-2023
float xp_recv_data[6][9] = { 0.0f },xp_recv_data_curr[6][9] = { 0 },xp_recv_data_prvs[6][9] = { 0 };
static float temp3X1_1[3][1]={0.0}, temp3X3_1[3][3]={0.0};
float R_v_b[3][3] ={0.0}, R_v_v1[3][3] ={0.0}, R_v1_v2[3][3] ={0.0},R_v2_b[3][3] ={0.0};
float fn_xplane_range_check(float* xp_range_data_in, float xp_data, float xp_range_max_init, float xp_range_max_norm, int xp_init_flg, int xp_rang_inc);
void fn_xp_fill_zero(int xp_fil_cols,int xp_r);

void fn_xplane_udp_open()
{
	//sock_send = aopenUDP("127.0.0.1", 49002, 49009);//send  //aopenUDP(IP*,Xplane port no.,client port no.) //do not change the port numbers
	//sock_recv = aopenUDP("127.0.0.1", 49001, 49007); //recv //aopenUDP(IP*,Xplane port no.,client port no.) //do not change the port numbers

	sock_send = aopenUDP("127.0.0.1", 49002, 49009);//send  //aopenUDP(IP*,Xplane port no.,client port no.) //do not change the port numbers
	sock_recv = aopenUDP("127.0.0.1", 49001, 49007); //recv //aopenUDP(IP*,Xplane port no.,client port no.) //do not change the port numbers
}



void fn_xplane_udp_close()
{
	closeUDP(sock_send);
	closeUDP(sock_recv);

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//static float t1=0;
//static float i = -1.2;
void fn_send_to_xplane_udp()
{
	//keep xp_out_rows value zero until you're not sending any data to xplane11, if you're sending any data mention the no.of rows that you're sending
//  commented 18 Aug 2023
	//keep xp_out_rows value zero until you're not sending any data to xplane11, if you're sending any data mention the no.of rows that you're sending
	 xp_states.xp_out_rows  = 2;

	float thrust_normalised =  (pwm_out_esc[4] - s_rotor.zero_fwv_thr_tor_pwm )*((1.0-0)/( -s_rotor.zero_fwv_thr_tor_pwm +  s_rotor.max_fwv_thr_tor_pwm));

    xp_snd_data[0][0] = 25; //engine_thrust in xplane data input output table
    // xp_snd_data[0][1] = 0.8;//0.00013760675 * fabs(rotor_force_out[4] * rotor_force_out[4]) + 0.027043305 * fabs(rotor_force_out[4]) + 0.02691607954;
	xp_snd_data[0][1] = thrust_normalised;//thrust_normalised;//0.003382819 * fabs(rotor_force_out[0] * rotor_force_out[0]) + 0.001695 * fabs(rotor_force_out[0]) + 0.11184058;  //
    xp_snd_data[0][2] = 0;//0.003382819 * fabs(rotor_force_out[1] * rotor_force_out[1]) + 0.001695 * fabs(rotor_force_out[1]) + 0.11184058;  //Right front
    xp_snd_data[0][3] = 0;//0.003382819 * fabs(rotor_force_out[2] * rotor_force_out[2]) + 0.001695 * fabs(rotor_force_out[2]) + 0.11184058;  //Right back
    xp_snd_data[0][4] = 0; //0.003382819 * fabs(rotor_force_out[3] * rotor_force_out[3]) + 0.001695 * fabs(rotor_force_out[3]) + 0.11184058;  //Left back;
    xp_snd_data[0][5] = 0;//0.003382819 * fabs(rotor_force_out[5] * rotor_force_out[5]) + 0.001695 * fabs(rotor_force_out[5]) + 0.11184058;  //Left front//pusher
    xp_snd_data[0][6] = 0;//0.003382819 * fabs(rotor_force_out[6] * rotor_force_out[6]) + 0.001695 * fabs(rotor_force_out[6]) + 0.11184058; 
    xp_snd_data[0][7] = 0; 
    xp_snd_data[0][8] = 0;  //example to follow, first element defines the which data you're sending rest all are data which you want modify values*/
//  commented 18 Aug 2023 till here



	/*To check control surfaces*/
	 xp_snd_data[1][0] = 138; //this is for Aileron/Elevator/Rudder


//  commented 18 Aug 2023
	  xp_snd_data[1][1] =  -actuator[ELEVATOR_COMMON].angle/(25.0f/57.3f);//Elevator 
	  xp_snd_data[1][2] = (actuator[AILERON_RIGHT].angle/(25.0f/57.3f)) * 2;//Aileron
	  xp_snd_data[1][3]  = -actuator[RUDDER_COMMON].angle/(25.0f/57.3f);

	 xp_snd_data[1][4] = 0; 
	 xp_snd_data[1][5] = 0;

	 xp_snd_data[1][6] = 0;
	 xp_snd_data[1][7] = 0;
	 xp_snd_data[1][8] = 0;
	 

	// sendDATA(sock_send, xp_snd_data, xp_states.xp_out_rows);

/******************************************************* for sending commands via dataref***********************************************************/

 char dref_4[]  = "sim/cockpit2/engine/actuators/throttle_ratio[0]\0";
 float value_4 = xp_snd_data[0][1];
 v_custom_send_dref(sock_send,dref_4,value_4);

// char dref_0[]  = "sim/cockpit2/engine/actuators/throttle_ratio_all\0";
// float value_0 = 1;//xp_snd_data[0][1]; //1;
//  v_custom_send_dref(sock_send,dref_0,value_0);


//  char dref_1[]  = "sim/cockpit2/engine/actuators/throttle_ratio[1]\0";
//  float value_1 = xp_snd_data[0][1]; //1;
//  v_custom_send_dref(sock_send,dref_1,value_1);

//  char dref_2[]  = "sim/cockpit2/engine/actuators/throttle_ratio[2]\0";
//  float value_2 = xp_snd_data[0][3]; //1;
//  v_custom_send_dref(sock_send,dref_2,value_2);
		
// char dref_3[]  = "sim/cockpit2/engine/actuators/throttle_ratio[3]\0";
//  float value_3 = xp_snd_data[0][4]; //1;
//  v_custom_send_dref(sock_send,dref_3,value_3);

//  char dref_4[]  = "sim/cockpit2/engine/actuators/throttle_ratio[4]\0";
//  float value_4 = xp_snd_data[0][5]; //1;
//  v_custom_send_dref(sock_send,dref_4,value_4);

// control surfaces

char dref_ele[]  = "sim/joystick/yoke_pitch_ratio\0";
 float value_ele = xp_snd_data[1][1]; //1;
 v_custom_send_dref(sock_send,dref_ele,value_ele);

char dref_ail[]  = "sim/joystick/yoke_roll_ratio\0";
 float value_ail = xp_snd_data[1][2]; //1;
 v_custom_send_dref(sock_send,dref_ail,value_ail);

char dref_rud[]  = "sim/joystick/yoke_heading_ratio\0";
 float value_rud = xp_snd_data[1][3]; //1;
 v_custom_send_dref(sock_send,dref_rud,value_rud);
		
}

/******************************************************OpenGL Conversion*****************************************************************************************************/



// const double EarthRadius = 6378137.0; // Earth's semi-major axis in meters


// double latitude, longitude, altitude;
// void latLonAltToOpenGL( double *openglX, double *openglY, double *openglZ) 
// {
//     // Convert latitude and longitude to radians
	
//      latitude = vehicle.lat * PI / 180.0;
//      longitude = vehicle.lon * PI / 180.0;
// 	 altitude = vehicle.alt_msl * 3.28084;

    // Calculate ECEF (Earth-Centered, Earth-Fixed) coordinates
    // double N = EarthRadius / sqrt(1.0 - (sin(latitude) * sin(latitude)));

    // *openglY = (N + altitude) * cos(latitude) * cos(longitude);
    // *openglX = (N + altitude) * cos(latitude) * sin(longitude);
    // *openglZ = (N * (1.0 - (0.081819192 * 0.081819192)) + altitude) * sin(latitude); // 0.08181919 is commonly used to represent the eccentricity of the Earth's ellipsoid

	// double maxCoordinate = EarthRadius; //+ altitude;
	// *openglY = *openglY - *openglY; // maxCoordinate;
	// *openglX = *openglX - *openglX; // maxCoordinate;
	// *openglZ = *openglZ - *openglZ; // maxCoordinate;
//}


/*****************************************************************New Geodetic***********************************************************************/ 
//    #define SEMI_MAJOR_AXIS 6378137.0 // Earth's semi-major axis in meters
// #define SEMI_MINOR_AXIS 6356752.314245 // Earth's semi-minor axis in meters
// #define ECCENTRICITY_SQUARED 0.006694379990141 // Eccentricity squared



// // Function to convert latitude and longitude to geodetic coordinates
// void latLongToGeodetic(double latitude, double longitude, double* altitude, double *openglX, double *openglY, double *openglZ) 
// {
//      latitude = vehicle.lat * PI / 180.0;
//      longitude = vehicle.lon * PI / 180.0;
//    // *altitude = vehicle.alt_msl * 3.28084;
//     double sinLat = sin(latitude);
//     double cosLat = cos(longitude);
    
//     double radiusOfCurvature = SEMI_MAJOR_AXIS / sqrt(1.0 - ECCENTRICITY_SQUARED * sinLat * sinLat);
    
//     double x = (radiusOfCurvature + *altitude) * cosLat * cos(longitude);
//     double y = (radiusOfCurvature + *altitude) * cosLat * sin(longitude);
//     double z = 2000; //((1.0 - ECCENTRICITY_SQUARED) * radiusOfCurvature + *altitude) * sinLat;
    
//     *altitude = sqrt(x * x + y * y + z * z) - radiusOfCurvature;

// 	*openglX = x;
// 	*openglY = y;
// 	*openglZ = z;

// }

/*********************************************************different method*********************************************************************************/

// # define  SCALE 9000
// void latLonAltToOpenGL( double *openglX, float *openglY, float *openglZ) 
// {
// 	double latitude, longitude, altitude, shiftX, shiftY;
// 	double x = 0.0, y = 0.0, w = 200.0f, N = 0.0, EarthRadius = 6378137.0;
	
// 	latitude = vehicle.lat * PI / 180.0;
// 	longitude = vehicle.lon * PI / 180.0;
// 	altitude = vehicle.alt_msl * 3.28084;

// 	y = (w / (2 * PI) * log(tan(PI / 4 + latitude / 2)) * SCALE);
// 	shiftY = -((w / (2 * PI) * log(tan(PI / 4 + latitude / 2)) * SCALE));
// 	y = y + shiftY;
// 	*openglY = y / EarthRadius ;
	
// 	x = ((w / (2 * PI)) * (longitude)* SCALE);
// 	shiftX = -(((w / (2 * PI)) * (longitude)* SCALE));
// 	x = -(x + shiftX);
// 	*openglX = x / EarthRadius;

// 	N = EarthRadius / sqrt(1.0 - (sin(latitude) * sin(latitude)));
// 	*openglZ = (N * (1.0 - (0.081819192 * 0.081819192)) + altitude) * sin(latitude); // 0.08181919 is commonly used to represent the eccentricity of the Earth's ellipsoid
// 	*openglZ = *openglZ / EarthRadius;
// }

/**************************************************************End of OpenGL conversion***************************************************************************************/


void fn_send_to_xplane_rk4_udp()
{

// 	double openglX, openglY, openglZ; // (altitude);


// /********************************************* for sending commands via dataref only for visualisation ***************************************************************/

// char dref_4[]  = "sim/cockpit2/engine/actuators/throttle_ratio[0]\0";
//  float value_4 = 0;//xp_snd_data[0][1];
//  v_custom_send_dref(sock_send,dref_4,value_4);

// //  char dref_physics_override[]  = "sim/operation/override/override_planepath[0]\0";
// //  float value_override = 1;
// //  v_custom_send_dref(sock_send,dref_physics_override,value_override);



//  char dref_phi[]  = "sim/flightmodel/position/phi\0";
//  float value_phi = vehicle.phi * 57.3;
//  v_custom_send_dref(sock_send,dref_phi,value_phi);

// char dref_theta[]  = "sim/flightmodel/position/theta\0";
// float value_theta = vehicle.theta * 57.3;
//  v_custom_send_dref(sock_send,dref_theta,value_theta);


//  char dref_psi[]  = "sim/flightmodel/position/psi\0";
//  float value_psi = vehicle.psi * 57.3;
//  v_custom_send_dref(sock_send,dref_psi,value_psi);

 
//   char dref_long[]  = "sim/flightmodel/position/local_x\0"; //"sim/flightmodel/position/latitude\0"; // 
//  float value_long = openglY; //openglY; 
//  v_custom_send_dref(sock_send,dref_long,value_long);

//  char dref_lat[]  =  "sim/flightmodel/position/local_z\0"; //"sim/flightmodel/position/longitude\0"; //
//  float value_lat = openglX; //-altitude; //openglX; //
//  v_custom_send_dref(sock_send,dref_lat,value_lat);

//  char dref_alt[]  = "sim/flightmodel/position/local_y\0";//"sim/flightmodel/position/elevation\0"; // 
//  float value_alt = openglZ; //-openglZ; 
//  v_custom_send_dref(sock_send,dref_alt,value_alt);
		
}


/*
******************************************************for receiving data from xplane*********************************************************************
*/

void fn_recv_frm_xplane_udp()
{
	int rows=999;

	xp_states.xp_in_rows  = 6;

	rows=readDATA(sock_recv, xp_recv_data, 	xp_states.xp_in_rows);
			//printf("%d\n",rows);
}


void fn_xp_fill_ideal_pls(float* ideal_plane_states, float* latitude_point , float* longitude_point , float* Alt)
{



	*ideal_plane_states      = xp_states.v_bd[0]; // body velocities in m/sec
	*(ideal_plane_states+1)  = xp_states.v_bd[1];
	*(ideal_plane_states+2)  = xp_states.v_bd[2];

	*(ideal_plane_states+3)  = xp_states.p; // body rates in rad/sec
	*(ideal_plane_states+4)  = xp_states.q;
	*(ideal_plane_states+5)  = xp_states.r;

	*(ideal_plane_states+6)  = xp_states.phi; // euler angles in rad
	*(ideal_plane_states+7)  = xp_states.theta;
	*(ideal_plane_states+8)  = xp_states.psi;

	*latitude_point  =  xp_states.lat;
	*longitude_point =  xp_states.longt;
	*Alt             =  xp_states.alt_msl;
}


void fn_xp_fill_xp_states()
{
	for (int i = 0 ; i< xp_states.xp_in_rows ; i++)
	{
		switch((int)xp_recv_data[i][0])
		{
 
		case 3:
			fn_xplane_range_check(&xp_states.v_tas, xp_recv_data[i][3]*0.514f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit kts but here converted to m/sec
			fn_xplane_range_check(&xp_states.v_in, xp_recv_data[i][1]*0.514f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit kts but here converted to m/sec
			fn_xplane_range_check(&xp_states.v_true_gnd, xp_recv_data[i][4]*0.514f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit kts but here converted to m/sec
			fn_xplane_range_check(&xp_states.v_in_eq, xp_recv_data[i][2]*0.514f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit kts but here converted to m/sec

			break;


		case 16:
			fn_xplane_range_check(&xp_states.p, xp_recv_data[i][2], 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit rad/sec
			fn_xplane_range_check(&xp_states.q, xp_recv_data[i][1], 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit rad/sec
			fn_xplane_range_check(&xp_states.r, xp_recv_data[i][3], 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit rad/sec

			break;


		case 17:
			fn_xplane_range_check(&xp_states.phi, xp_recv_data[i][2]/57.2958f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit degs but here converted to rad
			fn_xplane_range_check(&xp_states.theta, xp_recv_data[i][1]/57.2958f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit degs but here converted to rad
			fn_xplane_range_check(&xp_states.psi, xp_recv_data[i][3]/57.2958f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit degs but here converted to rad
			fn_xplane_range_check(&xp_states.mag_psi, xp_recv_data[i][4]/57.2958f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc);  // original unit degs but here converted to rad

			break;


		case 18:
			fn_xplane_range_check(&xp_states.aoa, xp_recv_data[i][1]/57.2958f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit degs but here converted to rad
			fn_xplane_range_check(&xp_states.ssa, xp_recv_data[i][2]/57.2958f, 1000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit degs but here converted to rad

			break;


		case 20:
			fn_xplane_range_check(&xp_states.lat, xp_recv_data[i][1], 1000.0, 0.002, xp_states.fst_fill, xp_states.valid_inc); // original unit degs
			fn_xplane_range_check(&xp_states.longt, xp_recv_data[i][2], 1000.0, 0.002, xp_states.fst_fill, xp_states.valid_inc); // original unit degs
			// negative sign multiplied to make Xplane compatible with going up negative direction, Xplane defaults send alt_msl in positive
			fn_xplane_range_check(&xp_states.alt_msl,-xp_recv_data[i][3]*0.3048f, 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit ft , but here converted to m
			fn_xplane_range_check(&xp_states.alt_agl,-xp_recv_data[i][4]*0.3048f, 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit ft , but here converted to m
            // -ve sign in altitude to ensure going up is negative
			break;


		case 21:
			fn_xplane_range_check(&xp_states.pos_ned[0], xp_recv_data[i][1], 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit m
			fn_xplane_range_check(&xp_states.pos_ned[1], xp_recv_data[i][2], 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit m
			fn_xplane_range_check(&xp_states.pos_ned[2], xp_recv_data[i][3], 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit m

			fn_xplane_range_check(&xp_states.v_ned[0], xp_recv_data[i][4], 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit m/sec
			fn_xplane_range_check(&xp_states.v_ned[1], xp_recv_data[i][5], 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit m/sec
			fn_xplane_range_check(&xp_states.v_ned[2], xp_recv_data[i][6], 50000.0, 20.0, xp_states.fst_fill, xp_states.valid_inc); // original unit m/sec

			xp_states.v_ned[0]=-xp_states.v_ned[2];
			xp_states.v_ned[1]= xp_states.v_ned[0];
			xp_states.v_ned[2]=-xp_states.v_ned[1];

			xp_states.v_bd[0]=xp_states.v_ned[0]*cos(xp_states.psi)*cos(xp_states.theta) - xp_states.v_ned[2]*sin(xp_states.theta) + xp_states.v_ned[1]*cos(xp_states.theta)*sin(xp_states.psi);
			xp_states.v_bd[1]=xp_states.v_ned[1]*(cos(xp_states.phi)*cos(xp_states.psi) + sin(xp_states.phi)*sin(xp_states.psi)*sin(xp_states.theta)) - xp_states.v_ned[0]*(cos(xp_states.phi)*sin(xp_states.psi) - cos(xp_states.psi)*sin(xp_states.phi)*sin(xp_states.theta)) + xp_states.v_ned[2]*cos(xp_states.theta)*sin(xp_states.phi);
			xp_states.v_bd[2]=xp_states.v_ned[0]*(sin(xp_states.phi)*sin(xp_states.psi) + cos(xp_states.phi)*cos(xp_states.psi)*sin(xp_states.theta)) - xp_states.v_ned[1]*(cos(xp_states.psi)*sin(xp_states.phi) - cos(xp_states.phi)*sin(xp_states.psi)*sin(xp_states.theta)) + xp_states.v_ned[2]*cos(xp_states.phi)*cos(xp_states.theta);
			break;
		default:

			break;

		}

	}
	//fn_xp_v_ned_to_vbd();

	/*comparing the valid_inc with no.of times you're calling the fn_xplane_range_check()-
	 * when this function succeeds it will increment the valid_inc
	 */
	if(xp_states.valid_inc == 23)
	{
		xp_states.fst_fill = 1;
	}
	else
	{

	}

}

void fn_xp_v_ned_to_vbd()
{

	R_v_v1[0][0]= cosf(xp_states.psi );
	R_v_v1[0][1]= sinf(xp_states.psi );
	R_v_v1[0][2]= 0.0;
	R_v_v1[1][0]= (-sinf(xp_states.psi ));
	R_v_v1[1][1]= cosf(xp_states.psi );
	R_v_v1[1][2]= 0.0;
	R_v_v1[2][0]= 0.0;
	R_v_v1[2][1]= 0.0;
	R_v_v1[2][2]= 1.0;

	R_v1_v2[0][0]= cosf(xp_states.theta );
	R_v1_v2[0][1]= 0.0;
	R_v1_v2[0][2]= (-sinf(xp_states.theta ));
	R_v1_v2[1][0]= 0.0;
	R_v1_v2[1][1]= 1.0;
	R_v1_v2[1][2]= 0.0;
	R_v1_v2[2][0]= sinf(xp_states.theta );
	R_v1_v2[2][1]= 0.0;
	R_v1_v2[2][2]= cosf(xp_states.theta );

	R_v2_b[0][0]=1.0;
	R_v2_b[0][1]=0.0;
	R_v2_b[0][2]=0.0;
	R_v2_b[1][0]=0.0;
	R_v2_b[1][1]=cosf(xp_states.phi );
	R_v2_b[1][2]=sinf(xp_states.phi );
	R_v2_b[2][0]=0.0;
	R_v2_b[2][1]= (-sinf(xp_states.phi ));
	R_v2_b[2][2]=cosf(xp_states.phi );

	MatrixMultiply_old(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
	MatrixMultiply_old(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);

	temp3X1_1[0][0]=xp_states.v_ned[0];
	temp3X1_1[1][0]=xp_states.v_ned[1];
	temp3X1_1[2][0]=xp_states.v_ned[2];

	MatrixMultiply_old(R_v_b,3,3,temp3X1_1,3,1,xp_states.v_bd);//% body velocity in body frame , unit will out in m/sec

}



float fn_xplane_range_check(float* xp_range_data_in, float xp_data, float xp_range_max_init, float xp_range_max_norm, int xp_init_flg, int xp_rang_inc)
{
	unsigned int xp_rg_val = 0;
	float xp_data_curr = 0.0;
	if(xp_data == -999.0)
	{

	}
	else
	{
		xp_data_curr =  xp_data;
		xp_rg_val = abs((*xp_range_data_in) - xp_data_curr);
		if(xp_init_flg == 1)
		{
			if (xp_rg_val <= xp_range_max_norm)
			{
				*xp_range_data_in = xp_data_curr;
				xp_rang_inc++;
			}
			else
			{
				xp_data_curr = (*xp_range_data_in);
			}
		}
		else if (xp_rg_val <= xp_range_max_init)
		{
			*xp_range_data_in = xp_data_curr;
			xp_rang_inc++;
		}
		else
		{

		}

	}
	// printf("data:%f\n",xp_snd_data[1][1]);
	//printf("data:%f\n",xp_data);
	//printf("data_curr:%f\n",xp_data_curr);
	//printf("data_prvs:%f\n",*xp_range_data_in);

	xp_states.valid_inc = xp_rang_inc ;
	return xp_data_curr;

}







void fn_xplane_init()
{
	xp_states.xp_valid_data;
	int inc =0;
	while(!xp_states.xp_recv_valid || !xp_states.fst_fill )
	{
		fn_recv_frm_xplane_udp();
		for (int i = 0 ; i< xp_states.xp_in_rows ; i++)
		{

			switch ((int)xp_recv_data[i][0])
			{
			case 3:
				xp_states.xp_in_cols = 5;//no.of cols u are using from xplane's data for ex. from [0][0] to [0][6] then you have to assign 7 cols
				fn_xp_fill_zero(xp_states.xp_in_cols,i); // xp_states.xp_in_cols = header + data columns

				break;

			case 16:
				xp_states.xp_in_cols = 4;
				fn_xp_fill_zero(xp_states.xp_in_cols,i);

				break;

			case 17:
				xp_states.xp_in_cols = 5;
				fn_xp_fill_zero(xp_states.xp_in_cols,i);

				break;

			case 18:
				xp_states.xp_in_cols = 3;
				fn_xp_fill_zero(xp_states.xp_in_cols,i);

				break;

			case 20:
				xp_states.xp_in_cols = 5;
				fn_xp_fill_zero(xp_states.xp_in_cols,i);

				break;

			case 21:
				xp_states.xp_in_cols = 7;
				fn_xp_fill_zero(xp_states.xp_in_cols,i);

				break;

			default:

				break;
			}

		}

		inc =0;
		for (int i = 0 ; i< xp_states.xp_in_rows ; i++)
		{
			for(int j=0;j<9;j++)
			{
				if(xp_recv_data[i][j] == -999.0)
				{
					break;
				}
				else
				{
					inc++;
				}
			}
		}
		if(inc == (9*(xp_states.xp_in_rows)))
		{
			xp_states.xp_recv_valid = 1;
		}

		fn_xp_fill_xp_states();

		sleep(0.3);

	}
}


void fn_xp_fill_zero(int xp_fil_cols,int xp_r)
{

	for(int j= xp_fil_cols;j<9;j++)
	{
		xp_recv_data[xp_r][j] =0.0;
	}

}







