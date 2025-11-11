/*
 * Earth_model.h
 
 *      Author: rajat
 */


extern void v_lla2ned(float lla_pos[3], float lla_0[3], int flag_earth_model, float nedPos[3]);

extern void v_lla2ecef(float lla_pos[3], float ecef_pos[3]);


void v_lla2ned_d(double lla_pos[3], double lla_0[3], int flag_earth_model, double nedPos[3]);

extern void v_lla2ecef_d(double lla_pos[3], double ecef_pos[3]);



