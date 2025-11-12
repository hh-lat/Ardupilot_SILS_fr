/*
 * Conversions_frame_rotations.h
 *
 *      Author: Rajat
 */

void oned_to_2d_3X1(float Vector_original[], float Vector_rotated[]);
void oned_to_2d_3X3(float Vector_original[], float Vector_rotated[]);
void NED_to_body(float [], float []);
void body_to_NED(float[] , float[]);
void windframe_to_body(float[] , float[]);
void NED_to_frame1(float[] , float[]);
void frame1_to_NED(float[] , float[]);
void frame1_to_frame2(float[] , float[]);
void frame2_to_frame1(float[] , float[]);
void frame1_to_body(float [] , float[]);
void body_to_frame1(float[] , float[]);
void body_to_frame2(float[] , float[]);
void frame2_to_body(float[] , float[]);
void NED_to_frame2(float[] , float[]);
void frame2_to_NED(float[] , float[]);
void v_rotation_matrices_update(float,float,float,float,float);
void Euler_angle_singularity_checker(float*, float*);
void rad_2_deg(float*);
void deg_2_rad(float*);




