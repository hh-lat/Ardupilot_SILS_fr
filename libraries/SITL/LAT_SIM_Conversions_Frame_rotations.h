/*
 * Conversions_frame_rotations.h
 *
 *      Author: Rajat
 */

extern void oned_to_2d_3X1(float Vector_original[], float Vector_rotated[]);
extern void oned_to_2d_3X3(float Vector_original[], float Vector_rotated[]);
extern void NED_to_body(float [], float []);
extern void body_to_NED(float[] , float[]);
extern void windframe_to_body(float[] , float[]);
extern void NED_to_frame1(float[] , float[]);
extern void frame1_to_NED(float[] , float[]);
extern void frame1_to_frame2(float[] , float[]);
extern void frame2_to_frame1(float[] , float[]);
extern void frame1_to_body(float [] , float[]);
extern void body_to_frame1(float[] , float[]);
extern void body_to_frame2(float[] , float[]);
extern void frame2_to_body(float[] , float[]);
extern void NED_to_frame2(float[] , float[]);
extern void frame2_to_NED(float[] , float[]);
extern void v_rotation_matrices_update(float,float,float,float,float);
extern void Euler_angle_singularity_checker(float*, float*);
extern void rad_2_deg(float*);
extern void deg_2_rad(float*);




