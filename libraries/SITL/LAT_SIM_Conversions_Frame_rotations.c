#include "LAT_SIM_Runner.h"
#include "LAT_SIM_Conversions_Frame_rotations.h"
#include "LAT_SIM_math_util.h"

static float temp3X1_1[3];
float V_b_d[3];
float temp3X3_1[3][3], temp3X3_2[3][3];
static float R_v_v1[3][3], R_v1_v2[3][3], R_v2_b[3][3], R_v_b[3][3];
static float R_s_w[3][3], R_b_s[3][3], R_w_b[3][3];
float Vector_original_2d_3X1[3][1], Vector_rotated_2d_3X1[3][1];
float Vector_original_2d_3X3[3][3], Vector_rotated_2d_3X3[3][3];

// Checks roll pitch euler angle are within 90 degree and
// does not allow more than 90 degree to enter the autopilot computation
void Euler_angle_singularity_checker(float *phi, float *theta)
{
	if((fabsf(*phi) > 89.0/57.3) && (fabsf(*phi < 91.0/57.3)))
	{
		*phi = (89.0/57.3) * sign_1(*phi);
	}

	if((fabsf(*theta)> 89.0/57.3) && (fabsf(*theta < 91.0/57.3)))
	{
		*theta = (89.0/57.3) * sign_1(*theta);
	}
}


// converts radian 2 degrees
void rad_2_deg(float *angle_)
{
	*angle_ = *angle_*(57.295779f);
}


// converts degrees 2 radian
void deg_2_rad(float *angle_)
{
	*angle_ = *angle_*(57.295779f);
}


// updates rotation matrices between different frames based on euler angles and wind angles (alpha , beta)
void v_rotation_matrices_update(float phi, float theta, float psi, float alpha, float beta)
{
	if((fabsf(phi)>89.0/57.3) && (fabsf(phi<91.0/57.3)))
	{
		phi = (fabsf(phi)/phi)*89/57.3;
	}

	if((fabsf(theta)>89.0/57.3) && (fabsf(theta<91.0/57.3)))
	{
		theta = (fabsf(theta)/theta)*89/57.3;
	}

	R_v_v1[0][0] = cosf(psi);
	R_v_v1[0][1] = sinf(psi);
	R_v_v1[0][2] = 0.0;
	R_v_v1[1][0] = -sinf(psi);
	R_v_v1[1][1] = cosf(psi);
	R_v_v1[1][2] = 0.0;
	R_v_v1[2][0] = 0.0;
	R_v_v1[2][1] = 0.0;
	R_v_v1[2][2] = 1.0;

	R_v1_v2[0][0] = cosf(theta);
	R_v1_v2[0][1] = 0.0;
	R_v1_v2[0][2] = -sinf(theta);
	R_v1_v2[1][0] = 0.0;
	R_v1_v2[1][1] = 1.0;
	R_v1_v2[1][2] = 0.0;
	R_v1_v2[2][0] = sinf(theta);
	R_v1_v2[2][1] = 0.0;
	R_v1_v2[2][2] = cosf(theta);

	R_v2_b[0][0] = 1.0;
	R_v2_b[0][1] = 0.0;
	R_v2_b[0][2] = 0.0;
	R_v2_b[1][0] = 0.0;
	R_v2_b[1][1] = cosf(phi);
	R_v2_b[1][2] = sinf(phi);
	R_v2_b[2][0] = 0.0;
	R_v2_b[2][1] = -sinf(phi);
	R_v2_b[2][2] = cosf(phi);

	R_b_s[0][0] = cosf(alpha);
	R_b_s[0][1] = 0.0;
	R_b_s[0][2] = sinf(alpha);
	R_b_s[1][0] = 0.0;
	R_b_s[1][1] = 1.0;
	R_b_s[1][2] = 0.0;
	R_b_s[2][0] = -sinf(alpha);
	R_b_s[2][1] = 0.0;
	R_b_s[2][2] = cosf(alpha);

	R_s_w[0][0] = cosf(beta);
	R_s_w[0][1] = sinf(beta);
	R_s_w[0][2] = 0.0;
	R_s_w[1][0] = -sinf(beta);
	R_s_w[1][1] = cosf(beta);
	R_s_w[1][2] = 0.0;
	R_s_w[2][0] = 0.0;
	R_s_w[2][1] = 0.0;
	R_s_w[2][2] = 1.0;

//	MatrixMultiply(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
	MatrixMultiply(3,3,R_v1_v2,3,3,R_v_v1,temp3X3_1);

//	MatrixMultiply(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);
    MatrixMultiply(3,3,R_v2_b,3,3,temp3X3_1,R_v_b);

    transposedmxnAToB(3,3,R_b_s, temp3X3_1);
	transposedmxnAToB(3,3,R_s_w, temp3X3_2);

//	MatrixMultiply(temp3X3_1,3,3,temp3X3_2,3,3,R_w_b);
	MatrixMultiply(3,3,temp3X3_1,3,3,temp3X3_2,R_w_b);

}



// Converts vector expressed in Ned frame to body frame
void NED_to_body(float Vector_original[], float  Vector_rotated[] )
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);

//	MatrixMultiply(R_v_b,3,3,Vector_original,3,1,Vector_rotated);
    MatrixMultiply(3,3,R_v_b,3,1,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

   *(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
   *(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
   *(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

    memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
    memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}


// Converts vector expressed in body frame to NED frame

void body_to_NED(float Vector_original[], float Vector_rotated[])
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);
	transposedmxnAToB(3,3,R_v_b,temp3X3_1);

//	MatrixMultiply(temp3X3_1,3,3,Vector_original,3,1,Vector_rotated);
    MatrixMultiply(3,3,temp3X3_1,3,1,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

    *(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
    *(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
    *(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

    memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
    memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}


// Converts vector expressed in windframe frame to body frame
void windframe_to_body(float Vector_original[], float Vector_rotated[])
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);

//	MatrixMultiply(R_w_b,3,3,Vector_original,3,1,Vector_rotated);
    MatrixMultiply(3,3,R_w_b,3,1,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

    *(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
    *(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
    *(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

    memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
    memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}

// Converts vector expressed in Ned frame to frame1
// frame1 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void NED_to_frame1(float Vector_original[], float Vector_rotated[])
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);

//	MatrixMultiply(R_v_v1,3,3,Vector_original,3,1,Vector_rotated);
    MatrixMultiply(3,3,R_v_v1,3,1,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

    *(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
    *(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
    *(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

    memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
    memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}

// Converts vector expressed in frame1 frame to NED
// frame1 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void frame1_to_NED(float Vector_original[], float Vector_rotated[])
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);
	transposedmxnAToB(3,3,R_v_v1,temp3X3_1);

//	MatrixMultiply(temp3X3_1,3,3,Vector_original,3,1,Vector_rotated);
    MatrixMultiply(3,3,temp3X3_1,3,1,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

    *(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
    *(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
    *(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

    memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
    memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}

// Converts vector expressed in frame1 frame to frame2
// frame1,frame2 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void frame1_to_frame2(float Vector_original[], float Vector_rotated[])
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);

//	MatrixMultiply(R_v1_v2,3,3,Vector_original,3,1,Vector_rotated);
    MatrixMultiply(3,3,R_v1_v2,3,1,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

    *(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
    *(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
    *(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

    memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
    memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}

// Converts vector expressed in frame2 frame to frame1
// frame1,frame2 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void frame2_to_frame1(float Vector_original[], float Vector_rotated[])
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);
	transposedmxnAToB(3,3,R_v1_v2,temp3X3_1);

//	MatrixMultiply(temp3X3_1,3,3,Vector_original,3,1,Vector_rotated);
    MatrixMultiply(3,3,temp3X3_1,3,1,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

    *(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
    *(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
    *(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

    memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
    memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}


// Converts vector expressed in frame1 frame to body
// frame1 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void frame1_to_body(float  Vector_original[], float Vector_rotated[])
{
	frame1_to_NED( Vector_original,  temp3X1_1);
	NED_to_body(temp3X1_1,Vector_rotated);
}

// Converts vector expressed in body to frame1
// frame1 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void body_to_frame1(float Vector_original[], float Vector_rotated[])
{
	body_to_NED( Vector_original,  temp3X1_1);
	NED_to_frame1(temp3X1_1,Vector_rotated);
}

// Converts vector expressed in body to frame2
// frame2 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void body_to_frame2(float Vector_original[], float Vector_rotated[])
{
	body_to_frame1( Vector_original,  temp3X1_1);
	frame1_to_frame2(temp3X1_1,Vector_rotated);
}

// Converts vector expressed in frame2 to body
// frame2 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void frame2_to_body(float Vector_original[], float Vector_rotated[])
{
	frame2_to_frame1( Vector_original,  temp3X1_1);
	frame1_to_body(temp3X1_1,Vector_rotated);
}

// Converts vector expressed in NED to frame2
// frame2 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void NED_to_frame2(float Vector_original[], float Vector_rotated[])
{
	oned_to_2d_3X1(Vector_original, Vector_rotated);

//	MatrixMultiply(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
	MatrixMultiply(3,3,R_v1_v2,3,3,R_v_v1,temp3X3_1);

//	MatrixMultiply(temp3X3_1,3,3,Vector_original,3,3,Vector_rotated);
	MatrixMultiply(3,3,temp3X3_1,3,3,Vector_original_2d_3X1,Vector_rotated_2d_3X1);

	*(Vector_rotated)     = Vector_rotated_2d_3X1[0][0];
	*(Vector_rotated + 1) = Vector_rotated_2d_3X1[1][0];
	*(Vector_rotated + 2) = Vector_rotated_2d_3X1[2][0];

	memset(Vector_original_2d_3X1,0,sizeof(Vector_original_2d_3X1));
	memset(Vector_rotated_2d_3X1,0,sizeof(Vector_original_2d_3X1));
}

// Converts vector expressed in frame2 to NED
// frame2 Reference :  Book- Small Unmanned Aircraft,Author-Randal Beard, Chapter 2: Coordinate Frames
void frame2_to_NED(float Vector_original[], float Vector_rotated[])
{
	frame2_to_frame1( Vector_original,  temp3X1_1);
	frame1_to_body(temp3X1_1,Vector_rotated);
}

// converts 1d, 3 element array to 2d 3X1 array
void oned_to_2d_3X1(float Vector_original[], float Vector_rotated[])
{
	for(int i = 0; i < 3;i++)
	{
		Vector_original_2d_3X1[i][0] = Vector_original[i];
		Vector_rotated_2d_3X1[i][0]  = Vector_rotated[i];
	}
}

// converts 1 d , 9 element array to 2d , 3X3 array
void oned_to_2d_3X3(float Vector_original[], float Vector_rotated[])
{
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3;j++)
		{
			Vector_original_2d_3X3[i][j] = *Vector_original++;
			Vector_rotated_2d_3X3[i][j]  = *Vector_rotated++;
		}
	}
}



// used to input transport delay in delay_array_T array
// the delay will be given by delay_array_length
/*float Delay_input_fn_T(float new_value_T)
{
	float temp_old_T = delay_array_T[delay_array_length-1];

	for (int id = delay_array_length-1 ; id > 0;  id = id-1)
	{
		delay_array_T[id] = delay_array_T[id-1];
	}

	delay_array_T[0] = new_value_T;

	return temp_old_T;
}

// used to input transport delay in delay_array_T1 array
// the delay will be given by delay_array_length
float Delay_input_fn_T1(float new_value_T)
{
	float temp_old_T = delay_array_T1[delay_array_length-1];

	for (int id = delay_array_length-1; id > 0 ; id = id-1)
	{
		delay_array_T1[id] = delay_array_T1[id-1];
	}
	delay_array_T1[0] = new_value_T;

	return temp_old_T;
}
*/

// wraps angle between -180 to 180
// used in heading wrapping
// returns wrapped angle
/*float Angle_Ranges(float p)
{
	if ( p > 360)
		p =	fmodf(p,360);

	if (p > 180)
		p = -180.0 + fmodf(p,180);

	if (p < -360)
		p = -fmodf(fabsf(p),360);

	if (p < -180)
		p= 180.0 - fmodf(fabsf(p),180);

/*	if (p>15.0)
	p = 15.0 ;//+ fmodf(p,5);

	if (p<-15.0)
	p = -15.0 ;//+ fmodf(p,5);*/
/*
	return p;
}
*/
