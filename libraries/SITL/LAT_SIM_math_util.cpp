#include <math.h>
#ifdef DEBUG
#include <stdio.h>
#endif
#include "LAT_SIM_math_util.h"

int sign(float x)
{
	if(x<0)
	{ 
		return -1;
	}
	else if(x>0)
	{
		return 1;
	}
	else
	{
		return 0;
	}	
	
}

float sign_1(float p)
{
	if (p > 0.0)
   	{
		return 1.0;
   	}
   	else
   	{
   		if (p < 0.0)
   		{
	   		return -1.0;
   		}
   		else
   		{
	   		return 1.0;
   		}
   	}
   	return 1.0;
}

float sec(float z_r)
{
  return 1 / cosf(z_r);
}
void array_initd(float* arr,int num)
{	
#ifdef DEBUG
	if(num < 0)
	{
		printf("Negative array index passed %s,%d",__FILE__,__LINE__);
	}
#endif
    int i = 0;
    for(i=0;i<num;i++)
        *((float*)arr+i) = 0.0;
}

void array_initd_3X3(float source[][3])
{
	int i,j;

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			source[i][j] = 0;		
		}
	}
}


void transposed3x3(float source[][3])
{
	int i,j;

	float tempd3x3[3][3];
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			tempd3x3[i][j] = source[j][i];		
		}
	}
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			source[i][j] = tempd3x3[i][j];		
		}
	}
}


void transposedmxnAToB(int m, int n,  float* source, float* dest)
{
    for(int i = 0; i < m; i++)
        for(int j = 0; j < n; j++)
            dest[j*m + i] = source[i*n + j];
}




void copyMatrixd3x3(float source[][3],float dest[][3])
{
	int i,j;
	for(i=0;i<3;i++)
	{
	    for(j=0;j<3;j++)
  	    {
		dest[i][j] = source[i][j];
	    }
	}
}


void MatrixMultiply(
     uint32_t rowsA, uint32_t colsA, float *A,
     uint32_t rowsB, uint32_t colsB, float *B,
    float *C)
{
    for (uint32_t i = 0; i < rowsA; i++) {
        for (uint32_t j = 0; j < colsB; j++) {
            float sum = 0.0f;
            for (uint32_t k = 0; k < colsA; k++) {
                sum += A[i * colsA + k] * B[k * colsB + j];
            }
            C[i * colsB + j] = sum;
        }
    }
}


float constrain_float1(float current_value, float low, float high)
{
    /* The check for NaN as a float prevents propagation of floating point
     * errors through any function that uses constrain_value(). The normal
     * float semantics already handle -Inf and +Inf
     */
    if (isnan(current_value)!=0)
    {
//    	    INTERNAL_ERROR(AP_InternalError::error_t::constraining_nan);
        return (low + high) / 2;
    }

    if(current_value < low)
    {
        return low;
    }

    else if (current_value > high)
    {
        return high;
    }

    else
	{
    	return current_value;
	}
}

void cross_product(float a[],float b[], float c[])
{
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];
}

void dot_product(float a[],float b[], float *c)
{
   *c = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
