#include <math.h>
#ifdef DEBUG
#include <stdio.h>
#endif
#include "LAT_SIM_math_util.h"
//plant
float determinant(float **a,int n);
void cofactor(float **a,int n,float **b);
void transpose(float **a, int n);

float Det3x3(float d3x3A[3][3]);
float Det4x4(float d4x4A[4][4]);
float Det6x6(float d6x6InputMx[6][6]);
float Det5x5(float d5x5InputMx[5][5]);

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

/*
void array_initd_n_m(int n,int m,float arr[][m])
{
        int i =0;j=0;
        for(i=0;i<n;i++)
			for(j=0;j<m;j++)
				arr[i][j]=0.0;
                
}
*/
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

void transposed3x2(float source[][2])
{
	int i,j;

	float tempd2x3[2][3];
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<2;j++)
		{
			tempd2x3[j][i] = source[i][j];		
		}
	}
	for(i=0;i<2;i++)
	{
		for(j=0;j<3;j++)
		{
			source[i][j] = tempd2x3[i][j];		
		}
	}
}

void transposedmxn(int m,int n,float source[][n])
{
	int i,j;

	float tempdnxm[n][m];
	
	for(i=0;i<n;i++)
	{
		for(j=0;j<m;j++)
		{
			tempdnxm[i][j] = source[j][i];	
		}
	}
	for(i=0;i<n;i++)
	{
		for(j=0;j<m;j++)
		{
			source[i][j] = tempdnxm[i][j];	
		}
	}
}


void transposedmxnAToB(int m, int n,float source[][n], float dest[][m])
{
	int i=0,j=0;
	for(i=0;i<n;i++)
		for(j=0;j<m;j++)
			dest[i][j] = source[j][i];	

}
void transposedlxmxnAToB(int l,int m,int n,float source[l][m][n],float dest[n][m][l])
{
	int i,j,k;
		for(i=0;i<n;i++)
			for(j=0;j<m;j++)
				for(k=0;k<l;k++)
				dest[i][j][k] = source[k][j][i];	
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





void MatrixMultiply_old(float *dpA,unsigned int usNoOfRowsOfA,unsigned int usNoOfColumnsOfA,float *dpB,unsigned int usNoOfRowsOfB,unsigned int usNoOfColumnsOfB,float *dpC)
{
	unsigned usRowIndA,usColIndB,usColIndA;//7x7x7x3  : 7x3
	float dSum;

	for(usRowIndA = 0;usRowIndA < usNoOfRowsOfA;usRowIndA++) //7
	{
		for(usColIndB = 0;usColIndB < usNoOfColumnsOfB;usColIndB++)//3
		{
			dSum = 0.0;
			for(usColIndA = 0;usColIndA < usNoOfColumnsOfA;usColIndA++)//7
			{
				dSum = dSum + (*(dpA+usRowIndA*usNoOfColumnsOfA+usColIndA)) * (*(dpB+usColIndA*usNoOfColumnsOfB+usColIndB));
			}
			*(dpC+usRowIndA*usNoOfColumnsOfB+usColIndB) = dSum;
		}
	}
}


void MatrixMultiply(unsigned int usNoOfRowsOfA,unsigned int usNoOfColumnsOfA,float dpA[][usNoOfColumnsOfA],unsigned int usNoOfRowsOfB,unsigned int usNoOfColumnsOfB,float dpB[][usNoOfColumnsOfB],float dpC[usNoOfRowsOfA][usNoOfColumnsOfB])
{
	unsigned usRowIndA,usColIndB,usColIndA;//7x7x7x3  : 7x3
	float dSum;

	for(usRowIndA = 0;usRowIndA < usNoOfRowsOfA;usRowIndA++) //7
	{
		for(usColIndB = 0;usColIndB < usNoOfColumnsOfB;usColIndB++)//3
		{
			dSum = 0.0;
			for(usColIndA = 0;usColIndA < usNoOfColumnsOfA;usColIndA++)//7
			{
				dSum = dSum + (*(*(dpA+usRowIndA) + usColIndA)) * (*(*(dpB+usColIndA)  + usColIndB));
			}
			*(*(dpC+usRowIndA) +usColIndB) = dSum;
		}
	}
}



void MatrixInverse3x3(float d3x3A[3][3],float d3x3AInv[3][3])
{
    float d3x3CoFactors[3][3],d3x3Trans[3][3];
    float dDet,dDetInv;
    unsigned short int usRowInd,usColInd;
    
    dDet  =  0.0;
    for(usRowInd = 0,usColInd = 0;usColInd < 3;usColInd++)
        {    
               if(usColInd==2) 
               dDet += d3x3A[usRowInd][usColInd] * d3x3A[usRowInd+1][0]   * d3x3A[usRowInd+2][1];
               else if(usColInd==1)
               dDet += d3x3A[usRowInd][usColInd] * d3x3A[usRowInd+1][usColInd+1] * d3x3A[usRowInd+2][0];
               else 
               dDet += d3x3A[usRowInd][usColInd] * d3x3A[usRowInd+1][usColInd+1] * d3x3A[usRowInd+2][usColInd+2];
        }
    
    for(usRowInd = 2,usColInd = 0;usColInd < 3;usColInd++)
        {    
               if(usColInd == 2)  
               dDet -= d3x3A[usRowInd][usColInd] * d3x3A[usRowInd-1][0]   * d3x3A[usRowInd-2][1];
               else if(usColInd == 1)
               dDet -= d3x3A[usRowInd][usColInd] * d3x3A[usRowInd-1][usColInd+1] * d3x3A[usRowInd-2][0];
               else
               dDet -= d3x3A[usRowInd][usColInd] * d3x3A[usRowInd-1][usColInd+1] * d3x3A[usRowInd-2][usColInd+2];
        }

    d3x3CoFactors[0][0] = d3x3A[1][1] * d3x3A[2][2] - (d3x3A[2][1] * d3x3A[1][2]);
    d3x3CoFactors[0][1] = (-1) * (d3x3A[1][0] * d3x3A[2][2] - (d3x3A[2][0] * d3x3A[1][2]));
    d3x3CoFactors[0][2] = d3x3A[1][0] * d3x3A[2][1] - (d3x3A[2][0] * d3x3A[1][1]);
    
    d3x3CoFactors[1][0] = (-1) * (d3x3A[0][1] * d3x3A[2][2] - d3x3A[2][1] * d3x3A[0][2]);
    d3x3CoFactors[1][1] = d3x3A[0][0] * d3x3A[2][2] - d3x3A[2][0] * d3x3A[0][2];
    d3x3CoFactors[1][2] = (-1) * (d3x3A[0][0]*d3x3A[2][1] - d3x3A[2][0] * d3x3A[0][1]);

    d3x3CoFactors[2][0] = d3x3A[0][1] * d3x3A[1][2] - d3x3A[1][1] * d3x3A[0][2];
    d3x3CoFactors[2][1] = (-1) * (d3x3A[0][0] * d3x3A[1][2] - d3x3A[1][0] * d3x3A[0][2]);
    d3x3CoFactors[2][2] = d3x3A[0][0] * d3x3A[1][1] - d3x3A[1][0] * d3x3A[0][1];

    for(usRowInd = 0;usRowInd < 3;usRowInd++)
     {   
          for(usColInd = 0;usColInd < 3;usColInd++)
          {    
    
               d3x3Trans[usRowInd][usColInd] = d3x3CoFactors[usColInd][usRowInd];
    
          }
     }   

    if(dDet >= -1.0E-20 && dDet <= 1.0E-20 )
    {
        for(usRowInd = 0;usRowInd < 3;usRowInd++)
         {
          for(usColInd = 0;usColInd < 3;usColInd++)
          {
               d3x3AInv[usRowInd][usColInd] = 0.0;
          }
     }
    }
    else
    {
        dDetInv = 1.0 / dDet;
        for(usRowInd = 0;usRowInd < 3;usRowInd++)
         {
          for(usColInd = 0;usColInd < 3;usColInd++)
          {
               d3x3AInv[usRowInd][usColInd] = d3x3Trans[usRowInd][usColInd] * dDetInv;
          }
     }
    }

}

float Det3x3(float d3x3A[3][3])
{
 float dDetValue3X3;

 dDetValue3X3 = d3x3A[0][0]*(d3x3A[1][1]*d3x3A[2][2] - d3x3A[1][2]*d3x3A[2][1]) - d3x3A[0][1]*(d3x3A[1][0]*d3x3A[2][2]-d3x3A[1][2]*d3x3A[2][0]) + d3x3A[0][2]*(d3x3A[1][0]*d3x3A[2][1]- d3x3A[1][1]*d3x3A[2][0]);

 return dDetValue3X3;
}

float Det4x4(float d4x4A[4][4])
{
 float d3x3A[3][3];
 float dDetValue,dDetValue00,dDetValue01,dDetValue02,dDetValue03;

 d3x3A[0][0] = d4x4A[1][1];
 d3x3A[0][1] = d4x4A[1][2];
 d3x3A[0][2] = d4x4A[1][3];
 d3x3A[1][0] = d4x4A[2][1];
 d3x3A[1][1] = d4x4A[2][2];
 d3x3A[1][2] = d4x4A[2][3];
 d3x3A[2][0] = d4x4A[3][1];
 d3x3A[2][1] = d4x4A[3][2];
 d3x3A[2][2] = d4x4A[3][3];

 dDetValue00 = Det3x3(d3x3A);

 d3x3A[0][0] = d4x4A[1][0];
 d3x3A[0][1] = d4x4A[1][2];
 d3x3A[0][2] = d4x4A[1][3];
 d3x3A[1][0] = d4x4A[2][0];
 d3x3A[1][1] = d4x4A[2][2];
 d3x3A[1][2] = d4x4A[2][3];
 d3x3A[2][0] = d4x4A[3][0];
 d3x3A[2][1] = d4x4A[3][2];
 d3x3A[2][2] = d4x4A[3][3];

 dDetValue01 = Det3x3(d3x3A);

 d3x3A[0][0] = d4x4A[1][0];
 d3x3A[0][1] = d4x4A[1][1];
 d3x3A[0][2] = d4x4A[1][3];
 d3x3A[1][0] = d4x4A[2][0];
 d3x3A[1][1] = d4x4A[2][1];
 d3x3A[1][2] = d4x4A[2][3];
 d3x3A[2][0] = d4x4A[3][0];
 d3x3A[2][1] = d4x4A[3][1];
 d3x3A[2][2] = d4x4A[3][3];
 dDetValue02 = Det3x3(d3x3A);

 d3x3A[0][0] = d4x4A[1][0];
 d3x3A[0][1] = d4x4A[1][1];
 d3x3A[0][2] = d4x4A[1][2];
 d3x3A[1][0] = d4x4A[2][0];
 d3x3A[1][1] = d4x4A[2][1];
 d3x3A[1][2] = d4x4A[2][2];
 d3x3A[2][0] = d4x4A[3][0];
 d3x3A[2][1] = d4x4A[3][1];
 d3x3A[2][2] = d4x4A[3][2];

 dDetValue03 = Det3x3(d3x3A);

 dDetValue = d4x4A[0][0]*dDetValue00 - d4x4A[0][1]*dDetValue01 + d4x4A[0][2]*dDetValue02 - d4x4A[0][3]*dDetValue03;

 return dDetValue;
}
void MatrixInverse4x4(float d4x4A[4][4],float d4x4B[4][4])
{
   float d4x4Cofactor[4][4],d4x4AdjA[4][4],d3x3A[3][3];
   float dDet4x4Value;
   unsigned short int i,j;

   //Getting Cofactor Matrix

   //Cofactor Matrix 1st Column
   d3x3A[0][0] = d4x4A[1][1];
   d3x3A[0][1] = d4x4A[1][2];
   d3x3A[0][2] = d4x4A[1][3];
   d3x3A[1][0] = d4x4A[2][1];
   d3x3A[1][1] = d4x4A[2][2];
   d3x3A[1][2] = d4x4A[2][3];
   d3x3A[2][0] = d4x4A[3][1];
   d3x3A[2][1] = d4x4A[3][2];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[0][0] = Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][1];
   d3x3A[0][1] = d4x4A[0][2];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[2][1];
   d3x3A[1][1] = d4x4A[2][2];
   d3x3A[1][2] = d4x4A[2][3];
   d3x3A[2][0] = d4x4A[3][1];
   d3x3A[2][1] = d4x4A[3][2];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[1][0] = -1*Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][1];
   d3x3A[0][1] = d4x4A[0][2];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[1][1];
   d3x3A[1][1] = d4x4A[1][2];
   d3x3A[1][2] = d4x4A[1][3];
   d3x3A[2][0] = d4x4A[3][1];
   d3x3A[2][1] = d4x4A[3][2];
   d3x3A[2][2] = d4x4A[3][3];
   d4x4Cofactor[2][0] = Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][1];
   d3x3A[0][1] = d4x4A[0][2];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[1][1];
   d3x3A[1][1] = d4x4A[1][2];
   d3x3A[1][2] = d4x4A[1][3];
   d3x3A[2][0] = d4x4A[2][1];
   d3x3A[2][1] = d4x4A[2][2];
   d3x3A[2][2] = d4x4A[2][3];

   d4x4Cofactor[3][0] = -1*Det3x3(d3x3A);

   //Cofactor Matrix 2nd Column

   d3x3A[0][0] = d4x4A[1][0];
   d3x3A[0][1] = d4x4A[1][2];
   d3x3A[0][2] = d4x4A[1][3];
   d3x3A[1][0] = d4x4A[2][0];
   d3x3A[1][1] = d4x4A[2][2];
   d3x3A[1][2] = d4x4A[2][3];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][2];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[0][1] = -1*Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][2];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[2][0];
   d3x3A[1][1] = d4x4A[2][2];
   d3x3A[1][2] = d4x4A[2][3];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][2];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[1][1] = Det3x3(d3x3A);
   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][2];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[1][0];
   d3x3A[1][1] = d4x4A[1][2];
   d3x3A[1][2] = d4x4A[1][3];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][2];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[2][1] = -1 * Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][2];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[1][0];
   d3x3A[1][1] = d4x4A[1][2];
   d3x3A[1][2] = d4x4A[1][3];
   d3x3A[2][0] = d4x4A[2][0];
   d3x3A[2][1] = d4x4A[2][2];
   d3x3A[2][2] = d4x4A[2][3];

   d4x4Cofactor[3][1] = Det3x3(d3x3A);

   //Cofactor Matrix 3rd Column

   d3x3A[0][0] = d4x4A[1][0];
   d3x3A[0][1] = d4x4A[1][1];
   d3x3A[0][2] = d4x4A[1][3];
   d3x3A[1][0] = d4x4A[2][0];
   d3x3A[1][1] = d4x4A[2][1];
   d3x3A[1][2] = d4x4A[2][3];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][1];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[0][2] = Det3x3(d3x3A);
   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][1];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[2][0];
   d3x3A[1][1] = d4x4A[2][1];
   d3x3A[1][2] = d4x4A[2][3];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][1];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[1][2] = -1*Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][1];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[1][0];
   d3x3A[1][1] = d4x4A[1][1];
   d3x3A[1][2] = d4x4A[1][3];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][1];
   d3x3A[2][2] = d4x4A[3][3];

   d4x4Cofactor[2][2] = Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][1];
   d3x3A[0][2] = d4x4A[0][3];
   d3x3A[1][0] = d4x4A[1][0];
   d3x3A[1][1] = d4x4A[1][1];
   d3x3A[1][2] = d4x4A[1][3];
   d3x3A[2][0] = d4x4A[2][0];
   d3x3A[2][1] = d4x4A[2][1];
   d3x3A[2][2] = d4x4A[2][3];

   d4x4Cofactor[3][2] = -1 * Det3x3(d3x3A);


   //Cofactor Matrix 4th Column

   d3x3A[0][0] = d4x4A[1][0];
   d3x3A[0][1] = d4x4A[1][1];
   d3x3A[0][2] = d4x4A[1][2];
   d3x3A[1][0] = d4x4A[2][0];
   d3x3A[1][1] = d4x4A[2][1];
   d3x3A[1][2] = d4x4A[2][2];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][1];
   d3x3A[2][2] = d4x4A[3][2];

   d4x4Cofactor[0][3] = -1 * Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][1];
   d3x3A[0][2] = d4x4A[0][2];
   d3x3A[1][0] = d4x4A[2][0];
   d3x3A[1][1] = d4x4A[2][1];
   d3x3A[1][2] = d4x4A[2][2];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][1];
   d3x3A[2][2] = d4x4A[3][2];

   d4x4Cofactor[1][3] = Det3x3(d3x3A);

   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][1];
   d3x3A[0][2] = d4x4A[0][2];
   d3x3A[1][0] = d4x4A[1][0];
   d3x3A[1][1] = d4x4A[1][1];
   d3x3A[1][2] = d4x4A[1][2];
   d3x3A[2][0] = d4x4A[3][0];
   d3x3A[2][1] = d4x4A[3][1];
   d3x3A[2][2] = d4x4A[3][2];

   d4x4Cofactor[2][3] = -1*Det3x3(d3x3A);
   d3x3A[0][0] = d4x4A[0][0];
   d3x3A[0][1] = d4x4A[0][1];
   d3x3A[0][2] = d4x4A[0][2];
   d3x3A[1][0] = d4x4A[1][0];
   d3x3A[1][1] = d4x4A[1][1];
   d3x3A[1][2] = d4x4A[1][2];
   d3x3A[2][0] = d4x4A[2][0];
   d3x3A[2][1] = d4x4A[2][1];
   d3x3A[2][2] = d4x4A[2][2];

   d4x4Cofactor[3][3] = Det3x3(d3x3A);

   //TransposeMatrix4x4(d4x4Cofactor,d4x4AdjA);
   transposedmxnAToB(4,4,d4x4Cofactor,d4x4AdjA);

   dDet4x4Value = Det4x4(d4x4A);

   if((dDet4x4Value >= -1.0E-20) && (dDet4x4Value <= 1.0E-20))
    {
        for(i = 0;i < 4;i++)
         {
          for(j = 0;j < 4;j++)
          {
               d4x4B[i][j] = 0.0;
          }
        }
     }
   else
   {
    for (i=0;i<4;i++)
    {
      for (j=0;j<4;j++)
      {
         d4x4B[i][j] =  d4x4AdjA[i][j]/dDet4x4Value;
      }
    }
   }
}
float Det6x6(float d6x6InputMx[6][6])
{
 float dMinorMx[5][5];
 float dDetValue = 0.0;
 unsigned short int i,j,k;

 for (i=0;i<6;i++)      //Col No of Input Matrix
 {
   for (j = 0;j<i;j++)
    {
      for (k=0;k<5;k++)     //Row No of Minor Matrix
       {
     dMinorMx[k][j] = d6x6InputMx[k+1][j];
        }
     }
     for (j=i+1;j<6;j++)    //Col No of Input Mx    
     {
       for (k=0;k<5;k++)    //Row No of Minor Mx
       {
     dMinorMx[k][j-1] = d6x6InputMx[k+1][j];
       }
     }

  dDetValue += powf(-1,i)*d6x6InputMx[0][i] * Det5x5(dMinorMx);
 }

  return dDetValue;
}

float Det5x5(float d5x5InputMx[5][5])
{
 float dMinorMx[4][4];
 float dDetValue = 0.0;
 unsigned short int i,j,k;

 for (i=0;i<5;i++)      //Col No of Input Matrix
 {
    for (j = 0;j<i;j++)
    {
      for (k=0;k<4;k++)     //Row No of Minor Matrix
      {
    dMinorMx[k][j] = d5x5InputMx[k+1][j];
      }
    }
    for (j=i+1;j<5;j++)     //Col No of Input Mx
    {
      for (k=0;k<4;k++)     //Row No of Minor Mx
     {
    dMinorMx[k][j-1] = d5x5InputMx[k+1][j];
      }
    }

  dDetValue += powf(-1,i)*d5x5InputMx[0][i] * Det4x4(dMinorMx);
 }
  return dDetValue;
}

void MatrixInverse6x6(float d6x6A[6][6],float d6x6B[6][6])
{
   float d6x6Cofactor[6][6],d6x6AdjA[6][6],d5x5MinorMx[5][5];
   float dDet6x6Value;
   unsigned short int i,j;
   unsigned short int RowNo,ColNo;
   unsigned short RowNoCofMx = 0,ColNoCofMx = 0;
   unsigned short NoOfTimes;

    for (NoOfTimes = 0; NoOfTimes<36; NoOfTimes++)
    {
      RowNo = 0;    //Row No of input 6x6 Matrix    
      ColNo = 0;    //Col No of input 6x6 Matrix

      for (i=0;i<5;i++)  //Row No of Minor Matrix
      {
    if (RowNoCofMx == RowNo)
      RowNo++;
    for (j=0;j<5;j++)   //Column No of Minor Matrix
    {
      if (ColNoCofMx == ColNo)
        ColNo++;

    d5x5MinorMx[i][j]  = d6x6A[RowNo][ColNo];
    ColNo++;
    }
      RowNo++;
      ColNo = 0;
      }
      d6x6Cofactor[RowNoCofMx][ColNoCofMx] = powf(-1,(RowNoCofMx+ColNoCofMx))*Det5x5(d5x5MinorMx);
      ColNoCofMx++;
      if (ColNoCofMx == 6)
      {
    RowNoCofMx++;
    ColNoCofMx = 0;
      }
    }

   //TransposeMatrix6x6(d6x6Cofactor,d6x6AdjA);
   transposedmxnAToB(6,6,d6x6Cofactor,d6x6AdjA);

   dDet6x6Value = Det6x6(d6x6A);
   if((dDet6x6Value >= -1.0E-20) && (dDet6x6Value <= 1.0E-20))
    {
      for(i = 0;i < 6;i++)
      {
    for(j = 0;j < 6;j++)
    {
           d6x6B[i][j] = 0.0;
    }
      }
    }
   else
   {
    for (i=0;i<6;i++)
    {
      for (j=0;j<6;j++)
      {
         d6x6B[i][j] =  d6x6AdjA[i][j]/dDet6x6Value;
      }
     }
   }
}

/*********************************************************************************************/

/*
void matinv(int n,float **mat)
{

	int i,j;
    //calculate determinant 
    // Prepare cofactor matrix 
    // Transpose cofactor matrix 
    //multiply each element with 1/det 
    float det = 0.0;
    float cofactor_mat[n][n];

    det = determinant(mat,n); // not required to call det here, as cofactor internally usinfg det
	#ifdef DEBUG
	printf("det = %f\n",det);
	#endif
    cofactor(mat,n,cofactor_mat);
    transpose(cofactor_mat,n);


	/*
    for(i=0;i<n;i++)
    {   
        for(j=0;j<n;j++)
        {
            cofactor_mat[i][j] = cofactor_mat[i][j]/det;
        }
    } 
	
    for(i=0;i<n;i++)
    {   
        for(j=0;j<n;j++)
        {
            //mat[i][j] = cofactor_mat[i][j];
			assert( det != 0 );
            *((float*)mat + (n*i) +j) = cofactor_mat[i][j]/det;
        }
    } 
	#ifdef DEBUG
    for(i=0;i<n;i++)
    {   
        for(j=0;j<n;j++)
        {
			printf("%f \t",cofactor_mat[i][j]/det);
        }
		printf("\n");
    } 
	#endif


}
*/
/* Recursive definition of determinant usinfg expansion by minors */
float determinant(float **a,int n)
{
   int i,j,j1,j2;
   float det = 0;
   float **m = NULL;

   if (n < 1) { /* Error */

   } else if (n == 1) { /* Shouldn't get used */
      det = a[0][0];
   } else if (n == 2) {
      //det = a[0][0] * a[1][1] - a[1][0] * a[0][1];
      det = *((float*)a ) * (*((float *)a + 3)) - (*((float *)a + 2)) * (*((float *)a + 1 ));
   } else {
      det = 0;
      for (j1=0;j1<n;j1++) {
         m = malloc((n-1)*sizeof(float *));

         for (i=0;i<n-1;i++)
            m[i] = malloc((n-1)*sizeof(float));

         for (i=1;i<n;i++) {
            j2 = 0;
            for (j=0;j<n;j++) {
               if (j == j1)
                  continue;
               //m[i-1][j2] = a[i][j];
               m[i-1][j2] = (*((float*)a +n*i +j));
               j2++;
            }
         }
         //det += powf(-1.0,j1+2.0) * a[0][j1] * determinant(m,n-1);
         det += powf(-1.0,j1+2.0) * (*((float*)a + j1)) * determinant(m,n-1);
         for (i=0;i<n-1;i++)
            free(m[i]);
         free(m);
      }
   }
   return(det);
}
/* Find the cofactor matrix of a square matrix */
void cofactor(float **a,int n,float **b)
{
   int i,j,ii,jj,i1,j1;
   float det;
   float **c;

   c = malloc((n-1)*sizeof(float *));
   for (i=0;i<n-1;i++)
     c[i] = malloc((n-1)*sizeof(float));

   for (j=0;j<n;j++) {
      for (i=0;i<n;i++) {

         /* Form the adjoint a_ij */
         i1 = 0;
         for (ii=0;ii<n;ii++) {
            if (ii == i)
               continue;
            j1 = 0;
            for (jj=0;jj<n;jj++) {
               if (jj == j)
                  continue;
               //c[i1][j1] = a[ii][jj];
               c[i1][j1] = (float) (*((float*)a + (n*ii + jj))); //   a[ii][jj];
               j1++;
            }
            i1++;
         }

         /* Calculate the determinate */
         det = determinant(c,n-1);

         /* Fill in the elements of the cofactor */
         //b[i][j] = powf(-1.0,i+j+2.0) * det;
		 *((float*)b + (n*i) + j) = powf(-1.0,i+j+2.0) * det;
		 //*((float*)b + (n*i) + j) = powf(-1.0,i+j+2.0) / det;
      }
   }
   for(i=0;i<n-1;i++)
      free(c[i]);
   free(c);
}

/* Transpose of a square matrix, do it in place */
void transpose(float **a,int n)
{
   int i,j;
   float tmp;

   for (i=1;i<n;i++) 
   {
      for (j=0;j<i;j++)
	  {
		 tmp = (float) (*((float*)a + (n*i + j)));
         //tmp = a[i][j];
		 (*((float*)a + (n*i + j))) = (*((float*)a + (n*j + i)));
         //a[i][j] = a[j][i];
         //a[j][i] = tmp;
		 *((float*)a + (n*j + i)) = tmp;
      }
   }
}

/*
matinv()
{
    int i,j,k,n;
    float a[10][10]={0},d;
    clrscr();
    cout<<"No of equations ? "; cin>>n;
    cout<<"Read all coefficients of matrix with b matrix too "<<endl;
    for(i=1;i<=n;i++)
        for(j=1;j<=n;j++)
            cin>>a[i][j];
            
    for(i=1;i<=n;i++)
        for(j=1;j<=2*n;j++)
            if(j==(i+n))
                a[i][j]=1;

    //  partial pivoting 
    for(i=n;i>1;i--)
    {
        if(a[i-1][1]<a[i][1])
        for(j=1;j<=n*2;j++)
        {
            d=a[i][j];
            a[i][j]=a[i-1][j];
            a[i-1][j]=d;
        }
    }
    cout<<"pivoted output: "<<endl;
    for(i=1;i<=n;i++)
    {
        for(j=1;j<=n*2;j++)
            cout<<a[i][j]<<"    ";
        cout<<endl;
    }
    // reducing to diagonal  matrix 

    for(i=1;i<=n;i++)
    {
        for(j=1;j<=n*2;j++)
        if(j!=i)
        {
            d=a[j][i]/a[i][i];
            for(k=1;k<=n*2;k++)
                a[j][k]-=a[i][k]*d;
        }
    }
    // reducing to unit matrix 
    for(i=1;i<=n;i++)
    {
    d=a[i][i];
        for(j=1;j<=n*2;j++)
            a[i][j]=a[i][j]/d;
    }


    cout<<"your solutions: "<<endl;
    for(i=1;i<=n;i++)
    {
        for(j=n+1;j<=n*2;j++)
            cout<<a[i][j]<<"    ";
        cout<<endl;
    }
}
*/


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
