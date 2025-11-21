#ifndef MATH_UTIL_H
#define MATH_UTIL_H
#include<stdlib.h>
#include "math.h"
#include <stdint.h>
void array_initd(float * arr,int num);

extern float constrain_float1(float , float , float);

float sec(float);
int sign(float);
extern float sign_1(float p);


extern void MatrixMultiply(
     uint32_t rowsA, uint32_t colsA, float *A,
     uint32_t rowsB, uint32_t colsB, float *B,
    float *C);
extern void transposed3x3(float[][3]);
extern void transposedmxnAToB(int m, int n,  float* source, float* dest);




extern void cross_product(float [], float [], float []);
extern void dot_product(float [], float [], float *);
extern void array_initd_3X3(float source[][3]);
extern void copyMatrixd3x3(float source[][3],float dest[][3]);
#endif
