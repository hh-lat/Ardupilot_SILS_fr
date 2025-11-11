#include <stdio.h>
#include "common_variable.h"


float x1 , x2, z1, y_1, y_2, y_3, y4, y5, y6, y7, y8, y9, y_10;//argha optimization
float Ix , Iy, Iz, Ixy, Ixz, Iyz;


float b1;  //=0.0000143  ; //% thrust factor
float d1; //=0.000000252 ; //% torque factor
float b1_inv; //=69930.0699;
float d_by_b;
float b1_fwv, d1_fwv, b1_fwv_inv, d_by_b_fwv;
float g , c, mass, e, s, b;
float c0, c1, c2, c3, c4, c5, c6, c7, c8, c3_inv, c8_inv;
 
float t, t1;
