/* Rajat*/


#include "LAT_SIM_Forces_and_moments_rotors.h"

#ifndef GROUND_MODEL_H
#define GROUND_MODEL_H


typedef struct
{
  float n_force_ned[3];
  float n_force_b[3];
  float kfx;
  float kfy;
  float kfz;
  float kl;
  float km;
  float kn;
  uint8_t init_takeoff_activate;
  uint8_t plane_on_ground;

}struct_grnd_model ;


extern struct_grnd_model s_grnd_model;

extern void v_grnd_model_run(float, float, float);
extern void v_ground_model_param_init();

#endif
