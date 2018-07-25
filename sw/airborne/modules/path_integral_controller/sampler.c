/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time_v2/PIControllerptr->c"
 * @author Bianca Bendris
 *
 */
#include <math.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <stdlib.h>

#include "modules/path_integral_controller/sampler.h"



void PIController_init(struct PIController *ptr)
{

    ptr->T                  = 10;
    ptr->dt                 = 0.04;
    ptr->iT                 = 500;
    ptr->H                  = 1;
    ptr->dh                 = 0.2;
    ptr->iH                 = 5;
    ptr->nu                 = 1;
    ptr->R                  = 1;
    ptr->var                = ptr->nu*ptr->dh;
    ptr->lambda             = ptr->R*ptr->nu;
    ptr->N                  = 1000;
    ptr->COLLISION_DISTANCE = 1.5;
    ptr->COHESION_DISTANCE  = 2.5;
    ptr->COLLISION_PENALTY  = 3; //10*ptr->COLLISION_DISTANCE;
    ptr->COHESION_PENALTY   = 5;//10*ptr->COHESION_DISTANCE;
    ptr->TARGET_PENALTY     = 10;
    ptr->HEADING_PENALTY    = 2;
    ptr->PARALLEL_PENALTY   = 10;
    ptr->MAX_SPEED          = 1.5;
    ptr->PARALLEL_THR       = 0.3;

    ptr->units              = 3;
    ptr->dimX               = 2;
    ptr->dimU               = 2;


    ptr->state[0]           = 3;
    ptr->state[1]           = 4;
    ptr->state[2]           = 0;
    ptr->state[3]           = 0;

    ptr->followers[0][0]    = 2;
    ptr->followers[0][1]    = 6;
    ptr->followers[0][2]    = 0;
    ptr->followers[0][3]    = 0;

    ptr->followers[1][0]    = 2;
    ptr->followers[1][1]    = 2;
    ptr->followers[1][2]    = 0;
    ptr->followers[1][3]    = 0;

    ptr->wps[0]             = 13;
    ptr->wps[1]             = 4;
    ptr->wps[2]             = 4;
    ptr->wps[3]             = 13;

    for(int h=0; h<ptr->iH; h++) {
      for(int u=0; u<ptr->dimU; u++) {
        ptr->u_exp[h][u] = 0;
      }
    }

    //float *u_exp_ptr_original = ptr->u_exp[0];
    //printf("Before the float_mat_zero\n");
    //float_mat_zero(&u_exp_ptr_original, 5,2);
    //printf("After the float_mat_zero\n");
}



/**
 * Function that computes the optimal controls
 */
void compute_optimal_controls(struct PIController *ptr){

  float noise[ptr->N][ptr->iH][ptr->dimU];
  float u_roll[ptr->iH][ptr->dimU];

  gsl_rng *r = gsl_rng_alloc(gsl_rng_mt19937);
  float stdv = sqrt(ptr->var);
  float samples_cost[ptr->N];
  float leader_state[4];
  float followers_state[2][4];
  //float *followers_ptr = followers_state[0];
  //float *followers_ptr_original = ptr->followers[0];
  float min_cost = 10000;
  //float_vect_zero(&samples_cost, ptr->N);

  printf("1. Computing sample cost");
  for (int n=0; n < ptr->N; n++ ){

    for (int h=0; h<ptr->iH; h++) {
      for(int i=0; i<ptr->dimU; i++) {
        noise[n][h][i] = gsl_ran_gaussian(r,1.) * stdv;
        u_roll[h][i] = ptr->u_exp[h][i] + noise[n][h][i];
       }
    }

    for (int i = 0; i< 4; i++){
      leader_state[i] = ptr->state[i];
      for (int o = 0; o<2; o++){
        followers_state[o][i] = ptr->followers[o][i];
      }
    }



    //float_vect_copy(&leader_state[0], &ptr->state[0], 4);
    //float_mat_copy(&followers_ptr, &followers_ptr_original, 2, 4);

    for (int h=0; h<ptr->iH; h++) {


      samples_cost[n] += ptr->R * 0.5 * abs(u_roll[h][0]*u_roll[h][0] + u_roll[h][1]*u_roll[h][1]);


      float dist_target = (leader_state[0]- ptr-> wps[0]) * (leader_state[0]- ptr-> wps[0]) - (leader_state[1]- ptr-> wps[1]) * (leader_state[1]- ptr-> wps[1]);
      samples_cost[n] += ptr->TARGET_PENALTY * dist_target * ptr->dh;

      float cross_product_3= leader_state[0] * ptr->wps[1] - leader_state[1] * ptr->wps[0];
      samples_cost[n] += ptr->HEADING_PENALTY * cross_product_3 * ptr->dh;

      for(int a=0; a < ptr->units-1; a++){
        float dist_unit = (leader_state[0]-  followers_state[a][0]) * (leader_state[0]- followers_state[a][0]) - (leader_state[1]- followers_state[a][1]) * (leader_state[1]- followers_state[a][1]);
        samples_cost[n] += ptr->COLLISION_PENALTY * dist_unit;
      }

      // Propagate leader unit
      leader_state[2] += u_roll[h][0]*ptr->dh;
      leader_state[3] += u_roll[h][1]*ptr->dh;
      leader_state[0] += leader_state[2]*ptr->dh;
      leader_state[1] += leader_state[3]*ptr->dh;

      // Propagate follower units
      for(int a=0; a < ptr->units-1; a++) {
        followers_state[a][0] += followers_state[a][2] * ptr->dh;
        followers_state[a][1] += followers_state[a][3] * ptr->dh;
      }

    }

    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}

  }
  printf("Min cost is:%f\n",min_cost);


  float w[ptr->N];
  float w_sum = 0;
  for (int n=0; n < ptr->N; n++ ){
    w[n] = exp(-(samples_cost[n] - min_cost)/ptr->lambda);
    w_sum += w[n];
  }

  float optimal_controls[ptr->dimU];
  for (int n=0; n < ptr->N; n++ ){
      w[n] = w[n]/w_sum;
      optimal_controls[0] += w[n] * noise[n][0][0];
      optimal_controls[1] += w[n] * noise[n][0][1];
    }

  optimal_controls[0] /= ptr->dh;
  optimal_controls[1] /= ptr->dh;
  optimal_controls[0] +=  ptr->u_exp[1][0];
  optimal_controls[1] +=  ptr->u_exp[1][1];

  printf("Optimal control 0:%f, Optimal control 1:%f\n",optimal_controls[0],optimal_controls[1]);
  //printf(&optimal_controls[0]);
  //return &optimal_controls[0];

}

/*
int * compute_weight(int *cost, struct PIController *ptr){

  float w[ptr->N];
  float cost[]
  for(int n=0; n < ptr->N; n++ ){
    w[n] = exp(-(cost + n ))
  }

}
*/






/**
 * Shift exploring controls

void PIController_shiftControls(struct PIController *ptr)
{
  for (int s = 1; s < ptr->iH; s++){
    for (int u = 1; u< ptr->units*ptr->dimU; u++ ){
      ptr->u_exp[s-1][u] = ptr->u_exp[s][u];
    }
  }
  ptr->u_exp[ptr->iH-1][0] = 0;
  ptr->u_exp[ptr->iH-1][1] = 0;
  ptr->u_exp[ptr->iH-1][2] = 0;
  ptr->u_exp[ptr->iH-1][3] = 0;
  ptr->u_exp[ptr->iH-1][4] = 0;
  ptr->u_exp[ptr->iH-1][5] = 0;
}

*/


