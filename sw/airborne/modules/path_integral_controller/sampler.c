/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time_v2/pi2Controllerpi2.c"
 * @author Bianca Bendris
 *
 */
#include <math.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist2->h>
//#include <stdlib.h>

#include "modules/path_integral_controller/sampler.h"

#include  <time.h>


void PIController_init(struct PIController * pi2 )
{
    pi2->H                  = 1;
    pi2->dh                 = 0.2;
    pi2->iH                 = 5;
    pi2->nu                 = 1;
    pi2->R                  = 1;
    pi2->var                = pi2->nu*pi2->dh;
    pi2->lambda             = pi2->R*pi2->nu;
    pi2->N                  = 1000;
    pi2->COLLISION_DISTANCE = 2.25;       //1.5;
    pi2->COHESION_DISTANCE  = 6.25;       //2.5;
    pi2->COLLISION_PENALTY  = 10;         //10*pi2.COLLISION_DISTANCE;;// //1000
    pi2->COHESION_PENALTY   = 5;          //*pi2.COHESION_DISTANCE;;////10*pi2.COHESION_DISTANCE;
    pi2->TARGET_PENALTY     = 50;         //10;
    pi2->HEADING_PENALTY    = 2;
    pi2->PARALLEL_PENALTY   = 10;         //1000;
    pi2->MAX_SPEED          = 1.5;
    pi2->PARALLEL_THR       = 0.3;

    pi2->units              = 3;
    pi2->dimX               = 2;
    pi2->dimU               = 2;

    pi2->wps[0]             = 1.25;
    pi2->wps[1]             = 1.25;
    pi2->wps[2]             = 4;
    pi2->wps[3]             = 13;

    for(int h=0; h<pi2->iH; h++) {
      for(int u=0; u<pi2->dimU; u++) {
        pi2->u_exp[h][u] = 0;
      }
    }

}


/**
 * Function that computes the optimal controls for the leader unit
 */
void compute_optimal_controls(struct PIController * pi2, struct PIstate * st2, struct Input * in){

  float noise[pi2->N][pi2->iH][pi2->dimU];
  float u_roll[pi2->iH][pi2->dimU];
  //gsl_rng *r = gsl_rng_alloc(gsl_rng_mt19937);
  float stdv = sqrt(pi2->var);
  float samples_cost2[pi2->N];
  struct PIstate internal_state;
  float min_cost2 = 10000;
  float inv_lambda = 1/pi2->lambda;

  // Create and compute cost2 of all samples N
  for (int n=pi2->N; n--;){ // n < pi2->N; n++ ){

    samples_cost2[n] = 0;


    for (int i = 0; i< 2; i++){
      internal_state.pos[i] = st2->pos[i];
      internal_state.vel[i] = st2->vel[i];
      for (int o = 0; o<2; o++){
        internal_state.pos_rel[o+2*i] = st2->pos_rel[o+2*i];
        internal_state.vel_rel[o+2*i] = st2->vel_rel[o+2*i];
      }
    }

    for (int h=0; h< pi2->iH; h++){

      for(int i=0; i<pi2->dimU; i++) {
        noise[n][h][i] = 0;//gsl_ran_gaussian(r,1.) * st2dv;
        u_roll[h][i] = pi2->u_exp[h][i] + noise[n][h][i];
       }

      samples_cost2[n] += pi2->R * 0.5 * abs(u_roll[h][0]*u_roll[h][0] + u_roll[h][1]*u_roll[h][1]);
      //printf("control cost2 %f\n", samples_cost2[n]);
      float dist2_target = (internal_state.pos[0]- pi2-> wps[0]) * (internal_state.pos[0]- pi2-> wps[0]) + (internal_state.pos[1]- pi2-> wps[1]) * (internal_state.pos[1]- pi2-> wps[1]);
      samples_cost2[n] += pi2->TARGET_PENALTY* pi2->dh * dist2_target;
      //printf("target cost2 %f\n", samples_cost2[n]);
      float cross_product_3 = abs(internal_state.vel[0] * pi2->wps[1] - internal_state.vel[1] * pi2->wps[0]);
      samples_cost2[n] += pi2->HEADING_PENALTY * pi2->dh * cross_product_3;
      //printf("heading cost2 %f\n", samples_cost2[n]);
      for(int a=0; a < pi2->units-1; a++){
       float dist2_unit = (internal_state.pos[0]-  internal_state.pos_rel[0+2*a]) * (internal_state.pos[0]- internal_state.pos_rel[0+2*a]) + (internal_state.pos[1]- internal_state.pos_rel[1+2*a]) * (internal_state.pos[1]- internal_state.pos_rel[1+2*a]);

       if(dist2_unit > pi2->COLLISION_DISTANCE ){}
       else{ samples_cost2[n] += exp(pi2->COLLISION_PENALTY *(pi2->COLLISION_DISTANCE - dist2_unit));}
              //printf("collision cost2 %f\n", samples_cost2[n]);
       }

       // Propagate leader unit
       internal_state.vel[0] += u_roll[h][0]*pi2->dh;
       internal_state.vel[1] += u_roll[h][1]*pi2->dh;
       internal_state.pos[0] += internal_state.vel[0]*pi2->dh;
       internal_state.pos[1] += internal_state.vel[1]*pi2->dh;

       // Propagate follower units
       for(int a=0; a < pi2->units-1; a++) {
         internal_state.pos_rel[0+2*a] += internal_state.vel_rel[0+2*a] * pi2->dh;
         internal_state.pos_rel[1+2*a] += internal_state.vel_rel[1+2*a] * pi2->dh;
         }

     }

    if(samples_cost2[n] < min_cost2) {min_cost2 = samples_cost2[n];}

  }

  //printf("Min cost2 is:%f\n",min_cost2);


  // Compute weight of all samples N
  float w[pi2->N];
  float w_sum = 0;
  for (int n=0; n < pi2->N; ){
    w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
    w_sum += w[n]; n++;//
    w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
    w_sum += w[n]; n++;
  }

  float internal_controls[pi2->iH][pi2->dimU];
  for(int h = 0; h < pi2->iH; h++){
        internal_controls[h][0] = 0;
        internal_controls[h][1] = 0;
  }

  for (int n=0; n < pi2->N; n++ ){
    w[n] = w[n]/w_sum;
    for(int h = 0; h < pi2->iH; h++){
      internal_controls[h][0] += w[n] * noise[n][h][0];
      internal_controls[h][1] += w[n] * noise[n][h][1];
    }
   }

  for(int h = 0; h < pi2->iH; h++){
    internal_controls[h][0] /= pi2->dh;
    internal_controls[h][1] /= pi2->dh;
  }

  // Compute optimal controls
  //printf("exploring control 0:%f,%f Internal control 1:%f %f\n",pi2->u_exp[0][0],pi2->u_exp[0][1],internal_controls[0][0],internal_controls[0][1]);
  in->oc_x = pi2->u_exp[0][0] + internal_controls[0][0];
  in->oc_y = pi2->u_exp[0][1] + internal_controls[0][1];

  for(int h = 0; h < pi2->iH; h++){
    pi2->u_exp[h][0] += internal_controls[h][0];
    pi2->u_exp[h][1] += internal_controls[h][1];
  }

  //printf("Optimal control 0:%f, Optimal control 1:%f\n",in->oc_x,in.oc_y);

}


/**
 * Function that computes the optimal controls for a follower unit
*/

void compute_optimal_controls_followers(struct PIController * pi2, struct PIstate * st2, struct Input * in){


  float noise[pi2->N][pi2->iH][pi2->dimU];
  float u_roll[pi2->iH][pi2->dimU];

  //gsl_rng *r = gsl_rng_alloc(gsl_rng_mt19937);
  float samples_cost2[pi2->N];
  struct PIstate internal_state;
  float min_cost2 = 10000;
  float inv_lambda = 1/ pi2->lambda;

  // Create and compute cost2 of all samples N
  for (int n=pi2->N; n--; ){

    samples_cost2[n] = 0;

    for (int i = 0; i< 2; i++){
      internal_state.pos[i] = st2->pos[i];
      internal_state.vel[i] = st2->vel[i];
      for (int o = 0; o<2; o++){
        internal_state.pos_rel[o+2*i] = st2->pos_rel[o+2*i];
        internal_state.vel_rel[o+2*i] = st2->vel_rel[o+2*i];
      }
    }

    for (int h=0; h < pi2->iH; h++) {

      for(int i=0; i < pi2->dimU; i++) {
        noise[n][h][i] = 0;//gsl_ran_gaussian(r,1.) * st2dv;
        u_roll[h][i] = pi2->u_exp[h][i] + noise[n][h][i];
       }

      samples_cost2[n] += pi2->R * 0.5 * abs(u_roll[h][0]*u_roll[h][0] + u_roll[h][1]*u_roll[h][1]);

      float dist2_leader = (internal_state.pos[0]- internal_state.pos_rel[0]) * (internal_state.pos[0]- internal_state.pos_rel[0]) + (internal_state.pos[1]- internal_state.pos_rel[1]) * (internal_state.pos[1]- internal_state.pos_rel[1]);
      if(dist2_leader < pi2->COHESION_DISTANCE ){}
      else{ samples_cost2[n] += exp(pi2->COHESION_PENALTY *(dist2_leader - pi2->COHESION_DISTANCE));}

      for(int a=0; a < pi2->units-1; a++){
        float dist2_unit = (internal_state.pos[0]-  internal_state.pos_rel[0+2*a]) * (internal_state.pos[0]- internal_state.pos_rel[0+2*a]) + (internal_state.pos[1]- internal_state.pos_rel[0+2*a]) * (internal_state.pos[1]- internal_state.pos_rel[0+2*a]);
        if(dist2_unit > pi2->COLLISION_DISTANCE ){}
        else{ samples_cost2[n] += exp(pi2->COLLISION_PENALTY *(pi2->COLLISION_DISTANCE - dist2_unit));}


        float cross_product_3 = abs(internal_state.pos[0] * internal_state.pos_rel[1+2*a] - internal_state.pos[1] * internal_state.pos_rel[0+2*a]);
        if(cross_product_3 > pi2->PARALLEL_THR){}
        else{samples_cost2[n] += exp(pi2->PARALLEL_PENALTY *(pi2->PARALLEL_THR - cross_product_3));}

      }

      // Propagate unit
      internal_state.vel[0] += u_roll[h][0]*pi2->dh;
      internal_state.vel[1] += u_roll[h][1]*pi2->dh;
      internal_state.pos[0] += internal_state.vel[0]*pi2->dh;
      internal_state.pos[1] += internal_state.vel[1]*pi2->dh;

      // Propagate neighboring units
      for(int a=0; a < pi2->units-1; a++) {
        internal_state.pos_rel[0+2*a] += internal_state.vel_rel[0+2*a] * pi2->dh;
        internal_state.pos_rel[1+2*a] += internal_state.vel_rel[1+2*a] * pi2->dh;
      }

    }


    if(samples_cost2[n] < min_cost2) {min_cost2 = samples_cost2[n];}

  }
  //printf("Min cost2 is:%f\n",min_cost2);

  // Compute weight of all samples N
    float w[pi2->N];
    float w_sum = 0;
    for (int n=0; n < pi2->N; ){
      w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
      w_sum += w[n]; n++;//
      w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
      w_sum += w[n]; n++;
      w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
      w_sum += w[n]; n++;
      w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
      w_sum += w[n]; n++;
      w[n] = exp(-(samples_cost2[n] - min_cost2)*inv_lambda);
      w_sum += w[n]; n++;
    }

    float internal_controls[pi2->iH][pi2->dimU];
    for(int h = 0; h < pi2->iH; h++){
          internal_controls[h][0] = 0;
          internal_controls[h][1] = 0;
    }

    for (int n=0; n < pi2->N; n++ ){
      w[n] = w[n]/w_sum;
      for(int h = 0; h < pi2->iH; h++){
        internal_controls[h][0] += w[n] * noise[n][h][0];
        internal_controls[h][1] += w[n] * noise[n][h][1];
      }
     }

    for(int h = 0; h < pi2->iH; h++){
      internal_controls[h][0] /= pi2->dh;
      internal_controls[h][1] /= pi2->dh;
    }

    // Compute optimal controls
    //printf("exploring control 0:%f,%f Internal control 1:%f %f\n",pi2->u_exp[0][0],pi2->u_exp[0][1],internal_controls[0][0],internal_controls[0][1]);
    in->oc_x = pi2->u_exp[0][0] + internal_controls[0][0];
    in->oc_y = pi2->u_exp[0][1] + internal_controls[0][1];

    for(int h = 0; h < pi2->iH; h++){
      pi2->u_exp[h][0] += internal_controls[h][0];
      pi2->u_exp[h][1] += internal_controls[h][1];
    }

}








/*
int * compute_weight(int *cost2, st2ruct pi2Controller *ptr){

  float w[pi2->N];
  float cost2[]
  for(int n=0; n < pi2->N; n++ ){
    w[n] = exp(-(cost2 + n ))
  }

}
*/






/**
 * Shift exploring controls

void pi2Controller_shiftControls(st2ruct pi2Controller *ptr)
{
  for (int s = 1; s < pi2->iH; s++){
    for (int u = 1; u< pi2->units*pi2->dimU; u++ ){
      pi2->u_exp[s-1][u] = pi2->u_exp[s][u];
    }
  }
  pi2->u_exp[pi2->iH-1][0] = 0;
  pi2->u_exp[pi2->iH-1][1] = 0;
  pi2->u_exp[pi2->iH-1][2] = 0;
  pi2->u_exp[pi2->iH-1][3] = 0;
  pi2->u_exp[pi2->iH-1][4] = 0;
  pi2->u_exp[pi2->iH-1][5] = 0;
}

*/


