/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time_v2/PIControllerpi.c"
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

#include  <time.h>


void PIController_init()
{
    pi.H                  = 1;
    pi.dh                 = 0.2;
    pi.iH                 = 5;
    pi.nu                 = 1;
    pi.R                  = 1;
    pi.var                = pi.nu*pi.dh;
    pi.lambda             = pi.R*pi.nu;
    pi.N                  = 1000;
    pi.COLLISION_DISTANCE = 2.25;       //1.5;
    pi.COHESION_DISTANCE  = 6.25;       //2.5;
    pi.COLLISION_PENALTY  = 10;         //10*pi.COLLISION_DISTANCE;;// //1000
    pi.COHESION_PENALTY   = 5;          //*pi.COHESION_DISTANCE;;////10*pi.COHESION_DISTANCE;
    pi.TARGET_PENALTY     = 50;         //10;
    pi.HEADING_PENALTY    = 2;
    pi.PARALLEL_PENALTY   = 10;         //1000;
    pi.MAX_SPEED          = 1.5;
    pi.PARALLEL_THR       = 0.3;

    pi.units              = 3;
    pi.dimX               = 2;
    pi.dimU               = 2;

    pi.wps[0]             = 1.25;
    pi.wps[1]             = 1.25;
    pi.wps[2]             = 4;
    pi.wps[3]             = 13;

    for(int h=0; h<pi.iH; h++) {
      for(int u=0; u<pi.dimU; u++) {
        pi.u_exp[h][u] = 0;
      }
    }

}


/**
 * Function that computes the optimal controls for the leader unit
 */
void compute_optimal_controls(){

  float noise[pi.N][pi.iH][pi.dimU];
  float u_roll[pi.iH][pi.dimU];
  gsl_rng *r = gsl_rng_alloc(gsl_rng_mt19937);
  float stdv = sqrt(pi.var);
  float samples_cost[pi.N];
  struct PIstate internal_state;
  float min_cost = 10000;
  float inv_lambda = 1/pi.lambda;

  // Create and compute cost of all samples N
  for (int n=pi.N; n--;){ // n < pi.N; n++ ){

    samples_cost[n] = 0;


    for (int i = 0; i< 2; i++){
      internal_state.pos[i] = st.pos[i];
      internal_state.vel[i] = st.vel[i];
      for (int o = 0; o<2; o++){
        internal_state.pos_rel[o+2*i] = st.pos_rel[o+2*i];
        internal_state.vel_rel[o+2*i] = st.vel_rel[o+2*i];
      }
    }

    for (int h=0; h< pi.iH; h++){

      for(int i=0; i<pi.dimU; i++) {
        noise[n][h][i] = gsl_ran_gaussian(r,1.) * stdv;
        u_roll[h][i] = pi.u_exp[h][i] + noise[n][h][i];
       }

      samples_cost[n] += pi.R * 0.5 * abs(u_roll[h][0]*u_roll[h][0] + u_roll[h][1]*u_roll[h][1]);
      //printf("control cost %f\n", samples_cost[n]);
      float dist_target = (internal_state.pos[0]- pi. wps[0]) * (internal_state.pos[0]- pi. wps[0]) + (internal_state.pos[1]- pi. wps[1]) * (internal_state.pos[1]- pi. wps[1]);
      samples_cost[n] += pi.TARGET_PENALTY* pi.dh * dist_target;
      //printf("target cost %f\n", samples_cost[n]);
      float cross_product_3 = abs(internal_state.vel[0] * pi.wps[1] - internal_state.vel[1] * pi.wps[0]);
      samples_cost[n] += pi.HEADING_PENALTY * pi.dh * cross_product_3;
      //printf("heading cost %f\n", samples_cost[n]);
      for(int a=0; a < pi.units-1; a++){
       float dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[0+2*a]) * (internal_state.pos[0]- internal_state.pos_rel[0+2*a]) + (internal_state.pos[1]- internal_state.pos_rel[1+2*a]) * (internal_state.pos[1]- internal_state.pos_rel[1+2*a]);

       if(dist_unit > pi.COLLISION_DISTANCE ){}
       else{ samples_cost[n] += exp(pi.COLLISION_PENALTY *(pi.COLLISION_DISTANCE - dist_unit));}
              //printf("collision cost %f\n", samples_cost[n]);
       }

       // Propagate leader unit
       internal_state.vel[0] += u_roll[h][0]*pi.dh;
       internal_state.vel[1] += u_roll[h][1]*pi.dh;
       internal_state.pos[0] += internal_state.vel[0]*pi.dh;
       internal_state.pos[1] += internal_state.vel[1]*pi.dh;

       // Propagate follower units
       for(int a=0; a < pi.units-1; a++) {
         internal_state.pos_rel[0+2*a] += internal_state.vel_rel[0+2*a] * pi.dh;
         internal_state.pos_rel[1+2*a] += internal_state.vel_rel[1+2*a] * pi.dh;
         }

     }

    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}

  }

  printf("Min cost is:%f\n",min_cost);


  // Compute weight of all samples N
  float w[pi.N];
  float w_sum = 0;
  for (int n=0; n < pi.N; ){
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    w_sum += w[n]; n++;//
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    w_sum += w[n]; n++;
  }

  float internal_controls[pi.iH][pi.dimU];
  for(int h = 0; h < pi.iH; h++){
        internal_controls[h][0] = 0;
        internal_controls[h][1] = 0;
  }

  for (int n=0; n < pi.N; n++ ){
    w[n] = w[n]/w_sum;
    for(int h = 0; h < pi.iH; h++){
      internal_controls[h][0] += w[n] * noise[n][h][0];
      internal_controls[h][1] += w[n] * noise[n][h][1];
    }
   }

  for(int h = 0; h < pi.iH; h++){
    internal_controls[h][0] /= pi.dh;
    internal_controls[h][1] /= pi.dh;
  }

  // Compute optimal controls
  //printf("exploring control 0:%f,%f Internal control 1:%f %f\n",pi.u_exp[0][0],pi.u_exp[0][1],internal_controls[0][0],internal_controls[0][1]);
  in.oc_x = pi.u_exp[0][0] + internal_controls[0][0];
  in.oc_y = pi.u_exp[0][1] + internal_controls[0][1];

  for(int h = 0; h < pi.iH; h++){
    pi.u_exp[h][0] += internal_controls[h][0];
    pi.u_exp[h][1] += internal_controls[h][1];
  }

  //printf("Optimal control 0:%f, Optimal control 1:%f\n",in.oc_x,in.oc_y);

}


/**
 * Function that computes the optimal controls for a follower unit
*/

void compute_optimal_controls_followers(){


  float noise[pi.N][pi.iH][pi.dimU];
  float u_roll[pi.iH][pi.dimU];

  gsl_rng *r = gsl_rng_alloc(gsl_rng_mt19937);
  float stdv = sqrt(pi.var);
  float samples_cost[pi.N];
  struct PIstate internal_state;
  float min_cost = 10000;
  float inv_lambda = 1/ pi.lambda;

  // Create and compute cost of all samples N
  for (int n=pi.N; n--; ){

    samples_cost[n] = 0;

    for (int i = 0; i< 2; i++){
      internal_state.pos[i] = st.pos[i];
      internal_state.vel[i] = st.vel[i];
      for (int o = 0; o<2; o++){
        internal_state.pos_rel[o+2*i] = st.pos_rel[o+2*i];
        internal_state.vel_rel[o+2*i] = st.vel_rel[o+2*i];
      }
    }

    for (int h=0; h < pi.iH; h++) {

      for(int i=0; i < pi.dimU; i++) {
        noise[n][h][i] = gsl_ran_gaussian(r,1.) * stdv;
        u_roll[h][i] = pi.u_exp[h][i] + noise[n][h][i];
       }

      samples_cost[n] += pi.R * 0.5 * abs(u_roll[h][0]*u_roll[h][0] + u_roll[h][1]*u_roll[h][1]);

      float dist_leader = (internal_state.pos[0]- internal_state.pos_rel[0]) * (internal_state.pos[0]- internal_state.pos_rel[0]) + (internal_state.pos[1]- internal_state.pos_rel[1]) * (internal_state.pos[1]- internal_state.pos_rel[1]);
      if(dist_leader < pi.COHESION_DISTANCE ){}
      else{ samples_cost[n] += exp(pi.COHESION_PENALTY *(dist_leader - pi.COHESION_DISTANCE));}

      for(int a=0; a < pi.units-1; a++){
        float dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[0+2*a]) * (internal_state.pos[0]- internal_state.pos_rel[0+2*a]) + (internal_state.pos[1]- internal_state.pos_rel[0+2*a]) * (internal_state.pos[1]- internal_state.pos_rel[0+2*a]);
        if(dist_unit > pi.COLLISION_DISTANCE ){}
        else{ samples_cost[n] += exp(pi.COLLISION_PENALTY *(pi.COLLISION_DISTANCE - dist_unit));}


        float cross_product_3 = abs(internal_state.pos[0] * internal_state.pos_rel[1+2*a] - internal_state.pos[1] * internal_state.pos_rel[0+2*a]);
        if(cross_product_3 > pi.PARALLEL_THR){}
        else{samples_cost[n] += exp(pi.PARALLEL_PENALTY *(pi.PARALLEL_THR - cross_product_3));}

      }

      // Propagate unit
      internal_state.vel[0] += u_roll[h][0]*pi.dh;
      internal_state.vel[1] += u_roll[h][1]*pi.dh;
      internal_state.pos[0] += internal_state.vel[0]*pi.dh;
      internal_state.pos[1] += internal_state.vel[1]*pi.dh;

      // Propagate neighboring units
      for(int a=0; a < pi.units-1; a++) {
        internal_state.pos_rel[0+2*a] += internal_state.vel_rel[0+2*a] * pi.dh;
        internal_state.pos_rel[1+2*a] += internal_state.vel_rel[1+2*a] * pi.dh;
      }

    }


    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}

  }
  printf("Min cost is:%f\n",min_cost);

  // Compute weight of all samples N
    float w[pi.N];
    float w_sum = 0;
    for (int n=0; n < pi.N; ){
      w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
      w_sum += w[n]; n++;//
      w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
      w_sum += w[n]; n++;
      w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
      w_sum += w[n]; n++;
      w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
      w_sum += w[n]; n++;
      w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
      w_sum += w[n]; n++;
    }

    float internal_controls[pi.iH][pi.dimU];
    for(int h = 0; h < pi.iH; h++){
          internal_controls[h][0] = 0;
          internal_controls[h][1] = 0;
    }

    for (int n=0; n < pi.N; n++ ){
      w[n] = w[n]/w_sum;
      for(int h = 0; h < pi.iH; h++){
        internal_controls[h][0] += w[n] * noise[n][h][0];
        internal_controls[h][1] += w[n] * noise[n][h][1];
      }
     }

    for(int h = 0; h < pi.iH; h++){
      internal_controls[h][0] /= pi.dh;
      internal_controls[h][1] /= pi.dh;
    }

    // Compute optimal controls
    //printf("exploring control 0:%f,%f Internal control 1:%f %f\n",pi.u_exp[0][0],pi.u_exp[0][1],internal_controls[0][0],internal_controls[0][1]);
    in.oc_x = pi.u_exp[0][0] + internal_controls[0][0];
    in.oc_y = pi.u_exp[0][1] + internal_controls[0][1];

    for(int h = 0; h < pi.iH; h++){
      pi.u_exp[h][0] += internal_controls[h][0];
      pi.u_exp[h][1] += internal_controls[h][1];
    }

}








/*
int * compute_weight(int *cost, struct PIController *ptr){

  float w[pi.N];
  float cost[]
  for(int n=0; n < pi.N; n++ ){
    w[n] = exp(-(cost + n ))
  }

}
*/






/**
 * Shift exploring controls

void PIController_shiftControls(struct PIController *ptr)
{
  for (int s = 1; s < pi.iH; s++){
    for (int u = 1; u< pi.units*pi.dimU; u++ ){
      pi.u_exp[s-1][u] = pi.u_exp[s][u];
    }
  }
  pi.u_exp[pi.iH-1][0] = 0;
  pi.u_exp[pi.iH-1][1] = 0;
  pi.u_exp[pi.iH-1][2] = 0;
  pi.u_exp[pi.iH-1][3] = 0;
  pi.u_exp[pi.iH-1][4] = 0;
  pi.u_exp[pi.iH-1][5] = 0;
}

*/


