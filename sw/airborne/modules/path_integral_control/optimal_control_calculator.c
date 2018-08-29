/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/path_integral_control/optimal_control_calculator.c"
 * @author Bianca Bendris
 * Calculates the x and y optimal velocities.
 */

#include "modules/path_integral_control/optimal_control_calculator.h"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>


#ifndef PI_LEADER
#define PI_LEADER 1
#endif
#ifndef PI_HORIZON
#define PI_HORIZON 1
#endif
#ifndef PI_DH
#define PI_DH 0.2
#endif
#ifndef PI_NU
#define PI_NU 1
#endif
#ifndef PI_R
#define PI_R 1
#endif
#ifndef PI_SAMPLES
#define PI_SAMPLES 1000
#endif
#ifndef PI_COLLISION_DISTANCE
#define PI_COLLISION_DISTANCE 2.25
#endif
#ifndef PI_COHESION_DISTANCE
#define PI_COHESION_DISTANCE 6.25
#endif
#ifndef PI_COLLISION_PENALTY
#define PI_COLLISION_PENALTY 10
#endif
#ifndef PI_COHESION_PENALTY
#define PI_COHESION_PENALTY 5
#endif
#ifndef PI_TARGET_PENALTY
#define PI_TARGET_PENALTY 50
#endif
#ifndef PI_HEADING_PENALTY
#define PI_HEADING_PENALTY 2
#endif
#ifndef PI_PARALLEL_PENALTY
#define PI_PARALLEL_PENALTY 10
#endif
#ifndef PI_MAX_SPEED
#define PI_MAX_SPEED 1.5
#endif
#ifndef PI_PARALLEL_THR
#define PI_PARALLEL_THR 0.3
#endif
#ifndef PI_UNITS
#define PI_UNITS 3
#endif
#ifndef PI_DIM_STATES
#define PI_DIM_STATES 2
#endif
#ifndef PI_DIM_U
#define PI_DIM_U 2
#endif
#ifndef PI_TARGET_WP_X
#define PI_TARGET_WP_X 1.25
#endif
#ifndef PI_TARGET_WP_Y
#define PI_TARGET_WP_Y 1.25
#endif



void pi_calc_init(struct path_integral_t *pi){

     pi->leader             = PI_LEADER;
     pi->H                  = PI_HORIZON;
     pi->dh                 = PI_DH;
     pi->iH                 = PI_HORIZON/PI_DH;
     pi->nu                 = PI_NU;
     pi->R                  = PI_R;
     pi->var                = PI_NU*PI_DH;
     pi->lambda             = PI_R*PI_NU;
     pi->N                  = PI_SAMPLES;
     pi->COLLISION_DISTANCE = PI_COLLISION_DISTANCE;
     pi->COHESION_DISTANCE  = PI_COHESION_DISTANCE;
     pi->COLLISION_PENALTY  = PI_COLLISION_PENALTY;
     pi->COHESION_PENALTY   = PI_COHESION_PENALTY;
     pi->TARGET_PENALTY     = PI_TARGET_PENALTY;
     pi->HEADING_PENALTY    = PI_HEADING_PENALTY;
     pi->PARALLEL_PENALTY   = PI_PARALLEL_PENALTY;
     pi->MAX_SPEED          = PI_MAX_SPEED;
     pi->PARALLEL_THR       = PI_PARALLEL_THR;

     pi->units              = PI_UNITS;
     pi->dimX               = PI_DIM_STATES;
     pi->dimU               = PI_DIM_U;

     pi->wps[0]             = PI_TARGET_WP_X;
     pi->wps[1]             = PI_TARGET_WP_Y;


     for(int h=0; h<pi->iH; h++) {
       for(int u=0; u<pi->dimU; u++) {
         pi->u_exp[h][u] = 0;
       }
     }

}


/**
 * Compute the optimal controls for the next timestep
 * @param[in]   *pi The path integral structure that keeps track of previous images
 * @param[in]   *st The state of the drone
 * @param[out]  *result the computed optimal controls
 */
bool pi_calc_timestep(struct path_integral_t *pi, struct pi_state_t *st, struct pi_result_t *result){

  bool success = false;

  float noise[pi->N][pi->iH][pi->dimU];
  float u_roll[pi->iH][pi->dimU];
  gsl_rng *r = gsl_rng_alloc(gsl_rng_mt19937);
  float stdv = sqrt(pi->var);
  float samples_cost[pi->N];
  struct pi_state_t internal_state;
  float min_cost = 10000;
  float inv_lambda = 1/pi->lambda;

  // Create and compute cost of all samples N
  for (int n=pi->N; n--;){ // n < pi->N; n++ ){

    samples_cost[n] = 0;

    for (int i = 0; i< 2; i++){
      internal_state.pos[i] = st->pos[i];
      internal_state.vel[i] = st->vel[i];
      for (int o = 0; o<2; o++){
        internal_state.pos_rel[o+2*i] = st->pos_rel[o+2*i];
        internal_state.vel_rel[o+2*i] = st->vel_rel[o+2*i];
      }
    }

    for (int h=0; h< pi->iH; h++){

      for(int i=0; i<pi->dimU; i++) {
        noise[n][h][i] = gsl_ran_gaussian(r,1.) * stdv;
        u_roll[h][i] = pi->u_exp[h][i] + noise[n][h][i];
       }

      samples_cost[n] += pi->R * 0.5 * abs(u_roll[h][0]*u_roll[h][0] + u_roll[h][1]*u_roll[h][1]);
      //printf("control cost %f\n", samples_cost[n]);

      if(pi->leader){
        float dist_target = (internal_state.pos[0]- pi-> wps[0]) * (internal_state.pos[0]- pi-> wps[0]) + (internal_state.pos[1]- pi-> wps[1]) * (internal_state.pos[1]- pi-> wps[1]);
        samples_cost[n] += pi->TARGET_PENALTY* pi->dh * dist_target;
        //printf("target cost %f\n", samples_cost[n]);
        float cross_product_3 = abs(internal_state.vel[0] * pi->wps[1] - internal_state.vel[1] * pi->wps[0]);
        samples_cost[n] += pi->HEADING_PENALTY * pi->dh * cross_product_3;
        //printf("heading cost %f\n", samples_cost[n]);
        for(int a=0; a < pi->units-1; a++){
          float dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[0+2*a]) * (internal_state.pos[0]- internal_state.pos_rel[0+2*a]) + (internal_state.pos[1]- internal_state.pos_rel[1+2*a]) * (internal_state.pos[1]- internal_state.pos_rel[1+2*a]);

          if(dist_unit > pi->COLLISION_DISTANCE ){}
          else{ samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));}
          //printf("collision cost %f\n", samples_cost[n]);
          }
      }
      else{
        float dist_leader = (internal_state.pos[0]- internal_state.pos_rel[0]) * (internal_state.pos[0]- internal_state.pos_rel[0]) + (internal_state.pos[1]- internal_state.pos_rel[1]) * (internal_state.pos[1]- internal_state.pos_rel[1]);
        if(dist_leader < pi->COHESION_DISTANCE ){}
        else{ samples_cost[n] += exp(pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));}

        for(int a=0; a < pi->units-1; a++){
          float dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[0+2*a]) * (internal_state.pos[0]- internal_state.pos_rel[0+2*a]) + (internal_state.pos[1]- internal_state.pos_rel[0+2*a]) * (internal_state.pos[1]- internal_state.pos_rel[0+2*a]);
          if(dist_unit > pi->COLLISION_DISTANCE ){}
          else{ samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));}


          float cross_product_3 = abs(internal_state.pos[0] * internal_state.pos_rel[1+2*a] - internal_state.pos[1] * internal_state.pos_rel[0+2*a]);
          if(cross_product_3 > pi->PARALLEL_THR){}
          else{samples_cost[n] += exp(pi->PARALLEL_PENALTY *(pi->PARALLEL_THR - cross_product_3));}

        }
      }

       // Propagate leader unit
       internal_state.vel[0] += u_roll[h][0]*pi->dh;
       internal_state.vel[1] += u_roll[h][1]*pi->dh;
       internal_state.pos[0] += internal_state.vel[0]*pi->dh;
       internal_state.pos[1] += internal_state.vel[1]*pi->dh;

       // Propagate follower units
       for(int a=0; a < pi->units-1; a++) {
         internal_state.pos_rel[0+2*a] += internal_state.vel_rel[0+2*a] * pi->dh;
         internal_state.pos_rel[1+2*a] += internal_state.vel_rel[1+2*a] * pi->dh;
         }

     }

    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}

  }

  printf("Min cost is:%f\n",min_cost);


  // Compute weight of all samples N
  float w[pi->N];
  float w_sum = 0;
  for (int n=0; n < pi->N; ){
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

  float internal_controls[pi->iH][pi->dimU];
  for(int h = 0; h < pi->iH; h++){
    internal_controls[h][0] = 0;
    internal_controls[h][1] = 0;
  }

  for (int n=0; n < pi->N; n++ ){
    w[n] = w[n]/w_sum;
    for(int h = 0; h < pi->iH; h++){
      internal_controls[h][0] += w[n] * noise[n][h][0];
      internal_controls[h][1] += w[n] * noise[n][h][1];
    }
   }

  for(int h = 0; h < pi->iH; h++){
    internal_controls[h][0] /= pi->dh;
    internal_controls[h][1] /= pi->dh;
  }

  // Compute optimal controls
  //printf("exploring control 0:%f,%f Internal control 1:%f %f\n",pi->u_exp[0][0],pi->u_exp[0][1],internal_controls[0][0],internal_controls[0][1]);
  result->pi_vel.x = pi->u_exp[0][0] + internal_controls[0][0];
  result->pi_vel.y = pi->u_exp[0][1] + internal_controls[0][1];

  for(int h = 0; h < pi->iH; h++){
    pi->u_exp[h][0] += internal_controls[h][0];
    pi->u_exp[h][1] += internal_controls[h][1];
  }
  if(result->pi_vel.x > 0 && result->pi_vel.y > 0){
    success = true;
  }

  return success;

}
