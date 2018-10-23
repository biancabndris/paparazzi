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
#include <math.h>
#include <stdio.h>

#ifndef PI_FREQ
#define PI_FREQ 15
#endif
#ifndef PI_LEADER
#define PI_LEADER 1
#endif
#ifndef PI_HORIZON
#define PI_HORIZON 2
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
#define PI_SAMPLES 300
#endif
#ifndef PI_COLLISION_DISTANCE
#define PI_COLLISION_DISTANCE 1//0.25//2.25
#endif
#ifndef PI_COHESION_DISTANCE
#define PI_COHESION_DISTANCE 4////6.25
#endif
#ifndef PI_COLLISION_PENALTY
#define PI_COLLISION_PENALTY 11//40//10
#endif
#ifndef PI_COHESION_PENALTY
#define PI_COHESION_PENALTY 1//0.1//5
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
#define PI_MAX_SPEED 0.5
#endif
#ifndef PI_PARALLEL_THR
#define PI_PARALLEL_THR 0.3
#endif
#ifndef PI_UNITS
#define PI_UNITS 2
#endif
#ifndef PI_DIM_U
#define PI_DIM_U 2
#endif
#ifndef CZ_LIMIT
#define CZ_LIMIT 2.8
#endif
#ifndef PI_OUTSIDECZ_PENALTY
#define PI_OUTSIDECZ_PENALTY 5
#endif



void pi_calc_init(struct path_integral_t *pi){

     pi->freq               = PI_FREQ;
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
     pi->OUTSIDECZ_PENALTY   = PI_OUTSIDECZ_PENALTY;

     pi->units              = PI_UNITS;
     pi->dimU               = PI_DIM_U;


     pi->seed               = gsl_rng_alloc(gsl_rng_mt19937);
     gsl_rng_set(pi->seed, 0);

     for(int h=0; h < pi->iH; h++) {
       for(int u=0; u < pi->dimU; u++) {
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
bool pi_calc_timestep(struct path_integral_t *pi, struct pi_state_t *st, struct pi_wp_t *wp, struct pi_result_t *result){

  bool success = false;
  float noise[pi->N][pi->iH][pi->dimU];
  float u_roll[pi->iH][pi->dimU];
  float stdv = sqrt(pi->var);
  float samples_cost[pi->N];
  struct pi_state_t internal_state;
  float min_cost = 1000000;
  float inv_lambda = 1/(pi->lambda);
  float dt = 1/pi->freq;
  float half_dh = pi->dh*0.5;

  // Shift controls with dt
  float shift = (pi->dh - dt)/pi->dh;
  float shift2 = dt/(pi->dh - dt);

  for (int h=0; h < pi->iH-1; h++){
    u_roll[h][0] = pi->u_exp[h][0]*shift;
    u_roll[h][1] = pi->u_exp[h][1]*shift;

    u_roll[h][0] += pi->u_exp[h+1][0]*shift2;
    u_roll[h][1] += pi->u_exp[h+1][1]*shift2;
  }
  u_roll[pi->iH-1][0] = pi->u_exp[pi->iH-1][0]*shift;
  u_roll[pi->iH-1][1] = pi->u_exp[pi->iH-1][1]*shift;

  // Create and compute cost of all samples N
  for (int n=0; n < pi->N; n++){

    samples_cost[n] = 0;

    for (int i = 0; i< 2; i++){
      internal_state.pos[i] = st->pos[i];
      internal_state.vel[i] = st->vel[i];
      internal_state.pos_rel[i] = st->pos_rel[i];
      internal_state.vel_rel[i] = st->vel_rel[i];
    }
    //printf("State %f, %f\n",internal_state.pos[0], internal_state.vel[0]);
    float applied_vel_n = internal_state.vel[0];
    float applied_vel_e = internal_state.vel[1];

    for (int h=0; h < pi->iH; h++){
      noise[n][h][0] = gsl_ran_gaussian(pi->seed,1.) * stdv;
      noise[n][h][1] = gsl_ran_gaussian(pi->seed,1.) * stdv;


      // Propagate leader unit
      internal_state.vel[0] = applied_vel_n + u_roll[h][0]*pi->dh + noise[n][h][0];
      internal_state.vel[1] = applied_vel_e + u_roll[h][1]*pi->dh + noise[n][h][1];

      // Check max velocity
      if(internal_state.vel[0] > pi->MAX_SPEED){
        internal_state.vel[0]  = pi->MAX_SPEED;
      }

      if(internal_state.vel[1] > pi->MAX_SPEED){
        internal_state.vel[1]  = pi->MAX_SPEED;
      }

      if(internal_state.vel[0] < -1*pi->MAX_SPEED){
        internal_state.vel[0]  = -1*pi->MAX_SPEED;
      }

      if(internal_state.vel[1] < -1*pi->MAX_SPEED){
        internal_state.vel[1]  = -1*pi->MAX_SPEED;
      }

      float rise_time = 4;
      float ratio = pi->dh/(rise_time+pi->dh);

      applied_vel_n = (1-ratio)*applied_vel_n + ratio*internal_state.vel[0];
      applied_vel_e = (1-ratio)*applied_vel_e + ratio*internal_state.vel[1];

      internal_state.pos[0] += applied_vel_n*pi->dh;
      internal_state.pos[1] += applied_vel_e*pi->dh;

      //Propagate follower units
      //for(int a=0; a < pi->units; a++) {
        internal_state.pos_rel[0] += internal_state.vel_rel[0] * pi->dh;
        internal_state.pos_rel[1] += internal_state.vel_rel[1] * pi->dh;
      //}
      // Control Cost
      samples_cost[n] += pi->R * fabs(u_roll[h][0]*u_roll[h][0]*half_dh + u_roll[h][0]*noise[n][h][0] + u_roll[h][1]*u_roll[h][1]*half_dh + u_roll[h][1]*noise[n][h][1]);

      if(pi->leader){
        // Target distance cost
        float dist_target = (internal_state.pos[0]- wp->pos_N) * (internal_state.pos[0]- wp->pos_N) + (internal_state.pos[1]- wp->pos_E ) * (internal_state.pos[1]- wp->pos_E );
        //samples_cost[n] += pi->TARGET_PENALTY* pi->dh * dist_target;
        samples_cost[n] += exp((dist_target - 4));

        //if(dist_target > 0.5f){
        //  float cross_product_3 = fabs(applied_vel_n * wp->pos_E - applied_vel_e* wp->pos_N);
        //  samples_cost[n] += pi->HEADING_PENALTY * pi->dh * cross_product_3;
        //}

        //for(int a=0; a < pi->units-1; a++){
          float dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[0]) * (internal_state.pos[0]- internal_state.pos_rel[0]) + (internal_state.pos[1]- internal_state.pos_rel[1]) * (internal_state.pos[1]- internal_state.pos_rel[1]);
          if(dist_unit > pi->COLLISION_DISTANCE ){}
          else{ samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));}
        //}
         float velocity_vec = applied_vel_n * applied_vel_n + applied_vel_e * applied_vel_e;
         if(velocity_vec >0.3){}
         else{
           samples_cost[n] += 100*(0.3 - velocity_vec)*(0.3 - velocity_vec);
         }


        if(internal_state.pos[0] > CZ_LIMIT){
          float outside = internal_state.pos[0] - CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state.pos[0] < -CZ_LIMIT){
        float outside = internal_state.pos[0] + CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state.pos[1] > CZ_LIMIT){
          float outside = internal_state.pos[1] - CZ_LIMIT;
          samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
       if(internal_state.pos[1] < -CZ_LIMIT){
          float outside = internal_state.pos[1] + CZ_LIMIT;
         samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
      }
      else{

        float dist_target = (internal_state.pos[0]- wp->pos_N) * (internal_state.pos[0]- wp->pos_N) + (internal_state.pos[1]- wp->pos_E ) * (internal_state.pos[1]- wp->pos_E );
        //samples_cost[n] += pi->TARGET_PENALTY* pi->dh * dist_target;
        samples_cost[n] += exp((dist_target - 4));

        float dist_leader = (internal_state.pos[0]- internal_state.pos_rel[0]) * (internal_state.pos[0]- internal_state.pos_rel[0]) + (internal_state.pos[1]- internal_state.pos_rel[1]) * (internal_state.pos[1]- internal_state.pos_rel[1]);
        //if(dist_leader < pi->COHESION_DISTANCE ){}
        //else{samples_cost[n] += exp(pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));}

        //for(int a=0; a < pi->units-1; a++){
          //float dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[0+2*a]) * (internal_state.pos[0]- internal_state.pos_rel[0+2*a]) + (internal_state.pos[1]- internal_state.pos_rel[0+2*a]) * (internal_state.pos[1]- internal_state.pos_rel[0+2*a]);
          if(dist_leader > pi->COLLISION_DISTANCE ){}
          else{ samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_leader));}
          //printf("Diff dist leader %f, diff dist unit %f \n", dist_leader - pi->COHESION_DISTANCE, pi->COLLISION_DISTANCE - dist_unit );
/*
          float cross_product_3 = fabs(internal_state.pos[0] * internal_state.pos_rel[1+2*a] - internal_state.pos[1] * internal_state.pos_rel[0+2*a]);
          if(cross_product_3 > pi->PARALLEL_THR){}
          else{samples_cost[n] += exp(pi->PARALLEL_PENALTY *(pi->PARALLEL_THR - cross_product_3));}
*/
        //}

        float velocity_vec = applied_vel_n * applied_vel_n + applied_vel_e * applied_vel_e;
        if(velocity_vec >0.3){}
        else{
          samples_cost[n] += 100*(0.3 - velocity_vec)*(0.3 - velocity_vec);
        }


        if(internal_state.pos[0] > CZ_LIMIT){
          float outside = internal_state.pos[0] - CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state.pos[0] < -CZ_LIMIT){
        float outside = internal_state.pos[0] + CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state.pos[1] > CZ_LIMIT){
          float outside = internal_state.pos[1] - CZ_LIMIT;
          samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
       if(internal_state.pos[1] < -CZ_LIMIT){
          float outside = internal_state.pos[1] + CZ_LIMIT;
         samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
      }
    }

    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}

  }

  result->min_cost = min_cost;
  printf("Min cost %f\n", min_cost);
  //Compute weight of all samples N
  float w[pi->N];
  float w_sum = 0;
  for (int n=0; n < pi->N; ){
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    w_sum += w[n]; n++;
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
  float ned_vel_n =  st->vel[0] + (u_roll[0][0] + internal_controls[0][0])*(dt);
  float ned_vel_e =  st->vel[1] + (u_roll[0][1] + internal_controls[0][1])*(dt);

  result->vel.x = ned_vel_n;
  result->vel.y = ned_vel_e;

  if(!isnan(result->vel.x) && !isnan(result->vel.y)){

    if(result->vel.x > pi->MAX_SPEED){
      result->vel.x  = pi->MAX_SPEED;
    }

    if(result->vel.y > pi->MAX_SPEED){
      result->vel.y  = pi->MAX_SPEED;
    }

    if(result->vel.x < -1*pi->MAX_SPEED){
      result->vel.x  = -1*pi->MAX_SPEED;
    }

    if(result->vel.y < -1*pi->MAX_SPEED){
      result->vel.y  = -1*pi->MAX_SPEED;
    }

    //Save controls for importance sampling
    for(int h = 0; h < pi->iH; h++){
      pi->u_exp[h][0] = u_roll[h][0] + internal_controls[h][0];
      pi->u_exp[h][1] = u_roll[h][1] + internal_controls[h][1];
    }

    success = true;
  }

  return success;

}

