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
#include <math/pprz_simple_matrix.h>
#include <math/pprz_algebra_float.h>
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
#ifndef SAMPLING_METHOD
#define SAMPLING_METHOD 0
#endif


// Internal functions
void rotate(int8_t angle, float ** Rmatrix);
void compute_probes(int8_t angle, uint8_t num_probes, float * v_init, float ** VProbes);
void select_probe(uint8_t num_probes, int8_t angle, float ** u_roll, struct path_integral_t *pi, struct pi_state_t *st, struct pi_wp_t *wp, float * best_probe_vel);


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
  struct pi_state_t internal_state, initial_state;
  float min_cost = 1000000;
  float inv_lambda = 1/(pi->lambda);
  float dt = 1/pi->freq;
  float half_dh = pi->dh*0.5;
  float cost_sum = 0;

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

  for (uint8_t i = 0; i< pi->dimU; i++){
    initial_state.pos[i] = st->pos[i];
    initial_state.vel[i] = st->vel[i];
    initial_state.pos_rel[i] = st->pos_rel[i];
    initial_state.vel_rel[i] = st->vel_rel[i];
  }

  switch (SAMPLING_METHOD){

    case 0 :

      printf("[sampling] Sampling method 0.\n");
      //printf("[sampling] Initial U %f, %f\n",u_roll[0][0], u_roll[0][1]);

      break;

    case 1 :

      printf("[sampling] Sampling method 1.\n");
      int8_t angle = 30;
      uint8_t num_probes = 7;
      MAKE_MATRIX_PTR(u_roll_ptr, u_roll, pi->iH);
      float best_probe_vel[pi->dimU];
      select_probe(num_probes, angle, u_roll_ptr, pi, st, wp, &best_probe_vel[0]);

      //printf("[sampling] Best probe %f, %f \n",best_probe_vel[0], best_probe_vel[1]);
      //printf("[sampling] Initial U %f, %f\n",u_roll[0][0], u_roll[0][1]);

      if(u_roll[0][0] != 0.0f){
        for(int h=0; h< pi->iH; h++){
          u_roll[h][0] -= (best_probe_vel[0] - initial_state.vel[0]);
          u_roll[h][1] -= (best_probe_vel[1] - initial_state.vel[1]);
        }
      }

      //printf("[sampling] Adjusted U %f, %f, diff %f, initial vel %f \n",u_roll[0][0], u_roll[0][1],(best_probe_vel[0] - initial_state.vel[0]), initial_state.vel[0]);
      initial_state.vel[0] = best_probe_vel[0];
      initial_state.vel[1] = best_probe_vel[1];

      pi->N = 100;
      break;
   }


  // Create and compute cost of all samples N
  for (int n=0; n < pi->N; n++){

    samples_cost[n] = 0;

    for (uint8_t i = 0; i< pi->dimU; i++){
      internal_state.pos[i] = initial_state.pos[i];   //st->pos[i];
      internal_state.vel[i] = initial_state.vel[i];   //st->vel[i];
      internal_state.pos_rel[i] = initial_state.pos_rel[i];   //st->pos_rel[i];
      internal_state.vel_rel[i] = initial_state.vel_rel[i];   //st->vel_rel[i];
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
      Bound(internal_state.vel[0],-pi->MAX_SPEED, pi->MAX_SPEED);
      Bound(internal_state.vel[1],-pi->MAX_SPEED, pi->MAX_SPEED);

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
    cost_sum += samples_cost[n];
    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}

  }

  float mean_cost = cost_sum/pi->N;
  result->min_cost = min_cost;
  //printf("Min cost %f\n", min_cost);
  //Compute weight of all samples N
  float w[pi->N];
  float w_sum = 0;
  float sample_var = 0;
  for (int n=0; n < pi->N; ){
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
  }

  printf("[variance] %f\n", sample_var);

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
    Bound(result->vel.x,-pi->MAX_SPEED, pi->MAX_SPEED);
    Bound(result->vel.y,-pi->MAX_SPEED, pi->MAX_SPEED);

    //Save controls for importance sampling
    for(int h = 0; h < pi->iH; h++){
      pi->u_exp[h][0] = u_roll[h][0] + internal_controls[h][0];
      pi->u_exp[h][1] = u_roll[h][1] + internal_controls[h][1];
    }

    success = true;
  }

  return success;

}




/**
 * Compute rotation matrix
 * @param[in]  angle [degrees]
 * @param[in]  *Rmatrix rotation matrix
 */
void rotate(int8_t angle, float ** Rmatrix){
  float radians = angle * (M_PI / 180.0);

  Rmatrix[0][0] = cosf(radians);
  Rmatrix[0][1] = -sinf(radians);
  Rmatrix[1][0] = sinf(radians);
  Rmatrix[1][1] = cosf(radians);
}


/**
 * Compute velocity probes by rotating a given velocity vector
 * @param[in]  angle [degrees]
 * @param[in]  num_probes
 * @param[in]  * v_init
 * @param[out]  **VProbes
 */
void compute_probes(int8_t angle, uint8_t num_probes, float * v_init, float ** VProbes){

  float _Rmatrix[2][2];
  MAKE_MATRIX_PTR(Rmatrix, _Rmatrix, 2);
  float _v[1][2] = { {*v_init, *(v_init+1)} };
  MAKE_MATRIX_PTR(v, _v, 1);
  float _CC[1][2];
  MAKE_MATRIX_PTR(CC, _CC, 1);
  float _C[1][2];
  MAKE_MATRIX_PTR(C,_C, 1);

  int half_probes = (num_probes-1)/2;
  for(uint8_t i=0; i<half_probes; i++){
    rotate((i+1)*angle, Rmatrix);
    MAT_MUL(1,2,2, CC, v, Rmatrix);
    VProbes[i][0] = CC[0][0];
    VProbes[i][1] = CC[0][1];

    rotate(-(i+1)*angle, Rmatrix);
    MAT_MUL(1,2,2, C, v, Rmatrix );
    VProbes[half_probes+i][0] = C[0][0];
    VProbes[half_probes+i][1] = C[0][1];
  }
  VProbes[num_probes-1][0] = _v[0][0];
  VProbes[num_probes-1][1] = _v[0][1];

}



/**
 * Propagate the velocity probes along the horizon and choose the one with minimum cost
 * @param[in]  num_probes
 * @param[in]  angle [degrees]
 * @param[in]  **u_roll controls over the horizon
 * @param[in]  *pi  path_integral struct with all parameters
 * @param[in]  *st  state struct
 * @param[in]  *wp  waypoint given to the leader
 * @param[out] best_probe_velocity the velocity of the probe with the least cost
 */
void select_probe(uint8_t num_probes, int8_t angle, float ** u_roll, struct path_integral_t *pi, struct pi_state_t *st, struct pi_wp_t *wp , float * best_probe_vel){

  float samples_cost_probes[num_probes];
  struct pi_state_t internal_state_probes, initial_state;
  float min_cost = 1000000;
  uint8_t min_cost_index = 0;
  float half_dh = pi->dh*0.5;
  float u_horizon[pi->iH][pi->dimU];

  //TODO: Send this to the function instead of doing it again
  for (int i = 0; i< pi->dimU; i++){
    initial_state.pos[i] = st->pos[i];
    initial_state.vel[i] = st->vel[i];
    initial_state.pos_rel[i] = st->pos_rel[i];
    initial_state.vel_rel[i] = st->vel_rel[i];
  }

  //initial_state.vel[0] = 0.2;
  //initial_state.vel[1] = 0;
  //printf("1. Initial state for probes: x %f, v %f x_rel %f\n", initial_state.pos[0], initial_state.vel[0], initial_state.pos_rel[0]);
  float _VProbes[num_probes][pi->dimU];
  MAKE_MATRIX_PTR(VProbes, _VProbes, num_probes);
  compute_probes(angle, num_probes, &initial_state.vel[0] , VProbes);

  //printf("2. Probes: v00 %f, v01 %f v10 %f, v11 %f\n", _VProbes[0][0], _VProbes[0][1], _VProbes[1][0], _VProbes[1][1]);
  //printf("2.1. Probes: v20 %f, v21 %f v30 %f, v31 %f\n", _VProbes[2][0], _VProbes[2][1], _VProbes[3][0], _VProbes[3][1]);
  //printf("2.1. Probes: v40 %f, v41 %f v50 %f, v51 %f\n", _VProbes[4][0], _VProbes[4][1], _VProbes[5][0], _VProbes[5][1]);
  //printf("2.1. Probes: v60 %f, v61 %f \n", _VProbes[6][0], _VProbes[6][1]);
  for(uint8_t p = 0; p < num_probes; p++){

    samples_cost_probes[p] = 0;
    for (int i = 0; i< pi->dimU; i++){
      internal_state_probes.pos[i] = initial_state.pos[i];
      internal_state_probes.vel[i] = _VProbes[p][i];
      internal_state_probes.pos_rel[i] = initial_state.pos_rel[i];
      internal_state_probes.vel_rel[i] = initial_state.vel_rel[i];
    }

    for(uint8_t h = 0; h > pi->iH; h++){

      if(u_horizon[0][0] != 0.0f){
        u_horizon[h][0] = u_roll[h][0] - (_VProbes[p][0] - initial_state.vel[0]);
        u_horizon[h][1] = u_roll[h][1] - (_VProbes[p][1] - initial_state.vel[1]);
      }
      else{
        u_horizon[h][0] = u_roll[h][0];
        u_horizon[h][1] = u_roll[h][1];
      }

      //printf("3. Adjust control: u_hor %f, u_roll %f Vprobe %f,initial vel %f \n", u_horizon[h][0], u_roll[h][0], _VProbes[p][0], initial_state.vel[0]);

      internal_state_probes.vel[0] += u_horizon[h][0]*pi->dh;
      internal_state_probes.vel[1] += u_horizon[h][1]*pi->dh;

      // Check max velocity
      Bound(internal_state_probes.vel[0],-pi->MAX_SPEED, pi->MAX_SPEED);
      Bound(internal_state_probes.vel[1],-pi->MAX_SPEED, pi->MAX_SPEED);

      // Propagate position
      internal_state_probes.pos[0] += internal_state_probes.vel[0] * pi->dh;
      internal_state_probes.pos[1] += internal_state_probes.vel[1] * pi->dh;
      internal_state_probes.pos_rel[0] += internal_state_probes.vel_rel[0] * pi->dh;
      internal_state_probes.pos_rel[1] += internal_state_probes.vel_rel[1] * pi->dh;

      // Control Cost
      samples_cost_probes[p] += pi->R * fabs(u_horizon[h][0]*u_horizon[h][0]*half_dh  + u_horizon[h][1]*u_horizon[h][1]*half_dh);

      if(pi->leader){
        // Target distance cost
        float dist_target = (internal_state_probes.pos[0]- wp->pos_N) * (internal_state_probes.pos[0]- wp->pos_N) + (internal_state_probes.pos[1]- wp->pos_E ) * (internal_state_probes.pos[1]- wp->pos_E );
        //samples_cost_probes[p] += pi->TARGET_PENALTY* pi->dh * dist_target;
        samples_cost_probes[p] += exp((dist_target - 4));

        // Collision cost
        float dist_unit = (internal_state_probes.pos[0]-  internal_state_probes.pos_rel[0]) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[0]) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[1]) * (internal_state_probes.pos[1]- internal_state_probes.pos_rel[1]);
        if(dist_unit > pi->COLLISION_DISTANCE ){}
        else{ samples_cost_probes[p] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));}

        // Min velocity cost
        float velocity_vec = internal_state_probes.vel[0] * internal_state_probes.vel[0] + internal_state_probes.vel[1] * internal_state_probes.vel[1];
        if(velocity_vec >0.3){}
        else{
          samples_cost_probes[p] += 100*(0.3 - velocity_vec)*(0.3 - velocity_vec);
        }


        // Out of bounds cost
        if(internal_state_probes.pos[0] > CZ_LIMIT){
          float outside = internal_state_probes.pos[0] - CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state_probes.pos[0] < -CZ_LIMIT){
          float outside = internal_state_probes.pos[0] + CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state_probes.pos[1] > CZ_LIMIT){
          float outside = internal_state_probes.pos[1] - CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state_probes.pos[1] < -CZ_LIMIT){
          float outside = internal_state_probes.pos[1] + CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
      }
      else{

        float dist_target = (internal_state_probes.pos[0]- wp->pos_N) * (internal_state_probes.pos[0]- wp->pos_N) + (internal_state_probes.pos[1]- wp->pos_E ) * (internal_state_probes.pos[1]- wp->pos_E );
        //samples_cost_probes[p] += pi->TARGET_PENALTY* pi->dh * dist_target;
        samples_cost_probes[p] += exp((dist_target - 4));

        float dist_leader = (internal_state_probes.pos[0]- internal_state_probes.pos_rel[0]) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[0]) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[1]) * (internal_state_probes.pos[1]- internal_state_probes.pos_rel[1]);
        //if(dist_leader < pi->COHESION_DISTANCE ){}
        //else{samples_cost_probes[p] += exp(pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));}

        //for(int a=0; a < pi->units-1; a++){
        //float dist_unit = (internal_state_probes.pos[0]-  internal_state_probes.pos_rel[0+2*a]) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[0+2*a]) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[0+2*a]) * (internal_state_probes.pos[1]- internal_state_probes.pos_rel[0+2*a]);
        if(dist_leader > pi->COLLISION_DISTANCE ){}
        else{ samples_cost_probes[p] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_leader));}

        float velocity_vec = internal_state_probes.vel[0] * internal_state_probes.vel[0] + internal_state_probes.vel[1] * internal_state_probes.vel[1];
        if(velocity_vec >0.3){}
        else{
          samples_cost_probes[p] += 100*(0.3 - velocity_vec)*(0.3 - velocity_vec);
        }

        // Out of bounds cost
        if(internal_state_probes.pos[0] > CZ_LIMIT){
          float outside = internal_state_probes.pos[0] - CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state_probes.pos[0] < -CZ_LIMIT){
          float outside = internal_state_probes.pos[0] + CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state_probes.pos[1] > CZ_LIMIT){
          float outside = internal_state_probes.pos[1] - CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
        if(internal_state_probes.pos[1] < -CZ_LIMIT){
          float outside = internal_state_probes.pos[1] + CZ_LIMIT;
          samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
        }
      }
    }

    if(samples_cost_probes[p] < min_cost){
      min_cost = samples_cost_probes[p];
      min_cost_index = p;
    }
  }

  best_probe_vel[0] = _VProbes[min_cost_index][0];
  best_probe_vel[1] = _VProbes[min_cost_index][1];

}



