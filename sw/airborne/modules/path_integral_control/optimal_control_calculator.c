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
#define PI_SAMPLES 100
#endif
#ifndef PI_COLLISION_DISTANCE
#define PI_COLLISION_DISTANCE 1//0.25//2.25
#endif
#ifndef PI_COHESION_DISTANCE
#define PI_COHESION_DISTANCE 6.25//4 //2.25////6.25
#endif
#ifndef PI_COLLISION_PENALTY
#define PI_COLLISION_PENALTY 11//40//10
#endif
#ifndef PI_COHESION_PENALTY
#define PI_COHESION_PENALTY 1 //100//1//0.1//5
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
#ifndef PI_MIN_SPEED
#define PI_MIN_SPEED 0.3
#endif
#ifndef PI_MIN_SPEED_PENALTY
#define PI_MIN_SPEED_PENALTY 100
#endif
#ifndef PI_PARALLEL_THR
#define PI_PARALLEL_THR 0.3
#endif
#ifndef PI_REL_UNITS
#define PI_REL_UNITS 3
#endif
#ifndef PI_DIM_U
#define PI_DIM_U 2
#endif
#ifndef PI_CZ_LIMIT
#define PI_CZ_LIMIT 2.8
#endif
#ifndef PI_OUTSIDECZ_PENALTY
#define PI_OUTSIDECZ_PENALTY 5 //5
#endif
#ifndef PI_RADIUS
#define PI_RADIUS 6.25
#endif
#ifndef PI_TASK
#define PI_TASK 0
#endif
#ifndef PI_SAMPLING_METHOD
#define PI_SAMPLING_METHOD 0
#endif
#ifndef PI_PROBE_ANGLE
#define PI_PROBE_ANGLE 30
#endif
#ifndef PI_NUM_PROBES
#define PI_NUM_PROBES 13  // 7 // 13
#endif
#ifndef PI_FOLLOWING_ID
#define PI_FOLLOWING_ID 140
#endif



// Internal functions
void rotate(int angle, float ** Rmatrix);
void compute_probes(int angle, uint8_t num_probes, float * v_init, float ** VProbes);
void select_probe(uint8_t sampling_method, uint8_t num_probes, int angle, float ** u_roll, struct path_integral_t *pi, struct pi_state_t *st, struct pi_wp_t *wp, float * best_probe_vel);
void compute_control_probes(int angle, uint8_t num_probes, uint8_t probe, float * u_init, float ** UProbes);
//void copy_state(uint8_t rel_units, uint8_t dimU, struct pi_state_t * dest, struct pi_state_t * origin );

void pi_calc_init(struct path_integral_t *pi){

     pi->freq               = PI_FREQ;
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
     pi->MIN_SPEED          = PI_MIN_SPEED;
     pi->MIN_SPEED_PENALTY  = PI_MIN_SPEED_PENALTY;
     pi->PARALLEL_THR       = PI_PARALLEL_THR;
     pi->OUTSIDECZ_PENALTY  = PI_OUTSIDECZ_PENALTY;
     pi->CZ_LIMIT           = PI_CZ_LIMIT;
     pi->RADIUS             = PI_RADIUS;
     pi->TASK               = PI_TASK;
     pi->SAMPLING_METHOD    = PI_SAMPLING_METHOD;
     pi->PROBE_ANGLE        = PI_PROBE_ANGLE;
     pi->NUM_PROBES         = PI_NUM_PROBES;
     pi->BEST_PROBE         = 100;

     pi->rel_units          = PI_REL_UNITS;
     pi->dimU               = PI_DIM_U;
     pi->following_id       = PI_FOLLOWING_ID;
     pi->following_index    = 0;

     pi->seed               = gsl_rng_alloc(gsl_rng_mt19937);
     gsl_rng_set(pi->seed, 0);

     init_controls(pi);

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

  // Initialize states
  for (uint8_t i = 0; i< pi->dimU; i++){
    initial_state.pos[i] = st->pos[i];
    initial_state.vel[i] = st->vel[i];
  }
  for(uint8_t a = 0; a< pi->rel_units; a++){
    initial_state.pos_rel[a].N = st->pos_rel[a].N;
    initial_state.pos_rel[a].E = st->pos_rel[a].E;
    initial_state.vel_rel[a].N = st->vel_rel[a].N;
    initial_state.vel_rel[a].E = st->vel_rel[a].E;
  }

  switch (pi->SAMPLING_METHOD){

    case 0 :

    {break;}

    case 2 :

    {
      //printf("[sampling] Sampling method 2.\n");

      MAKE_MATRIX_PTR(u_roll_ptr, u_roll, pi->iH);
      float best_probe_vel[pi->dimU];

      //todo remove sampling method and probe from function, already pass the pi pointer
      select_probe(pi->SAMPLING_METHOD, pi->NUM_PROBES, pi->PROBE_ANGLE, u_roll_ptr, pi, st, wp, &best_probe_vel[0]);

      //printf("[sampling] Best probe %f, %f \n",best_probe_vel[0], best_probe_vel[1]);
      //printf("[sampling] Modified U %f, %f\n",u_roll[0][0], u_roll[0][1]);

      //printf("[sampling] Adjusted U %f, %f, diff %f, initial vel %f \n",u_roll[0][0], u_roll[0][1],(best_probe_vel[0] - initial_state.vel[0]), initial_state.vel[0]);
      initial_state.vel[0] = best_probe_vel[0];
      initial_state.vel[1] = best_probe_vel[1];

      break;}
  }


  // Create and compute cost of all samples N
  for (int n=0; n < pi->N; n++){

    samples_cost[n] = 0;

    for (uint8_t i = 0; i< pi->dimU; i++){
      internal_state.pos[i] = initial_state.pos[i];
      internal_state.vel[i] = initial_state.vel[i];
    }
    for(uint8_t i = 0; i< pi->rel_units; i++){
      internal_state.pos_rel[i].N = initial_state.pos_rel[i].N;
      internal_state.pos_rel[i].E = initial_state.pos_rel[i].E;
      internal_state.vel_rel[i].N = initial_state.vel_rel[i].N;
      internal_state.vel_rel[i].E = initial_state.vel_rel[i].E;
    }

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

      float rise_time = 4;  //4
      float ratio = pi->dh/(rise_time+pi->dh);

      applied_vel_n = (1-ratio)*applied_vel_n + ratio*internal_state.vel[0];
      applied_vel_e = (1-ratio)*applied_vel_e + ratio*internal_state.vel[1];

      internal_state.pos[0] += applied_vel_n*pi->dh;
      internal_state.pos[1] += applied_vel_e*pi->dh;

      //Propagate follower rel_units
      for(int a=0; a < pi->rel_units; a++) {
        internal_state.pos_rel[a].N += internal_state.vel_rel[a].N * pi->dh;
        internal_state.pos_rel[a].E += internal_state.vel_rel[a].E * pi->dh;
      }

      // Control Cost
      samples_cost[n] += pi->R * fabs(u_roll[h][0]*u_roll[h][0]*half_dh + u_roll[h][0]*noise[n][h][0] + u_roll[h][1]*u_roll[h][1]*half_dh + u_roll[h][1]*noise[n][h][1]);


      // Out of boundaries cost
      if(internal_state.pos[0] > pi->CZ_LIMIT){
        float outside = internal_state.pos[0] - pi->CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }
      if(internal_state.pos[0] < -pi->CZ_LIMIT){
        float outside = internal_state.pos[0] + pi->CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }
      if(internal_state.pos[1] > pi->CZ_LIMIT){
        float outside = internal_state.pos[1] - pi->CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }
      if(internal_state.pos[1] < -pi->CZ_LIMIT){
        float outside = internal_state.pos[1] + pi->CZ_LIMIT;
        samples_cost[n] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }

      //float outside2 =  internal_state.pos[0]*internal_state.pos[0] + internal_state.pos[1]*internal_state.pos[1];
      //samples_cost[n] += 10/((6.25-outside2)*(6.25-outside2));
/*      float acc =  u_roll[h][0]*u_roll[h][0] + u_roll[h][1]*u_roll[h][1];
      if(acc > (pi->MAX_ACC * pi->MAX_ACC)){
        samples_cost[n] += (acc - (pi->MAX_ACC * pi->MAX_ACC)) * (acc - (pi->MAX_ACC * pi->MAX_ACC));
      }*/


      float dist_unit = 0;
      float dist_wp = 0;
      switch (pi->TASK){
        case 0:

        {
          //printf("[task] Leader task selected\n");

          // Distance from WP cost
          dist_wp = (internal_state.pos[0]- wp->pos_N) * (internal_state.pos[0]- wp->pos_N) + (internal_state.pos[1]- wp->pos_E ) * (internal_state.pos[1]- wp->pos_E );
          samples_cost[n] += pi->TARGET_PENALTY* pi->dh * dist_wp;

          // Heading cost
          if(dist_wp > 0.5f){
            float cross_product_3 = fabs(applied_vel_n * wp->pos_E - applied_vel_e* wp->pos_N);
            samples_cost[n] += pi->HEADING_PENALTY * pi->dh * cross_product_3;
          }

          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[a].N) * (internal_state.pos[0]- internal_state.pos_rel[a].N) + (internal_state.pos[1]- internal_state.pos_rel[a].E) * (internal_state.pos[1]- internal_state.pos_rel[a].E);
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{ samples_cost[n] += exp(5 * (pi->COLLISION_DISTANCE - dist_unit));} // pi->COLLISION_PENALTY
          }

          break;}


        case 1:

        {
          //printf("[task] Follower task selected\n");

          // Cohesion cost
          float dist_leader = (internal_state.pos[0]- internal_state.pos_rel[pi->following_index].N) * (internal_state.pos[0]- internal_state.pos_rel[pi->following_index].N) + (internal_state.pos[1]- internal_state.pos_rel[pi->following_index].E) * (internal_state.pos[1]- internal_state.pos_rel[pi->following_index].E);
          if(dist_leader < pi->COHESION_DISTANCE ){}
          else{
            samples_cost[n] += exp(pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));
            //samples_cost[n] += (pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));
          }

          ////////////////////////////////////////////////
          // Distance from WP cost
/*          dist_wp = (internal_state.pos[0]- wp->pos_N) * (internal_state.pos[0]- wp->pos_N) + (internal_state.pos[1]- wp->pos_E ) * (internal_state.pos[1]- wp->pos_E );
          samples_cost[n] += pi->TARGET_PENALTY* pi->dh * dist_wp;

          // Heading cost
          if(dist_wp > 0.5f){
            float cross_product_3 = fabs(applied_vel_n * wp->pos_E - applied_vel_e* wp->pos_N);
            samples_cost[n] += pi->HEADING_PENALTY * pi->dh * cross_product_3;
          }*/
          ///////////////////////////////////////////////


          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[a].N) * (internal_state.pos[0]- internal_state.pos_rel[a].N) + (internal_state.pos[1]- internal_state.pos_rel[a].E) * (internal_state.pos[1]- internal_state.pos_rel[a].E);
/*            if(a != pi->following_id){
              samples_cost[n] -= 0.2 * dist_unit;
            }*/
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{

              samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));

            }
          }

          break;}

        case 2:


        {
          //printf("[task] Circling task selected\n");

          // Distance from center cost
          dist_wp = (internal_state.pos[0]- wp->pos_N) * (internal_state.pos[0]- wp->pos_N) + (internal_state.pos[1]- wp->pos_E ) * (internal_state.pos[1]- wp->pos_E );
          samples_cost[n] += exp((dist_wp - pi->RADIUS));

          // Minimum velocity cost
          float velocity_vec = applied_vel_n * applied_vel_n + applied_vel_e * applied_vel_e;
          if(velocity_vec > pi->MIN_SPEED){}
          else{
            samples_cost[n] += pi->MIN_SPEED_PENALTY*(pi->MIN_SPEED - velocity_vec)*(pi->MIN_SPEED - velocity_vec);}


          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[a].N) * (internal_state.pos[0]- internal_state.pos_rel[a].N) + (internal_state.pos[1]- internal_state.pos_rel[a].E) * (internal_state.pos[1]- internal_state.pos_rel[a].E);
            samples_cost[n] -= dist_unit*pi->dh;
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{
              samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));

            }
          }

          break;}

        case 3:
        {

          dist_wp = (internal_state.pos[0]- wp->pos_N) * (internal_state.pos[0]- wp->pos_N) + (internal_state.pos[1]- wp->pos_E ) * (internal_state.pos[1]- wp->pos_E );
          samples_cost[n] += exp((dist_wp - pi->RADIUS));

          //printf("[task] Cat task selected\n");
          // Minimum velocity cost
          float velocity_vec = applied_vel_n * applied_vel_n + applied_vel_e * applied_vel_e;
          if(velocity_vec > pi->MIN_SPEED){}
          else{
            samples_cost[n] += pi->MIN_SPEED_PENALTY*(pi->MIN_SPEED - velocity_vec)*(pi->MIN_SPEED - velocity_vec);}

          // Follow mouse's position !!! Mouse has the 0 position !!!
          float dist_mouse = (internal_state.pos[0]- internal_state.pos_rel[pi->following_index].N) * (internal_state.pos[0]- internal_state.pos_rel[pi->following_index].N) + (internal_state.pos[1]- internal_state.pos_rel[pi->following_index].E) * (internal_state.pos[1]- internal_state.pos_rel[pi->following_index].E);
          samples_cost[n] += pi->TARGET_PENALTY *pi->dh * dist_mouse;

          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[a].N) * (internal_state.pos[0]- internal_state.pos_rel[a].N) + (internal_state.pos[1]- internal_state.pos_rel[a].E) * (internal_state.pos[1]- internal_state.pos_rel[a].E);
            samples_cost[n] -= dist_unit*pi->dh;
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{
              samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));
            }
          }

          break;
        }

        case 4:
        {

          dist_wp = (internal_state.pos[0]- wp->pos_N) * (internal_state.pos[0]- wp->pos_N) + (internal_state.pos[1]- wp->pos_E ) * (internal_state.pos[1]- wp->pos_E );
          samples_cost[n] += exp((dist_wp - pi->RADIUS));

          //printf("[task] Mouse task selected\n");

          // Minimum velocity cost
          /*float velocity_vec = applied_vel_n * applied_vel_n + applied_vel_e * applied_vel_e;
          if(velocity_vec > pi->MIN_SPEED){}
          else{
            samples_cost[n] += pi->MIN_SPEED_PENALTY*(pi->MIN_SPEED - velocity_vec)*(pi->MIN_SPEED - velocity_vec);}
*/
          // Escape cats
          for(int a=0; a < pi->rel_units; a++){
            float dist_cat = (internal_state.pos[0]- internal_state.pos_rel[a].N) * (internal_state.pos[0]- internal_state.pos_rel[a].N) + (internal_state.pos[1]- internal_state.pos_rel[a].E) * (internal_state.pos[1]- internal_state.pos_rel[a].E);
            samples_cost[n] += (pi->TARGET_PENALTY*pi->dh)/dist_cat;
          }


          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[a].N) * (internal_state.pos[0]- internal_state.pos_rel[a].N) + (internal_state.pos[1]- internal_state.pos_rel[a].E) * (internal_state.pos[1]- internal_state.pos_rel[a].E);
            samples_cost[n] -= dist_unit*pi->dh;
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{
              samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));
            }
          }
          break;
        }

      }
    }

    if(isnan(samples_cost[n])){
      samples_cost[n] = 1000000;
    }

    cost_sum += samples_cost[n];
    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}
    //printf("Samples cost %f\n",samples_cost[n]);
    //printf("Samples cost %f\n",samples_cost[n]);
  }

  //float mean_cost = cost_sum/pi->N;
  //printf("Min cost %f\n", min_cost);
  //Compute weight of all samples N
  float w[pi->N];
  float w_sum = 0;
  //float sample_var = 0;
  for (int n=0; n < pi->N; ){
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    //sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    //sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    //sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    //sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
    w[n] = exp(-(samples_cost[n] - min_cost)*inv_lambda);
    //sample_var += (samples_cost[n] - mean_cost) * (samples_cost[n] - mean_cost);
    w_sum += w[n]; n++;
  }

  //printf("[variance] %f\n", sample_var);

  float internal_controls[pi->iH][pi->dimU];
  for(int h = 0; h < pi->iH; h++){
    internal_controls[h][0] = 0;
    internal_controls[h][1] = 0;
  }

  float ESS_sum =0;
  float ESS = 0;
  for (int n=0; n < pi->N; n++ ){
    w[n] = w[n]/w_sum;
    ESS_sum += (w[n]*w[n]);
    for(int h = 0; h < pi->iH; h++){
      internal_controls[h][0] += w[n] * noise[n][h][0];
      internal_controls[h][1] += w[n] * noise[n][h][1];
    }
  }

  ESS = 1/ESS_sum;
  result->variance = ESS;

  for(int h = 0; h < pi->iH; h++){
    internal_controls[h][0] /= pi->dh;
    internal_controls[h][1] /= pi->dh;
  }

  // Compute optimal controls
  float ned_vel_n,  ned_vel_e;
  //printf("Uroll %f, %f, Internal controls %f, %f\n ",u_roll[0][0],u_roll[0][1],internal_controls[0][0],internal_controls[0][1]);
  if((initial_state.vel[0] - st->vel[0]) == 0.0){

    ned_vel_n =  st->vel[0] + (u_roll[0][0] + internal_controls[0][0])*(dt);
    ned_vel_e =  st->vel[1] + (u_roll[0][1] + internal_controls[0][1])*(dt);
  }
  else{
    //float dv_n = ((initial_state.vel[0] - st->vel[0])/dt + (u_roll[0][0] + internal_controls[0][0]));
    //float dv_e = ((initial_state.vel[1] - st->vel[1])/dt + (u_roll[0][1] + internal_controls[0][1]));
    //Bound(dv_n,-pi->MAX_ACC, pi->MAX_ACC);
    //Bound(dv_e,-pi->MAX_ACC, pi->MAX_ACC);
    ned_vel_n =  st->vel[0] + ((initial_state.vel[0] - st->vel[0])/dt + (u_roll[0][0] + internal_controls[0][0])) * dt;
    ned_vel_e =  st->vel[1] + ((initial_state.vel[1] - st->vel[1])/dt + (u_roll[0][1] + internal_controls[0][1])) * dt;
  }

  //printf(" state vel %f, u control %f, internal_control %f, result %f\n", st->vel[0], u_roll[0][0] , internal_controls[0][0],(u_roll[0][0] + internal_controls[0][0])*dt );

/*  float dv_n = (st->vel[0] - ned_vel_n);
  float dv_e = (st->vel[1] - ned_vel_e);

  Bound(dv_n,-0.4, 0.4);
  Bound(dv_e,-0.4, 0.4);*/

  result->vel.x = ned_vel_n;
  result->vel.y = ned_vel_e;
  //printf("result %f, %f\n",ned_vel_n,ned_vel_e);
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
void rotate(int angle, float ** Rmatrix){
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
void compute_control_probes(int angle, uint8_t num_probes, uint8_t probe, float * u_init, float ** UProbes){

  float _Rmatrix[2][2];
  MAKE_MATRIX_PTR(Rmatrix, _Rmatrix, 2);
  float _u[1][2] = { {*u_init, *(u_init+1)} };
  MAKE_MATRIX_PTR(u, _u, 1);
  float _CC[1][2];
  MAKE_MATRIX_PTR(CC, _CC, 1);
  float _C[1][2];
  MAKE_MATRIX_PTR(C,_C, 1);

  int half_probes = (num_probes-1)/2;
  if(probe < half_probes){
    rotate((probe+1) * angle, Rmatrix);
    MAT_MUL(1,2,2, CC, u, Rmatrix);
    UProbes[0][0] = CC[0][0];
    UProbes[0][1] = CC[0][1];
  }
  else if(probe >= half_probes){

    rotate(-(probe-half_probes +1) * angle, Rmatrix);
    MAT_MUL(1,2,2, C, u, Rmatrix );
    UProbes[0][0] = C[0][0];
    UProbes[0][1] = C[0][1];
  }

  if(probe == num_probes-1){
    UProbes[0][0] = _u[0][0];
    UProbes[0][1] = _u[0][1];
  }
}




/**
 * Compute velocity probes by rotating a given velocity vector
 * @param[in]  angle [degrees]
 * @param[in]  num_probes
 * @param[in]  * v_init
 * @param[out]  **VProbes
 */
void compute_probes(int angle, uint8_t num_probes, float * v_init, float ** VProbes){

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
void select_probe(uint8_t sampling_method ,uint8_t num_probes, int angle, float ** u_roll, struct path_integral_t *pi, struct pi_state_t *st, struct pi_wp_t *wp , float * best_probe_vel){

  float samples_cost_probes[num_probes];
  struct pi_state_t internal_state_probes;
  struct pi_state_t initial_state;
  float min_cost = 1000000;
  int8_t min_cost_index = num_probes -1;
  float half_dh = pi->dh*0.5;
  float u_horizon[pi->iH][pi->dimU];

  //printf("Sampling method %d\n", sampling_method);
  //TODO: Send this to the function instead of doing it again
  for (int i = 0; i < pi->dimU; i++){
    initial_state.pos[i] = st->pos[i];
    initial_state.vel[i] = st->vel[i];
  }
  //Propagate follower rel_units
  for(int a=0; a < pi->rel_units; a++) {
    initial_state.pos_rel[a].N = st->pos_rel[a].N;
    initial_state.pos_rel[a].E = st->pos_rel[a].E;
    initial_state.vel_rel[a].N = st->vel_rel[a].N;
    initial_state.vel_rel[a].E = st->vel_rel[a].E;
  }

  float _VProbes[num_probes][pi->dimU];
  MAKE_MATRIX_PTR(VProbes, _VProbes, num_probes);
  compute_probes(angle, num_probes, &initial_state.vel[0] , VProbes);


  /*float mean_N = 0;
  float mean_E = 0;
  for(int h = pi->iH/2; h < pi->iH; h++){
    mean_N += u_roll[h][0];
    mean_E += u_roll[h][1];
  }
  mean_N = mean_N/(pi->iH/2);
  mean_E = mean_E/(pi->iH/2);*/

  for(uint8_t p = 0; p < num_probes; p++){
    samples_cost_probes[p] = 0;

    for (uint8_t i = 0; i < pi->dimU; i++){
      internal_state_probes.pos[i] = initial_state.pos[i];
      internal_state_probes.vel[i] = _VProbes[p][i];
    }

    for(uint8_t a = 0; a < pi->rel_units; a++) {
      internal_state_probes.pos_rel[a].N = initial_state.pos_rel[a].N;
      internal_state_probes.pos_rel[a].E = initial_state.pos_rel[a].E;
      internal_state_probes.vel_rel[a].N = initial_state.vel_rel[a].N;
      internal_state_probes.vel_rel[a].E = initial_state.vel_rel[a].E;
    }

    float applied_vel_n = _VProbes[p][0];
    float applied_vel_e = _VProbes[p][1];

    for(int h = 0; h < pi->iH; h++){

      float _UProbes[1][pi->dimU];
      MAKE_MATRIX_PTR(UProbes, _UProbes, num_probes);
      compute_control_probes(angle, num_probes, p , &u_roll[h][0],UProbes);
/*      if(h==0){
        printf("Probe %d, control %f, %f\n", p, UProbes[0][0], UProbes[0][1]);
      }*/

      //if(p == 0 || p == 6 || p == 12 || h < pi->iH/2){ // 0 3 6 // 0 6 12
        //printf("Normal probe %d, h %d\n", p, h);
        //internal_state_probes.vel[0] += UProbes[0][0]*pi->dh;
        //internal_state_probes.vel[1] += UProbes[0][1]*pi->dh;
        internal_state_probes.vel[0] = applied_vel_n + UProbes[0][0]*pi->dh;
        internal_state_probes.vel[1] = applied_vel_e + UProbes[0][1]*pi->dh;
      //}
/*      else{
        if(h >= pi->iH/2){
          //printf("Mean probe %d, h %d, mean %f\n", p, h, mean_N);
          UProbes[0][0] = mean_N;
          UProbes[0][1] = mean_E;
          internal_state_probes.vel[0] += UProbes[0][0]*pi->dh;
          internal_state_probes.vel[1] += UProbes[0][1]*pi->dh;
        }
      }*/

      // Check max velocity
      Bound(internal_state_probes.vel[0],-pi->MAX_SPEED, pi->MAX_SPEED);
      Bound(internal_state_probes.vel[1],-pi->MAX_SPEED, pi->MAX_SPEED);

      // Take into account the delay
      float rise_time = 4;  //4
      float ratio = pi->dh/(rise_time+pi->dh);

      applied_vel_n = (1-ratio)*applied_vel_n + ratio*internal_state_probes.vel[0];
      applied_vel_e = (1-ratio)*applied_vel_e + ratio*internal_state_probes.vel[1];

      internal_state_probes.pos[0] += applied_vel_n * pi->dh;
      internal_state_probes.pos[1] += applied_vel_e * pi->dh;

      // Propagate position
      //internal_state_probes.pos[0] += internal_state_probes.vel[0] * pi->dh;
      //internal_state_probes.pos[1] += internal_state_probes.vel[1] * pi->dh;

      //Propagate follower rel_units
      for(int a=0; a < pi->rel_units; a++){
        internal_state_probes.pos_rel[a].N += internal_state_probes.vel_rel[a].N * pi->dh;
        internal_state_probes.pos_rel[a].E += internal_state_probes.vel_rel[a].E * pi->dh;
      }

      // Control Cost
      samples_cost_probes[p] += pi->R * fabs(u_horizon[h][0]*u_horizon[h][0]*half_dh  + u_horizon[h][1]*u_horizon[h][1]*half_dh);

      // Out of bounds cost
      if(internal_state_probes.pos[0] >  pi->CZ_LIMIT){
        float outside = internal_state_probes.pos[0] -  pi->CZ_LIMIT;
        samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }
      if(internal_state_probes.pos[0] < -pi->CZ_LIMIT){
        float outside = fabs(internal_state_probes.pos[0] +  pi->CZ_LIMIT);
        samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }
      if(internal_state_probes.pos[1] >  pi->CZ_LIMIT){
        float outside = internal_state_probes.pos[1] -  pi->CZ_LIMIT;
        samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }
      if(internal_state_probes.pos[1] < -pi->CZ_LIMIT){
        float outside = fabs(internal_state_probes.pos[1] +  pi->CZ_LIMIT);
        samples_cost_probes[p] += exp( pi->OUTSIDECZ_PENALTY* outside);
      }

/*      float acc =  UProbes[0][0]*UProbes[0][0] + UProbes[0][1]*UProbes[0][1];
      if(acc > (pi->MAX_ACC * pi->MAX_ACC)){
        samples_cost_probes[p] += (acc - (pi->MAX_ACC * pi->MAX_ACC)) * (acc - (pi->MAX_ACC * pi->MAX_ACC)) ;
      }*/

      float dist_wp = 0.0;
      float dist_unit = 0.0;
      switch(pi->TASK){
        case 0:
        {
          //printf("[task] Leader task selected\n");
          // Target distance cost
          dist_wp = (internal_state_probes.pos[0]- wp->pos_N) * (internal_state_probes.pos[0]- wp->pos_N) + (internal_state_probes.pos[1]- wp->pos_E ) * (internal_state_probes.pos[1]- wp->pos_E );
          samples_cost_probes[p] += pi->TARGET_PENALTY* pi->dh * dist_wp;


          // Heading cost
          if(dist_wp > 0.5f){
            //float cross_product_3 = fabs(internal_state_probes.vel[0] * wp->pos_E - internal_state_probes.vel[1]* wp->pos_N);
            float cross_product_3 = fabs(applied_vel_n * wp->pos_E - applied_vel_e * wp->pos_N);
            samples_cost_probes[p] += pi->HEADING_PENALTY * pi->dh * cross_product_3;
          }

          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state_probes.pos[0]-  internal_state_probes.pos_rel[a].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[a].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[a].E) * (internal_state_probes.pos[1] - internal_state_probes.pos_rel[a].E);
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{ samples_cost_probes[p] += exp(5 * (pi->COLLISION_DISTANCE - dist_unit));} //pi->COLLISION_PENALTY
            //samples_cost[n] += (1000*(pi->COLLISION_DISTANCE - dist_leader));
          }
          break;}

        case 1:

        {//printf("[task] Follower task selected\n");

          // Leader cohesion cost !!!! Assumed leader pos 0 !!!
          float dist_leader = (internal_state_probes.pos[0]- internal_state_probes.pos_rel[pi->following_index].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[pi->following_index].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[pi->following_index].E) * (internal_state_probes.pos[1]- internal_state_probes.pos_rel[pi->following_index].E);
          if(dist_leader < pi->COHESION_DISTANCE ){}
          else{
            samples_cost_probes[p] += exp(pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));
            //samples_cost_probes[p] += (pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));
          }
          ////////////////////////////////////////////////
/*          // Distance from WP cost
          dist_wp = (internal_state_probes.pos[0]- wp->pos_N) * (internal_state_probes.pos[0]- wp->pos_N) + (internal_state_probes.pos[1]- wp->pos_E ) * (internal_state_probes.pos[1]- wp->pos_E );
          samples_cost_probes[p] += pi->TARGET_PENALTY* pi->dh * dist_wp;

          // Heading cost
          if(dist_wp > 0.5f){
            float cross_product_3 = fabs(internal_state_probes.vel[0] * wp->pos_E - internal_state_probes.vel[1]* wp->pos_N);
            samples_cost_probes[p] += pi->HEADING_PENALTY * pi->dh * cross_product_3;
          }*/
          ///////////////////////////////////////////////

          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state_probes.pos[0]-  internal_state_probes.pos_rel[a].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[a].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[a].E) * (internal_state_probes.pos[1] - internal_state_probes.pos_rel[a].E);
/*            if(a != pi->following_id){
              samples_cost_probes[p] -= 0.2 * dist_unit;
            }*/

            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{

              samples_cost_probes[p] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));

              //samples_cost[n] += (1000*(pi->COLLISION_DISTANCE - dist_leader));
            }
          }
          break;}

        case 2:
        {
          //printf("[task] Circling task selected\n");
          // Target distance cost
          dist_wp = (internal_state_probes.pos[0]- wp->pos_N) * (internal_state_probes.pos[0]- wp->pos_N) + (internal_state_probes.pos[1]- wp->pos_E ) * (internal_state_probes.pos[1]- wp->pos_E );
          samples_cost_probes[p] += exp((dist_wp - pi->RADIUS));

          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state_probes.pos[0]-  internal_state_probes.pos_rel[a].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[a].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[a].E) * (internal_state_probes.pos[1] - internal_state_probes.pos_rel[a].E);
            samples_cost_probes[p] -= dist_unit*pi->dh;
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{
              samples_cost_probes[p] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));
              //samples_cost[n] += (1000*(pi->COLLISION_DISTANCE - dist_leader));
            }
          }

          // Min velocity cost
          //float velocity_vec = internal_state_probes.vel[0] * internal_state_probes.vel[0] + internal_state_probes.vel[1] * internal_state_probes.vel[1];
          float velocity_vec = applied_vel_n * applied_vel_n + applied_vel_e * applied_vel_e;
          if(velocity_vec >pi->MIN_SPEED){}
          else{
            samples_cost_probes[p] += pi->MIN_SPEED_PENALTY*(pi->MIN_SPEED - velocity_vec)*(pi->MIN_SPEED - velocity_vec);
          }
          break;}
        case 3:
        {
          //printf("[task] Cat task selected\n");

          dist_wp = (internal_state_probes.pos[0]) * (internal_state_probes.pos[0]) + (internal_state_probes.pos[1]) * (internal_state_probes.pos[1]);
          samples_cost_probes[p] += exp((dist_wp - pi->RADIUS));

          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state_probes.pos[0]-  internal_state_probes.pos_rel[a].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[a].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[a].E) * (internal_state_probes.pos[1] - internal_state_probes.pos_rel[a].E);
            samples_cost_probes[p] -= dist_unit*pi->dh;
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{
              samples_cost_probes[p] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));
              //samples_cost[n] += (1000*(pi->COLLISION_DISTANCE - dist_leader));
            }
          }

          // Min velocity cost
           float velocity_vec = applied_vel_n * applied_vel_n + applied_vel_e * applied_vel_e;
          if(velocity_vec >pi->MIN_SPEED){}
          else{
            samples_cost_probes[p] += pi->MIN_SPEED_PENALTY*(pi->MIN_SPEED - velocity_vec)*(pi->MIN_SPEED - velocity_vec);
          }


          // Follow mouse's position !!! Mouse has the 0 position !!!
          float dist_mouse = (internal_state_probes.pos[0]- internal_state_probes.pos_rel[pi->following_index].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[pi->following_index].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[pi->following_index].E) * (internal_state_probes.pos[1]- internal_state_probes.pos_rel[pi->following_index].E);
          samples_cost_probes[p] += pi->TARGET_PENALTY *pi->dh * dist_mouse;

          break;}


        case 4:
        {
          //printf("[task] Mouse task selected\n");

          dist_wp = (internal_state_probes.pos[0]) * (internal_state_probes.pos[0]) + (internal_state_probes.pos[1]) * (internal_state_probes.pos[1]);
          samples_cost_probes[p] += exp((dist_wp - pi->RADIUS));

          // Escape cats
          for(int a=0; a < pi->rel_units; a++){
            float dist_cat = (internal_state_probes.pos[0]- internal_state_probes.pos_rel[a].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[a].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[a].E) * (internal_state_probes.pos[1]- internal_state_probes.pos_rel[a].E);
            samples_cost_probes[p] += (pi->TARGET_PENALTY*pi->dh)/dist_cat;
          }


          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state_probes.pos[0]-  internal_state_probes.pos_rel[a].N) * (internal_state_probes.pos[0]- internal_state_probes.pos_rel[a].N) + (internal_state_probes.pos[1]- internal_state_probes.pos_rel[a].E) * (internal_state_probes.pos[1] - internal_state_probes.pos_rel[a].E);
            samples_cost_probes[p] -= dist_unit*pi->dh;
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{
              samples_cost_probes[p] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));
              //samples_cost[n] += (1000*(pi->COLLISION_DISTANCE - dist_leader));
            }
          }
          break;
        }
      }
    }

    if(samples_cost_probes[p] < min_cost){
      min_cost = samples_cost_probes[p];
      min_cost_index = p;
    }
  }
  //printf("Min cost index %d\n", min_cost_index);
  if(min_cost_index != (num_probes -1)){
    for(int h = 0; h < pi->iH; h++){
      float _UProbes_new[1][pi->dimU];
      MAKE_MATRIX_PTR(UProbes_new, _UProbes_new, num_probes);

      compute_control_probes(angle, num_probes, min_cost_index , &u_roll[h][0],UProbes_new);
      //if(min_cost_index == 0 || min_cost_index == 6 || min_cost_index == 12 || h < pi->iH/2){  // 0  3 6 // 0 6 12
        u_roll[h][0] = UProbes_new[0][0];
        u_roll[h][1] = UProbes_new[0][1];
      //}
/*      else {
        if(h >= pi->iH/2){
        UProbes_new[0][0] = mean_N;
        UProbes_new[0][1] = mean_E;
        u_roll[h][0] = UProbes_new[0][0];
        u_roll[h][1] = UProbes_new[0][1];
        }
      }*/
      //printf("H %d, UProbes control %f, %f\n", h,  UProbes_new[0][0], UProbes_new[0][1]);

    }
  }
  pi->BEST_PROBE = min_cost_index;
  best_probe_vel[0] = _VProbes[min_cost_index][0];
  best_probe_vel[1] = _VProbes[min_cost_index][1];

}

void init_controls(struct path_integral_t *pi){

  for(int h=0; h < pi->iH; h++) {
    for(int u=0; u < pi->dimU; u++) {
      pi->u_exp[h][u] = 0;
    }
  }
}


/*void copy_state(uint8_t rel_units, uint8_t dimU, struct pi_state_t * dest, struct pi_state_t * origin ){

  for (uint8_t i = 0; i< dimU; i++){
    dest->pos[i] = origin->pos[i];
    dest->vel[i] = origin->vel[i];
  }
  for(uint8_t u = 0; u< rel_units; u++){
    dest->pos_rel[u].N = origin->pos_rel[u].N;
    dest->pos_rel[u].E = origin->pos_rel[u].E;
    dest->vel_rel[u].N = origin->vel_rel[u].N;
    dest->vel_rel[u].E = origin->vel_rel[u].E;
  }

}*/

//TODO: Make the computation of the cost a separate function. For the probes the nr of samples is different
// So a new pi struct should be initialized. This is not ideal
/*void evaluate_samples(struct path_integral_t *pi, float ** u_roll){

  float noise[pi->N][pi->iH][pi->dimU];
  float stdv = sqrt(pi->var);
  float samples_cost[pi->N];
  struct pi_state_t internal_state, initial_state;
  float min_cost = 1000000;

}*/

