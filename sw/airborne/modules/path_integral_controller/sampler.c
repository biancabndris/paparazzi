/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time_v2/piControllerpi.c"
 * @author Bianca Bendris
 *
 */
#include <math.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist->h>
//#include <stdlib.h>

#include "modules/path_integral_controller/sampler.h"
#include  <time.h>

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
#define PI_COHESION_DISTANCE 2.25////6.25
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
#define PI_REL_UNITS 1
#endif
#ifndef PI_DIM_U
#define PI_DIM_U 2
#endif
#ifndef PI_CZ_LIMIT
#define PI_CZ_LIMIT 2.8
#endif
#ifndef PI_OUTSIDECZ_PENALTY
#define PI_OUTSIDECZ_PENALTY 5
#endif
#ifndef PI_RADIUS
#define PI_RADIUS 4
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
#define PI_NUM_PROBES 13 // 7
#endif


void init_controls(struct PIController *pi);
void gauss(float * mean, float * stdv, float * random);
void PIController_init(struct PIController * pi )
{
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

  init_controls(pi);

}


/**
 * Function that computes the optimal controls for the leader unit
 */
void compute_optimal_controls(struct PIController *pi, struct pi_state_t *st, struct pi_wp_t *wp, struct pi_result_t *result){

  float noise[pi->N][pi->iH][pi->dimU];
  float u_roll[pi->iH][pi->dimU];
  float samples_cost[pi->N];
  float stdv = sqrt(pi->var);
  float mean = 0.0;
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
      gauss(&stdv, &mean, &noise[n][h][0]);
      gauss(&stdv, &mean, &noise[n][h][1]);

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
            else{ samples_cost[n] += exp( 1 *(pi->COLLISION_DISTANCE - dist_unit));} // pi->COLLISION_PENALTY
          }

          break;}


        case 1:

        {
          //printf("[task] Follower task selected\n");

          // Cohesion cost
          float dist_leader = (internal_state.pos[0]- internal_state.pos_rel[0].N) * (internal_state.pos[0]- internal_state.pos_rel[0].N) + (internal_state.pos[1]- internal_state.pos_rel[0].E) * (internal_state.pos[1]- internal_state.pos_rel[0].E);
          if(dist_leader < pi->COHESION_DISTANCE ){}
          else{
            //samples_cost[n] += exp(pi->COHESION_PENALTY *(dist_leader - pi->COHESION_DISTANCE));
            samples_cost[n] += (100 *(dist_leader - pi->COHESION_DISTANCE));
          }

          // Collision cost
          for(int a=0; a < pi->rel_units; a++){
            dist_unit = (internal_state.pos[0]-  internal_state.pos_rel[a].N) * (internal_state.pos[0]- internal_state.pos_rel[a].N) + (internal_state.pos[1]- internal_state.pos_rel[a].E) * (internal_state.pos[1]- internal_state.pos_rel[a].E);
            if(dist_unit > pi->COLLISION_DISTANCE ){}
            else{
              samples_cost[n] += exp(pi->COLLISION_PENALTY *(pi->COLLISION_DISTANCE - dist_unit));
            }
          }

          break;}

      }
    }

    if(isnan(samples_cost[n])){
      samples_cost[n] = 1000000;
    }

    cost_sum += samples_cost[n];
    if(samples_cost[n] < min_cost) {min_cost = samples_cost[n];}

  }


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
  if((initial_state.vel[0] - st->vel[0]) == 0.0){

    ned_vel_n =  st->vel[0] + (u_roll[0][0] + internal_controls[0][0])*(dt);
    ned_vel_e =  st->vel[1] + (u_roll[0][1] + internal_controls[0][1])*(dt);
  }
  else{
    ned_vel_n =  st->vel[0] + ((initial_state.vel[0] - st->vel[0])/dt + (u_roll[0][0] + internal_controls[0][0]))*dt;
    ned_vel_e =  st->vel[1] + ((initial_state.vel[1] - st->vel[1])/dt + (u_roll[0][1] + internal_controls[0][1]))*dt;
  }


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

  }

}



void gauss(float * mean, float * stdv, float * random)
{
  float x = (float) rand() / RAND_MAX;
  float y = (float) rand() / RAND_MAX;
  float z = sqrt(-2 * logf(x)) * cosf(2 * M_PI * y);
  *random = z*(*stdv) + *mean;
}



void init_controls(struct PIController *pi){

  for(int h=0; h < pi->iH; h++) {
    for(int u=0; u < pi->dimU; u++) {
      pi->u_exp[h][u] = 0;
    }
  }
}




