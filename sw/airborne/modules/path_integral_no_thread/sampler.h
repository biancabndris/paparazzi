/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time/PIController.h"
 * @author Bianca Bendris
 *
 */


#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdint.h>
#include "state.h"
//#include "modules/multi/traffic_info.h"

struct PIController{
   float freq;
   uint8_t H;
   float dh;
   uint16_t iH;
   float nu;
   float R;
   float var;
   float lambda;
   uint16_t N;
   float COLLISION_DISTANCE;
   float COHESION_DISTANCE;
   float COLLISION_PENALTY;
   float COHESION_PENALTY;
   float TARGET_PENALTY;
   float HEADING_PENALTY;
   float PARALLEL_PENALTY;
   float MAX_SPEED;
   float MIN_SPEED;
   float MIN_SPEED_PENALTY;
   float PARALLEL_THR;
   float OUTSIDECZ_PENALTY;
   float CZ_LIMIT;
   float RADIUS;
   uint8_t TASK;
   uint8_t SAMPLING_METHOD;
   int PROBE_ANGLE;
   uint8_t NUM_PROBES;
   uint8_t BEST_PROBE;

   uint8_t rel_units;
   uint8_t dimU;

   float u_exp[10][2];
};

struct pi_result_t{
  struct FloatVect2 vel;
  float  variance;
};

struct pi_vectNE_t{
    float N;
    float E;
};

struct pi_state_t{
   float pos[2];
   float vel[2];
   float psi;
   struct pi_vectNE_t pos_rel[1];
   struct pi_vectNE_t vel_rel[1];
};

struct pi_wp_t{
   float pos_N;
   float pos_E;
   int wp_index;
};

struct traj_t{
  struct pi_wp_t wps[4];
};


extern void PIController_init(struct PIController * pi2 );
extern void compute_optimal_controls(struct PIController * pi, struct pi_state_t *st, struct pi_wp_t *wp, struct pi_result_t *result);

static inline void set_state(struct pi_state_t *st, uint8_t rel_units){

  st->pos[0]     = stateGetPositionNed_f()->x;
  st->pos[1]     = stateGetPositionNed_f()->y;
  st->vel[0]     = stateGetSpeedNed_f()->x;
  st->vel[1]     = stateGetSpeedNed_f()->y;
  st->psi        = stateGetNedToBodyEulers_f()->psi;


  //uint8_t *ac_ids = acInfoGetAcIds();
  for(uint8_t u=0; u< rel_units; u++){

    //uint8_t follower_id = *(ac_ids+2+u);
    //struct EnuCoor_f *ac_pos = 0;//acInfoGetPositionEnu_f(follower_id);
    st->pos_rel[u].N = 0.0;//ac_pos->y;
    st->pos_rel[u].E = 0.0;//ac_pos->x;

   // struct EnuCoor_f *ac_vel = acInfoGetVelocityEnu_f(follower_id);
    st->vel_rel[u].N = 0.0;//ac_vel->y;
    st->vel_rel[u].E = 0.0;//ac_vel->x;
  }

}


#endif // PI_CONTROLLER_H
