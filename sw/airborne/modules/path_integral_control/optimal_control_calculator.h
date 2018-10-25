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
 * @file "modules/path_integral_control/optimal_control_calculator.h"
 * @author Bianca Bendris
 * Calculates the x and y optimal velocities.
 */

#ifndef OPTIMAL_CONTROL_CALCULATOR_H
#define OPTIMAL_CONTROL_CALCULATOR_H



#include "pi_inter_thread_data.h"
#include "state.h"
#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "modules/multi/traffic_info.h"

struct path_integral_t{
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

   uint8_t units;
   uint8_t dimU;

   gsl_rng *seed;

   float u_exp[10][2];
};

struct traj_t{
  struct pi_wp_t wps[4];
};

extern void pi_calc_init(struct path_integral_t *pi);
bool pi_calc_timestep(struct path_integral_t *pi, struct pi_state_t *st, struct pi_wp_t *wp, struct pi_result_t *result);
extern void init_controls(struct path_integral_t *pi);

static inline void set_state(struct pi_state_t *st){

  st->pos[0]     = stateGetPositionNed_f()->x;
  st->pos[1]     = stateGetPositionNed_f()->y;
  st->vel[0]     = stateGetSpeedNed_f()->x;
  st->vel[1]     = stateGetSpeedNed_f()->y;
  st->psi        = stateGetNedToBodyEulers_f()->psi;

  uint8_t *ac_ids;
  ac_ids = acInfoGetAcIds();
  uint8_t follower_id = *(ac_ids+2);

  struct EnuCoor_f *ac_pos = acInfoGetPositionEnu_f(follower_id);
  st->pos_rel[0] = ac_pos->y;
  st->pos_rel[1] = ac_pos->x;
  //st->pos_rel[2] = 0;
  //st->pos_rel[3] = 0;

  struct EnuCoor_f *ac_vel = acInfoGetVelocityEnu_f(follower_id);
  st->vel_rel[0] = ac_vel->y;
  st->vel_rel[1] = ac_vel->x;
  //st->vel_rel[2] = 0;
  //st->vel_rel[3] = 0;
}



/*static inline void set_trajectory(struct traj_t *trajectory){

  trajectory->wps[0].pos_N = 0;//-1.5;
  trajectory->wps[0].pos_E = 0;//-1;
  trajectory->wps[0].wp_index = 0;

  trajectory->wps[1].pos_N = 0;//-1.5;
  trajectory->wps[1].pos_E = 0;//1;
  trajectory->wps[1].wp_index = 1;

  trajectory->wps[2].pos_N = 0;//1.5;
  trajectory->wps[2].pos_E = 0;//1;
  trajectory->wps[2].wp_index = 2;

  trajectory->wps[3].pos_N = 0;//1.5;
  trajectory->wps[3].pos_E = 0;//-1;
  trajectory->wps[3].wp_index = 3;
}*/


#endif
