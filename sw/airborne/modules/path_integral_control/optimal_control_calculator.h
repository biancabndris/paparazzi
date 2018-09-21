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

#define TRAJ_THR 0.8

#include "pi_inter_thread_data.h"
#include "state.h"
#include "navigation.h"
#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

struct path_integral_t{
   float freq;
   bool leader;
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
   float PARALLEL_THR;

   uint8_t units;
   uint8_t dimU;

   gsl_rng *seed;

   //float wps[2];
   float u_exp[5][2];
};

struct traj_t{
  struct pi_wp_t wps[4];
};


extern void pi_calc_init(struct path_integral_t *pi);
bool pi_calc_timestep(struct path_integral_t *pi, struct pi_state_t *st, struct pi_wp_t *wp, struct pi_result_t *result);


static inline void set_state(struct pi_state_t *st){

  st->pos[0]     = stateGetPositionNed_f()->x;
  st->pos[1]     = stateGetPositionNed_f()->y;
  st->vel[0]     = stateGetSpeedNed_f()->x;
  st->vel[1]     = stateGetSpeedNed_f()->y;
  st->psi        = stateGetNedToBodyEulers_f()->psi;

  st->pos_rel[0] = 4;
  st->pos_rel[1] = 4;
  st->pos_rel[2] = 2;
  st->pos_rel[3] = 2;

  st->vel_rel[0] = 0;
  st->vel_rel[1] = 0.2;
  st->vel_rel[2] = 0.2;
  st->vel_rel[3] = 0;
}

static inline void simulate_virtual_leader(struct pi_state_t *st){

  //Simulate a leader flying in a rectangle [-0.5,0.5] N and [-1,1] E
  //Virtual leader start position is 0E -0.5N

  //Propagate
  st->pos_rel[0] += st->vel_rel[0]*0.05;
  st->pos_rel[1] += st->vel_rel[1]*0.05;
  //printf("pos N %f, pos E %f\n",st->pos_rel[0], st->pos_rel[1]);

  float dist1 = (-0.5 - st->pos_rel[0])*(-0.5 - st->pos_rel[0]) + (1.2 - st->pos_rel[1])*(1.2 - st->pos_rel[1]);
  float dist2 = (0.5 - st->pos_rel[0])*(0.5 - st->pos_rel[0]) + (1.2 - st->pos_rel[1])*(1.2 - st->pos_rel[1]);
  float dist3 = (0.5 - st->pos_rel[0])*(0.5 - st->pos_rel[0]) + (-1.2 - st->pos_rel[1])*(-1.2 - st->pos_rel[1]);
  float dist4 = (-0.5 - st->pos_rel[0])*(-0.5 - st->pos_rel[0]) + (-1.2 - st->pos_rel[1])*(-1.2 - st->pos_rel[1]);
  //printf("dist1 %f, dist2  %f, dist3 %f, dist4  %f \n",dist1, dist2, dist3, dist4);
  if(dist1 <= 0.0325f){
    printf("yes");
    st->vel_rel[0]= 0.2;//v2[0];
    st->vel_rel[1] = 0;//v2[1];
  }
  else if(dist2 <= 0.0325f){
    st->vel_rel[0] = 0;
    st->vel_rel[1] = -0.2;
  }
  else if(dist3 <= 0.0325f){
    st->vel_rel[0] = -0.2;
    st->vel_rel[1] = 0;
  }
  else if(dist4 <= 0.0325f){
    st->vel_rel[0] = 0;
    st->vel_rel[1] = 0.2;
  }
  //printf("vel N %f, vel E %f\n",st->vel_rel[0], st->vel_rel[1]);
}

static inline void init_virtual_leader(struct pi_state_t *st){

  st->pos_rel[0] = -0.5;
  st->pos_rel[1] = 0;
  st->vel_rel[0] = 0;
  st->vel_rel[1] = 0.2;

}

static inline void check_wp(struct pi_wp_t *wp, struct traj_t *trajectory){

  struct EnuCoor_i current_wp = {wp->pos_E/0.0039063, wp->pos_N/0.0039063, 1/0.0039063};
  float dist = get_dist2_to_point(&current_wp);
  //printf("current wp N %f, E %f, distance %f\n", wp->pos_N, wp->pos_E, dist);
  if(dist < TRAJ_THR*TRAJ_THR){
    if(wp->wp_index < 3){
      //printf("current wp N %f, E %f, distance %f\n", wp->pos_N, wp->pos_E, dist);
      //printf("WP CHANGED");
      int index = wp->wp_index + 1;
      wp->pos_N = trajectory->wps[index].pos_N;
      wp->pos_E = trajectory->wps[index].pos_E;
      wp->wp_index = index;
    }
    else{
      int index2 = 0;
      //printf("current wp N %f, E %f, distance %f\n", wp->pos_N, wp->pos_E, dist);
      //printf("WP RESET");
      wp->pos_N = trajectory->wps[index2].pos_N;
      wp->pos_E = trajectory->wps[index2].pos_E;
      wp->wp_index = index2;
    }
  }

}

static inline void set_trajectory(struct traj_t *trajectory){

  trajectory->wps[0].pos_N = 1;//-2;
  trajectory->wps[0].pos_E = 1.5;//-1;
  trajectory->wps[0].wp_index = 0;

  trajectory->wps[1].pos_N = 1;//2;
  trajectory->wps[1].pos_E = 1.5;//1.5;//-1;
  trajectory->wps[1].wp_index = 1;

  trajectory->wps[2].pos_N = 1;//0.5;//0;
  trajectory->wps[2].pos_E = 1.5;//1.5;//0;
  trajectory->wps[2].wp_index = 2;

  trajectory->wps[3].pos_N = 1;//0.5;//0;
  trajectory->wps[3].pos_E = 1.5;//0;
  trajectory->wps[3].wp_index = 3;
}




#endif
