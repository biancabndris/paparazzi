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

struct path_integral_t{
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


   float wps[2];
   float u_exp[5][2];
};


extern void pi_calc_init(struct path_integral_t *pi);
extern bool pi_calc_timestep(struct path_integral_t *pi, struct pi_state_t *st, struct pi_result_t *temp_result);


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

#endif
