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
 * @file "modules/path_integral_control/pi_inter_thread_data.h"
 * @author Bianca Bendris
 * Data structures used to for inter-thread communication via Unix Domain sockets.
 */


#ifndef PI_INTER_THREAD_DATA_H
#define PI_INTER_THREAD_DATA_H

#include "math/pprz_algebra_float.h"

// The result of the optimal control calculation given as x and y body velocities
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


#endif
