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
 * @file "modules/path_integral_control/path_integral_control.c"
 * @author Bianca Bendris
 * Stochastic Optimal controller. Outputs x and y optimal velocities.
 */

#include "modules/path_integral_control/path_integral_control.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "subsystems/abi.h"


struct path_integral_t pi;
struct pi_result_t pi_result;
struct pi_state_t st;
static bool pi_got_result;
static pthread_mutex_t pi_mutex;



/**
 * Initialize the path integral module
 */
void pi_init(void)
{

  pi_got_result = false;
  pi_calc_init(&pi);

}

/**
 * Calculate the optimal controls
 */
void pi_calc(void){

  clock_t start, end;
  float cpu_time_used;
  struct pi_result_t temp_result;

  set_state(&st);

  start = clock();
  bool success = pi_calc_timestep(&pi, &st, &temp_result);
  end = clock();

  cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

  printf("------ ELAPSED TIME: %f\n----------",cpu_time_used);
  printf("Controls 0: %f , Controls 1: %f\n", in.oc_x, in.oc_y);

  // Copy the result if finished
  pthread_mutex_lock(&pi_mutex);
  pi_result = temp_result;
  pi_got_result = success;

  pthread_mutex_unlock(&pi_mutex);

}

void pi_run(void){
  pthread_mutex_lock(&pi_mutex);
  // Update the stabilization loops on the current calculation
  if (pi_got_result){
    uint32_t now_ts = get_sys_time_usec();
    pi_got_result = false;
  }
  pthread_mutex_unlock(&pi_mutex);
}


