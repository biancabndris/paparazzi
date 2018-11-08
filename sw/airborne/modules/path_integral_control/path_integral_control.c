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
#include <errno.h>
#include "state.h"
#include "subsystems/abi.h"
#include "modules/multi/traffic_info.h"

#include "../../subsystems/navigation/waypoints.h"
#include "navigation.h"
#include "generated/flight_plan.h"


struct path_integral_t pi;
struct pi_result_t pi_result;
struct pi_state_t st;
struct pi_wp_t wp;
struct traj_t trajectory;
static bool pi_got_result;
static pthread_t path_integral_thread;
static pthread_mutex_t pi_mutex;


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/**
 * Send path integral control telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void pi_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&pi_mutex);
  pprz_msg_send_PATH_INTEGRAL(trans, dev, AC_ID,
                               &pi_result.vel.x, &pi_result.vel.y,  &pi_result.variance, &wp.pos_E, &wp.pos_N, &pi.TASK, &pi.SAMPLING_METHOD, &pi.BEST_PROBE);
  pthread_mutex_unlock(&pi_mutex);
}

static void relative_ac_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&pi_mutex);
  pprz_msg_send_RELATIVE_AC(trans, dev, AC_ID, &st.pos_rel[0].N, &st.pos_rel[0].E, &st.vel_rel[0].N, &st.vel_rel[0].E, &st.pos_rel[0].N, &st.pos_rel[0].E, &st.vel_rel[0].N, &st.vel_rel[0].E,&st.pos_rel[0].N, &st.pos_rel[0].E, &st.vel_rel[0].N, &st.vel_rel[0].E);
  pthread_mutex_unlock(&pi_mutex);
}

#endif

#ifndef TRAJ_THR
#define TRAJ_THR 0.4
#endif

static void *pi_calc_thread(void *arg);
void set_wp(void);
void check_wp(void);

int counter, counter_probe;
float delayed_N = 0.0;
float delayed_E = 0.0;
/**
 * Initialize the path integral module
 */
void pi_init(void)
{

  pi_calc_init(&pi);
  pi_got_result = false;

  // Update state information
  set_state(&st, pi.rel_units);

  set_wp();
  counter = 0;
  counter_probe = 0;
  //set_trajectory(&trajectory);

  // Initialize the wp
  //static uint8_t traj[1] = {WP_p0};
  //printf("WP %d, %d \n",WP_p0, WP_p0);
  //wp.pos_N = trajectory.wps[0].pos_N;
  //wp.pos_E = trajectory.wps[0].pos_E;
  //wp.wp_index = trajectory.wps[0].wp_index;


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PATH_INTEGRAL, pi_telem_send);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RELATIVE_AC, relative_ac_telem_send);
#endif

}


void start_thread(void)
{

  if (path_integral_thread != 0){
    printf("[path_integral] Path integral thread already started!\n");
    return;
  }

  if (pthread_create(&path_integral_thread, NULL, pi_calc_thread, NULL) != 0) {
        printf("[path_integral] Could not create thread for path_integral: Reason: %d.\n", errno);
        return;
  }

  #ifndef __APPLE__
      pthread_setname_np(path_integral_thread, "path_integral");
  #endif
}


void stop_thread(void)
{

  if (pthread_cancel(path_integral_thread) != 0) {
    printf("[path_integral] Thread cancel did not work\n");
  }

}

static void *pi_calc_thread(void *arg __attribute__((unused)))
{
  while(true){
    struct timespec start, finish, finish2;
    clock_gettime(CLOCK_MONOTONIC, &start);

    //Copy the state
    pthread_mutex_lock(&pi_mutex);
    struct pi_state_t temp_state;
    struct pi_wp_t temp_wp;
    memcpy(&temp_state, &st, sizeof(struct pi_state_t));
    memcpy(&temp_wp, &wp, sizeof(struct pi_wp_t));
    pthread_mutex_unlock(&pi_mutex);

    //Compute the optimal controls
    struct pi_result_t temp_result;

    float elapsed, elapsed2;

    bool success = pi_calc_timestep(&pi, &temp_state, &temp_wp, &temp_result);

    pthread_mutex_lock(&pi_mutex);
    if(success){
      memcpy(&pi_result, &temp_result, sizeof(struct pi_result_t));
    }
    pi_got_result = success;
    pthread_mutex_unlock(&pi_mutex);

    //printf("CONTROLS 0: %f , Controls 1: %f\n", temp_result.vel.x, temp_result.vel.y);

    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec);
    elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    //printf("TIME %f\n", elapsed);

    float max_freq = 1/pi.freq;
    if(elapsed < max_freq){
      int waiting_time = (int)((max_freq -elapsed)*1000000);
      usleep(waiting_time);
      //printf("Max freq %f, waiting time %d\n", max_freq, waiting_time);
    }

    clock_gettime(CLOCK_MONOTONIC, &finish2);
    elapsed2 = (finish2.tv_sec - start.tv_sec);
    elapsed2 += (finish2.tv_nsec - start.tv_nsec) / 1000000000.0;
    printf("TIME2 %f\n", elapsed2);

      }
  return 0;
}


/**
 * Calculate the optimal controls
 */
void pi_run(void){

  pthread_mutex_lock(&pi_mutex);

  set_state(&st, pi.rel_units);
  check_wp();

  if(pi.TASK == 0){
   // check_wp();
  }

  else if(pi.TASK == 3 || pi.TASK == 4){
    /*float THR = 3;//0.3;
    float dist_cats_mouse = 0;
    for(uint8_t u = 0; u < pi.rel_units; u++){
      dist_cats_mouse += (st.pos[0]- st.pos_rel[u].N) * (st.pos[0]- st.pos_rel[u].N) + (st.pos[1]- st.pos_rel[u].E ) * (st.pos[1]- st.pos_rel[u].E);
    }
    if(dist_cats_mouse < THR){
      pi_result.vel.x = 0.0;
      pi_result.vel.y = 0.0;
    }*/
  }

    if (pi_got_result){
      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgPATH_INTEGRAL(PATH_INTEGRAL_ID, now_ts,
          pi_result.vel.x,
          pi_result.vel.y);
      pi_got_result = false;
    }

    //////////////////////////////////////

    if(pi.TASK == 1){
      if(counter == 0){
        delayed_N = st.pos_rel[0].N;
        delayed_E = st.pos_rel[0].E;
      }
      if(counter == 40){
        wp.pos_N = delayed_N;
        wp.pos_E = delayed_E;
        counter = -1;
      }
      counter++;

    }
    ////////////////////////////////////

  pthread_mutex_unlock(&pi_mutex);

}


uint8_t pi_follow_wps(void){

  pi.TASK = 0;
  printf("[task] Task changed to leader following wps");
  set_wp();
  init_controls(&pi);
  return pi.TASK;
}



uint8_t pi_follow_leader(void){

  pi.TASK = 1;
  printf("[task] Task changed to follower");
  init_controls(&pi);
  return pi.TASK;
}


uint8_t pi_circle_wp(void){

  pi.TASK = 2;
  printf("[task] Task changed to circling");
  set_wp();
  init_controls(&pi);
  return pi.TASK;
}


void set_wp(void){

  if(pi.TASK == 0){
    uint8_t target_wp[4] = {WP_p0,WP_p1,WP_p2,WP_p3};
    for(int i = 0; i < 4; i++){
      trajectory.wps[i].pos_N = waypoint_get_x(target_wp[i]);
      trajectory.wps[i].pos_E = waypoint_get_y(target_wp[i]);
      trajectory.wps[i].wp_index = i;
    }
    wp.pos_N = trajectory.wps[0].pos_N;
    wp.pos_E = trajectory.wps[0].pos_E;
    wp.wp_index = trajectory.wps[0].wp_index;

  }
  else if(pi.TASK == 1){
    wp.pos_N = 0.0;
    wp.pos_E = 0.0;
  }
  else if(pi.TASK == 2){
    wp.pos_N = waypoint_get_x(WP_centre);
    wp.pos_E = waypoint_get_y(WP_centre);
  }
}

void check_wp(void){

  struct EnuCoor_i current_wp = {wp.pos_E/0.0039063, wp.pos_N/0.0039063, 1/0.0039063};
  float dist = get_dist2_to_point(&current_wp);
  if(dist < TRAJ_THR*TRAJ_THR){
    if(wp.wp_index < 3){
      int index = wp.wp_index + 1;
      wp.pos_N = trajectory.wps[index].pos_N;
      wp.pos_E = trajectory.wps[index].pos_E;
      wp.wp_index = index;
    }
    else{
      int index2 = 0;
      wp.pos_N = trajectory.wps[index2].pos_N;
      wp.pos_E = trajectory.wps[index2].pos_E;
      wp.wp_index = index2;
    }
  }

}
