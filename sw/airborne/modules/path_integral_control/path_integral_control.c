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
#include "filters/low_pass_filter.h"

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
Butterworth2LowPass comm_vel[2];
float filtered_result[2];
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
      &pi_result.vel.x, &pi_result.vel.y, &pi_result.variance, &wp.pos_E, &wp.pos_N, &pi.TASK, &pi.SAMPLING_METHOD, &pi.BEST_PROBE);
  pthread_mutex_unlock(&pi_mutex);
}
//&filtered_result[0], &filtered_result[1], &pi_result.vel.x, &pi_result.vel.y
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
#ifndef NUM_WPS
#define NUM_WPS 4
#endif
#ifndef DELAY
#define DELAY 0
#endif

static void *pi_calc_thread(void *arg);
void set_wp(void);
void check_wp(void);
void pi_init_filters(void);
//bool check_wp_followers(void);

int counter, counter_probe;
uint8_t leader_pos_index = 0;
uint8_t f_index = DELAY;
float LeaderPos[1200][2];
float delayed_N = 0.0;
float delayed_E = 0.0;
bool leader_set = false;
bool catched = false;
bool pi_start = false;
uint8_t laps = 0;
/**
 * Initialize the path integral module
 */
void pi_init(void)
{

  pi_calc_init(&pi);
  pi_got_result = false;

  // Update state information
  set_state(&st, pi.rel_units);
  // Find leader index

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
  //pi_init_filters();

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

  if(!leader_set && pi.TASK == 1){
    for(uint8_t i = 0; i< pi.rel_units; i++){
      //printf("ac id %d leader id %d \n", st.pos_rel[i].AC_id, pi.following_id);
      if(st.pos_rel[i].AC_id == pi.following_id){
        pi.following_index = i;
        leader_set = true;
      }
    }

    //printf("leader id %d \n",pi.following_index);
  }

  //printf("check state %d\n", pi_got_result);
  //if(pi.TASK == 0){
   // check_wp();
  //}



  if (pi_got_result){
    /*if(catched){
      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgPATH_INTEGRAL(PATH_INTEGRAL_ID, now_ts,
          0.0,
          0.0);
      pi_got_result = false;
    }else{*/

    //update_butterworth_2_low_pass(&comm_vel[0], pi_result.vel.x);
    //update_butterworth_2_low_pass(&comm_vel[1], pi_result.vel.y);
    //printf("vel %f, filtered %f\n", pi_result.vel.x,comm_vel[0].o[0]);

    //pi_result.vel.x = comm_vel[0].o[0];
    //pi_result.vel.y = comm_vel[1].o[0];
    //Bound(pi_result.vel.x ,-pi.MAX_SPEED, pi.MAX_SPEED);
    //Bound(pi_result.vel.x ,-pi.MAX_SPEED, pi.MAX_SPEED);

    if(!isnan(pi_result.vel.x) && !isnan(pi_result.vel.y) ){
      //filtered_result[0] = pi_result.vel.x;
      //filtered_result[1] = pi_result.vel.y;

      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgPATH_INTEGRAL(PATH_INTEGRAL_ID, now_ts,
          pi_result.vel.x,
          pi_result.vel.y);
      pi_got_result = false;
    }



    //}
  }

    //////////////////////////////////////

    /*if(pi.TASK == 1 && pi_start){

      if(counter == 40){
        if(leader_pos_index < 1200){
          LeaderPos[leader_pos_index][0] = st.pos_rel[pi.following_index].N;
          LeaderPos[leader_pos_index][1] = st.pos_rel[pi.following_index].E;

        }else{
          leader_pos_index = 0;
          LeaderPos[leader_pos_index][0] = st.pos_rel[pi.following_index].N;
          LeaderPos[leader_pos_index][1] = st.pos_rel[pi.following_index].E;

        }

        if(leader_pos_index >= DELAY){
          wp.pos_N = LeaderPos[leader_pos_index][0];
          wp.pos_E = LeaderPos[leader_pos_index][1];
        }else{
          wp.pos_N = st.pos[0];
          wp.pos_E = st.pos[1];
        }
        bool next = check_wp_followers();
        if(next){
          f_index += 1;

        }
        leader_pos_index += 1;
        counter = -1;
      }

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
*/
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
  //pi_start = true;
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
    uint8_t target_wp[NUM_WPS] = {WP_p0,WP_p1,WP_p2,WP_p3};
    for(int i = 0; i < NUM_WPS; i++){
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
    if(wp.wp_index < NUM_WPS-1){
/*      uint8_t wp_index;
      if(laps >= 2){
        wp_index = wp.wp_index - 1;
      }
      else{
        wp_index = wp.wp_index + 1;
      }*/
      uint8_t wp_index = wp.wp_index + 1;
      wp.pos_N = trajectory.wps[wp_index].pos_N;
      wp.pos_E = trajectory.wps[wp_index].pos_E;
      wp.wp_index = wp_index;
    }
    else{
/*      if(laps >= 2 ){
        wp.pos_N = trajectory.wps[NUM_WPS-2].pos_N;
        wp.pos_E = trajectory.wps[NUM_WPS-2].pos_E;
        wp.wp_index = NUM_WPS-2;
      }*/
      wp.pos_N = trajectory.wps[0].pos_N;
      wp.pos_E = trajectory.wps[0].pos_E;
      wp.wp_index = 0;
      //laps += 1;
    }
  }

}

/*bool check_wp_followers(void){
  bool reached = false;
  struct EnuCoor_i current_wp = {wp.pos_E/0.0039063, wp.pos_N/0.0039063, 1/0.0039063};
  float dist = get_dist2_to_point(&current_wp);
  if(dist < TRAJ_THR*TRAJ_THR){
    reached = true;

  }
  return reached;
}*/

void pi_init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * 20);
  float sample_time = 1.0 / 15;
  // Filtering of gyroscope and actuators
  for (int8_t i = 0; i < 2; i++) {
    init_butterworth_2_low_pass(&comm_vel[i], tau, sample_time, 0.0);

  }
}



