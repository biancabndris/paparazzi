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
                               &pi_result.pi_vel.x, &pi_result.pi_vel.y, &st.pos_rel[0], &st.pos_rel[1]); //&st.pos_rel[0], &st.pos_rel[1]
  pthread_mutex_unlock(&pi_mutex);
}
#endif


static void *pi_calc_thread(void *arg);

/**
 * Initialize the path integral module
 */
void pi_init(void)
{

  pi_calc_init(&pi);
  pi_got_result = false;

  // Update state information
  set_state(&st);
  //init_virtual_leader(&st);
  set_trajectory(&trajectory);
  //printf("INIT STATE N %f, E %f\n",st.pos_rel[0],st.pos_rel[1]);
  // Initialize the wp
  wp.pos_N = trajectory.wps[0].pos_N;
  wp.pos_E = trajectory.wps[0].pos_E;
  wp.wp_index = trajectory.wps[0].wp_index;


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PATH_INTEGRAL, pi_telem_send);
#endif

}

/**
 * Starts the streaming of a all cameras
 */
void start_thread(void)
{

  if (path_integral_thread != 0) {
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


/*
 * Stop a video thread for a camera
 */
void stop_thread(void)
{

  if (pthread_cancel(path_integral_thread) != 0) {
    printf("[path_integral] Thread cancel did not work\n");
  }

}

static void *pi_calc_thread(void *arg __attribute__((unused)))
{
  while(true){

    // Copy the state
    pthread_mutex_lock(&pi_mutex);
    struct pi_state_t temp_state;
    struct pi_wp_t temp_wp;
    memcpy(&temp_state, &st, sizeof(struct pi_state_t));
    memcpy(&temp_wp, &wp, sizeof(struct pi_wp_t));
    pthread_mutex_unlock(&pi_mutex);

    // Compute the optimal controls
    struct pi_result_t temp_result;

    struct timespec start, finish;
    float elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);

    bool success = pi_calc_timestep(&pi, &temp_state, &temp_wp, &temp_result);

    // Copy the result if finished
    pthread_mutex_lock(&pi_mutex);
    if(success){
      memcpy(&pi_result, &temp_result, sizeof(struct pi_result_t));
    }
    pi_got_result = success;


    //printf("CONTROLS 0: %f , Controls 1: %f\n", temp_result.pi_vel.x, temp_result.pi_vel.y);

    pthread_mutex_unlock(&pi_mutex);

    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec);
    elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

    //printf("------ ELAPSED TIME: %f\n----------",elapsed);
    //Sleep to avoid having the optimal controls computed at a higher frequency
/*    if(elapsed < 1/pi.freq){
      sleep(1/pi.freq - elapsed);
    }*/
      }
  return 0;
}


/**
 * Calculate the optimal controls
 */
void pi_run(void){

  pthread_mutex_lock(&pi_mutex);

  // Update state information

  uint32_t now = get_sys_time_usec();
  //printf("[BEFORE SENDING ABI] %d x %f, y %f, vel x %f vel y %f Controls x %f, y %f\n", now, st.pos[0], st.pos[1], st.vel[0], st.vel[1], pi_result.pi_vel.x, pi_result.pi_vel.y);
  //simulate_virtual_leader(&st);
  //printf("VIRTUAL N %f, E %f\n",st.pos_rel[0],st.pos_rel[1]);


  // Update the stabilization loops on the current calculation
  if (pi_got_result){
    uint32_t now_ts = get_sys_time_usec();
    AbiSendMsgPATH_INTEGRAL(PATH_INTEGRAL_ID, now_ts,
                            pi_result.pi_vel.x,
                            pi_result.pi_vel.y);
    pi_got_result = false;
    //printf("!!!Event detected: Got result!!!");
    //printf("[SEND] %d x %f, y %f, vel x %f vel y %f Controls x %f, y %f\n", now_ts, st.pos[0], st.pos[1], st.vel[0], st.vel[1], pi_result.pi_vel.x, pi_result.pi_vel.y);
  }
  set_state(&st);

  uint32_t now2 = get_sys_time_usec();
  //printf("[AFTER SENDING ABI] %d x %f, y %f, vel x %f vel y %f Controls x %f, y %f\n", now2, st.pos[0], st.pos[1], st.vel[0], st.vel[1], pi_result.pi_vel.x, pi_result.pi_vel.y);
  check_wp(&wp, &trajectory);
  pthread_mutex_unlock(&pi_mutex);

}
