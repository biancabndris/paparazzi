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


struct path_integral_t pi;
struct pi_result_t pi_result;
struct pi_state_t st;
static bool pi_got_result;
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
                               &pi_result.pi_vel.x, &pi_result.pi_vel.y); // TODO: no noise measurement here...
  pthread_mutex_unlock(&pi_mutex);
}
#endif


static void *pi_calc_thread(void *arg);

/**
 * Initialize the path integral module
 */
void pi_init(void)
{

  pi_got_result = false;
  pi_calc_init(&pi);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PATH_INTEGRAL, pi_telem_send);
#endif

}

/**
 * Starts the streaming of a all cameras
 */
void start_thread(void)
{
  //if (!camera->thread.is_running)
  pthread_t tid;
  if (pthread_create(&tid, NULL, pi_calc_thread, NULL) != 0) {
        printf("[viewPI] Could not create thread for path_integral: Reason: %d.\n", errno);
        return;
      }
  #ifndef __APPLE__
      pthread_setname_np(tid, "path_integral");
  #endif
}


/*
 * Stop a video thread for a camera
 */
void stop_thread(void)
{
  //if (device->thread.is_running) {
    // Stop the streaming thread
    //device->thread.is_running = false;

  // Stop the thread
  //if (pthread_cancel(tid) < 0) {
  //  printf("Could not cancel thread for %s\n", tid);
  //  return false;
  //}

  // Wait for the thread to be finished
  //pthread_join(tid, NULL);
  //tid = (pthread_t) NULL;

}

static void *pi_calc_thread(void *arg)
{
  clock_t start, end;
  float cpu_time_used;
  struct pi_result_t temp_result;

  set_state(&st);

  start = clock();
  bool success = pi_calc_timestep(&pi, &st, &temp_result);
  end = clock();

  cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

  printf("------ ELAPSED TIME: %f\n----------",cpu_time_used);
  printf("Controls 0: %f , Controls 1: %f\n", temp_result.pi_vel.x, temp_result.pi_vel.y);

  // Copy the result if finished
  pthread_mutex_lock(&pi_mutex);
  pi_result = temp_result;
  pi_got_result = success;

  pthread_mutex_unlock(&pi_mutex);

  return 0;
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
  printf("Controls 0: %f , Controls 1: %f\n", temp_result.pi_vel.x, temp_result.pi_vel.y);

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
    AbiSendMsgPATH_INTEGRAL(PATH_INTEGRAL_ID, now_ts,
                            pi_result.pi_vel.x,
                            pi_result.pi_vel.y);
    pi_got_result = false;
    printf("!!!Event detected: Got result!!!");
  }
  pthread_mutex_unlock(&pi_mutex);
}


