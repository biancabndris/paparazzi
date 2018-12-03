/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time_v2/computation_time.c"
 * @author Bianca Bendris
 * Run a path integral algorithm and measure the running time needed to perform the computations
 */


#include "modules/path_integral_controller/path_integral_controller.h"
#include "modules/path_integral_controller/sampler.h"
#include  <stdio.h>
#include  "mcu_periph/sys_time.h"


#include <math.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist.h>
//#include <stdlib.h>

#include "../../subsystems/navigation/waypoints.h"
#include "navigation.h"
#include "generated/flight_plan.h"



#ifndef TRAJ_THR
#define TRAJ_THR 0.4
#endif
#ifndef NUM_WPS
#define NUM_WPS 4
#endif


struct PIController pi;
struct pi_state_t st;
struct pi_result_t pi_result;
struct pi_wp_t wp;
struct traj_t trajectory;
float start, stop, elapsed_time;
int counter, iterations;
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
/**
 * Send path integral control telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */

static void pi_telem_send(struct transport_tx *trans, struct link_device *dev)
{

  pprz_msg_send_PATH_INTEGRAL(trans, dev, AC_ID,
                               &pi_result.vel.x, &pi_result.vel.y,  &pi_result.variance, &wp.pos_E, &wp.pos_N, &pi.TASK, &pi.SAMPLING_METHOD, &pi.BEST_PROBE);

}


static void pi_timing_send(struct transport_tx *trans, struct link_device *dev)
{

  pprz_msg_send_PI_TIMING(trans, dev, AC_ID, &pi.N, &start, &stop, &elapsed_time);

}
#endif

void set_wp(void);
void check_wp(void);

void pi_init() {

  PIController_init(&pi);
  set_state(&st, pi.rel_units);
  set_wp();
  counter = 0;
  iterations = 1;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PATH_INTEGRAL, pi_telem_send);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PATH_INTEGRAL, pi_timing_send);
#endif


}


void compute_optimal_controls_periodic(){


  if (counter == 50 && iterations < 20){
    iterations += 1;
    pi.N = iterations * 100;
    counter = 0;
  }
  start = get_sys_time_float();

  set_state(&st, pi.rel_units);
  compute_optimal_controls(&pi, &st, &wp, &pi_result);
  check_wp();

  stop = get_sys_time_float();
  elapsed_time = stop - start;
  counter += 1;

}


void set_wp(void){


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


void check_wp(void){

  struct EnuCoor_i current_wp = {wp.pos_E/0.0039063, wp.pos_N/0.0039063, 1/0.0039063};
  float dist = get_dist2_to_point(&current_wp);
  if(dist < TRAJ_THR*TRAJ_THR){
    if(wp.wp_index < NUM_WPS-1){
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

