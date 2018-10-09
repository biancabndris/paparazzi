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
#include  <time.h>


#include <math.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <stdlib.h>

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
                               &in.oc_x, &in.oc_y); // TODO: no noise measurement here...
}
#endif

void pi_init() {

  PIController_init();//&pi
  initInput(); //&in

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PATH_INTEGRAL, pi_telem_send);
#endif

}


void compute_optimal_controls_periodic(){

  clock_t start, end;
  float cpu_time_used;

  setState(); //&st

  start = clock();
  compute_optimal_controls(); //&pi, &st, &in//compute_optimal_controls(&pi);
  end = clock();

  //printf("*controls %f\n", *controls);
  cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

  printf("------ ELAPSED TIME: %f\n----------",cpu_time_used);
  printf("Controls 0: %f , Controls 1: %f\n", in.oc_x, in.oc_y);
}

