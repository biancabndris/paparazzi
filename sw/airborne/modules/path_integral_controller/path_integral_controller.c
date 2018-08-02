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

struct PIController pi;

void pi_init() {

  PIController_init(&pi);

}


void compute_optimal_controls_periodic(){

  clock_t start, end;
  float cpu_time_used;


  start = clock();
  compute_optimal_controls_followers(&pi);//compute_optimal_controls(&pi);
  end = clock();

  //printf("*controls %f\n", *controls);
  cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

  printf("------ ELAPSED TIME: %f\n----------",cpu_time_used);
  //printf("Controls 0: %f , Controls 1: %f\n", *controls, *controls++);
}


