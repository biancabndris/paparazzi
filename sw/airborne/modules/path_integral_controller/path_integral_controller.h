/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time/computation_time.h"
 * @author Bianca Bendris
 * Run a path integral algorithm and measure the running time needed to perform the computations
 */

#ifndef COMPUTATION_TIME_H
#define COMPUTATION_TIME_H

float optimal_control[2];
void pi_init(void);
void compute_optimal_controls_periodic(void);


static inline float * get_pi_optimal_controls(void){
  return optimal_control;
}
#endif

