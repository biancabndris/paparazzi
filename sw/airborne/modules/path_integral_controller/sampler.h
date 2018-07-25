/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time/PIController.h"
 * @author Bianca Bendris
 *
 */


#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdint.h>


struct PIController{
   int8_t T;
   float dt;
   int16_t iT;
   int8_t H;
   float dh;
   int16_t iH;
   float nu;
   float R;
   float var;
   float lambda;
   int16_t N;
   float COLLISION_DISTANCE;
   float COHESION_DISTANCE;
   float COLLISION_PENALTY;
   float COHESION_PENALTY;
   float TARGET_PENALTY;
   float HEADING_PENALTY;
   float PARALLEL_PENALTY;
   float MAX_SPEED;
   float PARALLEL_THR;

   int8_t units;
   int8_t dimX;
   int8_t dimU;
   float state[4];
   float followers[2][4];

   float wps[4];
   float u_exp[5][2];
};



struct PIController;


extern void PIController_init(struct PIController *);
extern void compute_optimal_controls(struct PIController *ptr);

//extern void PIController_shiftControls(struct PIController *);


#endif // PI_CONTROLLER_H
