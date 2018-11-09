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
#include "state.h"

struct PIController{
   uint8_t H;
   float dh;
   uint16_t iH;
   float nu;
   float R;
   float var;
   float lambda;
   uint16_t N;
   float COLLISION_DISTANCE;
   float COHESION_DISTANCE;
   float COLLISION_PENALTY;
   float COHESION_PENALTY;
   float TARGET_PENALTY;
   float HEADING_PENALTY;
   float PARALLEL_PENALTY;
   float MAX_SPEED;
   float PARALLEL_THR;

   uint8_t units;
   uint8_t dimX;
   uint8_t dimU;


   float wps[4];
   float u_exp[5][2];
};

struct PIstate{
   float pos[2];
   float vel[2];
   float pos_rel[4];
   float vel_rel[4];
};

struct Input{
  float oc_x;
  float oc_y;
};




extern void PIController_init(struct PIController * pi2 );
extern void compute_optimal_controls(struct PIController * pi2, struct PIstate * st2, struct Input * in);
extern void compute_optimal_controls_followers(struct PIController * pi2, struct PIstate * st2, struct Input * in);
extern void shiftControls(void);

static inline void setState(struct PIstate * st2){
  st2->pos[0]     = stateGetPositionNed_f()->x;
  st2->pos[1]     = stateGetPositionNed_f()->y;
  st2->vel[0]     = stateGetSpeedNed_f()->x;
  st2->vel[1]     = stateGetSpeedNed_f()->z;

  st2->pos_rel[0] = 4;
  st2->pos_rel[1] = 4;
  st2->pos_rel[2] = 2;
  st2->pos_rel[3] = 2;

  st2->vel_rel[0] = 0;
  st2->vel_rel[1] = 0.2;
  st2->vel_rel[2] = 0.2;
  st2->vel_rel[3] = 0;
}

static inline void initInput(struct Input * in){
  in->oc_x = 0;
  in->oc_y = 0;

}

/*static inline struct Input *get_pi_optimal_controls(void){
  return &in;
}*/

#endif // PI_CONTROLLER_H
