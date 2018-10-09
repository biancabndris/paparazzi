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
 * @file "modules/actuator_modelling/actuator_modelling.h"
 * @author Bianca Bendris 
 * This module gives different velocity commands to the guided mode in different directions with the purpose of creating a model of the actuator response to these velocity commands. 
 */

#ifndef ACTUATOR_MODELLING_H
#define ACTUATOR_MODELLING_H

#include "stdint.h"

extern void init(void);
extern uint8_t check_command(void);

struct Input{
  float vel_x;
  float vel_y;
};

struct Input in;

static inline struct Input *get_commanded_vel(void){
  return &in;
}

#endif

