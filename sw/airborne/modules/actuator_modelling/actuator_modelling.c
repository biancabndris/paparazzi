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
 * @file "modules/actuator_modelling/actuator_modelling.c"
 * @author Bianca Bendris 
 * This module gives different velocity commands to the guided mode in different directions with the purpose of creating a model of the actuator response to these velocity commands. 
 */

#include "modules/actuator_modelling/actuator_modelling.h"
#include "subsystems/abi.h"
#include "state.h"
#include <stdio.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"

float vel_array[10] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1};
uint8_t run;
uint8_t item;
bool completed;
uint8_t stop;
float command_x[16];
float command_y[16];

//#if PERIODIC_TELEMETRY
//#include "subsystems/datalink/telemetry.h"
/**
 * Send path integral control telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
/*
static void pi_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&pi_mutex);
  pprz_msg_send_PATH_INTEGRAL(trans, dev, AC_ID,
                               &command_x[item], &command_y[item], 0, 0);
  pthread_mutex_unlock(&pi_mutex);
}
#endif
*/

void init(void){

  run = 0;
  item = 0;
  completed = true;
  stop = 0;

}

uint8_t check_command(void){
  float vel = vel_array[run];
  printf("vel assigned , %f\n",vel);
  float vel_xy = sqrt((vel*vel)*0.5);

  float pattern_x[16] = {vel,0.0,-vel,0.0,0.0,0.0,0.0,0.0,vel_xy,0.0,-vel_xy,0.0,vel_xy,0.0,-vel_xy,0.0};
  float pattern_y[16] = {0.0,0.0,0.0,0.0,vel,0.0,-vel,0.0,vel_xy,0.0,-vel_xy,0.0,-vel_xy,0.0,vel_xy,0.0};

  if(completed){
    if(run < 4){
      for(int i = 0; i < 16; i++){
        command_x[i] = pattern_x[i];
        command_y[i] = pattern_y[i];
      }

      completed = false;
    }
    else{
      stop = 1;
    }
  }
  else{
    in.vel_x = command_x[item];
    in.vel_y = command_y[item];
    printf("vel x %f, vel y %f\n", in.vel_x , in.vel_y);
    uint32_t now_ts = get_sys_time_usec();
    AbiSendMsgPATH_INTEGRAL(PATH_INTEGRAL_ID, now_ts,in.vel_x,in.vel_y);

    //guidance_h_set_guided_vel(in.vel_x, in.vel_y);

    //Convert from NED to body coordinates
    float psi = stateGetNedToBodyEulers_f()->psi;
    float vel_ned_n = stateGetSpeedNed_f()->x;
    float vel_ned_e = stateGetSpeedNed_f()->y;
    printf("Velocity N %f, velocity E %f\n",vel_ned_n,vel_ned_e);
    float vel_body_x = cosf(psi) * vel_ned_n  + sinf(psi) * vel_ned_e;
    float vel_body_y = -sinf(psi) * vel_ned_n  + cosf(psi) * vel_ned_e;
    printf("Velocity body x %f, bosy y  %f\n",vel_body_x,vel_body_y);
    float diff_x = fabs(command_x[item] - vel_body_x);
    float diff_y = fabs(command_y[item] - vel_body_y);

    if(diff_x < 0.01 && diff_y < 0.01){
      item += 1;
      printf("equal velocity\n");
    }

    if(item == 16){
      printf("item = 16\n");
      completed = true;
      run += 1;
      item = 0;
    }

  }

  return stop;

}




