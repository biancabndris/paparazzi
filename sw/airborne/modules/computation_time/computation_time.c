/*
 * Copyright (C) Bianca Bendris
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/computation_time/computation_time.c"
 * @author Bianca Bendris
 * Run a path integral algorithm and measure the running time needed to perform the computations
 */

#include "modules/computation_time/computation_time.h"
#include "modules/computation_time/main_function.h"

void pi_init() {

	int result = main_function("pi_UAV_joint.xml");

}


