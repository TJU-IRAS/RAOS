/*
 * Gas Distribution Mapping
 *
 * Author: Roice (LUO Bing)
 * Date: 2017-03-21 create this file
 */
#include "model/robot.h"
#include "model/SimModel.h"
#include <cmath>
#include <stdio.h>

bool hover_measure_init(void)
{
    return true;
}

void hover_measure_update(SimState_t *sim_state)
{
    (SimModel_get_robots())->at(0)->ref_state.pos[0] = 0;
    (SimModel_get_robots())->at(0)->ref_state.pos[1] = 0;
    (SimModel_get_robots())->at(0)->ref_state.pos[2] = 1.5;
}

void hover_measure_stop(void)
{
}

/* End of file gas_dist_mapping.cxx */
