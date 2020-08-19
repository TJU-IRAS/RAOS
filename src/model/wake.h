/*
 * Wakes in the simulation environment
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-08 create this file (RAOS)
 */

#ifndef WAKE_H
#define WAKE_H

#include "model/SimModel.h"
#include "model/environment.h"

void WakesInit(std::vector<Robot *> *);

void WakesUpdate(std::vector<Robot *> *, const char *, SimState_t *,
                 SimEnvInfo *); // update all of the wakes in the environment
void WakesFinish(void);

// for plume puffs
void WakesIndVelatPlumePuffsInit();

void WakesIndVelatPlumePuffsUpdate(std::vector<Robot *> *, std::vector<FilaState_t> *);

//for a cube around the robot wake
void WakesInvVelatAdjacentCubeInit();

void WakesInvVelatAdjacentCubeUpdate(std::vector<Robot*> *robots, std::vector<FilaState_t> *);

#endif

/* End of file wake.h */
