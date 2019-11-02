/*
 * Models of RAOS
 *
 * This file is included by SimMain.cxx
 * The implementations are written in SimModel.cxx, which contains functions
 * in files qrmod.cxx plume_model.cxx wind_model.cxx
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-23 create this file (RAOS)
 */
#ifndef SIMMODEL_H
#define SIMMODEL_H

#include "model/robot.h"

typedef struct
{
    double time;
    double dt;
    bool wake_initialized;
} SimState_t;

void SimModel_init(void);

void SimModel_update(void);

void SimModel_destroy(void);

std::vector<Robot *> *SimModel_get_robots(void);

SimState_t *SimModel_get_sim_state(void);

#endif
/* End of SimModel.h */
