/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : sensor_manager.h
   Author : tao.jing
   Date   : 19-10-15
   Brief  : 
**************************************************************************/

#ifndef RAOS_SENSOR_MANAGER_H
#define RAOS_SENSOR_MANAGER_H

#include <vector>
#include "mox_sensor.h"

typedef struct
{
    float pos[3];    /* position coordinate (earth axis x), volatile */
} SensorBaseRefPos;

class sensor_manager
{
public:
    static sensor_manager* instance();

    //bool add_mox_sensor(SensorBaseRefPos);

private:
    sensor_manager();

private:
    std::vector<mox_sensor> mox_sensor_array;
};

#endif //RAOS_SENSOR_MANAGER_H
