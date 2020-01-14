/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : tdlas_sensor.h
   Author : tao.jing
   Date   : 19-10-17
   Brief  : 
**************************************************************************/

#ifndef RAOS_TDLAS_SENSOR_H
#define RAOS_TDLAS_SENSOR_H

#include "common/math/raos_math.h"

enum e_scan_model
{
    e_tdlas_none = 0,
    e_tdlas_line = 1,
    e_tdlas_ring = 2,
    e_tdlas_external = 3,
};

class tdlas_sensor
{
public:
    tdlas_sensor(float _radius = 0.5f);

public:
    // Get current TDLAS sensor readings
    float get_current_tdlas_reading();
    // Update the TDLAS start point and end point
    void update_tdlas_scan(inner_point_t cur_robot_pos, inner_point_t ref_robot_pos);
    // Get sample concentration evaluation
    float get_simple_conc_evaluation(inner_point_t A, inner_point_t B, float z_min, float z_max);


    inner_point_t generate_scan_offset(e_scan_model scan_model);

public:
    // Start Point and End Point in world coordinate system
    inner_point_t start_point;
    inner_point_t end_point;

    inner_point_t offset;

private:
    float radius;

    unsigned long long scan_update_time;
};

#endif //RAOS_TDLAS_SENSOR_H
