/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : tdlas_sensor.cpp
   Author : tao.jing
   Date   : 19-10-17
   Brief  :
**************************************************************************/
#include <string.h>
#include <iostream>
#include "tdlas_sensor.h"
#include "common/math/raos_math.h"
#include "model/plume.h"

tdlas_sensor::tdlas_sensor(float _radius)
: radius(_radius)
, scan_update_time(0)
{
    offset = inner_point_t(0, 0, 0);
}

float tdlas_sensor::get_current_tdlas_reading()
{
    inner_point_t pointA(start_point[0], start_point[1], start_point[2]);
    inner_point_t pointB(end_point[0], end_point[1], end_point[2]);

    float tdlas_conc = 0.0f;
    std::vector<FilaState_t> *puffs = plume_get_fila_state();
    for (unsigned int i = 0; i < puffs->size(); i++)
    {
        inner_point_t pointP(puffs->at(i).pos[0], puffs->at(i).pos[1], puffs->at(i).pos[2]);

        if (!whether_point_projected_belong_line_segment(pointA, pointB, pointP))
        {
            continue;
        }

        float dist_filament = calc_point_to_line_dist(pointA, pointB, pointP);
        if ( dist_filament > radius + puffs->at(i).r )
        {
            continue;
        }

        float v_proportion = 0.0f;
        float v_filament = static_cast<float>(4.0f / 3.0f * M_PI * pow(puffs->at(i).r, 3));
        if ( dist_filament + puffs->at(i).r < radius  )
        {
            v_proportion = 1.0f;
        }
        else
        {
            if ( dist_filament > radius )
            {
                //spherical cap
                float h = puffs->at(i).r - ( dist_filament - radius );
                float r = static_cast<float>(sqrt(pow(puffs->at(i).r, 2) - pow(dist_filament - radius, 2)));
                float v_spherical_cap = calc_spherical_cap_volume(r, h);
                v_proportion = v_spherical_cap / v_filament;
            }
            else if( dist_filament < radius )
            {
                //spherical without cap
                float h = dist_filament + puffs->at(i).r - radius;
                float r = static_cast<float>(sqrt(pow(puffs->at(i).r, 2) - pow(radius - dist_filament, 2)));
                float v_spherical_without_cap = calc_spherical_without_cap_volume(r, h);
                v_proportion = v_spherical_without_cap / v_filament;
            }
        }

        tdlas_conc += 1.0 / 1.40995 * v_proportion;
    }

    return tdlas_conc;
}

// cur_robot_pos: new robot position; ref_robot_pos: tdlas position w.r.t robot center
void tdlas_sensor::update_tdlas_scan(inner_point_t cur_robot_pos, inner_point_t ref_robot_pos)
{
    //update tdlas scan line start point, namely the mounted point
    start_point = cur_robot_pos + ref_robot_pos;

    //update tdlas scan line end point
    end_point = generate_scan_offset(e_tdlas_external) + cur_robot_pos;

    scan_update_time ++;
}

float tdlas_sensor::get_simple_conc_evaluation(inner_point_t A, inner_point_t B, float z_max, float z_min)
{
    extern float get_certain_pos_conc(inner_point_t pos);

    float conc_ret = 0.0f;
    conc_ret += get_certain_pos_conc(A);
    conc_ret += get_certain_pos_conc(B);
    unsigned int z_count = 10;
    float z_delta = ( z_max - z_min ) / z_count;

    for (unsigned int z_idx = 0; z_idx < z_count; z_idx ++)
    {
        float z = z_min + z_delta * z_idx ;
        A.z = z;
        B.z = z;
        conc_ret += get_certain_pos_conc(A);
        conc_ret += get_certain_pos_conc(B);
    }
    return conc_ret;
}

inner_point_t tdlas_sensor::generate_scan_offset(e_scan_model scan_model)
{
    if ( scan_model == e_tdlas_none )
    {
        return inner_point_t(0, 0, 0);
    }

    if ( scan_model == e_tdlas_line )
    {
        //Method1 - scan reciprocate in line
        static int line_round = 1;
        // x_scan_pos and y_scan_pos are relative position w.r.t robot frame
        float x_scan_pos = 1;
        float y_scan_max_pos = 0.5f;
        //scan cycle 2s --- 0.001s one interval
        long long half_cycle = static_cast<long>( 2 / 0.01f) ;
        long long scan_idx = scan_update_time % half_cycle;

        if ( scan_idx == 0 )
        {
            line_round *= -1;
        }

        float scan_step = y_scan_max_pos * 2 / half_cycle;
        float y_scan_pos = line_round * y_scan_max_pos + (-line_round) * scan_idx * scan_step;
        return inner_point_t(-x_scan_pos, -y_scan_pos, 0);
    }

    if ( scan_model == e_tdlas_external )
    {
        //Method3- start & end point offset will be updated by external method
        return offset;
    }

    return inner_point_t(0, 0, 0);
}