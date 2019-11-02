/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : raos_math.cpp
   Author : tao.jing
   Date   : 19-10-17
   Brief  :
**************************************************************************/
#include "raos_math.h"

// Calc distance between pointT and the line determined by pointA and pointB
float calc_point_to_line_dist(const inner_point_t& pointA, const inner_point_t& pointB, const inner_point_t& pointP)
{
    // Vector from A to T and B
    inner_point_t vector_AP( pointP.x - pointA.x, pointP.y - pointA.y, pointP.z - pointA.z );
    inner_point_t vector_AB( pointB.x - pointA.x, pointB.y - pointA.y, pointB.z - pointA.z );

    inner_point_t cross_AP_AB(vector_AB.y * vector_AP.z - vector_AB.z * vector_AP.y,
                              vector_AB.x * vector_AP.z - vector_AB.z * vector_AP.x,
                              vector_AB.x * vector_AP.y - vector_AB.y * vector_AP.x);

    float distance_P_AB = cross_AP_AB.norm2() / vector_AB.norm2();

    return distance_P_AB;
}

bool whether_point_projected_belong_line_segment(const inner_point_t& pointA, const inner_point_t& pointB, const inner_point_t& pointP)
{
    inner_point_t vector_AP( pointP.x - pointA.x, pointP.y - pointA.y, pointP.z - pointA.z );
    inner_point_t vector_AB( pointB.x - pointA.x, pointB.y - pointA.y, pointB.z - pointA.z );

    float dot_product_AP_AB = vector_AP.x * vector_AB.x + vector_AP.y + vector_AB.y + vector_AP.z * vector_AB.z;
    float cos_AP_AB = dot_product_AP_AB / vector_AB.norm2() / vector_AP.norm2();

    inner_point_t vector_BP( pointP.x - pointB.x, pointP.y - pointB.y, pointP.z - pointB.z );
    inner_point_t vector_BA( pointA.x - pointB.x, pointA.y - pointB.y, pointA.z - pointB.z );

    float dot_product_BP_BA = vector_BP.x * vector_BA.x + vector_BP.y + vector_BA.y + vector_BP.z * vector_BA.z;
    float cos_BP_BA = dot_product_BP_BA / vector_BP.norm2() / vector_BA.norm2();

    if ( cos_BP_BA > -1 && cos_BP_BA < 1 && cos_AP_AB > -1 && cos_AP_AB < 1 )
    {
        return true;
    }

    return false;
}

float calc_spherical_cap_volume(float r_chord_len, float h_arch_height)
{
    return static_cast<float>(M_PI / 6.0f * (3.0f * pow(r_chord_len, 2) * h_arch_height + pow(h_arch_height, 3)));
}

float calc_spherical_without_cap_volume(float r_chord_len, float h_arch_height)
{
    float v_spherical_cap = static_cast<float>(M_PI / 6.0f * (3.0f * pow(r_chord_len, 2) * h_arch_height + pow(h_arch_height, 3)));
    float R_spherical = static_cast<float>( (pow(r_chord_len, 2) + pow(h_arch_height, 2)) / 2 / h_arch_height );
    float v_spherical = static_cast<float>( 4.0f / 3.0f * M_PI * pow(R_spherical, 3));
    return v_spherical - v_spherical_cap;
}