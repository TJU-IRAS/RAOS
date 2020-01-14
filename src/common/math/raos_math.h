/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : raos_math.h
   Author : tao.jing
   Date   : 19-10-17
   Brief  : 
**************************************************************************/

#ifndef RAOS_RAOS_MATH_H
#define RAOS_RAOS_MATH_H

#include <math.h>
#include <assert.h>

#ifdef FLOAT_EQUAL
#undef FLOAT_EQUAL
#endif
#define FLOAT_EQUAL(x, y) ((x > y + 1E-6) && (x < y - 1E-6))

class inner_point_t
{
public:
    inner_point_t()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
    {
    }

    inner_point_t(float _x, float _y, float _z)
    : x(_x)
    , y(_y)
    , z(_z)
    {
    }

    inner_point_t(float arry[3])
    : x(arry[0])
    , y(arry[1])
    , z(arry[2])
    {
    }

    inner_point_t(const inner_point_t& copy_value)
    : x(copy_value.x)
    , y(copy_value.y)
    , z(copy_value.z)
    {

    }

public:
    float norm2()
    {
        return static_cast<float>(sqrt( pow(x, 2) + pow(y, 2) + pow(z, 2) ));
    }

    //override [] operator
    float& operator[](int i)
    {
        assert(i <= 2 && i >= 0);
        float* p_element = nullptr;
        switch (i)
        {
            case 0:
                p_element = &x;
                break;
            case 1:
                p_element = &y;
                break;
            case 2:
                p_element = &z;
                break;
            default:
                break;
        }
        return *p_element;
    }

    //override = operator
    inner_point_t& operator=(const inner_point_t& right_value)
    {
        x = right_value.x;
        y = right_value.y;
        z = right_value.z;
        return *this;
    }

    inner_point_t operator+(const inner_point_t& right_value)
    {
        inner_point_t sum_point;
        sum_point.x = x + right_value.x;
        sum_point.y = y + right_value.y;
        sum_point.z = z + right_value.z;
        return sum_point;
    }

    inner_point_t operator-(const inner_point_t& right_value)
    {
        inner_point_t diff_point;
        diff_point.x = x - right_value.x;
        diff_point.y = y - right_value.y;
        diff_point.z = z - right_value.z;
        return diff_point;
    }

    bool operator<(const inner_point_t& right_value)
    {
        if (x < right_value.x)
        {
            return true;
        }
        else if (FLOAT_EQUAL(x, right_value.x) && y < right_value.y)
        {
            return true;
        }
        else if (FLOAT_EQUAL(x, right_value.x) && FLOAT_EQUAL(y, right_value.y) && z < right_value.z)
        {
            return true;
        }
        return false;
    }

public:
    float x;
    float y;
    float z;
};

// Calc distance between pointT and the line determined by pointA and pointB
float calc_point_to_line_dist(const inner_point_t& pointA, const inner_point_t& pointB, const inner_point_t& pointP);

bool  whether_point_projected_belong_line_segment(const inner_point_t& pointA, const inner_point_t& pointB, const inner_point_t& pointP);

// Calc the volume of a spherical cap with r as chord length and h as arch height
float calc_spherical_cap_volume(float r_chord_len, float h_arch_height);
// Calc the volume of a spherical without cap with r as chord length and h as arch height
float calc_spherical_without_cap_volume(float r_chord_len, float h_arch_height);


#endif //RAOS_RAOS_MATH_H
