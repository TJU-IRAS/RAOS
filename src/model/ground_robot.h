/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : ground_robot.h
   Author : tao.jing
   Date   : 19-10-18
   Brief  : 
**************************************************************************/

#ifndef RAOS_GROUND_ROBOT_H
#define RAOS_GROUND_ROBOT_H

#include "model/robot.h"

// Ground Robot frame
typedef struct GRframe_t
{
    float body_depth;
    float body_width;
    float body_height;
    float up_depth;
    float up_width;
    float up_height;
    float up_offset;

    float angle;

    GRframe_t()
    {
        body_depth = 2.5f; //X/east
        body_width = 1.7f; //Y/north
        body_height = 1.5f;

        up_depth = 1.2f;
        up_width = body_width;
        up_height = 1.0f;

        up_offset = body_height + up_height;

        angle = 0.0f;
    }

} GRframe_t;

// Ground Robot Model
class GRdynamic
{
public:
    GRdynamic(float *pos_ref, float *pos, GRframe_t *frm);

    void update();

public:
    GRframe_t* frame;

    float *GR_pos_ref; // reference position
    float GR_yaw_ref; // reference heading
    float *GR_pos; // actual position
};

#endif //RAOS_GROUND_ROBOT_H
