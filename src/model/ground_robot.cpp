/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : ground_robot.cpp
   Author : tao.jing
   Date   : 19-10-18
   Brief  : 
**************************************************************************/
#include "ground_robot.h"

GRdynamic::GRdynamic(float *pos_ref, float *pos, GRframe_t *frm)
{
    GR_pos_ref = pos_ref;
    GR_pos = pos;
    frame = frm;
}

void GRdynamic::update()
{

}

