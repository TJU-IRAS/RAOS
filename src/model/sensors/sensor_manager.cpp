/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : sensor_manager.cpp
   Author : tao.jing
   Date   : 19-10-15
   Brief  : 
**************************************************************************/
#include "sensor_manager.h"

sensor_manager* sensor_manager::instance()
{
    static sensor_manager inst;
    return &inst;
}

sensor_manager::sensor_manager()
{

}