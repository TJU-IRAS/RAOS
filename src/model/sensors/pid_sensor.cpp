/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : pid_sensor.cpp
   Author : tao.jing
   Date   : 19-10-16
   Brief  : 
**************************************************************************/
#include "pid_sensor.h"

pid_sensor::pid_sensor(gas_type_e _gas_type,
                       double _node_rate)
: gas_type(_gas_type)
, node_rate(_node_rate)
{

}

void pid_sensor::pid_sensor_config(gas_type_e _gas_type)
{
    gas_type = _gas_type;
}

double pid_sensor::get_current_pid_reading(double current_conc)
{
    /*
    if ( current_conc > -1 * 1E-6 )
    {
        conc_vec.push_back(current_conc);
    }
    */

    if ( PID_correction_factors[gas_type] < 1E-6 )
    {
        return current_conc;
    }
    return  current_conc / PID_correction_factors[gas_type] * 10;
}