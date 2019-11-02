/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : mox_sensor.cpp
   Author : tao.jing
   Date   : 19-10-15
   Brief  : 
**************************************************************************/
#include "math.h"
#include "mox_sensor.h"

mox_sensor::mox_sensor(gas_type_e _gas_type,
                       sensor_type_e _sensor_type,
                       double _node_rate)
: gas_type(_gas_type)
, sensor_type(_sensor_type)
, node_rate(_node_rate)
{
    RS_R0 = 0.0f;
    sensor_output = Sensitivity_Air[sensor_type];;
    previous_sensor_output = sensor_output;
}

void mox_sensor::mox_sensor_config(gas_type_e _gas_type, sensor_type_e _sensor_type)
{
    gas_type = _gas_type;
    sensor_type = _sensor_type;
}

double mox_sensor::get_current_mox_reading(double current_conc)
{
    /*
    if ( current_conc > -1 * 1E-6 )
    {
        conc_vec.push_back(current_conc);
    }
    */

    //1. Set Sensor Output based on gas concentrations (gas type dependent)
    //---------------------------------------------------------------------
    // RS/R0 = A*conc^B (a line in the loglog scale)
    float resistance_variation = 0.0;

    RS_R0 = sensitivity_lineloglog[sensor_type][gas_type][0] * pow(current_conc, sensitivity_lineloglog[sensor_type][gas_type][1]);

    //Ensure we never overpass the baseline level (max allowed)
    /*
    if (RS_R0 > Sensitivity_Air[sensor_type])
        RS_R0= Sensitivity_Air[sensor_type];
    */
    //Increment with respect the Baseline
    resistance_variation += Sensitivity_Air[sensor_type] - RS_R0;

    //Calculate final RS_R0 given the final resistance variation
    RS_R0 = Sensitivity_Air[sensor_type] - resistance_variation;

    //Ensure a minimum sensor resitance
    if (RS_R0 <= 0.0)
        RS_R0 = 0.01;

    //2. Simulate transient response (dynamic behaviour, tau_r and tau_d)
    //---------------------------------------------------------------------
    float tau;
    if (RS_R0 < previous_sensor_output)  //rise
        tau = tau_value[sensor_type][0][0];
    else //decay
        tau = tau_value[sensor_type][0][1];

    // Use a low pass filter
    //alpha value = At/(tau+At)
    double alpha = (1/node_rate) / (tau+(1/node_rate));

    //filtered response (uses previous estimation):
    sensor_output = (alpha*RS_R0) + (1-alpha)*RS_R0;//previous_sensor_output;

    //Update values
    previous_sensor_output = sensor_output;

    float lnA = log(sensitivity_lineloglog[sensor_type][gas_type][0]);
    //float lnRS_R0 = log((sensor_output * R0[sensor_type]));
    float lnRS_R0 = log((sensor_output));
    float conc_output = exp( (lnRS_R0 - lnA) / sensitivity_lineloglog[sensor_type][gas_type][1] );

    //return (sensor_output * R0[sensor_type]);
    return conc_output;
}