/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : pid_sensor.h
   Author : tao.jing
   Date   : 19-10-16
   Brief  : 
**************************************************************************/
#ifndef RAOS_PID_SENSOR_H
#define RAOS_PID_SENSOR_H

#include <vector>
#include "sensor_def.h"

//PID correction factors for gas concentration
//--------------------------------------------
//Ethanol, Methane, Hydrogen, Propanol, Chlorine, Fluorine, Acetone
// http://www.intlsensor.com/pdf/pidcorrectionfactors.pdf
// Here we simulate a lamp of 11.7eV to increase the range of detectable gases
// A 0.0 means the PID is not responsive to that gas
const float PID_correction_factors[7] = {10.47, 12.51, 15.43, 10.22, 11.48, 0.0, 9.71};


class pid_sensor
{
public:
    pid_sensor(gas_type_e _gas_type = DEFAULT_GAS_TYPE,
               double _node_rate = 5);

public:
    void pid_sensor_config(gas_type_e _gas_type);

    double get_current_pid_reading(double current_conc);

public:
    std::vector<double> conc_vec;

private:
    gas_type_e    gas_type;

    double node_rate;
};

#endif //RAOS_PID_SENSOR_H
