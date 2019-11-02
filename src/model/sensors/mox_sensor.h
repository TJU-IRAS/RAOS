/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : mox_sensor.h
   Author : tao.jing
   Date   : 19-10-15
   Brief  : 
**************************************************************************/

#ifndef RAOS_MOX_SENSOR_H
#define RAOS_MOX_SENSOR_H

#include <vector>
#include "sensor_def.h"


const float R0[5] = {3000, 50000, 3740, 3740, 4500};      //[Ohms] Reference resistance (see datasheets)

//Time constants (Rise, Decay), unit:second
const float tau_value[5][7][2] =      //5 sensors, 7 gases , 2 Time Constants
    {
        {  //TGS2620
            {2.96, 15.71},  //ethanol
            {2.96, 15.71},  //methane
            {2.96, 15.71},  //hydrogen
            {2.96, 15.71},  //propanol
            {2.96, 15.71},  //chlorine
            {2.96, 15.71},  //fluorine
            {2.96, 15.71}   //Acetone
        },

        {  //TGS2600
            {4.8,  18.75},   //ethanol
            {4.8,  18.75},   //methane
            {4.8,  18.75},   //hydrogen
            {4.8,  18.75},   //propanol
            {4.8,  18.75},   //chlorine
            {4.8,  18.75},   //fluorine
            {4.8,  18.75}    //Acetone
        },

        {  //TGS2611
            {3.44, 6.35},   //ethanol
            {3.44, 6.35},   //methane
            {3.44, 6.35},   //hydrogen
            {3.44, 6.35},   //propanol
            {3.44, 6.35},   //chlorine
            {3.44, 6.35},   //fluorine
            {3.44, 6.35}    //Acetone
        },

        {  //TGS2610
            {3.44, 6.35},   //ethanol
            {3.44, 6.35},   //methane
            {3.44, 6.35},   //hydrogen
            {3.44, 6.35},   //propanol
            {3.44, 6.35},   //chlorine
            {3.44, 6.35},   //fluorine
            {3.44, 6.35}    //Acetone
        },

        {  //TGS2612
            {3.44, 6.35},   //ethanol
            {3.44, 6.35},   //methane
            {3.44, 6.35},   //hydrogen
            {3.44, 6.35},   //propanol
            {3.44, 6.35},   //chlorine
            {3.44, 6.35},   //fluorine
            {3.44, 6.35}    //Acetone
        }
    };

// MOX sensitivity. Extracted from datasheets and curve fitting
//--------------------------------------------------------------
const float Sensitivity_Air[5] = {21, 1, 8.8, 10.3, 19.5};      //RS/R0 when exposed to clean air (datasheet)

// RS/R0 = A*conc^B (a line in the loglog scale)
const float sensitivity_lineloglog[5][7][2] = {   //5 Sensors, 7 Gases, 2 Constants: A, B
    {  //TGS2620
        {62.32,  -0.7155},   //Ethanol
        {120.6, -0.4877},   //Methane
        {24.45,  -0.5546},   //Hydrogen
        {120.6, -0.4877},   //propanol (To review)
        {120.6, -0.4877},   //chlorine (To review)
        {120.6, -0.4877},   //fluorine (To review)
        {120.6, -0.4877}    //Acetone (To review)
    },

    {  //TGS2600
        {0.6796, -0.3196},   //ethanol
        {1.018, -0.07284},   //methane
        {0.6821, -0.3532},    //hydrogen
        {1.018, -0.07284},   //propanol (To review)
        {1.018, -0.07284},   //chlorine (To review)
        {1.018, -0.07284},   //fluorine (To review)
        {1.018, -0.07284}    //Acetone (To review)
    },

    {  //TGS2611
        {51.11,  -0.3658},    //ethanol
        {38.46, -0.4289},    //methane
        {41.3,   -0.3614},     //hydrogen
        {38.46, -0.4289},   //propanol (To review)
        {38.46, -0.4289},   //chlorine (To review)
        {38.46, -0.4289},   //fluorine (To review)
        {38.46, -0.4289}    //Acetone (To review)
    },

    {  //TGS2610
        {106.1,  -0.5008},     //ethanol
        {63.91, -0.5372},     //methane
        {66.78,  -0.4888},     //hydrogen
        {63.91, -0.5372},   //propanol (To review)
        {63.91, -0.5372},   //chlorine (To review)
        {63.91, -0.5372},   //fluorine (To review)
        {63.91, -0.5372}    //Acetone (To review)
    },

    {  //TGS2612
        {31.35,  -0.09115},   //ethanol
        {146.2, -0.5916},    //methane
        {19.5,   0.0},         //hydrogen
        {146.2, -0.5916},   //propanol (To review)
        {146.2, -0.5916},   //chlorine (To review)
        {146.2, -0.5916},   //fluorine (To review)
        {146.2, -0.5916}    //Acetone (To review)
    }
};

//definition of mox sensor
class mox_sensor
{
public:
    mox_sensor(gas_type_e _gas_type = DEFAULT_GAS_TYPE,
               sensor_type_e _sensor_type = DEFAULT_SENSOR_TYPE,
               double _node_rate = 5);

public:
    void mox_sensor_config(gas_type_e _gas_type, sensor_type_e _sensor_type);

    double get_current_mox_reading(double current_conc);

private:
    gas_type_e    gas_type;
    sensor_type_e sensor_type;

    double node_rate;

    std::vector<double> conc_vec;

    //Ideal sensor response based on sensitivity
    double RS_R0;
    double sensor_output;
    double previous_sensor_output;
};

#endif //RAOS_MOX_SENSOR_H
