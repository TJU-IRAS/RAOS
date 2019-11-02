/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : sensor_def.h
   Author : tao.jing
   Date   : 19-10-16
   Brief  :
**************************************************************************/

#ifndef RAOS_SENSOR_DEF_H
#define RAOS_SENSOR_DEF_H


enum gas_type_e
{
    e_ethanol_id = 0,
    e_methane_id = 1,
    e_hydrogen_id = 2,
    e_propanol_id = 3,
    e_chloride_id = 4,
    e_flurorine_id = 5,
    e_acetone_id = 6,
    e_neon_id = 7,
    e_helium_id = 8,
    e_hotair_id = 9,
};

enum sensor_type_e
{
    e_tgs2620_id = 0,
    e_tgs2600_id = 1,
    e_tgs2611_id = 2,
    e_tgs2610_id = 3,
    e_tgs2612_id = 4,
};

#define DEFAULT_GAS_TYPE    e_methane_id
#define DEFAULT_SENSOR_TYPE e_tgs2620_id

#endif //RAOS_SENSOR_DEF_H
