/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : virtual_plume.h
   Author : tao.jing
   Date   : 2020/6/19
   Brief  : 
**************************************************************************/
#ifndef RAOS_VIRTUAL_PLUME_H
#define RAOS_VIRTUAL_PLUME_H

#include "plume.h"

typedef struct rotor_state_s_t
{
    float pos[3];       // rotor position
    float psi;          // blade azimuth, [0, 360), volatile
    float thrust;       // the thrust this rotor produce, N
    float marker_num;   // markers of this rotor - convert to float
}rotor_state_t;

class Robot;
class virtual_plume
{
public:
    static virtual_plume* instance();

    void init();

    void update_rotor_info(std::vector<Robot *> * robots);

    void save_virtual_plume_info();

private:
    virtual_plume();

public:
    std::vector<FilaState_t>   plumes;
    std::vector<rotor_state_t> rotor_states;
};

#endif //RAOS_VIRTUAL_PLUME_H
