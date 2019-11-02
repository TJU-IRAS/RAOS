/*
 * get wind model
 *
 * Author: Zhangqi kang
 * Date: 2017-12-4 create this file
 */
#include <cmath>
#include <vector>
#include <time.h> // for random seed
#include "model/windvector.h"
//#include "model/environment.h"
#include "SimConfig.h"
#include "ziggurat.h" // for normal distribution number
#include "SimModel.h"

#include <string.h>
#include <stdlib.h>
#include <iostream>

GetWind *getwind = new GetWind();
long update_time = 0;

/*==============   getwind Model  ============*/

std::vector<Wind_vel> *wind_get_saved_data(void)
{
    return &getwind->windData;
}


void GetWind::updatePos(float *robot_pos)
{
#ifdef FOLLOW_ROBOT
    vectorstate.at(index_robot).pos[0] = robot_pos[0];
    vectorstate.at(index_robot).pos[1] = robot_pos[1];
    vectorstate.at(index_robot).pos[2] = robot_pos[2] - FOLLOW_ROBOT_DOWN;
#endif
    //printf("pos1:%f,pos2:%f,pos3:%f,vel1:%f,vel2:%f,vel3:%f\n",vectorstate.back().pos[0],vectorstate.back().pos[1],vectorstate.back().pos[2],vectorstate.back().vel[0],vectorstate.back().vel[1],vectorstate.back().vel[2]);
}

void GetWind::updateVel(void)
{
    float wind[3];
    float vm_x, vm_y, vm_z;
    Wind_vel new_wind_data;

    for (unsigned int num = 0; num < vectorstate.size(); num++)
    {
        PL_env_info->measure_wind(vectorstate.at(num).pos, wind);

        float wind_noise_coef = 0.01; // 0.3 for office
        vm_x = wind_noise_coef * r4_nor(seed, kn, fn, wn);
        vm_y = wind_noise_coef * r4_nor(seed, kn, fn, wn);
        vm_z = wind_noise_coef * r4_nor(seed, kn, fn, wn);

        // vectorstate.at(num).vel[0] += (vm_x + wind[0]);
        // vectorstate.at(num).vel[1] += (vm_y + wind[1]);
        // vectorstate.at(num).vel[2] += (vm_z + wind[2]);
        vectorstate.at(num).vel[0] += wind[0];
        vectorstate.at(num).vel[1] += wind[1];
        vectorstate.at(num).vel[2] += wind[2];

#ifdef SAVE_WIND
        if (num == 0) //选择保存的位置
        {
            save_wind_count++;
            if (save_wind_count >= SAVE_WIND_PER)
            {
                save_wind_count = 0;
                memcpy(new_wind_data.vel, vectorstate.at(num).vel, 3 * sizeof(float));
                windData.push_back(new_wind_data);
            }
        }
#endif
    }
}

void GetWind::windget(float *pos, float *wind)
{
    long current_update;
    FilaState_t new_windvector;
    new_windvector.pos[0] = pos[0];
    new_windvector.pos[1] = pos[1];
    new_windvector.pos[2] = pos[2];
    new_windvector.vel[0] = 0;
    new_windvector.vel[1] = 0;
    new_windvector.vel[2] = 0;
    new_windvector.r = 0;
    vectorstate.push_back(new_windvector);

    current_update = update_time;
    while (update_time > current_update + 1);//确保cuda至少更新一次，防止在cuda运行的时候，更新vectorstate

    wind[0] = vectorstate.back().vel[0];
    wind[0] = vectorstate.back().vel[1];
    wind[0] = vectorstate.back().vel[2];

    vectorstate.pop_back();
}

void GetWind::init(SimEnvInfo *sim_env_info, float *robot_pos)
{
    PL_env_info = sim_env_info;
    SimConfig_t *sim_config = SimConfig_get_configs();

    float SimAreaX = sim_config->arena.w,
        SimAreaY = sim_config->arena.l;

    //Scene Relate Param

    //Office
    float bottom_x = ceil(-SimAreaX / 2),
        bottom_y = ceil(-SimAreaY / 2),
        top_x = ceil(SimAreaX / 2),
        top_y = ceil(SimAreaY / 2);
    /*
    //Wind Tunnel
    float bottom_x = (-SimAreaX / 2),
        bottom_y = (-SimAreaY / 2),
        top_x = (SimAreaX / 2),
        top_y = (SimAreaY / 2);
    */

    std::cout << "bottom_x: " << bottom_x << std::endl;
    std::cout << "bottom_y: " << bottom_y << std::endl;
    std::cout << "top_x: " << top_x << std::endl;
    std::cout << "top_y: " << top_y << std::endl;

    //printf("bottom_x:%f,bottom_y:%f,top_x:%f,top_y:%f\n",bottom_x,bottom_y,top_x,top_y );
#ifdef DRAW_GRID_VECTOR
    for (float id_x = bottom_x; id_x < top_x; id_x += WINDVECTOR_GRID)
    {
        for (float id_y = bottom_y; id_y < top_y; id_y += WINDVECTOR_GRID)
        {
            FilaState_t new_windvector;
            new_windvector.pos[0] = id_x;
            new_windvector.pos[1] = id_y;
            new_windvector.pos[2] = WINDVECTOR_HEIGHT;
            new_windvector.vel[0] = 0;
            new_windvector.vel[1] = 0;
            new_windvector.vel[2] = 0;
            new_windvector.r = 0;
            vectorstate.push_back(new_windvector);
        }
    }
#endif

#ifdef CUSTOMPOINTX
    FilaState_t new_windvector1;
    new_windvector1.pos[0] = CUSTOMPOINTX;
    new_windvector1.pos[1] = CUSTOMPOINTY;
    new_windvector1.pos[2] = CUSTOMPOINTZ;
    new_windvector1.vel[0] = 0;
    new_windvector1.vel[1] = 0;
    new_windvector1.vel[2] = 0;
    new_windvector1.r = 0;
    vectorstate.push_back(new_windvector1);
#endif

#ifdef FOLLOW_ROBOT
    FilaState_t new_windvector2;
    new_windvector2.pos[0] = robot_pos[0];
    new_windvector2.pos[1] = robot_pos[1];
    new_windvector2.pos[2] = robot_pos[2] - FOLLOW_ROBOT_DOWN;
    new_windvector2.vel[0] = 0;
    new_windvector2.vel[1] = 0;
    new_windvector2.vel[2] = 0;
    new_windvector2.r = 0;
    vectorstate.push_back(new_windvector2);
    index_robot = vectorstate.size() - 1;
#endif

    /* setup ziggurat method to generate normal distribution numbers */
    r4_nor_setup(kn, fn, wn);
    seed = time(NULL);
}


void windget_init(SimEnvInfo *sim_env_info, float *robot_pos)
{
    getwind->init(sim_env_info, robot_pos);
}


void windget_updatePos(float *robot_pos)
{
    getwind->updatePos(robot_pos);
}

void windget_updateVel(void)
{
    getwind->updateVel();
}

void windget_destroy(void)
{
    if (getwind)
    {
        getwind->vectorstate.clear();
        std::vector<FilaState_t>().swap(getwind->vectorstate);
        getwind->windData.clear();
        //delete getwind;
        //getwind = NULL;
    }
}

void windget(float *pos, float *wind)
{
#ifdef RAOS_FEATURE_WAKES
    getwind->windget(pos, wind);
#endif
}

std::vector<FilaState_t> *wind_get_vector_state(void)
{
    return &getwind->vectorstate;
}



/* End of windvector.cxx */
