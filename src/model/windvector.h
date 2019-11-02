/*
 * get wind model
 *
 * Author: Zhangqi kang
 * Date: 2017-12-4 create this file
 */
#ifndef WINDVECTOR_H
#define WINDVECTOR_H

#include <vector>
#include "model/SimModel.h"
#include "model/environment.h"
#include "model/plume.h"

#define DRAW_GRID_VECTOR


#define SAVE_WIND
#define SAVE_WIND_PER 3

//Scene Relate Param
//Office
#define WINDVECTOR_GRID    1.0
#define WINDVECTOR_HEIGHT  1.5
//Wind Tunnel
//#define WINDVECTOR_GRID    0.05
//#define WINDVECTOR_HEIGHT  0.4

#define GET_WIND_BY_CUDA true

// #define CUSTOMPOINTX 2.0
#define CUSTOMPOINTY 0.0
#define CUSTOMPOINTZ 2.0

#define FOLLOW_ROBOT      true
#define FOLLOW_ROBOT_DOWN 0.0


typedef struct
{
    float vel[3];
} Wind_vel;

class GetWind
{
public:
    std::vector<FilaState_t> vectorstate; // state of windvector, a list
    std::vector<Wind_vel> windData;
    SimEnvInfo *PL_env_info;

    void init(SimEnvInfo *sim_env_info, float *robot_pos); // windget initialization
    void updatePos(float *robot_pos);

    void updateVel(void);

    void windget(float *pos, float *wind);

private:
    float fn[128];
    unsigned int kn[128];
    float wn[128];
    unsigned int seed;

    int save_wind_count = 0;

    int index_robot = 0;
};


void windget_init(SimEnvInfo *sim_env_info, float *robot_pos);

void windget_updatePos(float *robot_pos);

void windget_updateVel(void);

void windget_destroy(void);

void windget(float *pos, float *wind);

std::vector<FilaState_t> *wind_get_vector_state(void);

std::vector<Wind_vel> *wind_get_saved_data(void);

#endif

/* End of windvector.h */
