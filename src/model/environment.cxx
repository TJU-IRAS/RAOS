#include <string.h>
#include <ctime>
#include <random>
#include "model/environment.h"
#include "SimWind.h"
#include "SimConfig.h"
#include "SimWind.h"
#include "windvector.h"
#include <iostream>

extern float winddata_X[X_N][Y_N][Z_N];
extern float winddata_Y[X_N][Y_N][Z_N];
extern float winddata_Z[X_N][Y_N][Z_N];

#define Noise_Ratio 10.0

SimEnvInfo::SimEnvInfo(float delta_t)
{
    dt = delta_t;

    SimConfig_t *sim_config = SimConfig_get_configs();

    Env_Size.width = sim_config->arena.w;
    Env_Size.length = sim_config->arena.l;
    Env_Size.height = sim_config->arena.h;


#ifdef ENV_WIND_WEIBULL
    random_generator = new std::default_random_engine;
    weibull_dist_wind[0] = new std::weibull_distribution<float>(100, 0.3);
    weibull_dist_wind[1] = new std::weibull_distribution<float>(100, 0.3);
    weibull_dist_wind[2] = new std::weibull_distribution<float>(100, 0.3);
#endif

#ifdef ENV_WIND_CN
    // wind_cn_params.damping = 0.3;
    wind_cn_params.damping = 0.3;
    //wind_cn_params.bandwidth = 0.05;
    wind_cn_params.bandwidth = 0.15;
    // wind_cn_params.G = 10.0;
    wind_cn_params.G = 10.0;
    wind_cn_state = (Wind_Colored_Noise_State_t *) malloc(sizeof(Wind_Colored_Noise_State_t) * 3);
    memset(wind_cn_state, 0, sizeof(Wind_Colored_Noise_State_t) * 3); // 3 components
    std::srand(std::time(0)); // use current time as seed for random generator
#endif
}

#ifdef ENV_WIND_CN

float colored_noise(Wind_Colored_Noise_Params_t *params, Wind_Colored_Noise_State_t *state, float dt)
{
    // sample from standard normal#include "SimConfig.h"
    float u = (float) std::rand() / (float) RAND_MAX - 0.5;
    // init local params
    float dx[2] = {0};
    dx[0] = state->x[1];
    dx[1] = -2 * params->damping * params->bandwidth * dx[0] +
            params->bandwidth * params->bandwidth * (params->G * u - state->x[0]);
    state->x[1] += dx[1] * dt;
    state->x[0] += state->x[1] * dt;
    return state->x[0];
}

#endif

void SimEnvInfo::measure_wind(float *pos, float *wind)
{
    int id_x, id_y, id_z;

    //Scene Related Param
    //Office
    /*
    id_x = (int) round((pos[0] + 9.15) * 10);
    id_y = (int) round((pos[1] + 7.15) * 10);
    id_z = (int) round((pos[2]) * 3.3333);
    */
    //Wind Tunnel
    ///*
    id_x = (int)round((pos[0] + Env_Size.width / 2.0) * 1.0f / WINDVECTOR_GRID);
    id_y = (int)round((pos[1] + Env_Size.length / 2.0) * 1.0f / WINDVECTOR_GRID);
    id_z = (int)round((pos[2]) * 1.0f / WINDVECTOR_GRID);
    //*/
    //printf("id_x:%d,id_y:%d,id_z:%d\n",id_x,id_y,id_z );

    //limit mat index range method 1: if out of range ,then set the index to boundary.

    if (id_x < 0)
        id_x = 0;
    else if (!(id_x < X_N))
        id_x = X_N - 1;

    if (id_y < 0)
        id_y = 0;
    else if (!(id_y < Y_N))
        id_y = Y_N - 1;

    if (id_z < 0)
        id_z = 0;
    else if (!(id_z < Z_N))
        id_z = Z_N - 1;


    //limit mat index range method 1: if out of range ,then set the wind value to zero.
    if (id_x < 0 || !(id_x < X_N) || id_y < 0 || !(id_y < Y_N) || id_z < 0 || !(id_z < Z_N))
    {
        wind[0] = 0;
        wind[1] = 0;
        wind[2] = 0;
        return;
    }

#ifdef ENV_WIND_WEIBULL
    // wind[0] = winddata_X[int(ceil((pos[0]+5)*4))][int(ceil((pos[1]+5)*4)+12)][int(ceil((pos[2])*4)+10)]+(*weibull_dist_wind[0])(*random_generator);
    // wind[1] = winddata_Y[int(ceil((pos[0]+5)*4))][int(ceil((pos[1]+5)*4)+12)][int(ceil((pos[2])*4)+10)]+(*weibull_dist_wind[1])(*random_generator);
    // wind[2] = winddata_Z[int(ceil((pos[0]+5)*4))][int(ceil((pos[1]+5)*4)+12)][int(ceil((pos[2])*4)+10)]+(*weibull_dist_wind[2])(*random_generator);
    // printf("SimEnvInfo: generated wind = [ %f, %f, %f ]\n", wind[0], wind[1], wind[2]);
    wind[0] = 1.5+(*weibull_dist_wind[0])(*random_generator);
    wind[1] = 0.0+(*weibull_dist_wind[1])(*random_generator);
    wind[2] = 0.0+(*weibull_dist_wind[2])(*random_generator);
#endif

#ifdef ENV_WIND_CN

    //ToDo Scene Related Param
    //Office
    /*
    wind[0] = 0.5 * winddata_X[id_x][id_y][id_z] + colored_noise(&wind_cn_params, &wind_cn_state[0], dt) + 0.0;
    wind[1] = 0.5 * winddata_Y[id_x][id_y][id_z] + colored_noise(&wind_cn_params, &wind_cn_state[1], dt) + 0.0;
    wind[2] = 0.5 * winddata_Z[id_x][id_y][id_z] + colored_noise(&wind_cn_params, &wind_cn_state[2], dt) + 0.0;
    */

    //Wind Tunnel
    /*
    wind[0] = winddata_X[id_x][id_y][id_z];
    wind[1] = winddata_Y[id_x][id_y][id_z];
    wind[2] = winddata_Z[id_x][id_y][id_z];
    */

    //Buildings
    wind[0] = winddata_X[id_x][id_y][id_z] / 5;
    wind[1] = winddata_Y[id_x][id_y][id_z] / 5;
    wind[2] = winddata_Z[id_x][id_y][id_z] / 5;

    //Wind from left to right
    //wind[0] = 0.2;
    //wind[1] = 0;
    //wind[2] = 0;

    //wind[0] = colored_noise(&wind_cn_params, &wind_cn_state[0], dt) + 1.5;
    //wind[1] = colored_noise(&wind_cn_params, &wind_cn_state[1], dt) + 0.0;
    //wind[2] = colored_noise(&wind_cn_params, &wind_cn_state[2], dt) + 0.0;

    //printf("SimEnvInfo: generated wind = [ %f, %f, %f ]\n", wind[0], wind[1], wind[2]);
#endif
}

void SimEnvInfo::destroy(void)
{
#ifdef ENV_WIND_WEIBULL
    for (int i = 0; i < 3; i++)
        delete weibull_dist_wind[i];
    delete random_generator;
#endif

#ifdef ENV_WIND_CN
    free(wind_cn_state);
#endif
}
