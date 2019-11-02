#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <random>

typedef struct
{
    float width;
    float length;
    float height;
} Sim_Env_Size;

#ifndef ENV_WIND_CN
#define ENV_WIND_CN
#endif


// #ifndef ENV_WIND_WEIBULL
//   #define ENV_WIND_WEIBULL
// #endif

#ifdef ENV_WIND_CN
typedef struct
{
    float damping;
    float bandwidth;
    float G;
} Wind_Colored_Noise_Params_t;

typedef struct
{
    float x[2];
} Wind_Colored_Noise_State_t;
#endif

class SimEnvInfo
{
public:
    // init settings
    //SimConfig_t* sim_config = SimConfig_get_configs();
    Sim_Env_Size Env_Size;

    SimEnvInfo(float);

    void measure_wind(float *, float *);

    void destroy(void);

private:
    float dt;
#ifdef ENV_WIND_WEIBULL
    std::default_random_engine* random_generator;
    std::weibull_distribution<float>* weibull_dist_wind[3];
#endif
#ifdef ENV_WIND_CN
    Wind_Colored_Noise_Params_t wind_cn_params;
    Wind_Colored_Noise_State_t *wind_cn_state;
#endif
};

#endif
