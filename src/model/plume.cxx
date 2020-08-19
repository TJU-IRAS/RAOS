/*
 * plume model
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-26 create this file (RAOS)
 */
#include <cmath>
#include <vector>
#include <cstring>
#include <iostream>
#include <sstream>
#include <ctime> // for random seed
#include "model/plume.h"
#include "model/environment.h"
#include "SimConfig.h"
#include "ziggurat.h" // for normal distribution number
#include "SimModel.h"

#include <highfive/H5File.hpp>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>

unsigned char cube_obstacle_judge(float* pos);

/*============== Filament Model ============*/
#if defined(USE_FILAMENT_MODEL)
typedef struct
{
    float source_pos[3]; // position of the source
    double pps; // parcels released per second
    double mpp; // molecules per parcel
    float lambda; // N(mu, lambda), vm
} PlumeConfig_t;

class FilaModel
{
public:
    std::vector<FilaState_t> state; // state of fila, a list
    PlumeConfig_t config; // plume model settings
    SimEnvInfo *PL_env_info;

    /* parameters for ziggurat method */
    float fn[128];
    unsigned int kn[128];
    float wn[128];
    unsigned int seed;

    void init(SimEnvInfo *sim_env_info) // plume initialization
    {
        PL_env_info = sim_env_info;

        // init settings
        SimConfig_t *sim_config = SimConfig_get_configs();
        config.pps = sim_config->source.pps;
        config.mpp = sim_config->source.mpp;
        config.source_pos[0] = sim_config->source.x;
        config.source_pos[1] = sim_config->source.y;
        config.source_pos[2] = sim_config->source.z;
        config.lambda = sim_config->plume.lambda;

        /* init fila state */
        state.reserve(MAX_NUM_PUFFS + 10); // make room for MAX_NUM_PUFFS fila
        /* release first odor pack at source */
        fila_release();

        /* clear number of fila need to release */
        fila_num_need_release = 0.0;

        /* setup ziggurat method to generate normal distribution numbers */
        r4_nor_setup(kn, fn, wn);
        seed = time(NULL);
    }

    void update(SimState_t *sim_state) // update fila
    {
#ifdef RAOS_FEATURE_WAKES
        if (!sim_state->wake_initialized) return;
#endif

        float wind[3];
        float vm_x, vm_y, vm_z;

        /* Step 1: update positions of fila */
        for (unsigned int i = 0; i < state.size(); i++) // for each fila
        {
            // get wind
            PL_env_info->measure_wind(state.at(i).pos, wind);
            // calculate centerline relative dispersion
            /*
            vm_x = (drand48() - 0.5);
            vm_y = (drand48() - 0.5);
            vm_z = (drand48() - 0.5);
            */
            vm_x = 1.8 * r4_nor(seed, kn, fn, wn);
            vm_y = 1.8 * r4_nor(seed, kn, fn, wn);
            vm_z = 1.8 * r4_nor(seed, kn, fn, wn);
            // vm_x = 0; vm_y = 0; vm_z = 0;
            // calculate pos increment
            float last_pos[3] = {0};
            memcpy(reinterpret_cast<float*>(last_pos), state.at(i).pos, 3 * sizeof(float));

            // here the integrated velocity contains three parts:
            //  wind[*]: the free stream wind field
            //  vm_*: the random noise wind speed
            //  state.at(i).vel[*]: the induced velocity (induced by wakes, updated in WakesIndVelatPlumePuffsUpdate)
            state.at(i).pos[0] += (wind[0] + vm_x + state.at(i).vel[0]) * sim_state->dt;
            state.at(i).pos[1] += (wind[1] + vm_y + state.at(i).vel[1]) * sim_state->dt;
            state.at(i).pos[2] += (wind[2] + vm_z + state.at(i).vel[2]) * sim_state->dt;
            if (state.at(i).pos[2] < 0.0)
                state.at(i).pos[2] = -state.at(i).pos[2];

            /*
            if ( i == 0 )
            {
                std::cout << state.at(i).pos[0] << " " << state.at(i).pos[1] << " " << state.at(i).pos[2] << std::endl;
            }
            */

            // obstacle judgement
            while ( cube_obstacle_judge(state.at(i).pos) == 0x07 )
            {
                // In cube obstacle
                unsigned char ret = cube_obstacle_judge(last_pos);
                if ( (ret & static_cast<unsigned char>(1 << 0)) == 0 )
                {
                    // X collision
                    state.at(i).pos[0] -= (wind[0] + vm_x + state.at(i).vel[0]) * sim_state->dt;
                    state.at(i).pos[0] -= (wind[0] + vm_x + state.at(i).vel[0]) * sim_state->dt;
                    //state.at(i).pos[1] += 0.5;
                }
                if ( (ret & static_cast<unsigned char>(1 << 1)) == 0 )
                {
                    // Y collision
                    state.at(i).pos[1] -= (wind[1] + vm_y + state.at(i).vel[1]) * sim_state->dt;
                    state.at(i).pos[1] -= (wind[1] + vm_y + state.at(i).vel[1]) * sim_state->dt;
                }
                if ( (ret & static_cast<unsigned char>(1 << 2)) == 0 )
                {
                    // Z collision
                    state.at(i).pos[2] -= (wind[2] + vm_y + state.at(i).vel[2]) * sim_state->dt;
                    state.at(i).pos[2] -= (wind[2] + vm_y + state.at(i).vel[2]) * sim_state->dt;
                }
            }

        }
        /* Step 2: update sizes of fila */
        for (unsigned int i = 0; i < state.size(); i++) // for each fila
        {
            if (state.at(i).r < 0.10)
            {
                //Scene Related Param
                //Wind Tunnel
                //state.at(i).r += 0.0001 / 2;
                //Office like
                state.at(i).r += 0.0001;
            }
            //state.at(i).r += 0.00001;
        }
        /* Step 3: fila maintainance */
        fila_num_need_release += config.pps * sim_state->dt;
        // remove fila which moved outside sim area
        SimConfig_t *configs = SimConfig_get_configs(); // get runtime configs
        int n = state.size(), i = 0;
        bool moved_outside = false;
        while (i != n)
        {
            if (std::abs(state.at(i).pos[0]) > configs->arena.w / 2.0 or
                std::abs(state.at(i).pos[1]) > configs->arena.l / 2.0 or
                //std::abs(state.at(i).pos[2]) > configs->arena.h/2.0
                std::abs(state.at(i).pos[2]) > configs->arena.h)
                moved_outside = true;
            if (moved_outside == true)
            {
                state.erase(state.begin() + i);
                n--;
                moved_outside = false;
            } else
                i++;
        }
        // release & remove fila
        while (fila_num_need_release >= 1.0)
        {
            fila_release();
            fila_num_need_release -= 1.0;
            if (state.size() > MAX_NUM_PUFFS)
                state.erase(state.begin());
        }
    }

private:
    float fila_num_need_release; // "buffer" fila
    void fila_release(void) // release a filament (puff)
    {
        FilaState_t new_fila;
        new_fila.pos[0] = config.source_pos[0];
        new_fila.pos[1] = config.source_pos[1];
        new_fila.pos[2] = config.source_pos[2];
        new_fila.vel[0] = 0;
        new_fila.vel[1] = 0;
        new_fila.vel[2] = 0;
        new_fila.r = 0.015;
        //new_fila.r = 0.01;
        state.push_back(new_fila);
    }
};

#endif

/*==============   Plume Model  ============*/
#if defined(USE_FILAMENT_MODEL)
FilaModel *fila = NULL;
#endif

void plume_init(SimEnvInfo *sim_env_info)
{
#if defined(USE_FILAMENT_MODEL)
    fila = new FilaModel();
    fila->init(sim_env_info);
#endif
}

void plume_update(SimState_t *sim_state)
{
#if defined(USE_FILAMENT_MODEL)
    fila->update(sim_state);
#endif
}

void plume_destroy(void)
{
#if defined(USE_FILAMENT_MODEL)
    if (fila)
    {
        fila->state.clear();
        std::vector<FilaState_t>().swap(fila->state);
        delete fila;
        fila = NULL;
    }
#endif
}

#if defined(USE_FILAMENT_MODEL)

std::vector<FilaState_t> *plume_get_fila_state(void)
{
    return &fila->state;
}

void save_fila_induced_vel_field()
{
    return ;
    std::cout << "Begin save plume induced velocity field..." << std::endl;
    using namespace HighFive;
    File file("PlumeIndVel.h5", File::ReadWrite | File::Create);

    static unsigned int save_idx = 0;

    std::stringstream dataset_ss;
    dataset_ss << "PlumeFrame" << save_idx ++;
    std::string dataset_name = dataset_ss.str();

    unsigned int fila_vec_len = fila->state.size();
    std::cout << "Dataset name " << dataset_ss.str() << " len " << fila_vec_len << std::endl;

    std::vector<size_t> dims(2);
    dims[0] = fila_vec_len;
    dims[1] = 6;
    DataSet dataset =
        file.createDataSet<float>(dataset_name, DataSpace(dims));

    std::vector<std::vector<float>> plume_ind_vel;
    for ( auto iter : fila->state)
    {
        std::vector<float> data_vec;
        for ( unsigned int idx=0; idx < sizeof(iter.pos) / sizeof(iter.pos[0]); idx ++ )
        {
            data_vec.emplace_back(iter.pos[idx]);
        }
        for ( unsigned int idx=0; idx < sizeof(iter.vel) / sizeof(iter.vel[0]); idx ++ )
        {
            data_vec.emplace_back(iter.vel[idx]);
        }
        plume_ind_vel.emplace_back(data_vec);
    }
    dataset.write(plume_ind_vel);
    std::cout << "Finish save plume induced velocity field..." << std::endl;
}

unsigned char cube_obstacle_judge(float* pos)
{
    //return 0x00;    // Disable obstacle judgement

    float x_range[3][2] = {
        {9.0f, 11.0f}, {14.0f, 16.0f}, {19.0f, 21.0f}
    };

    float y_range[3][2] = {
        {9.0f, 11.0f}, {14.0f, 16.0f}, {19.0f, 21.0f}
    };

    float z_range[1][2] = {{0.0f, 4.0f}};

    float x_offset = 15.0f;
    float y_offset = 15.0f;
    float z_offset = 0.0f;

    bool x_in_range = false;
    for (unsigned char idx = 0; idx < sizeof(x_range) / sizeof(x_range[0]); idx++)
    {
        float cur_x = pos[0] + x_offset;
        if ( cur_x >= x_range[idx][0] && cur_x <= x_range[idx][1] )
        {
            x_in_range = true;
            break;
        }
    }

    bool y_in_range = false;
    for (unsigned char idx = 0; idx < sizeof(y_range) / sizeof(y_range[0]); idx++)
    {
        float cur_y = pos[1] + y_offset;
        if ( cur_y >= y_range[idx][0] && cur_y <= y_range[idx][1] )
        {
            y_in_range = true;
            break;
        }
    }

    bool z_in_range = false;
    for (unsigned char idx = 0; idx < sizeof(z_range) / sizeof(z_range[0]); idx++)
    {
        float cur_z = pos[2] + z_offset;
        if ( cur_z >= z_range[idx][0] && cur_z <= z_range[idx][1] )
        {
            z_in_range = true;
            break;
        }
    }

    unsigned char ret = 0;
    unsigned char x_ret = x_in_range ? static_cast<unsigned char>(1 << 0) : 0 ;
    unsigned char y_ret = y_in_range ? static_cast<unsigned char>(1 << 1) : 0 ;
    unsigned char z_ret = z_in_range ? static_cast<unsigned char>(1 << 2) : 0 ;
    ret += x_ret;
    ret += y_ret;
    ret += z_ret;

    if ( ret == 0x07 )
    {
        //std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    }

    return ret;
}

#endif

/* End of plume.cxx */
