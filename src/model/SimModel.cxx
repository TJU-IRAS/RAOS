/*
 * Models of RAOS
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-23 create this file (RAOS)
 */
#include <iostream>
#include <vector>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include "method/simulated_annealing_method.h"
#include "model/robot.h"
#include "model/plume.h"
#include "model/virtual_plume.h"


#ifdef RAOS_FEATURE_WAKES
#include "model/wake.h"
#endif

#include "model/SimModel.h"
#include "method/method.h"
#include "SimConfig.h"
#include <cmath>
#include <stdio.h>

#include "model/windvector.h"

#include <highfive/H5File.hpp>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>

//#include "ui/draw/draw_windvector.h"

/* Pointers of instances of RAOS models */
static std::vector<Robot *> robots; // pointer array of robot instances

static SimState_t sim_state;

static SimEnvInfo *sim_env_info;

#define ESTABLISH_WAKE_DT_DIVIDE 18.0

//#define METHOD_HOVER

extern long update_time;

void SimModel_init(void)
{
    SimConfig_t *configs = SimConfig_get_configs(); // get runtime configs

    /* init timing */
    sim_state.time = 0.0;
#ifdef RAOS_FEATURE_WAKES
    sim_state.dt = configs->common.dt / ESTABLISH_WAKE_DT_DIVIDE;
    sim_state.wake_initialized = false;
#else
    sim_state.dt = configs->common.dt; // 50 Hz
#endif

    /* create environment class */
    sim_env_info = new SimEnvInfo(configs->common.dt);

    /* Create wind vector display */
    //WindVector_Init(configs->arena.w,configs->arena.l, 2.0 , sim_env_info);

    /* create & init robot */
    Robot *new_robot = new Robot("quadrotor", "Micro Bee", "PID", configs->common.dt, sim_env_info);
    //Robot *new_robot = new Robot("groundrobot", "Micro Bee", "PID", configs->common.dt, sim_env_info);
    robots.push_back(new_robot);

    /* init plume */
    plume_init(sim_env_info);

#ifdef RAOS_FEATURE_WAKES
    /* init parallelization of rotor wakes computation */
    WakesInit(&robots);

    //init windvector
    windget_init(sim_env_info, robots.at(0)->state.pos);
    /* init para... of wakes-induced velocity at puffs */
    WakesIndVelatPlumePuffsInit();
    //init para... of wakes-induced velocity at adjacent cube
    WakesInvVelatAdjacentCubeInit();
    virtual_plume::instance()->init();
#endif

    /* init method */
#ifdef METHOD_HOVER
    method_init(METHOD_HOVER_MEASURE);
#else
    //method_init(METHOD_GAS_DIST_MAPPING);  // gas distribution mapping
    method_init(METHOD_SIMULATED_ANNEALING); // simulated annealing source seaching
#endif

}

void SimModel_update(void)
{
    /* update strategy */
#ifdef RAOS_FEATURE_WAKES
    if (sim_state.wake_initialized)
#endif
    {
        method_update(&sim_state);
    }

    /* update robot */
    for (unsigned int i = 0; i < robots.size(); i++)
        robots.at(i)->update();
#ifdef RAOS_FEATURE_WAKES
    /* update rotor wakes */
    WakesUpdate(&robots, "Euler", &sim_state, sim_env_info);

    /* update plume */
    WakesIndVelatPlumePuffsUpdate(&robots, plume_get_fila_state());

    //update windvector's Pos
    windget_updatePos(robots.at(0)->state.pos);
    /* update windvector */

#ifdef GET_WIND_BY_CUDA
    WakesIndVelatPlumePuffsUpdate(&robots, wind_get_vector_state());
#endif

    //save induced velocity field around the robot
    //std::cout << virtual_plume::instance()->plumes.size() << std::endl;

    //auto start_time = std::chrono::high_resolution_clock::now();
    WakesInvVelatAdjacentCubeUpdate(&robots, &(virtual_plume::instance()->plumes));
    //auto end_time = std::chrono::high_resolution_clock::now();
    //auto diff = end_time - start_time;
    //std::chrono::microseconds counter = std::chrono::duration_cast<std::chrono::microseconds>(diff);
    //std::cout << "Calculation time consumption: " << counter.count() / 1000.0f << " ms." << std::endl;
    virtual_plume::instance()->update_rotor_info(&robots);
    //virtual_plume::instance()->save_virtual_plume_info();
#endif
    update_time++;
    windget_updateVel();

    /* debug */
    /*
    std::vector<FilaState_t>* plume = plume_get_fila_state();
    printf("v_z = %f, size_m = %d\n", plume->back().vel[2], robots.at(0)->wakes.at(0)->wake_state[0]->size());
    */

    plume_update(&sim_state);
    //save_fila_induced_vel_field();

    /* calculate concentration at robot's position */
    std::vector<FilaState_t> *puffs = plume_get_fila_state();
    //Refer to Roice thesis 2.2.2, sigma represents the distortion matrix for odor package
    float inv_sigma[3] = {1.42857, 9.34579, 9.34579};
    float delta[3];
    float power;
    float conc = 0.0;
    for (unsigned int i = 0; i < puffs->size(); i++)
    {
        power = 0.0;
        // get delta
        for (int k = 0; k < 3; k++)
            delta[k] = robots.at(0)->state.pos[k] - puffs->at(i).pos[k];
        for (int k = 0; k < 3; k++)
            power += delta[k] * inv_sigma[k] * delta[k];
        // calculate delta^T*sigma*delta
        conc += 1.0 / 1.40995 * exp(-power);
    }
    //conc *= 50; // Q --> 50Q
    robots.at(0)->state.gas_sensor = conc;
    //std::cout << "Current concentration at robot position: " << conc << std::endl;

    double mox_reading = robots.at(0)->center_mox_sensor.sensor.get_current_mox_reading(conc);
    double pid_reading = robots.at(0)->center_pid_sensor.sensor.get_current_pid_reading(conc);

    // Update tdlas scan start point and end point according to its scan model
    robots.at(0)->center_tdlas_sensor.sensor.update_tdlas_scan(inner_point_t(robots.at(0)->state.pos),
                                                               inner_point_t(robots.at(0)->center_tdlas_sensor.pos));
    /*
    robots.at(0)->center_tdlas_sensor.sensor.start_point[0] = robots.at(0)->state.pos[0]
        + robots.at(0)->state.tdlas_ref_start_point[0];
    robots.at(0)->center_tdlas_sensor.sensor.start_point[1] = robots.at(0)->state.pos[1]
        + robots.at(0)->state.tdlas_ref_start_point[1];
    robots.at(0)->center_tdlas_sensor.sensor.start_point[2] = robots.at(0)->state.pos[2]
        + robots.at(0)->state.tdlas_ref_start_point[2];
    robots.at(0)->center_tdlas_sensor.sensor.end_point[0] = robots.at(0)->state.pos[0]
        + robots.at(0)->state.tdlas_ref_end_point[0];
    robots.at(0)->center_tdlas_sensor.sensor.end_point[1] = robots.at(0)->state.pos[1]
        + robots.at(0)->state.tdlas_ref_end_point[1];
    robots.at(0)->center_tdlas_sensor.sensor.end_point[2] = robots.at(0)->state.pos[2]
        + robots.at(0)->state.tdlas_ref_end_point[2];
    */
    double tdlas_reading = robots.at(0)->center_tdlas_sensor.sensor.get_current_tdlas_reading();
    // Update tdlas start and end points for drawing
    robots.at(0)->state.tdlas_ref_start_point = inner_point_t(robots.at(0)->center_tdlas_sensor.pos);
    robots.at(0)->state.tdlas_ref_end_point = inner_point_t(robots.at(0)->center_tdlas_sensor.sensor.end_point) -
                                              inner_point_t(robots.at(0)->state.pos);

    {
        //update mox sensor reading
        extern void update_mox_conc_disp(float conc);
        extern void update_pid_conc_disp(float conc);
        extern void update_tdlas_conc_disp(float conc);
        update_mox_conc_disp(mox_reading);
        update_pid_conc_disp(pid_reading);
        update_tdlas_conc_disp(tdlas_reading);
    }

    /* update timing */
    sim_state.time += sim_state.dt;
    //usleep(1000 * 5);
#ifdef RAOS_FEATURE_WAKES
    if (sim_state.time > 0.5 && sim_state.wake_initialized == false)
    {
        sim_state.wake_initialized = true;
        sim_state.dt = ESTABLISH_WAKE_DT_DIVIDE * sim_state.dt;
    }
#endif
}

void SimModel_destroy(void)
{
#ifdef RAOS_FEATURE_WAKES
    // free memory of GPU computation
    WakesFinish();
#endif
    // destroy plumes
    plume_destroy();
    //destroy windget
    windget_destroy();
    // destroy robots
    if (robots.size())
    {
        for (unsigned int i = 0; i < robots.size(); i++)
        {
            robots.at(i)->destroy();
            delete robots.at(i);
        }
        robots.clear();
    }
    // free dynamic memory in sim_env_info
    sim_env_info->destroy();
    delete sim_env_info;
}

float get_certain_pos_conc(inner_point_t pos)
{
    /* calculate concentration at certain position */
    std::vector<FilaState_t> *puffs = plume_get_fila_state();
    //Refer to Roice thesis 2.2.2, sigma represents the distortion matrix for odor package
    float inv_sigma[3] = {1.42857, 9.34579, 9.34579};
    float delta[3];
    float power;
    float conc = 0.0;
    for (unsigned int i = 0; i < puffs->size(); i++)
    {
        power = 0.0;
        // get delta
        for (int k = 0; k < 3; k++)
            delta[k] = pos[k] - puffs->at(i).pos[k];
        for (int k = 0; k < 3; k++)
            power += delta[k] * inv_sigma[k] * delta[k];
        // calculate delta^T*sigma*delta
        conc += 1.0 / 1.40995 * exp(-power);
    }
    return conc;
}

void SimModel_savesnap()
{
    float x_range[2] = {-15.0f, 15.0f};
    float y_range[2] = {-15.0f, 15.0f};
    float z_range[2] = {0.0f, 10.0f};

    std::cout << "Begin save concentration snap" << std::endl;

    using namespace HighFive;
    File file("ConcSnap.h5", File::ReadWrite | File::Create | File::Truncate);
    //0.Save current conc map
    std::vector<size_t> dims(2);
    dims[0] = 150 + 1;
    dims[1] = 150 + 1;
    DataSet dataset =
        file.createDataSet<float>("Conc", DataSpace(dims));
    std::vector<std::vector<float>> conc_snap;
    for ( unsigned int x_idx = 0; x_idx < dims[0]; x_idx ++ )
    {
        std::vector<float> y_conc;
        for ( unsigned int y_idx = 0; y_idx < dims[1]; y_idx ++ )
        {
            inner_point_t pos;
            pos.x = x_idx * 0.2 + x_range[0];
            pos.y = y_idx * 0.2 + y_range[0];
            pos.z = 2.0f;
            float conc = get_certain_pos_conc(pos);
            y_conc.emplace_back(conc);
        }
        conc_snap.emplace_back(y_conc);
    }
    dataset.write(conc_snap);

    //1.Save conc history
    std::vector<size_t> dims_conc_history(1);
    dims_conc_history[0] = simulated_annealing_method::instance()->search_conc_trajectory.size();
    DataSet dataset_conc_history =
        file.createDataSet<float>("ConcHistory", DataSpace(dims_conc_history));
    dataset_conc_history.write(simulated_annealing_method::instance()->search_conc_trajectory);

    //2.Save searching path
    std::vector<size_t> dims_path(2);
    dims_path[0] = simulated_annealing_method::instance()->search_path_trajectory.size();
    dims_path[1] = 3;
    DataSet dataset_path =
        file.createDataSet<float>("Path", DataSpace(dims_path));
    std::vector<std::vector<float>> path_data;
    for ( unsigned int path_idx = 0; path_idx < dims_path[0]; path_idx ++ )
    {
        inner_point_t point = simulated_annealing_method::instance()->search_path_trajectory[path_idx];
        std::vector<float> pos_vec;
        for ( unsigned int pos_idx = 0; pos_idx < dims_path[1]; pos_idx ++ )
        {
            pos_vec.emplace_back(point[pos_idx]);
        }
        path_data.emplace_back(pos_vec);
    }
    dataset_path.write(path_data);

    //3.Save searching points type
    std::vector<size_t> dims_path_type(1);
    dims_path_type[0] = simulated_annealing_method::instance()->search_path_type.size();
    DataSet dataset_path_type =
        file.createDataSet<float>("PathType", DataSpace(dims_path_type));
    dataset_path_type.write(simulated_annealing_method::instance()->search_path_type);

    //3.Save start point
    std::vector<size_t> dims_start_point(1);
    dims_start_point[0] = 3;
    DataSet dataset_start_point =
        file.createDataSet<float>("StartPoint", DataSpace(dims_start_point));
    std::vector<float> start_pos;
    for ( unsigned int idx = 0; idx < 3; idx ++ )
    {
        start_pos.emplace_back(simulated_annealing_method::instance()->start_pos[idx]);
    }
    dataset_start_point.write(start_pos);

    std::cout << "Finish saving concentration snap" << std::endl;
}

SimState_t *SimModel_get_sim_state(void)
{
    return &sim_state;
}

// get the pointer to the robots
std::vector<Robot *> *SimModel_get_robots(void)
{
    return &robots;
}

/* End of SimModel.cxx */
