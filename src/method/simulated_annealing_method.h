/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : simulated_annealing_method.h
   Author : tao.jing
   Date   : 2020/1/11
   Brief  : 
**************************************************************************/
#ifndef RAOS_SIMULATED_ANNEALING_METHOD_H
#define RAOS_SIMULATED_ANNEALING_METHOD_H

#include <map>
#include <vector>
#include <list>
#include "common/math/raos_math.h"
#include "model/SimModel.h"

enum next_pos_model
{
    e_finding_plume   = 0,
    e_searching       = 1,
    e_leave_local_max = 2,
};

class Robot;
class simulated_annealing_method
{
public:
    static simulated_annealing_method* instance();

private:
    simulated_annealing_method();

public:
    void init();
    void update(SimState_t* sim_state);
    void stop();

    void generate_next_pos(next_pos_model next_pos_model = e_searching);
    bool judge_new_solution(float diff_conc);
    void update_dir_probability();

    bool judge_local_maximum();

    // Update UAV pos towards next position, true-arrived, false-not arrived
    bool update_pos_towards_next_pos(SimState_t* sim_state);

    unsigned char judge_cube_obstacle(inner_point_t pos);
    bool judge_near_boundary(inner_point_t pos);

public:
    float temperature;
    float exit_temperature;
    float cool_coef;
    float conc_threshold;

    float x_pos_probability;
    float y_pos_probability;
    float z_pos_probability;

    float last_conc;
    float cur_conc;
    float conc_array[128];

    unsigned long update_count;
    unsigned long accept_count;
    unsigned long reject_count;
    unsigned long exit_count;
    unsigned long max_exit_count;

    unsigned long refind_plume_num;

    inner_point_t current_pos;
    inner_point_t next_pos;

    inner_point_t start_pos;

    std::vector<inner_point_t>     search_path_trajectory; //Search history info - path
    std::vector<float>             search_conc_trajectory; //Search history info - conc
    std::vector<unsigned int>      search_path_type; // 0-search 1-finding plume 2-leave local

    unsigned int recent_record_point_num;
    std::list<inner_point_t> recent_points;

    unsigned int x_path_history[150];
    unsigned int y_path_history[150];
    unsigned int z_path_history[50];

    unsigned int path_history[150][150];

    bool finding_plume;
    bool sa_moving;
    unsigned int sample_count_for_one_position;    //Sample counts at one target position
    unsigned int circle_sample_count;
    float theta_delta;
    float max_direction_theta;

    bool in_local_maximum;

    bool exit_search;
};

#endif //RAOS_SIMULATED_ANNEALING_METHOD_H
