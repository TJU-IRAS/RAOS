/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : simulated_annealing_method.cpp
   Author : tao.jing
   Date   : 2020/1/11
   Brief  :
**************************************************************************/
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "model/robot.h"
#include "model/SimModel.h"
#include "simulated_annealing_method.h"

simulated_annealing_method* simulated_annealing_method::instance()
{
    static simulated_annealing_method inst;
    return &inst;
}

simulated_annealing_method::simulated_annealing_method()
{

}

void simulated_annealing_method::init()
{
    temperature = 1E6;
    exit_temperature = 1E-6;
    cool_coef = 0.9f;
    conc_threshold = 0.1f;

    x_pos_probability = 0.5;
    y_pos_probability = 0.5;
    z_pos_probability = 0.5;

    last_conc = 0.0f;
    cur_conc = 0.0f;

    update_count = 0;
    accept_count = 0;
    reject_count = 0;
    exit_count   = 0;

    max_exit_count = 30;

    refind_plume_num = 3;

    finding_plume = true;
    sa_moving = false;

    current_pos = inner_point_t(2, -8, 2.5);
    //current_pos = inner_point_t(0, 2, 2.5);
    next_pos = current_pos;

    sample_count_for_one_position = 36 * 4;
    circle_sample_count = 36;
    float theta_range[2] = {0, 2 * M_PI};
    theta_delta = (theta_range[1] - theta_range[0]) / static_cast<float>(circle_sample_count);
    max_direction_theta = 0.0f;

    in_local_maximum = false;

    exit_search = false;

    recent_record_point_num = 5;

    memset(x_path_history, 0, sizeof(x_path_history));
    memset(y_path_history, 0, sizeof(y_path_history));
    memset(z_path_history, 0, sizeof(z_path_history));
    memset(path_history, 0, sizeof(path_history));
    memset(conc_array, 0, sizeof(conc_array));
}

void simulated_annealing_method::update(SimState_t* sim_state)
{
    Robot* robot = (SimModel_get_robots())->at(0);
    if ( exit_search )
    {
        return;
    }

    //Plume finding
    if (finding_plume)
    {
        //Sample concentration at present position
        //cur_conc = robot->state.gas_sensor;
        robot->center_tdlas_sensor.sensor.offset = inner_point_t(0, 0, 2.5);
        cur_conc = robot->center_tdlas_sensor.sensor.get_current_tdlas_reading();
        last_conc = cur_conc;
        if ( cur_conc > conc_threshold )
        {
            std::cout << "[SA] Found plume, conc " << cur_conc << std::endl;
            finding_plume = false;
            sa_moving = true;
            generate_next_pos(e_searching);
        }
        else
        {
            //std::cout << "[SA] Next pos x " << next_pos.x << " y " << next_pos.y << " z " << next_pos.z << std::endl;
            bool target_arrived = update_pos_towards_next_pos(sim_state);
            if ( target_arrived )
            {
                current_pos = next_pos;
                search_path_trajectory.emplace_back(current_pos);
                search_path_type.emplace_back(e_finding_plume);
                search_conc_trajectory.emplace_back(robot->state.gas_sensor);
                generate_next_pos(e_finding_plume);
            }
        }
        return;
    }

    //SA
    if ( temperature < exit_temperature )
    {
        std::cout << "[SA] Temperature lower than exit condition, exit SA loop." << std::endl;
        return;
    }

    if ( sa_moving )
    {
        // Wait to reach next target position
        bool target_arrived = update_pos_towards_next_pos(sim_state);
        if ( target_arrived )
        {
            inner_point_t tmp_pos = current_pos;
            current_pos = next_pos;
            next_pos = tmp_pos;
            sa_moving = false;
        }
    }
    else
    {
        // Hovering for sampling
        static unsigned int sample_count = 0;
        //cur_conc = (cur_conc * sample_count + robot->state.gas_sensor) / ( sample_count + 1 );
        //cur_conc = robot->center_tdlas_sensor.sensor.get_current_tdlas_reading();
        if ( sample_count < sample_count_for_one_position )
        {
            sample_count ++;

            const float radius = 1.0f;

            static unsigned int theta_idx = 0;
            float theta = static_cast<float>(theta_idx) * theta_delta;

            float x_offset = radius * static_cast<float>(cos(theta));
            float y_offset = radius * static_cast<float>(sin(theta));

            robot->center_tdlas_sensor.sensor.offset = inner_point_t(x_offset, y_offset, 2.5);

            unsigned int one_theta_scan_time = sample_count_for_one_position / circle_sample_count;
            static unsigned int one_theta_scan_idx = 0;
            one_theta_scan_idx ++;
            if ( one_theta_scan_idx >= one_theta_scan_time )
            {
                conc_array[theta_idx] = robot->center_tdlas_sensor.sensor.get_current_tdlas_reading();
                //inner_point_t end_point = current_pos+ inner_point_t(x_offset, y_offset, 0);
                //inner_point_t middle_point = current_pos+ inner_point_t(x_offset/2, y_offset/2, 0);
                //conc_array[theta_idx] =
                //    robot->center_tdlas_sensor.sensor.get_simple_conc_evaluation(end_point, middle_point, 0.1f, 7.0f);
                //std::cout << "Current tdlas conc: " << conc_array[theta_idx] << std::endl;

                // Get max conc and direction
                float max_conc = 0.0f;
                unsigned int max_idx = 0;
                for (unsigned int i = 0; i < circle_sample_count; ++i)
                {
                    if (conc_array[i] > max_conc)
                    {
                        max_conc = conc_array[i];
                        max_idx = i;
                    }
                }
                max_direction_theta = static_cast<float>(max_idx) * theta_delta;
                float max_conc_x_offset = radius * static_cast<float>(cos(max_direction_theta));
                float max_conc_y_offset = radius * static_cast<float>(sin(max_direction_theta));
                robot->state.target_offset = inner_point_t(max_conc_x_offset, max_conc_y_offset, 0.0f);

                theta_idx ++;
                theta_idx = theta_idx >= circle_sample_count ? 0 : theta_idx;
                one_theta_scan_idx = 0;
            }
        }
        else
        {
            // Get max conc and direction
            float max_conc = 0.0f;
            unsigned int max_idx = 0;
            for (unsigned int i = 0; i < circle_sample_count; ++i)
            {
                if (conc_array[i] > max_conc)
                {
                    max_conc = conc_array[i];
                    max_idx = i;
                }
            }
            memset(conc_array, 0, sizeof(conc_array));

            cur_conc = max_conc;

            if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) < 0.99 )
            {
                max_direction_theta = static_cast<float>(max_idx) * theta_delta;
            }
            else
            {
                auto rand_idx = static_cast<unsigned int>( rand() / RAND_MAX * circle_sample_count );
                max_direction_theta = static_cast<float>(rand_idx) * theta_delta;
            }
            //std::cout << max_idx << " " << max_direction_theta << std::endl;
            // Finish hovering sampling
            sample_count = 0;
            search_path_trajectory.emplace_back(current_pos);
            if (!in_local_maximum)
            {
                search_path_type.emplace_back(e_leave_local_max);
            }
            else
            {
                search_path_type.emplace_back(e_searching);
            }
            search_conc_trajectory.emplace_back(max_conc);

            if ( recent_points.size() < recent_record_point_num )
            {
                recent_points.emplace_back(current_pos);
            }
            else
            {
                recent_points.pop_front();
                recent_points.emplace_back(current_pos);
            }

            if ( recent_points.size() == recent_record_point_num && judge_local_maximum() )
            {
                std::cout << "[SA] === Local maximum! === " << std::endl;
                recent_points.clear();
                in_local_maximum = true;
            }

            // Judge if stop condition satisfied
            bool reject = false;
            float conc_diff = last_conc - cur_conc;
            if ( judge_new_solution(conc_diff) && cur_conc > conc_threshold )
            {
                if ( FLOAT_EQUAL(conc_diff, 0.0f) )
                {
                    exit_count ++;
                }
                else
                {
                    exit_count = 1;
                }

                accept_count ++;
            }
            else
            {
                exit_count ++;
                reject_count ++;
                reject = true;
            }

            update_count++;
            temperature *= cool_coef;

            if ( exit_count > max_exit_count )
            {
                exit_search = true;
                std::cout << "[SA] SA search exit." << std::endl;
            }
            else
            {
                if ( !reject )
                {
                    //not reject: generate a new target position, or go back to last position
                    if (in_local_maximum)
                    {
                        generate_next_pos(e_leave_local_max);
                    }
                    else
                    {
                        generate_next_pos(e_searching);
                    }
                }
                else
                {
                    //generate_next_pos();
                    if (reject_count >= refind_plume_num)
                    {
                        finding_plume = true;
                        std::cout << "Find plume again..." << std::endl;
                    }
                }
                sa_moving = true;
                //std::cout << "[SA] next pos: " << next_pos.x << ' ' << next_pos.y << ' ' << next_pos.z << std::endl;
            }
        }
    }
}

void simulated_annealing_method::stop()
{
    std::cout << "[SA] SA Searching stop." << std::endl;
}

void simulated_annealing_method::generate_next_pos(next_pos_model next_pos_model)
{
    //next_pos = inner_point_t(3,8,5);
    //return ;

    float x_offset = 0.0f;
    float y_offset = 0.0f;
    float z_offset = 0.0f;
    float sign     = 1.0f;

    float x_max_offset = 0.5f;
    float y_max_offset = 0.5f;
    float z_max_offset = 0.5f;

    if (next_pos_model == e_finding_plume)
    {
        static unsigned int dir_idx = 0; // 0:x+ 1:y+ 2:x- 3:y-

        switch( dir_idx )
        {
            case 0:
                x_offset = -1.0f;
                y_offset = 1.0f;
                break;
            case 1:
                x_offset = 1.0f;
                y_offset = 1.0f;
                break;
            case 2:
                x_offset = 1.0f;
                y_offset = -1.0f;
                break;
            case 3:
                x_offset = -1.0f;
                y_offset = -1.0f;
                break;
        }

        inner_point_t target_pos = current_pos + inner_point_t(x_offset, y_offset, z_offset);
        target_pos.z = 3;
        if (judge_cube_obstacle(target_pos) == 0x07 || judge_near_boundary(target_pos))
        {
            dir_idx ++;
            dir_idx = dir_idx >= 4 ? 0 : dir_idx;
        }
    }
    else if ( next_pos_model == e_searching )
    {
        x_max_offset = 1.0f;
        y_max_offset = 1.0f;
        z_max_offset = 0.2f;

        x_offset = (0.6 + static_cast<float>(rand()) / RAND_MAX * x_max_offset) * static_cast<float>(cos(max_direction_theta));
        y_offset = (0.6 + static_cast<float>(rand()) / RAND_MAX * y_max_offset) * static_cast<float>(sin(max_direction_theta));
        sign = z_pos_probability > static_cast<float>(rand()) / RAND_MAX ? 1.0f : -1.0f;
        z_offset = static_cast<float>(rand()) / RAND_MAX * z_max_offset * sign;
    }
    else if ( next_pos_model == e_leave_local_max )
    {
        static unsigned int leave_local_count = 0;
        leave_local_count ++;
        if (leave_local_count >= 6)
        {
            in_local_maximum = false;
            leave_local_count = 0;
        }

        float signx = 0.5 > static_cast<float>(rand()) / RAND_MAX ? 1.0f : -1.0f;
        x_offset = 1.0f * signx;
        float signy = 0.8 > static_cast<float>(rand()) / RAND_MAX ? 1.0f : -1.0f;
        y_offset = 1.0f * signy;
    }

    inner_point_t last_pos = next_pos;
    next_pos = current_pos + inner_point_t(x_offset, y_offset, z_offset);
    next_pos.z = 4;

    // Judge if invalid position
    // X Y Z limit validation
    if (next_pos.x < -14.0f || next_pos.x > 14.0f)
    {
        next_pos.x += x_offset * -1.1f;
    }
    if (next_pos.y < -14.0f || next_pos.y > 14.0f)
    {
        next_pos.y += y_offset * -1.1f;
    }
    if (next_pos.z < 0.5 || next_pos.z > 8.5f)
    {
        next_pos.z += z_offset * -1.1f;
    }

    // buildings obstacle validation
    while (judge_cube_obstacle(next_pos) == 0x07 )
    {
        // In cube obstacle
        unsigned char ret = judge_cube_obstacle(last_pos);
        if ( (ret & static_cast<unsigned char>(1 << 0)) == 0 )
        {
            // X collision
            next_pos[0] += x_offset * -1.1f;
            next_pos[0] += x_offset * -1.1f;
        }
        if ( (ret & static_cast<unsigned char>(1 << 1)) == 0 )
        {
            // Y collision
            next_pos[1] += y_offset * -1.1f;
            next_pos[1] += y_offset * -1.1f;
        }
        if ( (ret & static_cast<unsigned char>(1 << 2)) == 0 )
        {
            // Z collision
            next_pos[2] += z_offset * -1.1f;
            next_pos[2] += z_offset * -1.1f;
        }
    }
}

bool simulated_annealing_method::judge_new_solution(float diff_conc)
{
    if ( diff_conc < 0 )
    {
        // New concentration is higher than last concentration, accept
        return true;
    }
    else
    {
        // New concentration is higher than last concentration, accept in probability
        float d = exp(-1 * (1E4 * diff_conc / temperature));
        if ( d > rand() / RAND_MAX )
        {
            //accept worse solution
            return true;
            //return false;
        }
        else
        {
            return false;
        }
    }
}

void simulated_annealing_method::update_dir_probability()
{
    unsigned int x_idx = int((current_pos.x + 15.0f) / 0.2f );
    unsigned int y_idx = int((current_pos.y + 15.0f) / 0.2f );
    path_history[x_idx][y_idx] ++;

    unsigned long total_count = 0;
    unsigned long n1=0, n2=0, n3=0, n4=0;
    for (unsigned int x_iter = 0; x_iter < 150; x_iter ++)
    {
        for (unsigned int y_iter = 0; y_iter < 150; y_iter ++)
        {
            x_idx = 75;
            y_idx = 75;
            n1 += x_iter > x_idx && y_iter > y_idx ? path_history[x_iter][y_iter] : 0;
            n2 += x_iter <= x_idx && y_iter > y_idx ? path_history[x_iter][y_iter] : 0;
            n3 += x_iter <= x_idx && y_iter <= y_idx ? path_history[x_iter][y_iter] : 0;
            n4 += x_iter > x_idx && y_iter <= y_idx ? path_history[x_iter][y_iter] : 0;
        }
    }
    total_count = n1 + n2 + n3 + n4;
    unsigned long x_pos_count = n1 + n4;
    unsigned long y_pos_count = n1 + n2;
    x_pos_probability = 1 - (static_cast<float>(x_pos_count) / total_count);
    y_pos_probability = 1 - (static_cast<float>(y_pos_count) / total_count);

    /*
    std::cout << "Probability " << x_pos_probability << " "
              << y_pos_probability << " "
              << z_pos_probability << std::endl;
    */
    return ;
}

bool simulated_annealing_method::judge_local_maximum()
{
    float x_max = 0.0f;
    float x_min = 1E6;
    float y_max = 0.0f;
    float y_min = 1E6;
    for ( auto iter = recent_points.begin(); iter != recent_points.end(); ++iter )
    {
        float x = iter->x;
        float y = iter->y;
        x_max = x > x_max ? x : x_max;
        x_min = x_min > x ? x : x_min;
        y_max = y > y_max ? y : y_max;
        y_min = y_min > y ? y : y_min;
    }

    float dist = (x_max - x_min) * (x_max - x_min) + (y_max - y_min) * (y_max - y_min);
    std::cout << dist << std::endl;
    static unsigned int in_range_times = 0;
    if ( dist < 21.0f )
    {
        in_range_times ++;
    }
    else
    {
        in_range_times = 0;
    }
    if( in_range_times >= 3 )
    {
        in_range_times = 0;
        return true;
    }
    return false;
}

bool simulated_annealing_method::update_pos_towards_next_pos(SimState_t* sim_state)
{
    Robot* robot = (SimModel_get_robots())->at(0);

    // Get robot's position
    float vel_nrm = 1.0;
    float *ref_pos = &(robot->ref_state.pos[0]);
    float dist[3];
    float ref_vel[3] = {0};
    // calculate the direction to fly
    for (int i = 0; i < 3; i++)
        dist[i] = next_pos[i] - ref_pos[i];
    double dist_nrm = sqrt(dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2]);
    for (int i = 0; i < 3; i++)
    {
        if (fabs(dist[i]) < 0.001)
            ref_vel[i] = 0;
        else
            ref_vel[i] = float(dist[i]) * vel_nrm / dist_nrm;
    }
    // update robot's position
    for (int i = 0; i < 3; i++)
        ref_pos[i] += ref_vel[i] * sim_state->dt;

    if (fabs(dist[0]) < 0.1 && fabs(dist[1]) < 0.1 && fabs(dist[2]) < 0.1)
    {
        current_pos = next_pos;
        update_dir_probability();
        return true;
    }
    return false;
}

unsigned char simulated_annealing_method::judge_cube_obstacle(inner_point_t pos)
{
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

bool simulated_annealing_method::judge_near_boundary(inner_point_t pos)
{
    if ( pos.x > 13.5f || pos.x < -13.5f || pos.y > 13.5f || pos.y < -13.5f )
    {
        return true;
    }
    return false;
}