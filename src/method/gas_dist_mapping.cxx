/*
 * Gas Distribution Mapping
 *
 * Author: Roice (LUO Bing)
 * Date: 2017-03-16 create this file
 */
#include "model/robot.h"
#include "model/SimModel.h"
#include <cmath>
#include <stdio.h>
#include <vector>
#include <common/math/raos_math.h>

#define RouteMap1
//#define RouteMap2

class GasDistMapping
{
public:
    GasDistMapping(void);

    void waypoint_update(Robot *, SimState_t *);

private:
    std::vector<inner_point_t> waypoints;
    unsigned int next_waypoint_index;
};

GasDistMapping::GasDistMapping(void)
{
    //flow height
    float fly_height = 1.5; // m

#ifdef RouteMap1
    // The route map for office like environment

    waypoints.emplace_back(inner_point_t(8, -5, fly_height));
    waypoints.emplace_back(inner_point_t(6, -5, fly_height));
    waypoints.emplace_back(inner_point_t(6, 6, fly_height));
    waypoints.emplace_back(inner_point_t(6, 0, fly_height));
    waypoints.emplace_back(inner_point_t(0, 0, fly_height));
    waypoints.emplace_back(inner_point_t(0, -6, fly_height));
    waypoints.emplace_back(inner_point_t(0, 0, fly_height));
    waypoints.emplace_back(inner_point_t(-6, 0, fly_height));
    waypoints.emplace_back(inner_point_t(-6, -6, fly_height));
    waypoints.emplace_back(inner_point_t(-6, 0, fly_height));
    waypoints.emplace_back(inner_point_t(-3, 0, fly_height));
    waypoints.emplace_back(inner_point_t(-3, 2, fly_height));
    waypoints.emplace_back(inner_point_t(1, 2, fly_height));
    waypoints.emplace_back(inner_point_t(1, 6, fly_height));
    waypoints.emplace_back(inner_point_t(-3, 6, fly_height));
    waypoints.emplace_back(inner_point_t(-6, 2, fly_height));
    waypoints.emplace_back(inner_point_t(-6, 5.5, fly_height));
    waypoints.emplace_back(inner_point_t(-6, 6.5, fly_height));
    waypoints.emplace_back(inner_point_t(-5.5f, 6.0, fly_height));
    waypoints.emplace_back(inner_point_t(-5.5f, 6.0, fly_height));

#endif


#ifdef RouteMap2
    for (int i = 0; i < 20; i++) {
        zigzag_waypoints[i][2] = z;

        if (i == 0 || i== 3 || i==4 || i==7 || i==8 || i==11 || i==12 ||
                i== 15 || i==16 || i==19)
            zigzag_waypoints[i][1] = 1.5;
        else
            zigzag_waypoints[i][1] = -1.5;
    }
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 2; j++)
            zigzag_waypoints[i*2+j][0] = 4.5-i*1.0;
#endif
    // init waypoint indexing
    next_waypoint_index = 0;
}

void GasDistMapping::waypoint_update(Robot *robot, SimState_t *sim_state)
{
    float vel_nrm = 1.0; // velocity

    // get robot's position
    float *ref_pos = &(robot->ref_state.pos[0]);
    float dist[3];

    float ref_vel[3] = {0};

    if (next_waypoint_index >= waypoints.size())
        return;

    // calculate the direction to fly
    for (int i = 0; i < 3; i++)
        dist[i] = waypoints[next_waypoint_index][i] - ref_pos[i];
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
        next_waypoint_index++;
        printf("next_way_point: %d\n", next_waypoint_index);
        fflush(stdout);
    }
}

GasDistMapping *gdm;

bool gas_dist_mapping_init(void)
{
    gdm = new GasDistMapping();
    return true;
}

void gas_dist_mapping_update(SimState_t *sim_state)
{
    gdm->waypoint_update((SimModel_get_robots())->at(0), sim_state);
}

void gas_dist_mapping_stop(void)
{
    delete gdm;
}

/* End of file gas_dist_mapping.cxx */
