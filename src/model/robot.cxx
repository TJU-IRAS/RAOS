/*
 *          Robot model
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-08 create this file (RAOS)
 */

#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <string.h> // memset
#include "model/robot.h"
#include "model/helicopter.h" // for HLframe_t
#include "model/quadrotor.h" // for QRframe_t and euler rotation functions
#include "model/ground_robot.h" // for GRframe_t
#include "model/environment.h"

Robot::Robot(const char *robot_type, const char *robot_name, const char *controller_name, float delta_t,
             SimEnvInfo *sim_env_info)
{
    // save args
    dt = delta_t;

    /* =============== Create Robot ================== */
    /******* Helicopter *******/
    if (strcmp(robot_type, "helicopter") == 0)
    {
        config.type = helicopter;
#ifdef RAOS_FEATURE_WAKES
        config.wake = on; // calculate wake by default
#endif
        config.frame = (HLframe_t *) malloc(sizeof(HLframe_t));
        ((HLframe_t *) (config.frame))->main_prop_radius = 0.1; // meter
        ((HLframe_t *) (config.frame))->main_prop_blades = 2; // two-blade
        ((HLframe_t *) (config.frame))->main_prop_chord = 0.01; // chord

        memset(state.pos, 0, sizeof(state.pos));
        memset(state.vel, 0, sizeof(state.vel));
        memset(state.acc, 0, sizeof(state.acc));
        memset(state.att, 0, sizeof(state.att));
        memset(state.att_vel, 0, sizeof(state.att_vel));
    }
        /******* QuadRotor *******/
    else if (strcmp(robot_type, "quadrotor") == 0)
    {
        config.type = quadrotor;
#ifdef RAOS_FEATURE_WAKES
        config.wake = on; // calculate wake by default
#endif
        config.frame = (QRframe_t *) malloc(sizeof(QRframe_t));
        /* init quadrotor dynamic model */
        memset(state.pos, 0, sizeof(state.pos));
        memset(state.vel, 0, sizeof(state.vel));
        memset(state.acc, 0, sizeof(state.acc));
        memset(state.att, 0, sizeof(state.att));
        memset(state.att_vel, 0, sizeof(state.att_vel));
        memset(ref_state.pos, 0, sizeof(ref_state.pos));
        state.pos[0] = 8.0;
        state.pos[1] = -5.0;
        state.pos[2] = 1.5;
        ref_state.pos[0] = 8.0;
        ref_state.pos[1] = -5.0;
        ref_state.pos[2] = 1.5;
        model = new QRdynamic(ref_state.pos, state.pos, state.vel, state.acc, state.att, state.att_vel, dt, robot_name,
                              controller_name, (QRframe_t *) (config.frame), sim_env_info, state.wind,
                              state.wind_est_incl, state.wind_est_leso);
    }
    else if (strcmp(robot_type, "groundrobot") == 0)
    {
        config.type = ground_robot;
        config.frame = (GRframe_t *) malloc(sizeof(GRframe_t));
        /* init quadrotor dynamic model */
        memset(state.pos, 0, sizeof(state.pos));
        memset(state.vel, 0, sizeof(state.vel));
        memset(state.acc, 0, sizeof(state.acc));
        memset(state.att, 0, sizeof(state.att));
        memset(state.att_vel, 0, sizeof(state.att_vel));
        memset(ref_state.pos, 0, sizeof(ref_state.pos));
        state.pos[0] = 8.0;
        state.pos[1] = -5.0;
        state.pos[2] = 1.5;
        ref_state.pos[0] = 8.0;
        ref_state.pos[1] = -5.0;
        ref_state.pos[2] = 1.5;
        model = new GRdynamic(ref_state.pos, state.pos, (GRframe_t *) (config.frame));
    }
        /******* Other *******/
    else
    {// cannot recognize robot type
        // Exit program
    }
    /* ================ Init Sensors ================= */
    center_mox_sensor.pos[0] = 0.0f;
    center_mox_sensor.pos[1] = 0.0f;
    center_mox_sensor.pos[2] = 0.0f;
    center_mox_sensor.sensor.mox_sensor_config(DEFAULT_GAS_TYPE, DEFAULT_SENSOR_TYPE);

    center_pid_sensor.pos[0] = 0.0f;
    center_pid_sensor.pos[1] = 0.0f;
    center_pid_sensor.pos[2] = 0.0f;
    center_pid_sensor.sensor.pid_sensor_config(DEFAULT_GAS_TYPE);

    /* ================ Init Robot ================= */
    /******* Helicopter *******/
    if (config.type == helicopter)
    {
#ifdef RAOS_FEATURE_WAKES
        /* init rotor wake */
        if (config.wake == on)
        {
            RotorWake *new_wake = new RotorWake();
            new_wake->rotor_state.frame.radius
                = ((HLframe_t *) (config.frame))->main_prop_radius;
            new_wake->rotor_state.frame.n_blades
                = ((HLframe_t *) (config.frame))->main_prop_blades;
            new_wake->rotor_state.frame.chord
                = ((HLframe_t *) (config.frame))->main_prop_chord;
            new_wake->rotor_state.thrust = 0.2 * 9.8; // 0.1 kg
            memcpy(new_wake->rotor_state.pos, state.pos, sizeof(state.pos));
            memcpy(new_wake->rotor_state.att,
                   state.att, sizeof(state.att));
            new_wake->init();
            wakes.push_back(new_wake);
        }
#endif
    }
        /******* Quadrotor *******/
    else if (config.type == quadrotor)
    {
#ifdef RAOS_FEATURE_WAKES
        /* init rotor wakes */
        if (config.wake == on)
        {
            for (int i = 0; i < 4; i++) // init four rotor wakes
            {
                RotorWake *new_wake = new RotorWake();
                new_wake->rotor_state.frame.radius
                    = ((QRframe_t *) (config.frame))->prop_radius;
                new_wake->rotor_state.frame.n_blades
                    = ((QRframe_t *) (config.frame))->prop_blades;
                new_wake->rotor_state.frame.chord
                    = ((QRframe_t *) (config.frame))->prop_chord;
                new_wake->rotor_state.thrust
                    = ((QRframe_t *) (config.frame))->mass * 9.8 / 4.0; // N
                if (i % 2 == 0)
                    new_wake->rotor_state.direction = WAKE_ROTOR_CLOCKWISE;
                else
                    new_wake->rotor_state.direction = WAKE_ROTOR_CCLOCKWISE;
                memcpy(new_wake->rotor_state.att,
                       state.att, sizeof(state.att));
                wakes.push_back(new_wake);
            }
            // init four rotor pos for wake computation
            QRCalculateAllRotorPos(state.pos, state.att, ((QRframe_t *) (config.frame))->size / 2.0,
                                   wakes.at(0)->rotor_state.pos, wakes.at(1)->rotor_state.pos,
                                   wakes.at(2)->rotor_state.pos, wakes.at(3)->rotor_state.pos);
            for (int i = 0; i < 4; i++) // init four rotor wakes
                wakes.at(i)->init();
        }
#endif

        /* init leds */
        state.leds = 1;
    }
        /******* Other *******/
    else
    {
        // Exit program
    }
}

void Robot::update(void)
{

    /******* Helicopter *******/
    if (config.type == helicopter)
    {
#ifdef RAOS_FEATURE_WAKES
        /* syncronize rotor att and pos (for wake computation) with robot */
        memcpy(wakes.at(0)->rotor_state.pos, state.pos, sizeof(state.pos));
        memcpy(wakes.at(0)->rotor_state.att,
               state.att, sizeof(state.att));
#endif
    }
        /******* Quadrotor *******/
    else if (config.type == quadrotor)
    {
#ifdef RAOS_FEATURE_WAKES
        /* syncronize rotors' att and pos (for wake computation) with robot */
        for (int i = 0; i < 4; i++) // init four rotor wakes
            memcpy(wakes.at(i)->rotor_state.att,
                   state.att, sizeof(state.att));
        QRCalculateAllRotorPos(state.pos, state.att,
                               ((QRframe_t *) (config.frame))->size / 2.0,
                               wakes.at(0)->rotor_state.pos,
                               wakes.at(1)->rotor_state.pos,
                               wakes.at(2)->rotor_state.pos,
                               wakes.at(3)->rotor_state.pos);
#endif
        /* update quadrotor dynamic model */
        ((QRdynamic *) model)->update();
    }
    else if (config.type == ground_robot)
    {
        /* update ground dynamic model */
        ((GRdynamic *) model)->update();
    }
        /******* Other *******/
    else
    {
        // Exit program
    }

    /* Save record */
    record.push_back(state);
}

void Robot::destroy(void)
{
#ifdef RAOS_FEATURE_WAKES
    if (wakes.size())
    {
        for (unsigned int i = 0; i < wakes.size(); i++)
        {
            wakes.at(i)->destroy();
            delete wakes.at(i);
        }
        wakes.clear();
        std::vector<RotorWake *>().swap(wakes);
    }
#endif
    if (config.type == quadrotor)
    {
        delete ((QRdynamic *) model)->estimator_wind_incl;
        delete ((QRdynamic *) model)->estimator_wind_leso;
        delete ((QRdynamic *) model);
    }
    if (config.type == ground_robot)
    {
        delete ((GRdynamic *) model);
    }
    record.clear();
    std::vector<RobotState_t>().swap(record);
}



/* End of file robot.cxx */
