/*
 *          Robot model
 *
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-08 create this file (RAOS)
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

#ifdef RAOS_FEATURE_WAKES

#include "wake_rotor.h"

#endif

#include "model/environment.h"

#include "model/sensors/mox_sensor.h"
#include "model/sensors/pid_sensor.h"
#include "model/sensors/tdlas_sensor.h"
#include "common/math/raos_math.h"

enum RobotType
{
    helicopter, quadrotor, ground_robot
};

enum FuncSwitch
{
    on, off
};

typedef struct
{
    /* Robot type */
    enum RobotType type; // robot type
    void *frame; // robot frame
#ifdef RAOS_FEATURE_WAKES
    /* rotor wake switch */
    enum FuncSwitch wake; // indicating whether to calculate wake or not
#endif
} RobotConfig_t;

typedef struct
{
    float pos[3];    // relative position w.r.t center of robot
    mox_sensor sensor;
}RobotMOXSensor_t;

typedef struct
{
    float pos[3];    // relative position w.r.t center of robot
    pid_sensor sensor;
}RobotPIDSensor_t;

typedef struct
{
    float pos[3];    // relative position w.r.t center of robot
    tdlas_sensor sensor;
}RobotTDLASSensor_t;

typedef struct
{
    float pos[3];    /* position coordinate (earth axis x), volatile */
    float vel[3];
    float acc[3];
    float att[3]; // roll, pitch, yaw
    float att_vel[3]; // \dot{att}
    int leds;
    float gas_sensor; // gas conc ground truth
    // environment
    float wind[3];
    // sensory result
    float wind_est_incl[3]; // wind estimation result using inclination method
    float wind_est_leso[3]; // wind estimation result using leso
    // tdlas sensor
    int tdlas_enable;   // 0-disable 1-enable
    inner_point_t tdlas_ref_start_point;
    inner_point_t tdlas_ref_end_point;
    inner_point_t target_offset;
} RobotState_t;

typedef struct
{
    float pos[3];    /* position coordinate (earth axis x), volatile */
} RobotRefState_t;

class Robot
{
public:
    Robot(const char *, const char *, const char *, float, SimEnvInfo *); // constructor
    void update(void);

    void destroy(void);

#ifdef RAOS_FEATURE_WAKES
    std::vector<RotorWake *> wakes; // pointer of array containing pointers of RotorWake instances describing wakes of rotors (if has)
#endif
    void *model; // robot dynamic model
    RobotState_t state; // robot state
    RobotRefState_t ref_state; // robot reference state
    RobotConfig_t config; // robot configs
    std::vector<RobotState_t> record;

    RobotMOXSensor_t center_mox_sensor;
    RobotPIDSensor_t center_pid_sensor;
    RobotTDLASSensor_t center_tdlas_sensor;
private:
    float dt;
};

#endif
/* End of file robot.h */
