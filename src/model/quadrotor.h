/*----------------------------------------------------------------------------
 *
 * quadrotor -- simple quad rotor model
 *
 *
 * Date: 2016-02-23 Roice (LUO Bing) modified qrmod.h for RAOS project
 *       2016-03-09 Roice modified this file to make it compatible with
 *                  robot.cxx and robot.h
 *----------------------------------------------------------------------------
 */

#ifndef QUADROTOR_H
#define QUADROTOR_H

#include "model/environment.h"

/* use dynamic model or not flag */
//#define QR_NODYNAMIC true

typedef struct
{
    float size; // distance between two diagonal motors
    float prop_radius; // propeller radius
    float prop_chord; // chord of propellers
    int prop_blades; // number of blades of a propeller
    float mass;
    float I[9];
    float k;
    float b;
    float c_x;
    float c_y;
    float c_z;
} QRframe_t;

/* calculate four rotors' pos according to quadrotor's pos and att */
void QRCalculateAllRotorPos(const float *pos, const float *att, float strut, float *rpos1, float *rpos2, float *rpos3,
                            float *rpos4);

typedef struct
{
    float pos[3]; // position, inertial frame
    float vel[3]; // velocity, inertial frame
    float eta[3]; // rotation, phi(roll) theta(pitch) psi(yaw)  body frame
    float eta_dot[3]; // rotation speed, \dot{\phi} \dot{\theta} \dot{\psi}, body frame
    float motor_rot_speed[4];
    // environment
    float wind[3];
} QRstate_t;

typedef enum
{
    QRcontroller_PID,
    QRcontroller_ADRC,
} QRcontroller_name_t;

typedef struct
{
    // position control
    float P_ALT;    // altitude, P
    float P_VEL;    // altitude velocity (U), P
    float I_VEL;
    float D_VEL;
    float P_POS;    // position, P
    float P_POSR;   // position velocity (NE), P
    float I_POSR;
    float D_POSR;
    float P_MAG;    // heading, P
    float I_MAG;
    float D_MAG;
    // attitude control
    float P_ROLL;
    float I_ROLL;
    float D_ROLL;
    float P_PITCH;
    float I_PITCH;
    float D_PITCH;
} QRcontroller_PID_Params_t;

typedef struct
{
    // position control
    float errIntegral_alt;
    float errIntegral_pos[2];
    float errIntegral_yaw;
    // attitude control
    float err_pitch;
    float err_roll;
    float errIntegral_roll;
    float errIntegral_pitch;
} QRcontroller_PID_State_t;

class QRestimator_INCL
{
public:
    QRestimator_INCL(float *vel, float *att, float dt);

    void update(void);

    float angle_incl;
    float e[3]; // -R_B^I * [0,0,1]^T
    float e_proj[3]; // e . [1,1,0]
    float v[3];
    float wind_estimated[3];
private:
    float dt;
    float *QR_vel;
    float *QR_att;

    void f_inc(float);
};

class QRestimator_LESO
{
public:
    QRestimator_LESO(float *pos, float *vel, float *acc, float *att, float *omega, float dt);

    void update(void);

    /* parameters */
    float w0;
    float m;
    float c_x;
    float c_y;
    float c_z;
    /* state */
    float z1[3]; // e, n, u
    float z2[3];
    float z3[3];
    float u[3];
    /* result */
    float wind_estimated[3];
private:
    float dt;
    float *QR_pos; // actual position
    float *QR_vel; // actual velocity
    float *QR_acc; // actual acceleration
    float *QR_att; // actual attitude
    float *QR_omega; // motor rotation speed
};

class QRdynamic
{
public:
    QRdynamic(float *pos_ref, float *pos, float *vel, float *acc, float *att, float *att_vel, float delta_t,
              const char *, const char *, QRframe_t *, SimEnvInfo *, float *wind, float *wind_est_incl,
              float *wind_est_leso); // constructor
    void update(void);

    /* quadrotor attributes */
    QRframe_t frame;
    /* quadrotor states */
    QRstate_t state;
    /* quadrotor wind estimation */
    QRestimator_INCL *estimator_wind_incl;
    QRestimator_LESO *estimator_wind_leso;
private:
    void configure(const char *, const char *, QRframe_t *);

    /* input/output of QRdynamic */
    float dt;
    float *QR_pos_ref; // reference position
    float QR_yaw_ref; // reference heading
    float *QR_pos; // actual position
    float *QR_vel; // actual velocity
    float *QR_acc; // actual acceleration
    float *QR_att; // actual attitude
    float *QR_att_vel;
    float *QR_wind; // wind vector at the position of quadrotor
    float *QR_wind_est_incl; // wind vector estimation using inclination method proposed by Neumann
    float *QR_wind_est_leso;
    SimEnvInfo *QR_env_info;

    /* quadrotor model, input = motor rotation speeds */
    void quadrotor_model(void);

    /* quadrotor controller, output = motor rotation speeds */
    typedef struct
    {
        // name (type) of controller
        QRcontroller_name_t name;
        // parameters of controller according to its type
        void *params;
        // state of controller according to its type
        void *state;

        // controller refresh routine
        void (QRdynamic::*refresh)(); // cannot use name 'update' since it's in the public already
    } QRcontroller_t;
    QRcontroller_t QRcontroller;

    void quadrotor_controller_pid(void);
};

#endif /* End of file quadrotor.h */
