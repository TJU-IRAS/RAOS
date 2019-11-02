/*--------------------------------------------------------------------------
 *
 * quadrotor -- Simple quad rotor model
 *
 *
 *
 * Date: 2016-02-23 Roice (LUO Bing) modified qrmod.cxx for RAOS project
 *       2016-03-09 Roice modified this file to make it compatible with
 *                  robot.cxx
 *
 *--------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <string.h>
#include "cblas.h"
#include "common/math/rotation.h"
#include "common/math/linalg.h"
#include "model/quadrotor.h"
#include "model/environment.h"

#ifndef RAD2DEG
#define RAD2DEG (180.0/M_PI)
#endif

/* calculate four rotors' pos according to quadrotor's pos and att */
void QRCalculateAllRotorPos(const float *pos, const float *att, float strut, float *rpos1, float *rpos2, float *rpos3,
                            float *rpos4)
{
    // init rotor pos vector
    memcpy(rpos1, pos, 3 * sizeof(float));
    memcpy(rpos2, pos, 3 * sizeof(float));
    memcpy(rpos3, pos, 3 * sizeof(float));
    memcpy(rpos4, pos, 3 * sizeof(float));

#if 0
    // shape: +
    float v1[3] = {strut, 0.0, 0.0};
    float v2[3] = {0.0, strut, 0.0};
    float v3[3] = {-strut, 0.0, 0.0};
    float v4[3] = {0.0, -strut, 0.0};
#else
    // shape: x
    float v1[3] = {strut / (float) 1.41421356, strut / (float) 1.41421356, 0.0};
    float v2[3] = {-strut / (float) 1.41421356, strut / (float) 1.41421356, 0.0};
    float v3[3] = {-strut / (float) 1.41421356, -strut / (float) 1.41421356, 0.0};
    float v4[3] = {strut / (float) 1.41421356, -strut / (float) 1.41421356, 0.0};
#endif

    rotate_vector(v1, rpos1, att[0] * RAD2DEG, att[1] * RAD2DEG, att[2] * RAD2DEG);
    rotate_vector(v2, rpos2, att[0] * RAD2DEG, att[1] * RAD2DEG, att[2] * RAD2DEG);
    rotate_vector(v3, rpos3, att[0] * RAD2DEG, att[1] * RAD2DEG, att[2] * RAD2DEG);
    rotate_vector(v4, rpos4, att[0] * RAD2DEG, att[1] * RAD2DEG, att[2] * RAD2DEG);

#if 0
    printf("rpos1 = [ %f, %f, %f ]\n", rpos1[0], rpos1[1], rpos1[2]);
    printf("rpos2 = [ %f, %f, %f ]\n", rpos2[0], rpos2[1], rpos2[2]);
    printf("rpos3 = [ %f, %f, %f ]\n", rpos3[0], rpos3[1], rpos3[2]);
    printf("rpos4 = [ %f, %f, %f ]\n", rpos4[0], rpos4[1], rpos4[2]);
#endif
}

QRdynamic::QRdynamic(float *pos_ref, float *pos, float *vel, float *acc, float *att, float *att_vel, float delta_t,
                     const char *robot_name, const char *ctl_name, QRframe_t *frm, SimEnvInfo *sim_env_info,
                     float *wind, float *wind_est_incl, float *wind_est_leso)
{
    // save parameters and addr
    dt = delta_t;
    QR_pos_ref = pos_ref;
    QR_yaw_ref = 0.;
    QR_pos = pos;
    QR_vel = vel;
    QR_acc = acc;
    QR_att = att;
    QR_att_vel = att_vel;
    QR_env_info = sim_env_info;
    QR_wind = wind;
    QR_wind_est_incl = wind_est_incl;
    QR_wind_est_leso = wind_est_leso;

    // configure quadrotor frame
    configure(robot_name, ctl_name, frm);

    // init parameters
    memset(&state, 0, sizeof(state));

    memcpy(QR_pos, state.pos, 3 * sizeof(float));
    memcpy(QR_att, state.eta, 3 * sizeof(float));

    // init wind estimator
    estimator_wind_incl = new QRestimator_INCL(vel, att, delta_t);
    estimator_wind_leso = new QRestimator_LESO(pos, vel, acc, att, state.motor_rot_speed, delta_t);
}

void QRdynamic::update(void)
{
    /* update environment */
    QR_env_info->measure_wind(state.pos, QR_wind); // get wind velocity
    /* update quad_rotor_model */
    quadrotor_model();
    /* update controller */
    void (QRdynamic::*p_controller_refresh)() = QRcontroller.refresh;
    (this->*p_controller_refresh)();
    /* wind estimation */
    estimator_wind_incl->update();
    estimator_wind_leso->update();
    memcpy(QR_wind_est_incl, estimator_wind_incl->wind_estimated, 3 * sizeof(float));
    memcpy(QR_wind_est_leso, estimator_wind_leso->wind_estimated, 3 * sizeof(float));

#ifdef QR_NODYNAMIC
    memcpy(QR_pos, QR_pos_ref, 3*sizeof(float));
    memset(QR_vel, 0, 3*sizeof(float));
    memset(QR_acc, 0, 3*sizeof(float));
    memset(QR_att, 0, 3*sizeof(float));
#endif

#if 0
    /* get f_inc of INCL, should close measure_wind */
    static int count = 0;
    if (count++ > 50*10 and QR_wind[0] <= 5.0) {
        //printf("Inclination: %.2f deg, wind speed: %.3f m/s\n", estimator_wind_incl->angle_incl*180./M_PI, QR_wind[0]);
        printf("%.2f, ", estimator_wind_incl->angle_incl*180./M_PI);
        count= 0;
        QR_wind[0] += 0.1;
    }
#endif

#if 0
    /* estimate wind resistance parameter, c_x, c_y */
    static int count = 0;
    static float beta = -M_PI;
    if (count++ > 50*20 and beta < M_PI) {
        count = 0;
        QR_pos_ref[0] = 0;
        if (QR_pos_ref[1] > 0.)
            QR_pos_ref[1] = -50;
        else
            QR_pos_ref[1] = 50;
        QR_pos_ref[2] = 2.0;
        beta += 10.*M_PI/180.;
        QR_yaw_ref = beta;
    }
#endif

#if 0
    /* estimate wind resistance parameter, c_z */
    static int count = 0;
    if (count++ > 50*20) {
        count = 0;
        QR_pos_ref[0] = 0;
        QR_pos_ref[1] = 0;
        if (QR_pos_ref[2] > 2.)
            QR_pos_ref[2] = 2. - 0.5;
        else
            QR_pos_ref[2] = 2. + 0.5;
    }
#endif

#if 0
    /* gust estimation */
    //QR_pos_ref[2] = 2.0;
    static int count = 0;
    if (count++ > 50*10) {
        count = 0;
        if (QR_wind[0] == 0.)
            QR_wind[0] = 1.0;
        else
            QR_wind[0] = 0.;
    }
#endif
}

void QRdynamic::quadrotor_model(void)
{
    float L = frame.size / 2.0;
    float k = frame.k;
    float b = frame.b;
    float c_x = frame.c_x;
    float c_y = frame.c_y;
    float c_z = frame.c_z;
    float *motor_rot_speed = state.motor_rot_speed;

    // compute thrust given rotation speeds of motors and thrust coefficient (k), body frame
    float T_B[3] = {0., 0., k * (motor_rot_speed[0] * motor_rot_speed[0] +
                                 motor_rot_speed[1] * motor_rot_speed[1] +
                                 motor_rot_speed[2] * motor_rot_speed[2] +
                                 motor_rot_speed[3] * motor_rot_speed[3])};
    //printf("T_B = [ %f, %f, %f ]\n", T_B[0], T_B[1], T_B[2]);

    // compute torques, given rotation speeds of motors, length from quadrotor center to motor (L), drag coefficient (b), thrust coefficient (k), body frame
    float tau_B[3] = {
        L * k * (motor_rot_speed[0] * motor_rot_speed[0] + motor_rot_speed[1] * motor_rot_speed[1] -
                 motor_rot_speed[2] * motor_rot_speed[2] - motor_rot_speed[3] * motor_rot_speed[3]),
        L * k * (-motor_rot_speed[0] * motor_rot_speed[0] + motor_rot_speed[1] * motor_rot_speed[1] +
                 motor_rot_speed[2] * motor_rot_speed[2] - motor_rot_speed[3] * motor_rot_speed[3]),
        b * (motor_rot_speed[0] * motor_rot_speed[0] - motor_rot_speed[1] * motor_rot_speed[1] +
             motor_rot_speed[2] * motor_rot_speed[2] - motor_rot_speed[3] * motor_rot_speed[3])
    };
    // body frame to inertial frame
    float R[9] = {
        std::cos(state.eta[1]) * std::cos(state.eta[2]),
        std::sin(state.eta[0]) * std::sin(state.eta[1]) * std::cos(state.eta[2]) -
        std::cos(state.eta[0]) * std::sin(state.eta[2]),
        std::cos(state.eta[2]) * std::sin(state.eta[1]) * std::cos(state.eta[0]) +
        std::sin(state.eta[0]) * std::sin(state.eta[2]),
        std::cos(state.eta[1]) * std::sin(state.eta[2]),
        std::sin(state.eta[1]) * std::sin(state.eta[0]) * std::sin(state.eta[2]) +
        std::cos(state.eta[0]) * std::cos(state.eta[2]),
        std::cos(state.eta[0]) * std::sin(state.eta[1]) * std::sin(state.eta[2]) -
        std::sin(state.eta[0]) * std::cos(state.eta[2]),
        -std::sin(state.eta[1]),
        std::sin(state.eta[0]) * std::cos(state.eta[1]),
        std::cos(state.eta[0]) * std::cos(state.eta[1])
    };
    // compute acceleration a, given T_B, R, friction coefficient c, translation velocity in inertial frame (state.vel)
    float T[3] = {0., 0., 0.}; // thrust, inertial frame
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, R, 3, T_B, 1, 1.0, T, 1);
    float flight_vel[3]; // flight velocity
    memcpy(flight_vel, state.vel, sizeof(float) * 3); // get ground velocity
    cblas_saxpy(3, -1.0, QR_wind, 1, flight_vel, 1); // triangle
    float flight_vel_B[3];
    solve_linear_equations(3, 1, R, flight_vel_B, flight_vel); // flight_vel I to B
    float Fd_B[3] = {-c_x * flight_vel_B[0], -c_y * flight_vel_B[1], -c_z * flight_vel_B[2]};
    float Fd[3] = {0., 0., 0.};
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, R, 3, Fd_B, 1, 1.0, Fd, 1); // Fd B to I
    float a[3] = {
        (float) 0. + (T[0] + Fd[0]) / frame.mass,
        (float) 0. + (T[1] + Fd[1]) / frame.mass,
        (float) -9.8 + (T[2] + Fd[2]) / frame.mass
    };

    // compute w (omega), given etadot_to_w matrix and rotation angle (eta), rotational speed (eta_dot)
    float etadot_to_w[9] = {
        1.,
        0.,
        -std::sin(state.eta[1]),
        0.,
        std::cos(state.eta[0]),
        std::cos(state.eta[1]) * std::sin(state.eta[0]),
        0.,
        -std::sin(state.eta[0]),
        std::cos(state.eta[1]) * std::cos(state.eta[0])
    };
    float w[3] = {0., 0., 0.};
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, etadot_to_w, 3, state.eta_dot, 1, 1.0, w, 1);
    // compute angular acceleration (wdot), given tau_B, w, inertial matrix of quadrotor (I)
    float Iw[3] = {0., 0., 0.}; // I*w
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, frame.I, 3, w, 1, 1.0, Iw, 1);
    float w_cross_Iw[3] = {
        w[1] * Iw[2] - w[2] * Iw[1],
        w[2] * Iw[0] - w[0] * Iw[2],
        w[0] * Iw[1] - w[1] * Iw[0]
    }; // w x (Iw)
    float tau_sub_w_cross_Iw[3]; // tau - w x (Iw)
    memcpy(tau_sub_w_cross_Iw, tau_B, sizeof(tau_B));
    cblas_saxpy(3, -1.0, w_cross_Iw, 1, tau_sub_w_cross_Iw, 1);
    float wdot[3];
    solve_linear_equations(3, 1, frame.I, wdot, tau_sub_w_cross_Iw);

    // refresh w
    cblas_saxpy(3, dt, wdot, 1, w, 1); // w = w + wdot * dt
    // refresh eta_dot, given w_to_etadot matrix and w
    float eta_dot[3];
    solve_linear_equations(3, 1, etadot_to_w, eta_dot, w);
    memcpy(state.eta_dot, eta_dot, sizeof(eta_dot));
    // refresh eta
    cblas_saxpy(3, dt, state.eta_dot, 1, state.eta, 1); // eta = eta + eta_dot * dt
    // refresh velocity
    cblas_saxpy(3, dt, a, 1, state.vel, 1); // vel = vel + a * dt
    // refresh position
    cblas_saxpy(3, dt, state.vel, 1, state.pos, 1); // pos = pos + vel * dt

    // constrain eta to [-pi ~ pi]
    for (int i = 0; i < 3; i++)
        if (std::abs(state.eta[i]) > M_PI)
            while (std::abs(state.eta[i]) > M_PI)
            {
                if (state.eta[i] > M_PI)
                    state.eta[i] -= 2 * M_PI;
                else
                    state.eta[i] += 2 * M_PI;
            }

    // constrain of ground
    //if (state.pos[2] < 0.)
    //    state.pos[2] = 0.;

    // save to global
    memcpy(QR_pos, state.pos, 3 * sizeof(float));
    memcpy(QR_vel, state.vel, 3 * sizeof(float));
    memcpy(QR_acc, a, 3 * sizeof(float));
    memcpy(QR_att, state.eta, 3 * sizeof(float));
    memcpy(QR_att_vel, state.eta_dot, 3 * sizeof(float));
}

static float constrain(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

static float applyDeadband(float value, float deadband)
{
    if (fabs(value) < deadband)
    {
        value = 0;
    } else if (value > 0)
    {
        value -= deadband;
    } else if (value < 0)
    {
        value += deadband;
    }
    return value;
}

void QRdynamic::quadrotor_controller_pid(void)
{
    QRcontroller_PID_Params_t *profile = (QRcontroller_PID_Params_t *) (QRcontroller.params);
    QRcontroller_PID_State_t *pid_state = (QRcontroller_PID_State_t *) (QRcontroller.state);
/* Step 1: Position Control */
    /*  Phase 1: Throttle, altitude control */
    // Altitude P-Controller
    float err_alt = constrain(QR_pos_ref[2] - QR_pos[2], -1.0, 1.0); // -0.5 - 0.5 m boundary
    err_alt = applyDeadband(err_alt,
                            0.01); // 1 cm deadband, remove small P parameter to reduce noise near zero position
    float setVel_alt = constrain(profile->P_ALT * err_alt, -2.0, 2.0); // limit velocity to +/- 2.0 m/s
    // Velocity PID-Controller
    //      P
    float err_alt_vel = setVel_alt - QR_vel[2];
    float result_alt = constrain((profile->P_VEL * err_alt_vel), -300, 300); // limit to +/- 300
    //      I
    pid_state->errIntegral_alt += (profile->I_VEL * err_alt_vel);
    pid_state->errIntegral_alt = constrain(pid_state->errIntegral_alt, -700.0, 700.0); // limit to +/- 700
    result_alt += pid_state->errIntegral_alt;
    //      D
    result_alt -= constrain(profile->D_VEL * QR_acc[2], -100, 100); // limit
    //printf("Alt error is %f m, vel error is %f, result is %f\n", err_alt, err_alt_vel, result_alt);
    result_alt = constrain(1050 + result_alt, 1050, 1950); // PPM value

    /*  Phase 2: Roll & Pitch, horizontal position control */
    // get position error vector in earth coordinate
    float err_en[2];
    for (int i = 0; i < 2; i++)
        err_en[i] = QR_pos_ref[i] - QR_pos[i];
    // Velocity-PID
    float target_vel[2]; // in inertial frame
    float err_pos_vel;
    float result_pos_i[2]; // in inertial frame
    float result_pos[2]; // in body frame
    for (int i = 0; i < 2; i++) // 0 for roll, 1 for pitch
    {
        // Position PID-Controller for east(x)/north(y) axis
        target_vel[i] = constrain(profile->P_POS * err_en[i], -5.0, 5.0); // limit error to +/- 2 m/s;
        target_vel[i] = applyDeadband(target_vel[i], 0.01); // 1 cm/s
        // Velocity PID-Controller
        err_pos_vel = target_vel[i] - QR_vel[i];
        // P
        result_pos_i[i] = constrain((profile->P_POSR * err_pos_vel), -200, 200); // limit to +/- 200
        // I
        pid_state->errIntegral_pos[i] += (profile->I_POSR * err_pos_vel);
        pid_state->errIntegral_pos[i] = constrain(pid_state->errIntegral_pos[i], -200.0, 200.0); // limit to +/- 200
        result_pos_i[i] += pid_state->errIntegral_pos[i];
        // D
        result_pos_i[i] -= constrain(profile->D_POSR * QR_acc[i], -100, 100); // limit
    }
    // transform result_pos from inertial frame to body frame
    float heading_angle = QR_att[2]; // in inertial frame
    result_pos[0] = std::cos(heading_angle) * result_pos_i[0] + std::sin(heading_angle) * result_pos_i[1];
    result_pos[1] = -std::sin(heading_angle) * result_pos_i[0] + std::cos(heading_angle) * result_pos_i[1];
    // update roll/pitch value
    for (int i = 0; i < 2; i++)
        result_pos[i] = constrain(1500 + result_pos[i], 1050, 1950);
#if 0
    printf("ref pos    = [ %f, %f, %f ]\n", QR_pos_ref[0], QR_pos_ref[1], QR_pos_ref[2]);
    printf("pos =        [ %f, %f, %f ]\n", QR_pos[0], QR_pos[1], QR_pos[2]);
    printf("err_en     = [ %f, %f ]\n", err_en[0], err_en[1]);
    printf("target_vel = [ %f, %f ]\n", target_vel[0], target_vel[1]);
    printf("err_pos_vel =[ %f, %f ]\n", target_vel[0] - QR_vel[0], target_vel[1] - QR_vel[1]);
    printf("result_pos = [ %f, %f]\n", result_pos[0], result_pos[1]);
#endif
    /*  Phase 3: Yaw, heading control */
    float err_yaw = QR_yaw_ref - QR_att[2]; // ref = 0. (North)
    float err_yaw_sign;
    if (std::abs(err_yaw) > M_PI)
    {
        if (err_yaw > M_PI)
            err_yaw_sign = -1.;
        else
            err_yaw_sign = 1.;
        err_yaw = err_yaw_sign * std::abs(2 * M_PI - std::abs(err_yaw));
    }
    err_yaw = applyDeadband(err_yaw, 1.0 * M_PI /
                                     180.); // 1 degree deadband, remove small P parameter to reduce noise near zero position
    // PID-Controller
    //      P
    float result_yaw = constrain((profile->P_MAG * err_yaw), -300, 300); // limit to +/- 300
    //      I
    pid_state->errIntegral_yaw += (profile->I_MAG * err_yaw);
    pid_state->errIntegral_yaw = constrain(pid_state->errIntegral_yaw, -700.0, 700.0); // limit to +/- 700
    result_yaw += pid_state->errIntegral_yaw;
    //      D
    result_yaw -= constrain(profile->D_MAG * QR_att_vel[2], -100, 100); // limit
    result_yaw = constrain(1500 + result_yaw, 1050, 1950); // PPM value
#if 0
    printf("QR_att[2] = %f\n", QR_att[2]);
    printf("yaw: QR_att_vel[2] = %f\n", QR_att_vel[2]);
    printf("yaw: err_yaw = %f\n", err_yaw);
    printf("yaw: P value = %f\n", constrain((profile->P_MAG*err_yaw), -300, 300));
    printf("yaw: I value = %f\n", constrain(pid_state->errIntegral_yaw, -700.0, 700.0));
    printf("yaw: D value = %f\n", -constrain(profile->D_MAG*QR_att_vel[2], -100, 100));
    printf("result_yaw = %f\n", result_yaw);
#endif

    //result_pos[0] = 1500;
    //result_pos[1] = 1500;
    //result_yaw = 1500 + 10;

/* Step 2: Attitude Control */
    float u[4];
    /* Phase 1: calculate u1 from throttle PPM value, m*u1 = f0 + f1 + f2 + f3 */
    u[0] = (result_alt - 1050.) / (1950. - 1050.) * 5. * 9.8; // 0 ~ 5 g acceleration
    /* Phase 2: calculate \ddot{roll, pitch}, roll, pitch denotes angle */
    float roll_ref = (result_pos[0] - 1500) / 450. * 45. * M_PI / 180.; // max 45 degree
    float pitch_ref = (1500 - result_pos[1]) / 450. * 45. * M_PI / 180.; // max 45 degree
    float err_roll = roll_ref - state.eta[1]; // because QR head to North(Y-axis)
    float err_pitch = pitch_ref - state.eta[0];
    float err_roll_dot = (err_roll - pid_state->err_roll) / dt;
    float err_pitch_dot = (err_pitch - pid_state->err_pitch) / dt;
    pid_state->err_roll = err_roll;
    pid_state->err_pitch = err_pitch;
    //  roll PID
    float result_roll = constrain(profile->P_ROLL * err_roll, -5.0, 5.0); // 0 ~ 5 g
    pid_state->errIntegral_roll += profile->I_ROLL * err_roll;
    pid_state->errIntegral_roll = constrain(pid_state->errIntegral_roll, -5.0, 5.0);
    result_roll += pid_state->errIntegral_roll;
    result_roll += constrain(profile->D_ROLL * err_roll_dot, -5., 5.);
    result_roll = constrain(result_roll, -5., 5.);
    //  pitch PID
    float result_pitch = constrain(profile->P_PITCH * err_pitch, -5.0, 5.0);
    pid_state->errIntegral_pitch += profile->I_PITCH * err_pitch;
    pid_state->errIntegral_pitch = constrain(pid_state->errIntegral_pitch, -5.0, 5.0);
    result_pitch += pid_state->errIntegral_pitch;
    result_pitch += constrain(profile->D_PITCH * err_pitch_dot, -5., 5.);
    result_pitch = constrain(result_pitch, -5., 5.);
    /* Phase 3: map \ddot{roll, pitch} to u2, u3 */
    u[1] = result_pitch;
    u[2] = result_roll;
    /* Phase 4: map result_yaw to u4 */
    u[3] = (result_yaw - 1500.) / 450. * 10; // 0 ~ 10 g
#if 0
    printf("roll_ref    = %f, pitch_ref    = %f\n", roll_ref, pitch_ref);
    printf("err_roll    = %f, err_pitch    = %f\n", err_roll, err_pitch);
    printf("result_roll = %f, result_pitch = %f\n", result_roll, result_pitch);
#endif

    /* Phase 5: map u to f, QR heading to North (Y-axis)
     * m*u1 =    f1 + f2 + f3 + f4
     * m*u2 = L*(f1 + f2 - f3 - f4)
     * m*u3 = L*(f2 + f3 - f1 - f4)
     * m*u4 = b/k*(f1 + f3 - f2 - f4);
     *
     * write to matrix form:
     *  A*f = [m*u1, u2/L, u3/L, u4/p]^T
     *  A = [ 1 1 1 1
     *        1 1 -1 -1
     *        -1 1 1 -1
     *        1 -1 1 -1 ]
     * */
    float A[16] = {1, 1, 1, 1,
                   1, 1, -1, -1,
                   -1, 1, 1, -1,
                   1, -1, 1, -1};

    float f[4] = {u[0] * frame.mass,
                  u[1] * frame.mass / frame.size / (float) 2.,
                  u[2] * frame.mass / frame.size / (float) 2.,
                  u[3] * frame.mass / frame.b * frame.k /* how to determine this? */ };
    solve_linear_equations(4, 1, A, f, f);
    for (int i = 0; i < 4; i++) // the rotor cannot rotate inversely
        f[i] = f[i] > 0 ? f[i] : 0;
    for (int i = 0; i < 4; i++)
        state.motor_rot_speed[i] = std::sqrt(f[i] / frame.k);
}

void QRdynamic::configure(const char *robot_name, const char *controller_name, QRframe_t *frm)
{
    /* frame */
    if (strcmp(robot_name, "Micro Bee") == 0)
    {
        frm->size = 0.22; // m
        frm->prop_radius = 0.0725; // m
        frm->prop_chord = 0.01; // m
        frm->prop_blades = 2;
        frm->mass = 0.122; // kg
        frm->I[0] = 0.0002632;
        frm->I[1] = 0.0;
        frm->I[2] = 0.0;
        frm->I[3] = 0.0;
        frm->I[4] = 0.0002745;
        frm->I[5] = 0.0;
        frm->I[6] = 0.0;
        frm->I[7] = 0.0;
        frm->I[8] = 0.00091175;
        frm->k = 0.0000542;
        frm->b = 0.000011;
        frm->c_x = 0.2;
        frm->c_y = 0.2;
        frm->c_z = 0.83;
    } else
    { // Super Bee by default
        if (strcmp(robot_name, "Super Bee") != 0)
            printf(
                "Warning: quadrotor robot name \'%s\' not recognized, use the configuration of Super Bee by default.\n",
                robot_name);
        frm->size = 0.45; // m
        frm->prop_radius = 0.15; // m
        frm->prop_chord = 0.016; // m
        frm->prop_blades = 2;
        frm->mass = 0.8; // kg
        frm->I[0] = 0.0081;
        frm->I[1] = 0.0;
        frm->I[2] = 0.0;
        frm->I[3] = 0.0;
        frm->I[4] = 0.0081;
        frm->I[5] = 0.0;
        frm->I[6] = 0.0;
        frm->I[7] = 0.0;
        frm->I[8] = 0.0142;
        frm->k = 0.0000542;
        frm->b = 0.000011;
        frm->c_x = 0.2;
        frm->c_y = 0.2;
        frm->c_z = 0.83;
    };

    /* controller name */
    if (strcmp(controller_name, "PID") == 0)
        QRcontroller.name = QRcontroller_PID;
    else if (strcmp(controller_name, "ADRC") == 0)
        QRcontroller.name = QRcontroller_ADRC;
    else
    {
        printf("Warning: QR controller name \'%s\' not recognized, use PID by default.\n", controller_name);
        QRcontroller.name = QRcontroller_PID;
    }

    /* controller parameters */
    switch (QRcontroller.name)
    {
        case QRcontroller_PID:
            QRcontroller.params = (QRcontroller_PID_Params_t *) malloc(sizeof(QRcontroller_PID_Params_t));
            if (strcmp(robot_name, "Micro Bee") == 0)
            {
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_ALT = 1.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_VEL = 400.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_VEL = 10.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_VEL = 0.06;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_POS = 1.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_POSR = 200.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_POSR = 1;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_POSR = 10;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_MAG = 10;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_MAG = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_MAG = 1;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_ROLL = 1.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_ROLL = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_ROLL = 0.1;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_PITCH = 1.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_PITCH = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_PITCH = 0.1;
            } else
            { // Super Bee by default
                if (strcmp(robot_name, "Super Bee") != 0)
                    printf(
                        "Warning: quadrotor robot name \'%s\' not recognized, use the PID setting of Super Bee by default.\n",
                        robot_name);
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_ALT = 1.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_VEL = 400.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_VEL = 10.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_VEL = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_POS = 1.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_POSR = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_POSR = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_POSR = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_MAG = 1.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_MAG = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_MAG = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_ROLL = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_ROLL = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_ROLL = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->P_PITCH = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->I_PITCH = 0.0;
                ((QRcontroller_PID_Params_t *) QRcontroller.params)->D_PITCH = 0.0;
            }
            break;
        case QRcontroller_ADRC:
            break;
    }

    /* controller state */
    switch (QRcontroller.name)
    {
        case QRcontroller_PID:
            QRcontroller.state = (QRcontroller_PID_State_t *) malloc(sizeof(QRcontroller_PID_State_t));
            memset(QRcontroller.state, 0, sizeof(QRcontroller_PID_State_t));
            break;
        case QRcontroller_ADRC:
            break;
    }

    /* controller refresh routine */
    switch (QRcontroller.name)
    {
        case QRcontroller_PID:
            QRcontroller.refresh = &QRdynamic::quadrotor_controller_pid;
            break;
        case QRcontroller_ADRC:
            break;
    }

    // copy frm (external) to frame (class private)
    memcpy(&frame, frm, sizeof(QRframe_t));
}

QRestimator_INCL::QRestimator_INCL(float *vel, float *att, float delta_t)
{
    // save parameters and addr
    dt = delta_t;
    QR_vel = vel;
    QR_att = att;
}

void QRestimator_INCL::update(void)
{
    // body frame to inertial frame
    float R[9] = {
        std::cos(QR_att[1]) * std::cos(QR_att[2]),
        std::sin(QR_att[0]) * std::sin(QR_att[1]) * std::cos(QR_att[2]) - std::cos(QR_att[0]) * std::sin(QR_att[2]),
        std::cos(QR_att[2]) * std::sin(QR_att[1]) * std::cos(QR_att[0]) + std::sin(QR_att[0]) * std::sin(QR_att[2]),
        std::cos(QR_att[1]) * std::sin(QR_att[2]),
        std::sin(QR_att[1]) * std::sin(QR_att[0]) * std::sin(QR_att[2]) + std::cos(QR_att[0]) * std::cos(QR_att[2]),
        std::cos(QR_att[0]) * std::sin(QR_att[1]) * std::sin(QR_att[2]) - std::sin(QR_att[0]) * std::cos(QR_att[2]),
        -std::sin(QR_att[1]),
        std::sin(QR_att[0]) * std::cos(QR_att[1]),
        std::cos(QR_att[0]) * std::cos(QR_att[1])
    };

    // e = -R_B^I * [0,0,1]^T
    memset(e, 0, 3 * sizeof(float));
    float unit[3] = {0., 0., -1.};
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, R, 3, unit, 1, 1.0, e, 1); // Fd B to I
    // e_proj = e . [1,1,0]
    memcpy(e_proj, e, 2 * sizeof(float));
    // |e_proj|
    float nrm_e_proj = std::sqrt(e_proj[0] * e_proj[0] + e_proj[1] * e_proj[1]);
    if (nrm_e_proj < 0.001)
    {
        // no wind
        angle_incl = 0.;
        memset(v, 0, 3 * sizeof(float));
    } else
    {
        angle_incl = std::asin(nrm_e_proj);
        f_inc(nrm_e_proj);
    }
    // triangle
    //memcpy(wind_estimated, QR_vel, 3*sizeof(float));
    //cblas_saxpy(3, -1.0, v, 1, wind_estimated, 1); // wind <- -1.0*v+vel
    memcpy(wind_estimated, v, 3 * sizeof(float));
}

void QRestimator_INCL::f_inc(float norm_e_prj)
{
    float alpha = std::asin(norm_e_prj) * 180. / M_PI;
    float strength = 0.00086 * alpha * alpha + 0.08794 * alpha + 0.06383;
    for (int i = 0; i < 2; i++)
        v[i] = strength * e_proj[i] / norm_e_prj;
}

QRestimator_LESO::QRestimator_LESO(float *pos, float *vel, float *acc, float *att, float *omega, float delta_t)
{
    // save parameters and addr
    dt = delta_t;
    QR_pos = pos;
    QR_vel = vel;
    QR_acc = acc;
    QR_att = att;
    QR_omega = omega;

    // init state
    memset(z1, 0, sizeof(z1));
    memset(z2, 0, sizeof(z2));
    memset(z3, 0, sizeof(z3));
    memset(u, 0, sizeof(u));

    // parameters
    w0 = 18;
    m = 0.122;
    c_x = 0.20001;
    c_y = 0.20001;
    c_z = 0.830005;
}

void QRestimator_LESO::update(void)
{
    // body frame to inertial frame
    float R_BI[9] = {
        std::cos(QR_att[1]) * std::cos(QR_att[2]),
        std::sin(QR_att[0]) * std::sin(QR_att[1]) * std::cos(QR_att[2]) - std::cos(QR_att[0]) * std::sin(QR_att[2]),
        std::cos(QR_att[2]) * std::sin(QR_att[1]) * std::cos(QR_att[0]) + std::sin(QR_att[0]) * std::sin(QR_att[2]),
        std::cos(QR_att[1]) * std::sin(QR_att[2]),
        std::sin(QR_att[1]) * std::sin(QR_att[0]) * std::sin(QR_att[2]) + std::cos(QR_att[0]) * std::cos(QR_att[2]),
        std::cos(QR_att[0]) * std::sin(QR_att[1]) * std::sin(QR_att[2]) - std::sin(QR_att[0]) * std::cos(QR_att[2]),
        -std::sin(QR_att[1]),
        std::sin(QR_att[0]) * std::cos(QR_att[1]),
        std::cos(QR_att[0]) * std::cos(QR_att[1])
    };
    // inertial frame to body frame
    float R_IB[9] = {
        std::cos(QR_att[2]) * std::cos(QR_att[1]),
        std::sin(QR_att[2]) * std::cos(QR_att[1]),
        -std::sin(QR_att[1]),
        std::sin(QR_att[0]) * std::cos(QR_att[2]) * std::sin(QR_att[1]) - std::cos(QR_att[0]) * std::sin(QR_att[2]),
        std::sin(QR_att[0]) * std::sin(QR_att[2]) * std::sin(QR_att[1]) + std::cos(QR_att[0]) * std::cos(QR_att[2]),
        std::sin(QR_att[0]) * std::cos(QR_att[1]),
        std::cos(QR_att[0]) * std::cos(QR_att[2]) * std::sin(QR_att[1]) + std::sin(QR_att[0]) * std::sin(QR_att[2]),
        std::cos(QR_att[0]) * std::sin(QR_att[2]) * std::sin(QR_att[1]) - std::sin(QR_att[0]) * std::cos(QR_att[2]),
        std::cos(QR_att[0]) * std::cos(QR_att[1])
    };

    float leso_err[3] = {
        QR_pos[0] - z1[0],
        QR_pos[1] - z1[1],
        QR_pos[2] - z1[2]};

    // get motor value and calculate force vector
    float scale_motor_value = 0.01;
    float thrust_U_B[3] = {
        0.,
        0.,
        (float) (std::pow(QR_omega[0] * scale_motor_value, 2) + std::pow(QR_omega[1] * scale_motor_value, 2) +
                 std::pow(QR_omega[2] * scale_motor_value, 2) + std::pow(QR_omega[3] * scale_motor_value, 2))};
    //printf("thrust_U_B = [ %f, %f, %f ]\n", thrust_U_B[0], thrust_U_B[1], thrust_U_B[2]);
    float thrust_U0 = 2.2059;
    float thrust_B[3] = {0., 0., thrust_U_B[2] * (float) 9.8 / thrust_U0};
    float thrust[3] = {0};
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, R_BI, 3, thrust_B, 1, 1.0, thrust, 1); // B to I
    // kappa
    float kappa[3] = {0, 0, -9.8};
    cblas_saxpy(3, 1.0, thrust, 1, kappa, 1); // kappa <- 1.0*thrust+G

    for (int i = 0; i < 3; i++) // 0 for roll, 1 for pitch, 2 for throttle
    {
        z1[i] += dt * (z2[i] + 3 * w0 * leso_err[i]);
        z2[i] += dt * (z3[i] + 3 * std::pow(w0, 2) * leso_err[i] + kappa[i]);
        z3[i] += dt * (std::pow(w0, 3) * leso_err[i]);
    }

    float a_v[3] = {-z3[0], -z3[1], -z3[2]};

    // convert a_v to wind vector
    //                 [ 1/c_x, 0, 0 ]
    // v = m * R_B^I * [ 0, 1/c_y, 0 ] * R_I^B * a_v
    //                 [ 0, 0, 1/c_z ]
    // u = qr_speed - v
    float c[9] = {(float) 1. / c_x, 0., 0.,
                  0., (float) 1. / c_y, 0.,
                  0., 0., (float) 1. / c_z};
    float c_B[9] = {0};
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1.0, R_BI, 3, c, 3, 0.0, c_B, 3); // B to I
    float c_BI[9] = {0};
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1.0, c_B, 3, R_IB, 3, 0.0, c_BI, 3); // B to I
    float v[3] = {0};
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, m, c_BI, 3, a_v, 1, 0.0, v, 1);
    float u[3];
    memcpy(u, QR_vel, 3 * sizeof(float));
    cblas_saxpy(3, -1.0, v, 1, u, 1); // u <- -1.0*v+QR_vel
    memcpy(wind_estimated, u, 3 * sizeof(float));

#if 0
    printf("pos = [ %f, %f, %f ]\n", QR_pos[0], QR_pos[1], QR_pos[2]);
    printf("z1  = [ %f, %f, %f ]\n", z1[0], z1[1], z1[2]);
    printf("z2  = [ %f, %f, %f ]\n", z2[0], z2[1], z2[2]);
    printf("z3  = [ %f, %f, %f ]\n", z3[0], z3[1], z3[2]);
    printf("u   = [ %f, %f, %f ]\n", kappa[0], kappa[1], kappa[2]);
    printf("a_v = [ %f, %f, %f ]\n", a_v[0], a_v[1], a_v[2]);
    printf("c_B = [ %f, %f, %f \n \
                    %f, %f, %f \n \
                    %f, %f, %f ]\n", c_B[0], c_B[1], c_B[2], c_B[3], c_B[4], c_B[5], c_B[6], c_B[7], c_B[8]);
    printf("v   = [ %f, %f, %f ]\n", v[0], v[1], v[2]);
    printf("u   = [ %f, %f, %f ]\n", u[0], u[1], u[2]);
#endif

#if 0 // calculate c
    // vel_B
    float vel_B[3];
    solve_linear_equations(3, 1, R_BI, vel_B, QR_vel); // I to B
    // av_B
    float av_B[3];
    solve_linear_equations(3, 1, R_BI, av_B, a_v); // I to B
    float c_est[3];
    for (int i = 0; i < 3; i++)
        c_est[i] = m*av_B[i]/vel_B[i];
    memcpy(wind_estimated, c_est, 3*sizeof(float));
#endif
}

/* End of file quadrotor.cxx */
