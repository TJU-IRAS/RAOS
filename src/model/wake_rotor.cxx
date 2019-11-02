/*
 * single-rotor wake model running on GPU
 *      Free vortex method
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-02 create this file (RAOS)
 *       2016-03-08 CPU too slow, modified to GPU
 */

#include <cmath>
#include <stdio.h>
#include <stdlib.h> // drand48 and srand48 on Linux
#include <string.h> // memset
#include "cblas.h" // CBLAS for vector arithmetic
#include "model/wake_rotor.h"
#include "common/math/rotation.h"

#ifndef RAD2DEG
#define RAD2DEG (180.0/M_PI)
#endif

#define Gamma_Ratio 4.0
#define R_Ratio 1

void RotorWake::init(void) // rotor wake initialization
{// by far config and rotor_state should be artificially configured

    // init vortex markers state
    max_markers = ceil(360.0 * config.rounds /
                       config.dpsi); // calculate max lagrangian markers of a vortex filament, 10.0 degree is an arbitrary number
#if defined(WAKE_IGE)
    wake_state = (std::vector<VortexMarker_t>**)malloc(rotor_state.frame.n_blades*2*sizeof(std::vector<VortexMarker_t>*));
#else
    wake_state = (std::vector<VortexMarker_t> **) malloc(
        rotor_state.frame.n_blades * sizeof(std::vector<VortexMarker_t> *));
#endif
    for (int i = 0; i < rotor_state.frame.n_blades; i++)
    {
        wake_state[i] = new std::vector<VortexMarker_t>;
        wake_state[i]->reserve(
            max_markers + ceil(360.0 / config.dpsi) + 10); // 10 is an arbitrary number, can be num>=1
#if defined(WAKE_IGE)
        wake_state[i+rotor_state.frame.n_blades] = new std::vector<VortexMarker_t>;
        wake_state[i+rotor_state.frame.n_blades]->reserve(max_markers+ceil(360.0/config.dpsi)+10); // 10 is an arbitrary number, can be num>=1
#endif
    }

#if defined(WAKE_BC_INIT)
    // initial wake geometry, using Landgrebe's prescribed wake model
    // < Principles of helicopter >, JG Leishman, page 604-606
    init_wake_geometry();
#else
    // release first group of markers (one for each blade) at the tips
    marker_release();
#endif
}

void RotorWake::maintain(const char *marker_maintain_type)
{/* marker maintainance */
    if (strcmp(marker_maintain_type, "one_by_one") == 0)
    {
#if defined(WAKE_BC_FAR)
        if (wake_state[0]->size() >= max_markers) {// consider far wake BC
            // maintain markers
            maintain_markers();
            // update far wake BC
            update_far_wake_bc();
        }
        else // do not consider far wake BC, develop to max_markers first
            maintain_markers();
#else
        maintain_markers();
#endif
    } else if (strcmp(marker_maintain_type, "turn_by_turn") == 0)
    {
        /* calculate average velocity at the rotor */
        int markers_per_turn = ceil(360.0 / config.dpsi);
        double sum_vel[3] = {0.0, 0.0, 0.0};
        float vel[3];
        int count = 0;
        for (int i = 0; i < rotor_state.frame.n_blades; i++)
            for (unsigned int j = wake_state[i]->size() - 1; j > wake_state[i]->size() - markers_per_turn - 1; j--)
            {
                count++;
                for (int k = 0; k < 3; k++)
                {
                    sum_vel[k] += wake_state[i]->at(j).vel[k];
                }
            }
        for (int i = 0; i < 3; i++)
            vel[i] = sum_vel[i] / double(count);
        /* release a turn of markers */
        VortexMarker_t new_marker;
        float pos_marker_v[3] = {rotor_state.frame.radius, 0, 0}; // vector used to calculate pos marker in body axis
        float pos_marker_p[3] = {0}; // position of marker in body axis
        memset(new_marker.vel, 0, sizeof(new_marker.vel));
        // calculate the tip vortex circulation: Gamma
        //new_marker.Gamma = -(rotor_state.direction)*2.0f*rotor_state.thrust/rotor_state.frame.n_blades/(rotor_state.frame.radius*rotor_state.frame.radius)/1.185f/rotor_state.Omega; // tip vortex circulation
        //new_marker.Gamma = -(rotor_state.direction)*2.0f*rotor_state.thrust/rotor_state.frame.n_blades/(rotor_state.frame.radius*rotor_state.frame.radius)/1.185f/rotor_state.Omega; // tip vortex circulation
        new_marker.Gamma = -(rotor_state.direction) * Gamma_Ratio * rotor_state.thrust / rotor_state.frame.n_blades /
                           (rotor_state.frame.radius * rotor_state.frame.radius) / 1.185f /
                           rotor_state.Omega; // tip vortex circulation
        // calculate initial radius of the tip vortex
        //new_marker.r_init = 0.03*rotor_state.frame.chord;
        new_marker.r_init = 0.03 * rotor_state.frame.chord;
        for (int i = 0; i < rotor_state.frame.n_blades; i++)
        {
            for (int j = markers_per_turn - 1; j >= 0; j--)
            {
                // calculate the position of this marker
                memcpy(new_marker.pos, rotor_state.pos, sizeof(new_marker.pos));
                memset(pos_marker_p, 0, sizeof(pos_marker_p));
                rotate_vector(pos_marker_v, pos_marker_p, 0, 0,
                              360.0 / rotor_state.frame.n_blades * i + rotor_state.psi +
                              (-rotor_state.direction) * config.dpsi * j);
                rotate_vector(pos_marker_p, new_marker.pos, rotor_state.att[0] * RAD2DEG, rotor_state.att[1] * RAD2DEG,
                              rotor_state.att[2] * RAD2DEG);
                for (int k = 0; k < 3; k++) // integrate
                    new_marker.pos[k] += vel[k] * j * (config.dpsi * M_PI / 180.0) / rotor_state.Omega;
                // calculate radius of vortex core
                new_marker.r = sqrt(new_marker.r_init * new_marker.r_init +
                                    R_Ratio * 4.0f * 1.25643f * 4.0 * 0.01834f * j * (config.dpsi * M_PI / 180.0) /
                                    rotor_state.Omega);
                // push to wake state array
                wake_state[i]->push_back(new_marker);
#if defined(WAKE_IGE)
                new_marker.pos[2] = -new_marker.pos[2];
                new_marker.Gamma = -new_marker.Gamma;
                wake_state[i+rotor_state.frame.n_blades]->push_back(new_marker);
                new_marker.Gamma = -new_marker.Gamma;
#endif
            }
        }
        /* remove a turn of markers */
        for (int i = 0; i < rotor_state.frame.n_blades; i++)
            if (wake_state[i]->size() > static_cast<unsigned int>(max_markers))
                for (int j = 0; j < markers_per_turn; j++)
                {
                    wake_state[i]->erase(wake_state[i]->begin());
#if defined(WAKE_IGE)
                    wake_state[i+rotor_state.frame.n_blades]->erase(wake_state[i+rotor_state.frame.n_blades]->begin());
#endif
                }
    }
}

#if defined(WAKE_BC_FAR)
void RotorWake::update_far_wake_bc(void) {
    float center[3] = {0.0, 0.0, 1000000.0}; // 1000000.0 is a number that is impossible
    float radius = 0.0;
    int markers_in_one_turn = ceil(360.0/config.dpsi);
    // calculate vortex ring center
    for (int i = 0; i < rotor_state.frame.n_blades; i++)
        for (int j = markers_in_one_turn-1; j >= 0; j--)
            for (int k = 0; k < 2; k++) // x,y
                center[k] += wake_state[i]->at(j).pos[k];
    for (int k = 0; k < 2; k++)
        center[k] = center[k]/(markers_in_one_turn*rotor_state.frame.n_blades);
    for (int k = 0; k < 2; k++) {
        if (center[0] != center[0] || center[1] != center[1]) { // NaN
            center[0] = rotor_state.pos[0];
            center[1] = rotor_state.pos[1];
        }
    }
    for (int i = 0; i < rotor_state.frame.n_blades; i++)
        for (int j = markers_in_one_turn-1; j >= 0; j--) {
            if (wake_state[i]->at(j).pos[2] < center[2])
                center[2] = wake_state[i]->at(j).pos[2];
        }
    far_wake.gap = wake_state[0]->at(0).pos[2]-wake_state[0]->at(markers_in_one_turn-1).pos[2];
    center[2] += far_wake.gap;
    // calculate vortex ring radius
    for (int i = 0; i < rotor_state.frame.n_blades; i++)
        for (int j = markers_in_one_turn-1; j >= 0; j--)
            radius += sqrtf(powf(wake_state[i]->at(j).pos[0]-center[0],2)
                + powf(wake_state[i]->at(j).pos[1]-center[1],2));
    radius = radius/(markers_in_one_turn*rotor_state.frame.n_blades);
    if (radius <= rotor_state.frame.radius || radius != radius)
        radius = rotor_state.frame.radius;
    // save to far wake state
    memcpy(&far_wake.center[0], &center[0], 3*sizeof(float));
    far_wake.radius = radius;
    far_wake.Gamma = wake_state[0]->at(0).Gamma;
    far_wake.core_radius = wake_state[0]->at(0).r;
    //far_wake.core_radius = 0.001 ;
    far_wake.initialized = true;
}
#endif

void RotorWake::maintain_markers(void)
{
    // release new markers
    marker_release();
    // remove old markers
    if (wake_state[0]->size() > static_cast<unsigned int>(max_markers))
    {
        for (int i = 0; i < rotor_state.frame.n_blades; i++)
        {
            wake_state[i]->erase(wake_state[i]->begin());
#if defined(WAKE_IGE)
            wake_state[i+rotor_state.frame.n_blades]->erase(wake_state[i+rotor_state.frame.n_blades]->begin());
#endif
        }
    }
}

void RotorWake::marker_release(void)
{

    VortexMarker_t new_marker;
    float pos_marker_v[3] = {rotor_state.frame.radius, 0, 0}; // vector used to calculate pos marker in body axis
    float pos_marker_p[3] = {0}; // position of marker in body axis

    // clear velocity
    memset(new_marker.vel, 0, sizeof(new_marker.vel));

    // calculate the tip vortex circulation: Gamma
    new_marker.Gamma = -(rotor_state.direction) * 2.0f * rotor_state.thrust / rotor_state.frame.n_blades /
                       (rotor_state.frame.radius * rotor_state.frame.radius) / 1.185f /
                       rotor_state.Omega; // tip vortex circulation

    // calculate initial radius of the tip vortex
    new_marker.r_init = 0.03 * rotor_state.frame.chord;
    new_marker.r = new_marker.r_init;

    for (int i = 0; i < rotor_state.frame.n_blades; i++)
    {
        // calculate the position of this marker
        memcpy(new_marker.pos, rotor_state.pos, sizeof(new_marker.pos));
        memset(pos_marker_p, 0, sizeof(pos_marker_p));
        rotate_vector(pos_marker_v, pos_marker_p, 0, 0, 360.0 / rotor_state.frame.n_blades * i + rotor_state.psi);
        rotate_vector(pos_marker_p, new_marker.pos, rotor_state.att[0] * RAD2DEG, rotor_state.att[1] * RAD2DEG,
                      rotor_state.att[2] * RAD2DEG);
        // push to wake state array
        wake_state[i]->push_back(new_marker);
#if defined(WAKE_IGE)
        new_marker.pos[2] = -new_marker.pos[2];
        new_marker.Gamma = -new_marker.Gamma;
        wake_state[i+rotor_state.frame.n_blades]->push_back(new_marker);
        new_marker.Gamma = -new_marker.Gamma;
#endif
    }

    // update blade azimuth
    rotor_state.psi += (rotor_state.direction) * config.dpsi;
    if (rotor_state.psi >= 360.0)
        rotor_state.psi -= 360.0;
}

#if defined(WAKE_BC_INIT)
void RotorWake::init_wake_geometry(void) {
    VortexMarker_t new_marker;
    float pos_marker_v[3] = {0}; // vector used to calculate pos marker in body axis
    float pos_marker_p[3] = {0}; // position of marker in body axis
    float z_tip, r_tip, C_T, sigma, psi_rad;

    // clear velocity
    memset(new_marker.vel, 0, sizeof(new_marker.vel));

    // calculate the tip vortex circulation: Gamma
    new_marker.Gamma = -(rotor_state.direction)*2.0f*rotor_state.thrust/rotor_state.frame.n_blades/(rotor_state.frame.radius*rotor_state.frame.radius)/1.185f/rotor_state.Omega; // tip vortex circulation

    // calculate initial radius of the tip vortex
    new_marker.r_init = 0.1*rotor_state.frame.chord;
    new_marker.r = new_marker.r_init;

    // calculate thrust coefficient and sigma
    C_T = rotor_state.thrust/(1.185f*M_PI*pow(rotor_state.Omega,2)*pow(rotor_state.frame.radius,4));
    sigma = rotor_state.frame.n_blades*rotor_state.frame.chord/(M_PI*rotor_state.frame.radius);

    for (int j = max_markers-1; j >= 0; j--) {
        // calculate psi
        psi_rad = j*config.dpsi*M_PI/180.0f;
        // calculate r_tip
        r_tip = rotor_state.frame.radius*(0.78f+(1.0f-0.78f)*exp(-(0.145+27*C_T)*psi_rad));
        pos_marker_v[0] = r_tip;
        // calculate z_tip
        if (psi_rad <= 2*M_PI/rotor_state.frame.n_blades)
            z_tip = rotor_state.frame.radius*(psi_rad*(-0.25)*(C_T/sigma)); // assume blade twist is 10 deg
        else
            z_tip = rotor_state.frame.radius*(
                    2*M_PI/rotor_state.frame.n_blades*(-0.25)*(C_T/sigma)
                    + (psi_rad-2*M_PI/rotor_state.frame.n_blades)
                    * (-1.41*sqrt(C_T/2.0)));
        for (int i = 0; i < rotor_state.frame.n_blades; i++) {
            // calculate the position of this marker
            memcpy(new_marker.pos, rotor_state.pos, sizeof(new_marker.pos));
            new_marker.pos[2] += z_tip;
            rotor_state.psi = -(rotor_state.direction)*(psi_rad - floor(psi_rad/(2*M_PI))*(2*M_PI))*180.0f/M_PI;
            memset(pos_marker_p, 0, sizeof(pos_marker_p));
            rotate_vector(pos_marker_v, pos_marker_p, 0, 0, 360.0/rotor_state.frame.n_blades*i+rotor_state.psi);
            rotate_vector(pos_marker_p, new_marker.pos, rotor_state.att[0]*RAD2DEG, rotor_state.att[1]*RAD2DEG, rotor_state.att[2]*RAD2DEG);
            // push to wake state array
            wake_state[i]->push_back(new_marker);
        }
    }
    // save psi
}
#endif

// Constructor
RotorWake::RotorWake(void)
{
    // init configurations
    config.rounds = 20; // 20 rounds
    config.dpsi = 20; // 10 deg

    // init rotor state
    rotor_state.frame.radius = 0.1; // meter
    rotor_state.frame.chord = 0.01; // meter
    rotor_state.frame.n_blades = 2; // two-blade
    memset(rotor_state.pos, 0, sizeof(rotor_state.pos));
    memset(rotor_state.att, 0, sizeof(rotor_state.att));
    rotor_state.Omega = 50 * 2 * M_PI; // rad/s
    rotor_state.psi = 0;
    rotor_state.thrust = 1.0; // 1 N, ~100 g
    rotor_state.direction = WAKE_ROTOR_CLOCKWISE;

    max_markers = 1000;

#if defined(WAKE_BC_FAR)
    far_wake.initialized = false;
#endif
}

void RotorWake::destroy(void)
{
    // destroy wake_state
    for (int i = 0; i < rotor_state.frame.n_blades; i++)
    {
        wake_state[i]->clear();
        delete wake_state[i];
    }
    free(wake_state);
}

/* End of file wake_rotor.cxx */
