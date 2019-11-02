/*
 * single-rotor wake model running on GPU
 *      Free vortex method
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-02 create this file (RAOS)
 *       2016-03-08 CPU too slow, modified to GPU
 */

#ifndef WAKE_ROTOR_H
#define WAKE_ROTOR_H

#include <vector>

//#define WAKE_BC_INIT // consider initial boundary condition
//#define WAKE_BC_FAR // consider far wake boundary condition
//#define WAKE_IGE // consider in-ground-effect

#define WAKE_ROTOR_CLOCKWISE 1.0f
#define WAKE_ROTOR_CCLOCKWISE (-1.0f)

typedef struct
{
    float radius; // propeller radius
    float chord;
    int n_blades; // number of blades
} RotorFrame_t; // mechanical frame (constant)

typedef struct
{
    float pos[3];    /* position coordinate (earth axis x), volatile */
    float att[3]; // [yaw, pitch, roll]
    float Omega; // rotation angular speed, volatile
    float direction; // rotation direction, 1.0 for clockwise, -1.0 for counter-clockwise
    float psi; // blade azimuth, [0, 360), volatile
    float thrust; // the thrust this rotor produce, N
    RotorFrame_t frame; // const
} RotorState_t;

typedef struct
{
    float pos[3]; // 3D position of this marker
    float vel[3]; // 3D velocity of this marker
    float Gamma; // circulation of i-th segment (i-th marker to (i+1)-th marker)
    float r; // vortex core radius
    float r_init; // r_0
} VortexMarker_t;

typedef struct
{
    float rounds; // number of revolutions (number of vortex rings to maintain) 
    float dpsi; // delta azimuth angle to release/maintain marker
} RotorWakeConfig_t; // const since sim init

#if defined(WAKE_BC_FAR)
typedef struct {// use vortex ring to describe far wake
    float center[3]; // center of vortex ring
    float radius; // radius of vortex ring
    float Gamma;
    float core_radius;
    float gap; // distance to the latest marker
    bool initialized;
}FarWakeState_t; // state of far wake BC
#endif

class RotorWake
{
public:
    RotorWake(void); // constructor
    void init(void); // rotor wake init
    void maintain(const char *); // marker release & remove, to maintain a fixed length of vortex filament
    void destroy(void);

    std::vector<VortexMarker_t> **wake_state; // wake_state[i] is the pointer of a std::vector, which contains the markers of a whole filament
    RotorWakeConfig_t config; // configures of rotor wake simulation
    RotorState_t rotor_state;
#if defined(WAKE_BC_FAR)
    FarWakeState_t far_wake;
#endif
    int max_markers;
    float dt;

private:
    void marker_release(void);

    void maintain_markers(void);

#if defined(WAKE_BC_FAR)
    void update_far_wake_bc(void);
#endif
#if defined(WAKE_BC_INIT)
    void init_wake_geometry(void);
#endif
};

#endif
/* End of file wake_rotor.h */
