/*
 * Wakes in the simulation environment
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-08 create this file (RAOS)
 */

#include <stdio.h>
#include "model/robot.h"
#include "model/wake_rotor.h"
#include "model/error_cuda.h"
#include "model/plume.h" // for wake-induced velocity computation of plume puffs
#include "model/SimModel.h"
#include "model/environment.h"

#define PI 3.14159265358979323846

// storing cuda device properties, for the sheduling of parallel computing
static cudaDeviceProp prop;

// data
VortexMarker_t *wake_markers; // on-host array containing all of the markers, for convenience mem copying with GPU
VortexMarker_t *dev_wake_markers; // on-device array ...
VortexMarker_t *dev_wake_markers_mediate; // on-device array ...
int *idx_end_marker_fila; // on-host array containing index of endpoint of all vortex filaments, for parallel computing
int *dev_idx_end_marker_fila; // on-device array ...
#if defined(WAKE_BC_FAR)
FarWakeState_t* far_wakes;
FarWakeState_t* dev_far_wakes; // on-device array containing the far wake states of rotors
#endif
// wind
typedef struct
{
    float v[3];
} Wake_FreeStreamVector_t;
Wake_FreeStreamVector_t *free_stream;
Wake_FreeStreamVector_t *dev_free_stream; // on-device ... free stream vector at robots' position
int *idx_end_marker_robot; // endpoint of vortices of robots
int *dev_idx_end_marker_robot;

// vel += biot savert induction from segement a-b to position p
__device__ float3
biot_savart_induction(VortexMarker_t
a,
VortexMarker_t b, float3
p,
float3 vel
)
{
float3 ap, bp, ab, ba, ind;
float dbnrm_ab, dbnrm_ap, dbnrm_bp; // |vector|^2
float cos_apab, cos_babp, db_sin_apab; // cos(ap,ab) cos(ba,bp) sin(ap,ab)^2
float db_h, scale; // h^2, |ind|

// if marker a or b is at p, pass
if ( a.pos[0]==p.
x &&a
.pos[1]==p.
y &&a
.pos[2]==p.z ||
b.pos[0]==p.
x &&b
.pos[1]==p.
y &&b
.pos[2]==p.z ||
a.pos[0]==b.pos[0] && a.pos[1]==b.pos[1] && a.pos[2]==b.pos[2]) {
return
vel;
}

// get vectors AP, BP and AB & BA  [12 FLOPS]
//  AP [3 FLOPS]
ap.
x = p.x - a.pos[0];
ap.
y = p.y - a.pos[1];
ap.
z = p.z - a.pos[2];
/* // __fsub_ru() costs equal to simply minus
ap.x = __fsub_ru(p.x, a.pos[0]);
ap.y = __fsub_ru(p.y, a.pos[1]);
ap.z = __fsub_ru(p.z, a.pos[2]);
*/
//  BP [3 FLOPS]
bp.
x = p.x - b.pos[0];
bp.
y = p.y - b.pos[1];
bp.
z = p.z - b.pos[2];
/*
bp.x = __fsub_ru(p.x, b.pos[0]);
bp.y = __fsub_ru(p.y, b.pos[1]);
bp.z = __fsub_ru(p.z, b.pos[2]);
*/
//  AB [3 FLOPS]
ab.
x = b.pos[0] - a.pos[0];
ab.
y = b.pos[1] - a.pos[1];
ab.
z = b.pos[2] - a.pos[2];
/*
ab.x = __fsub_ru(b.pos[0], a.pos[0]);
ab.y = __fsub_ru(b.pos[1], a.pos[1]);
ab.z = __fsub_ru(b.pos[2], a.pos[2]);
*/
//  BA [3 FLOPS]
ba.
x = a.pos[0] - b.pos[0];
ba.
y = a.pos[1] - b.pos[1];
ba.
z = a.pos[2] - b.pos[2];
/*
ba.x = __fsub_ru(a.pos[0], b.pos[0]);
ba.y = __fsub_ru(a.pos[1], b.pos[1]);
ba.z = __fsub_ru(a.pos[2], b.pos[2]);
*/

// cos(ap-ab) and cos(ba-bp) [31 FLOPS]
dbnrm_ab = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z; // [5 FLOPS]
dbnrm_ap = ap.x * ap.x + ap.y * ap.y + ap.z * ap.z; // [5 FLOPS]
dbnrm_bp = bp.x * bp.x + bp.y * bp.y + bp.z * bp.z; // [5 FLOPS]
/* // using __powf(x,y) to calculate squares costs more than simply multiply
dbnrm_ab = __powf(ab.x,2) + __powf(ab.y,2) + __powf(ab.z,2);
dbnrm_ap = __powf(ap.x,2) + __powf(ap.y,2) + __powf(ap.z,2);
dbnrm_bp = __powf(bp.x,2) + __powf(bp.y,2) + __powf(bp.z,2);
*/
cos_apab = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) * __frsqrt_rn(dbnrm_ap * dbnrm_ab); // [8 FLOPS]
cos_babp = (ba.x * bp.x + ba.y * bp.y + ba.z * bp.z) * __frsqrt_rn(dbnrm_ab * dbnrm_bp); // [8 FLOPS]

// h, perpendicular distance from P to AB [3 FLOPS]
db_sin_apab = 1 - cos_apab * cos_apab; //[2 FLOPS]
if (db_sin_apab <= 0.0f) // sometimes |cos_apab| will be slightly larger than 1.0f due to computation err
db_h = 0.0f;
else {
db_h = dbnrm_ap * db_sin_apab; // [1 FLOPS]
}

// strength
scale = a.Gamma * db_h * 10000.0f * __frsqrt_rn(__powf(a.r * 100, 4) + db_h * db_h * 10000.0f * 10000.0f) *
        (cos_apab + cos_babp) / (4 * PI);

// get induced velocity
ind.
x = ap.y * bp.z - ap.z * bp.y;
ind.
y = ap.z * bp.x - ap.x * bp.z;
ind.
z = ap.x * bp.y - ap.y * bp.x;
scale = scale * __frsqrt_rn(ind.x * ind.x + ind.y * ind.y + ind.z * ind.z);
ind.
x = scale * ind.x;
ind.
y = scale * ind.y;
ind.
z = scale * ind.z;

// add induced velocity [3 FLOPS]
vel.x += ind.
x;
vel.y += ind.
y;
vel.z += ind.
z;

return
vel;
}

__device__ float3
tile_calculation_vel_markers
(float3
pos,
float3 vel,
int tile,
int tile_size,
int row_sgmts, VortexMarker_t
* markers,
int *index_end,
int num_fila,
int num_markers
)
{
int i; // the i-th colum marker in this tile
int idx_m; // the idx_m -th colum marker of whole N markers
int idx_fila; // index of fila
int shared_mem_offset = (tile % row_sgmts) * tile_size;

bool isend; // the marker to be calculated is an end point or not

extern __shared__ VortexMarker_t
tile_markers[];

// if this tile is not the first tile, we will use the last tile's end
if (tile > 0)
{
isend = false;
for (
idx_fila = 0;
idx_fila<num_fila;
idx_fila++) {
if (
tile *tile_size
-1 == index_end[idx_fila]) {
isend = true;
break;
}
}
if (isend == false &&
tile *tile_size<num_markers
)
vel = biot_savart_induction(markers[tile * tile_size - 1], tile_markers[shared_mem_offset], pos, vel);
}

for (
i = 0;
i<tile_size-1; i++)
{// compule all colum markers in this tile
idx_m = tile * tile_size + i;
if (idx_m >= num_markers-1)
break;
else
{
isend = false;
// check whether this marker is end point or not
for (
idx_fila = 0;
idx_fila<num_fila;
idx_fila++) {
if (idx_m == index_end[idx_fila]) {
isend = true;
break;
}
}
if (isend == false) // if this marker is not an end point
vel = biot_savart_induction(tile_markers[shared_mem_offset + i], tile_markers[shared_mem_offset + i + 1], pos, vel);
}
}

return
vel;
}

/* calculate velocities of markers, running on GPU
Note:
    This routine uses one-dimensional blocks and threads
    row_sgms is the number of segments to parallelly compute a row
    blockDim must equal to tile_size*row_sgmts
    gridDim == (num_markers + tile_size - 1) / tile_size
 */
__global__ void CalculateVelofMarkers(VortexMarker_t * markers, int * idx_end, int
num_fila,
int num_markers,
int tile_size,
int row_sgmts, Wake_FreeStreamVector_t
* wind,
int *idx_end_robot,
int num_robot
)
{
extern __shared__ VortexMarker_t
tile_markers[];

int idx, i;
float3 pos; // position of vortex marker to calculate velocity in this thread
float3 vel = {0.0f, 0.0f, 0.0f}; // velocity of this marker
int row = threadIdx.x % tile_size + blockIdx.x * tile_size; // get row number (the index of marker to be evaluated)

// get the marker of which the velocity to be calculated
if (row<num_markers) {
pos.
x = markers[row].pos[0];
pos.
y = markers[row].pos[1];
pos.
z = markers[row].pos[2];
}

// compute each tile
i = 0;
while (true) // every thread can enter this loop
{
idx = threadIdx.x + blockDim.x * i;
if (idx<num_markers)
tile_markers[threadIdx.x] = markers[idx]; // copy markers to shared mem for tile calculation
__syncthreads(); // make sure the shared mem has been loaded
if (row<num_markers)
vel = tile_calculation_vel_markers(pos, vel, idx / tile_size, tile_size, row_sgmts, markers, idx_end, num_fila,
                                   num_markers);

__syncthreads(); // make sure every thread has done the calc of this tile

if (blockDim.
x *(i
+1) < num_markers)
i++;
else
break;
}

// save the result of this segment to the end of the shared mem
//  at present the shared memory is of no use for computation, so it can be used to temporarily store the segment results
if (row<num_markers) {
tile_markers[threadIdx.x].vel[0] = vel.
x;
tile_markers[threadIdx.x].vel[1] = vel.
y;
tile_markers[threadIdx.x].vel[2] = vel.
z;
}

__syncthreads(); // make sure the segment results are stored

if (threadIdx.x<tile_size && row < num_markers) {
// sum the velocities computed by multiple (num of row_sgms) threads
vel.
x = 0.0f;
vel.
y = 0.0f;
vel.
z = 0.0f;
for (
i = 0;
i<row_sgmts;
i++) {
vel.x += tile_markers[threadIdx.x%tile_size+
i *tile_size
].vel[0];
vel.y += tile_markers[threadIdx.x%tile_size+
i *tile_size
].vel[1];
vel.z += tile_markers[threadIdx.x%tile_size+
i *tile_size
].vel[2];
}
// Save the result in global memory for the integration step.
/*TODO:*/
for (
i = 0;
i<num_robot;
i++) {
if (row <= idx_end_robot[i]) {
markers[row].vel[0] = vel.x + wind[i].v[0];
markers[row].vel[1] = vel.y + wind[i].v[1];
markers[row].vel[2] = vel.z + wind[i].v[2];
break;
}
}
}
}

/* calculate new positions of markers, running on GPU 
    Forward Euler  
 */
__global__ void CalculatePosofMarkersFE(VortexMarker_t * markers, int
num_markers,
float dt
)
{
VortexMarker_t mkr;
int tid = threadIdx.x + blockIdx.x * blockDim.x;

if (tid<num_markers) {
mkr = markers[tid];

for (
int i = 0;
i < 3; i++)
mkr.pos[i] += mkr.vel[i]*
dt;
}
if (tid<num_markers)
markers[tid] =
mkr;
}

/* average velocity of two sequence of markers
   the number of markers should be the same
   This routine is used for the correction step of predictor-corrector integration
   scheme of FVM, which involves the average computation of previous velocity and
   predicted velocity

   the vel of array markers and markers_temp will be averaged and stored in markers
 */
__global__ void AverageVelofMarkers(VortexMarker_t * markers, VortexMarker_t * markers_temp, int
num_markers)
{
VortexMarker_t mkr, mkr_temp;
int tid = threadIdx.x + blockIdx.x * blockDim.x;

if (tid<num_markers) {
mkr = markers[tid];
mkr_temp = markers_temp[tid];

for (
int i = 0;
i < 3; i++)
mkr.vel[i] = 0.5*(mkr.vel[i] + mkr_temp.vel[i]);
}
if (tid<num_markers)
markers[tid] =
mkr;
}

/* calculate vortex core radius of markers */
__global__ void CalculateVtxCoreofMarkers(VortexMarker_t * markers, int
num_markers,
int *idx_end,
int num_fila,
float dt
)
{
VortexMarker_t mkr;
float t; // time t, second
int fila_start = 0, fila_end = idx_end[num_fila - 1]; // start and end index of a filament in markers
int tid = threadIdx.x + blockIdx.x * blockDim.x;

if (tid<num_markers) {
mkr = markers[tid];

// calculate life time of this marker (vortex segment)
for (
int i = 0;
i<num_fila;
i++) {
if (idx_end[i] < tid) { // find which filament this marker belongs to
if (idx_end[i] > fila_start)
fila_start = idx_end[i];
}
else {
if (idx_end[i] < fila_end)
fila_end = idx_end[i];
}
}
t = (fila_end - tid) * dt;
/* r_c(t) = sqrt( r_init^2 + 4*alpha*delta*nu*t )
   Here we assume r_init is 0.005 m
   alpha is Lamb's constant which is 1.25643
   delta is is a function of vortex Reynolds number, delta > 1 (laminar when delta = 1)
   here we assume delta = 8
   nu is viscous constant of air at 25 degree temperature: 0.01834
*/
mkr.
r = __fsqrt_ru(mkr.r_init * mkr.r_init + 4.0f * 1.25643f * 4.0 * 0.01834 * t);
}
if (tid<num_markers)
markers[tid] =
mkr;
}

__device__ float complete_elliptic_int_first(float k)
{
    return PI / 2.0 * (1.0 + 0.5 * 0.5 * k * k + 0.5 * 0.5 * 0.75 * 0.75 * __powf(k, 4));
}

__device__ float complete_elliptic_int_second(float k)
{
    return PI / 2.0 * (1.0 - 0.5 * 0.5 * k * k - 0.5 * 0.5 * 0.75 * 0.75 * __powf(k, 4) / 3.0);
}

__device__ void
induced_velocity_vortex_ring(float *center, float radius, float Gamma, float core_radius, VortexMarker_t *mkr)
{
    float op[3], op_z, op_r, db_op_z, db_op_r;
    float u_z, u_r, m, a, b;
    float db_radius, db_delta;

    db_radius = radius * radius;
    db_delta = core_radius * core_radius;
    // vector op
    for (int j = 0; j < 3; j++)
        op[j] = mkr->pos[j] - center[j];
    if (op[0] == 0.0f && op[1] == 0.0f && op[2] == 0.0f) // P is at center
        mkr->vel[2] += Gamma / (2 * radius);
    else
    {
        // op_z, cylindrical coord
        op_z = op[2]; // implies ring direction of (0, 0, 1)
        db_op_z = op_z * op_z;
        // op_r, cylindrical coord
        db_op_r = op[0] * op[0] + op[1] * op[1] + op[2] * op[2] - op_z * op_z;
        op_r = __fsqrt_ru(db_op_r);
        // a, A, m
        a = __fsqrt_ru((op_r + radius) * (op_r + radius) + db_op_z + db_delta);
        b = (op_r - radius) * (op_r - radius) + db_op_z + db_delta;
        m = 4 * op_r * radius / (a * a);
        // u_z, cylindrical coord 
        u_z = Gamma / (2 * PI * a) * ((-(db_op_r - db_radius + db_op_z + db_delta) / b)
                                      * complete_elliptic_int_second(m) + complete_elliptic_int_first(m));
        // u_r, cylindrical coord
        u_r = Gamma * op_z / (2 * PI * op_r * a) * (((db_op_r + db_radius + db_op_z + db_delta) / b)
                                                    * complete_elliptic_int_second(m) - complete_elliptic_int_first(m));
        // map u_z, u_r to cartesian coord
        mkr->vel[2] += u_z; // downward
        mkr->vel[0] += op[0] * u_r / __fsqrt_ru(op[0] * op[0] + op[1] * op[1]);
        mkr->vel[1] += op[1] * u_r / __fsqrt_ru(op[0] * op[0] + op[1] * op[1]);
    }
}

#if defined(WAKE_BC_FAR)
/* calculate & add induced velocity of far wake BC */
__global__ void CalculateIndVelofFarWakeBC(VortexMarker_t* markers, int num_markers, FarWakeState_t* rings, int num_rings)
{
    VortexMarker_t mkr;
    float radius, Gamma; // radius and strength of a vortex ring 
    float center[3]; // center of the vortex ring
    int tid = threadIdx.x + blockIdx.x * blockDim.x;

    if (tid < num_markers) {
        mkr = markers[tid];
        /* calculate induced velocity of far wake vortex ring */
        for (int i = 0; i < num_rings; i++) {
            if (rings[i].initialized == true) {
                radius = rings[i].radius;
                Gamma = __fsqrt_ru(rings[i].Gamma*rings[i].Gamma); // Gamma>=0
                center[0] = rings[i].center[0];
                center[1] = rings[i].center[1];
                for (int j = 0; j < 1; j++) {
                    center[2] = rings[i].center[2] + j*rings[i].gap;
                    induced_velocity_vortex_ring(&center[0], radius, Gamma, rings[i].core_radius, &mkr);
                }
            }
        }
    }
    if (tid < num_markers)
        markers[tid] = mkr;
}
#endif

/* update all of the rotor wakes in the environment
 * this routine will traverse all of the robot instances
 * and update all of the Lagrangian markers for all robots */
void WakesUpdate(std::vector<Robot *> *robots, const char *integration_scheme, SimState_t *sim_state,
                 SimEnvInfo *sim_env_info)
{
    unsigned int idx_robot, idx_rotor;
    int idx_blade;
    int num_blade = 0;
    int addr_cp_markers = 0; // index for copy marker states from rotor wake to wake_markers
#if defined(WAKE_BC_FAR)
    int addr_cp_rotors = 0;
#endif

/* Step 1: update velocity & position of markers */

    // Phase 1: collect all vortex markers to a memory, for GPU computing
    //  the markers are placed contiguously, fila to fila
    for (idx_robot = 0; idx_robot < robots->size(); idx_robot++)
    {
        for (idx_rotor = 0; idx_rotor < robots->at(idx_robot)->wakes.size(); idx_rotor++)
        {
#if defined(WAKE_BC_FAR)
            memcpy(&far_wakes[addr_cp_rotors++], &(robots->at(idx_robot)->wakes.at(idx_rotor)->far_wake), sizeof(FarWakeState_t));
#endif
            for (idx_blade = 0;
                 idx_blade < robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades; idx_blade++)
            {
                std::copy(robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->begin(),
                          robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->end(),
                          &wake_markers[addr_cp_markers]);
                addr_cp_markers += robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->size();
                idx_end_marker_fila[num_blade] = addr_cp_markers - 1; // the address of the last element, hence -1
                num_blade++;
            }
        }
        idx_end_marker_robot[idx_robot] = addr_cp_markers - 1; // the address of the last element of a robot
    }// traversed all rotor wakes and got total number of markers
#if defined(WAKE_IGE)
    for(idx_robot = 0; idx_robot < robots->size(); idx_robot++) {
        for (idx_rotor = 0; idx_rotor < robots->at(idx_robot)->wakes.size(); idx_rotor++) {
            for (idx_blade = 0; idx_blade < robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades; idx_blade++) {
                std::copy(robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade+robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades]->begin(),
                    robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade+robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades]->end(),
                    &wake_markers[addr_cp_markers]);
                addr_cp_markers += robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade+robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades]->size();
                idx_end_marker_fila[num_blade] = addr_cp_markers-1; // the address of the last element, hence -1
                num_blade++;
            }
        }
    }
#endif
    // get wind vectors
    for (unsigned int i = 0; i < robots->size(); i++)
        sim_env_info->measure_wind(robots->at(i)->state.pos, free_stream[i].v);

    // Phase 2: copy array wake_markers & idx_wake_markers to GPU's dev_wake_markers
    HANDLE_ERROR(cudaMemcpy(dev_wake_markers, wake_markers,
                            addr_cp_markers * sizeof(VortexMarker_t), cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(dev_idx_end_marker_fila, idx_end_marker_fila,
                            num_blade * sizeof(int), cudaMemcpyHostToDevice));
#if defined(WAKE_BC_FAR)
    HANDLE_ERROR( cudaMemcpy(dev_far_wakes, far_wakes,
                addr_cp_rotors*sizeof(FarWakeState_t), cudaMemcpyHostToDevice) );
#endif
    HANDLE_ERROR(cudaMemcpy(dev_free_stream, free_stream,
                            robots->size() * sizeof(Wake_FreeStreamVector_t), cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(dev_idx_end_marker_robot, idx_end_marker_robot,
                            robots->size() * sizeof(int), cudaMemcpyHostToDevice));

    // Phase 3: parallel computing
    //  determine threads per block and blocks number, at present addr_cp_markers contains total num of markers
    int p, q, threads, blocks;
    p = prop.warpSize; // tile size
    q = 4; // number of segments of a row
    threads = std::min(p * q, prop.maxThreadsPerBlock);

    //  launch gpu computing
    //   the end address of shared memory contains the end element of 'last' tile for 'next' tile computation
    cudaError_t err;

    //  <1> calculate velocity of Lagrangian markers, n-1 state
    blocks = (addr_cp_markers + p - 1) / p;
    //  Note: here omitted checks for max number of blocks, since in RAO problem the vortex markers
    //        rarely exceeds 65535*threads.
    CalculateVelofMarkers << < blocks, threads, (threads) * sizeof(VortexMarker_t) >> >
                                                (dev_wake_markers, dev_idx_end_marker_fila, num_blade, addr_cp_markers, p, q, dev_free_stream, dev_idx_end_marker_robot, robots->size());
    err = cudaGetLastError();
    if (err != cudaSuccess)
        printf("Error: %s\n", cudaGetErrorString(err));
    //      add far wake boundary condition effect
#if defined(WAKE_BC_FAR)
    blocks = (addr_cp_markers + threads -1)/threads;
    CalculateIndVelofFarWakeBC<<<blocks, threads>>>(dev_wake_markers, addr_cp_markers, dev_far_wakes, addr_cp_rotors);
    err = cudaGetLastError();
    if (err != cudaSuccess)
        printf("Error: %s\n", cudaGetErrorString(err));
#endif

    //  <2> calculate position of Lagrangian markers, predict
    blocks = (addr_cp_markers + threads - 1) / threads;
    CalculatePosofMarkersFE << < blocks, threads >> > (dev_wake_markers, addr_cp_markers, sim_state->dt);
    err = cudaGetLastError();
    if (err != cudaSuccess)
        printf("Error: %s\n", cudaGetErrorString(err));

    if (strcmp(integration_scheme, "PC") == 0) // predictor-corrector
    {
        //  <3> calculate mediate velocity of predicted positions for corrector
        HANDLE_ERROR(cudaMemcpy(dev_wake_markers_mediate, dev_wake_markers,
                                addr_cp_markers * sizeof(VortexMarker_t), cudaMemcpyDeviceToDevice));
        blocks = (addr_cp_markers + p - 1) / p;
        CalculateVelofMarkers << < blocks, threads, (threads) * sizeof(VortexMarker_t) >> >
                                                    (dev_wake_markers_mediate, dev_idx_end_marker_fila, num_blade, addr_cp_markers, p, q, dev_free_stream, dev_idx_end_marker_robot, robots->size());
        err = cudaGetLastError();
        if (err != cudaSuccess)
            printf("Error: %s\n", cudaGetErrorString(err));
        //  <4> calculate final position of markers, correct
        blocks = (addr_cp_markers + threads - 1) / threads;
        AverageVelofMarkers << < blocks, threads >> > (dev_wake_markers, dev_wake_markers_mediate, addr_cp_markers);
        err = cudaGetLastError();
        if (err != cudaSuccess)
            printf("Error: %s\n", cudaGetErrorString(err));
        CalculatePosofMarkersFE << < blocks, threads >> > (dev_wake_markers, addr_cp_markers, sim_state->dt);
        err = cudaGetLastError();
        if (err != cudaSuccess)
            printf("Error: %s\n", cudaGetErrorString(err));
    }
    //  <5> calculate vortex core growth
    //    the argument num_blade, num_markers and dt are used to compute lifetime
    blocks = (addr_cp_markers + threads - 1) / threads;
    CalculateVtxCoreofMarkers << < blocks, threads >> >
                                           (dev_wake_markers, addr_cp_markers, dev_idx_end_marker_fila, num_blade,
                                               20.0 / 360.0 / 50.0);
    err = cudaGetLastError();
    if (err != cudaSuccess)
        printf("Error: %s\n", cudaGetErrorString(err));

    // Phase 4: retrieve data dev_wake_markers from GPU to wake_markers
    HANDLE_ERROR(cudaMemcpy(wake_markers, dev_wake_markers,
                            addr_cp_markers * sizeof(VortexMarker_t), cudaMemcpyDeviceToHost));

    // Phase 5: GPU computed, distribute the memory to every rotor wake states
    addr_cp_markers = 0;
    for (idx_robot = 0; idx_robot < robots->size(); idx_robot++)
    {
        for (idx_rotor = 0; idx_rotor < robots->at(idx_robot)->wakes.size(); idx_rotor++)
        {
            for (idx_blade = 0;
                 idx_blade < robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades; idx_blade++)
            {
                std::copy(&wake_markers[addr_cp_markers],
                          &wake_markers[addr_cp_markers +
                                        robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->size()],
                          robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->data());
                addr_cp_markers += robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->size();
            }
        }
    }
#if defined(WAKE_IGE)
    for(idx_robot = 0; idx_robot < robots->size(); idx_robot++) {
        for (idx_rotor = 0; idx_rotor < robots->at(idx_robot)->wakes.size(); idx_rotor++) {
            for (idx_blade = 0; idx_blade < robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades; idx_blade++) {
                std::copy(&wake_markers[addr_cp_markers],
                    &wake_markers[addr_cp_markers+robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade+robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades]->size()],
                robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade+robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades]->data());
                addr_cp_markers += robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade+robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades]->size();
            }
        }
    }
#endif


/* Step 2: maintain markers of wakes */
    for (idx_robot = 0; idx_robot < robots->size(); idx_robot++)
    {
        for (idx_rotor = 0; idx_rotor < robots->at(idx_robot)->wakes.size(); idx_rotor++)
        {
            if (sim_state->wake_initialized)
                robots->at(idx_robot)->wakes.at(idx_rotor)->maintain("turn_by_turn");
            else
                robots->at(idx_robot)->wakes.at(idx_rotor)->maintain("one_by_one");
        }
    }
}

/* init parallelization of wakes computation */
void WakesInit(std::vector<Robot *> *robots)
{
    /* print the properties of all the graphic cards this machine has */
    int count; // number of devices
    HANDLE_ERROR(cudaGetDeviceCount(&count));
    for (int i = 0; i < count; i++)
    {// print out info of all graphic cards
        HANDLE_ERROR(cudaGetDeviceProperties(&prop, i));
        printf("======== Card %d ========\n", i + 1);
        printf("Graphic card name: %s\n", prop.name);
        printf("Compute Capability: %d.%d\n", prop.major, prop.minor);
        printf("Total global memory: %ld MByte\n", prop.totalGlobalMem / 1024 / 1024);
        printf("Total constant memoty: %ld kByte\n", prop.totalConstMem / 1024);
        printf("Multiprocessor count: %d\n", prop.multiProcessorCount);
        printf("Shared mem per mp: %ld\n", prop.sharedMemPerBlock);
        printf("Registers per mp: %d\n", prop.regsPerBlock);
        printf("Threads in warp: %d\n", prop.warpSize);
        printf("Max threads per block: %d\n", prop.maxThreadsPerBlock);
        printf("Max thread dimensions: (%d, %d, %d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1],
               prop.maxThreadsDim[2]);
        printf("Max grid dimensions: (%d, %d, %d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
        printf("\n");
    }
    if (count > 1)
    {// multiple graphic cards
        if (count == 2)
        {
            HANDLE_ERROR(cudaSetDevice(1));   //Set the second graphic to calculate
        } else
        {
            printf(
                "Warning: Multiple graphic cards have been found on this machine. Please modify the function WakeInit in the file src/model/wake.cu to choose the most appropriate card.\n");
            exit(EXIT_FAILURE); // force the user to choose which card to use
        }
    } else if (count <= 0)
    {// no graphic card found
        printf(
            "Error: No graphic cards have been found on this machine. Please run this program on the machine with NVIDIA graphic cards.\n");
        exit(EXIT_FAILURE);
    }

    /* Init GPU computation */
    // Note: the number of rotor wakes & vortex filaments are fixed since simulation starts

    // traverse all rotor wakes and got total max number of markers and fila, for allocating mem
    int max_num_markers = 0;
    int max_num_rotors = 0;
    int max_num_fila = 0;
    for (unsigned int idx_robot = 0; idx_robot < robots->size(); idx_robot++)
    {// traverse all robots
        for (unsigned int idx_rotor = 0; idx_rotor < robots->at(idx_robot)->wakes.size(); idx_rotor++)
        {// traverse all rotors
            max_num_rotors++;
            for (int idx_blade = 0;
                 idx_blade < robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades; idx_blade++)
            {// traverse all blades
#if defined(WAKE_IGE)
                max_num_fila+=2;
#else
                max_num_fila++;
#endif
#if defined(WAKE_IGE)
                max_num_markers += 2*robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->capacity();
#else
                max_num_markers += robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->capacity();
#endif
            }
        }
    }

    // allocate a page-locked host memory containing all of the marker states
    HANDLE_ERROR(cudaHostAlloc((void **) &wake_markers,
                               max_num_markers * sizeof(*wake_markers), cudaHostAllocDefault));
    // allocate device memory as big as the host's
    HANDLE_ERROR(cudaMalloc((void **) &dev_wake_markers,
                            max_num_markers * sizeof(*dev_wake_markers)));
    HANDLE_ERROR(cudaMalloc((void **) &dev_wake_markers_mediate,
                            max_num_markers * sizeof(*dev_wake_markers_mediate)));

    // allocate host memory containing the index of wake_markers
    HANDLE_ERROR(cudaHostAlloc((void **) &idx_end_marker_fila,
                               max_num_fila * sizeof(*idx_end_marker_fila), cudaHostAllocDefault));
    // allocate device memory containing the indexes
    HANDLE_ERROR(cudaMalloc((void **) &dev_idx_end_marker_fila,
                            max_num_fila * sizeof(*dev_idx_end_marker_fila)));

    // allocate host memory containing wind vectors at robots' positions
    HANDLE_ERROR(cudaHostAlloc((void **) &free_stream,
                               robots->size() * sizeof(Wake_FreeStreamVector_t), cudaHostAllocDefault));
    // allocate device memory
    HANDLE_ERROR(cudaMalloc((void **) &dev_free_stream,
                            robots->size() * sizeof(Wake_FreeStreamVector_t)));

    // allocate host memory containing index to identify which robot the vortices belong to
    HANDLE_ERROR(cudaHostAlloc((void **) &idx_end_marker_robot,
                               robots->size() * sizeof(*idx_end_marker_robot), cudaHostAllocDefault));
    // allocate device memory
    HANDLE_ERROR(cudaMalloc((void **) &dev_idx_end_marker_robot,
                            robots->size() * sizeof(*dev_idx_end_marker_robot)));

#if defined(WAKE_BC_FAR)
    // allocate host memory containing the far wake states
    HANDLE_ERROR( cudaHostAlloc((void**)&far_wakes,
        max_num_rotors*sizeof(*far_wakes), cudaHostAllocDefault) );
    // allocate device memory containing the far wake states
    HANDLE_ERROR( cudaMalloc((void**)&dev_far_wakes,
        max_num_rotors*sizeof(*dev_far_wakes)) );
#endif
}

/* close GPU computation */
void WakesFinish(void)
{
    // free device memory
    HANDLE_ERROR(cudaFree(dev_idx_end_marker_fila));
    HANDLE_ERROR(cudaFree(dev_wake_markers));
    HANDLE_ERROR(cudaFree(dev_free_stream));
    HANDLE_ERROR(cudaFree(dev_idx_end_marker_robot));
#if defined(WAKE_BC_FAR)
    HANDLE_ERROR( cudaFree(dev_far_wakes) );
#endif
    // free host memory
    HANDLE_ERROR(cudaFreeHost(idx_end_marker_fila));
    HANDLE_ERROR(cudaFreeHost(wake_markers));
    HANDLE_ERROR(cudaFreeHost(free_stream));
    HANDLE_ERROR(cudaFreeHost(idx_end_marker_robot));
#if defined(WAKE_BC_FAR)
    HANDLE_ERROR( cudaFreeHost(far_wakes) );
#endif
}

/*************************************************************************/
/*************** Calculate Induced Velocity at Plume puffs ***************/
/*************************************************************************/
// these functions can be called after WakesInit

FilaState_t *plume_puffs; // on-host ...
FilaState_t *dev_plume_puffs; // on-device array containing the states of plume puffs

__global__ void CalculateIndVelatPlumePuffs(VortexMarker_t * markers, int * idx_end, int
num_fila,
int num_markers,
int tile_size,
int row_sgmts, FilaState_t
* plume,
int num_puffs
)
{
extern __shared__ VortexMarker_t
tile_markers[];
float3 pos; // position of plume puff to calculate velocity in this thread
float3 vel = {0.0f, 0.0f, 0.0f}; // velocity of this marker
int idx, i;
int row = threadIdx.x % tile_size + blockIdx.x * tile_size; // get row number (the index of puff to be evaluated)

// get the plume fila (puff) which the velocity to be calculated
if (row<num_puffs)
{
pos.
x = plume[row].pos[0];
pos.
y = plume[row].pos[1];
pos.
z = plume[row].pos[2];
}

// compute each tile
i = 0;
while (true) // every thread can enter this loop
{
idx = threadIdx.x + blockDim.x * i;
if (idx<num_markers)
tile_markers[threadIdx.x] = markers[idx]; // copy markers to shared mem for tile calculation
__syncthreads(); // make sure the shared mem has been loaded
if (row<num_puffs)
vel = tile_calculation_vel_markers(pos, vel, idx / tile_size, tile_size, row_sgmts, markers, idx_end, num_fila,
                                   num_markers);

__syncthreads(); // make sure every thread has done the calc of this tile

if (blockDim.
x *(i
+1) < num_markers)
i++;
else
break;
}

// save the result of this segment to the end of the shared mem
//  at present the shared memory is of no use for computation, so it can be used to temporarily store the segment results
if (row<num_puffs) {
tile_markers[threadIdx.x].vel[0] = vel.
x;
tile_markers[threadIdx.x].vel[1] = vel.
y;
tile_markers[threadIdx.x].vel[2] = vel.
z;
}

__syncthreads(); // make sure the segment results are stored

if (threadIdx.x<tile_size && row < num_puffs) {
// sum the velocities computed by multiple (num of row_sgms) threads
vel.
x = 0.0f;
vel.
y = 0.0f;
vel.
z = 0.0f;
for (
i = 0;
i<row_sgmts;
i++) {
vel.x += tile_markers[threadIdx.x%tile_size+
i *tile_size
].vel[0];
vel.y += tile_markers[threadIdx.x%tile_size+
i *tile_size
].vel[1];
vel.z += tile_markers[threadIdx.x%tile_size+
i *tile_size
].vel[2];
}
// Save the result in global memory for the integration step.
plume[row].vel[0] = vel.
x;
plume[row].vel[1] = vel.
y;
plume[row].vel[2] = vel.
z;
}
}

void WakesIndVelatPlumePuffsUpdate(std::vector<Robot *> *robots, std::vector<FilaState_t> *plume)
{
    unsigned int idx_robot, idx_rotor;
    int idx_blade;
    int addr_cp_markers = 0, num_blade = 0;
    // Step 1: collect all vortex markers & puffs to one memory, respectively, for GPU computing
    //  the markers are placed contiguously, fila to fila
    for (idx_robot = 0; idx_robot < robots->size(); idx_robot++)
    {
        for (idx_rotor = 0; idx_rotor < robots->at(idx_robot)->wakes.size(); idx_rotor++)
        {
            for (idx_blade = 0;
                 idx_blade < robots->at(idx_robot)->wakes.at(idx_rotor)->rotor_state.frame.n_blades; idx_blade++)
            {
                std::copy(robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->begin(),
                          robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->end(),
                          &wake_markers[addr_cp_markers]);
                addr_cp_markers += robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->size();
                idx_end_marker_fila[num_blade] = addr_cp_markers - 1; // the address of the last element, hence -1
                num_blade++;
            }
        }
    }// traversed all rotor wakes and got total number of markers
    //  collect puffs to a memory
    std::copy(plume->begin(), plume->end(), &plume_puffs[0]);

    // Step 2: copy array wake_markers, idx_wake_markers, plume_puffs to GPU's version
    HANDLE_ERROR(cudaMemcpy(dev_wake_markers, wake_markers,
                            addr_cp_markers * sizeof(VortexMarker_t), cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(dev_idx_end_marker_fila, idx_end_marker_fila,
                            num_blade * sizeof(int), cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(dev_plume_puffs, plume_puffs,
                            plume->size() * sizeof(FilaState_t), cudaMemcpyHostToDevice));

    // Step 3: compute induced velocity of plume puffs
    //  determine threads per block and blocks number, at present addr_cp_markers contains total num of markers
    int p, q, threads, blocks;
    p = prop.warpSize; // tile size
    q = 4; // number of segments of a row
    threads = std::min(p * q, prop.maxThreadsPerBlock);
    //  launch gpu computing
    cudaError_t err;
    //  <1> calculate velocity of Lagrangian markers, n-1 state
    blocks = (plume->size() + p - 1) / p;
    //  Note: here omitted checks for max number of blocks, since in RAO problem the vortex markers
    //        rarely exceeds 65535*threads.
    CalculateIndVelatPlumePuffs << < blocks, threads, threads * sizeof(VortexMarker_t) >> >
                                                      (dev_wake_markers, dev_idx_end_marker_fila, num_blade, addr_cp_markers, p, q, dev_plume_puffs, plume->size());
    err = cudaGetLastError();
    if (err != cudaSuccess)
        printf("Error: %s\n", cudaGetErrorString(err));

    // Step 4: distribute puffs to host and give back
    HANDLE_ERROR(cudaMemcpy(plume_puffs, dev_plume_puffs,
                            plume->size() * sizeof(FilaState_t), cudaMemcpyDeviceToHost));
    std::copy(&plume_puffs[0], &plume_puffs[plume->size()], plume->data());
}

void WakesIndVelatPlumePuffsInit(std::vector<Robot *> *robots, std::vector<FilaState_t> *plume)
{
    // allocate a page-locked host memory containing all of the plume puffs states
    HANDLE_ERROR(cudaHostAlloc((void **) &plume_puffs,
                               MAX_NUM_PUFFS * sizeof(*plume_puffs), cudaHostAllocDefault));
    // allocate device memory as big as the host's
    HANDLE_ERROR(cudaMalloc((void **) &dev_plume_puffs,
                            MAX_NUM_PUFFS * sizeof(*dev_plume_puffs)));
}

/* End of file wake.cu */
