RAOS -- Simulation Framework for Robot Active Olfaction
=======

Copyright @ TJU-IRAS. Only for academic research.



[toc]

Dependencies:
----
(1) OpenGL library<br>
(2) boost<br>
(3) make, cmake, gcc, g++, gfortran<br>
(4) for rotor wake calculation:<br>
    cuda toolkit, and a graphic card<br>

Note: For the following libs, we provide pre-built lib files in 3rdparty, which may be dependent on your system configuration.

If any of these libs is not working properlly, you can download the source of the library and compile it on your own PC. And also change the cmake file configuration.

(5) glm ([OpenGL Mathematics](http://glm.g-truc.net/)): https://github.com/g-truc/glm, https://sourceforge.net/projects/ogl-math/

Note: For the following libs, the install.sh file will install the environment automatically.

If any problems occurs, you may need to debug the install process according to the prompted infomation.

(6) SOIL (Simple OpenGL Image Library) : http://www.lonesock.net/soil.html

(7) GLFW: https://www.glfw.org/

(8) GLEW (The OpenGL Extension Wrangler Library): http://glew.sourceforge.net/

Install:
----

Run the install.sh to enable some 3rdparty. You may need chmod a+x install.sh.

Compilation:
----

General CMake compilation method (Review some libraries path configuration):

mkdir build & cd build

cmake ..

Help:
----
(1) How to check GPU compute compability ?

Refer to https://developer.nvidia.com/cuda-gpus#compute;



## OpenGL

### 1. How to change NASA airplane to OpenGL coordinates

```c
glX = state->pos[0];
glZ = -state->pos[1];
glY = state->pos[2];
```





## Wake Simulation

### RotorWake

The class for wake computation of **one rotor**.



#### class member

(1) <span style="color:red">**max_markers**</span>: max Lagrangian markers of a vortex filament, 10.0 degree is an arbitrary number, see Fig.2-3

The number of max_markers is calculated in **RotorWake::init**

`typedef struct
{
    float rounds; // number of revolutions (number of vortex rings to maintain) 
    float dpsi;   // delta azimuth angle to release/maintain marker
} RotorWakeConfig_t`

`max_markers = ceil(360.0 * config.rounds /  config.dpsi);`

(2) <span style="color:red">**wake_state**</span>: filament markers container handlers

`std::vector<VortexMarker_t> **wake_state`

wake_state[i] is the pointer of a std::vector, which contains the markers of a whole filament.

One vectors for each blade.  (If consider in-ground-effect, two vectors for each blade.)

Allocate memory for wake_state in **RotorWake::init**

No in-ground-effect:

```c
wake_state = (std::vector<VortexMarker_t> **) malloc(
        rotor_state.frame.n_blades * sizeof(std::vector<VortexMarker_t> *));
```

With in-ground-effect:

```c
    wake_state = (std::vector<VortexMarker_t>**)malloc(rotor_state.frame.n_blades*2*sizeof(std::vector<VortexMarker_t>*));
```

Use std::vector::reverse to allocate memory space for vortex markers.



```c
typedef struct
{
    float pos[3]; // 3D position of this marker
    float vel[3]; // 3D velocity of this marker
    //Gamma: vortex strength
    float Gamma;  // circulation of i-th segment (i-th marker to (i+1)-th marker)
    float r;      // vortex core radius
    float r_init; // r_0, initial radius of the tip vortex
} VortexMarker_t;
```



#### Wake geometry initialization

**RotorWake::init_wake_geometry** ---> Eqs (2-15),  Landgrebe prescribed wake model

called in RotorWake::init()



#### Wake update

SimModel_update ---> WakesUpdate   (in wake.cu)  ---> RotorWake::maintain

**<span style='color:red'>1. RotorWake::maintain</span>**

**Called by WakesUpdates()**

① marker_maintain_type == one_by_one

If far wake BC is not considered, call **RotorWake::maintain_markers**

② marker_maintain_type == turn_by_turn

Generate a turn of vortex, markers_per_turn is the number of vortex of a blade;

```c
int markers_per_turn = ceil(360.0 / config.dpsi);
```



<span style="color:red">**2. RotorWake::maintain_markers**</span>

**Called by RotorWake::maintain()**

Release new marker and remove old markers

call **RotorWake::marker_release**

Calculation of Gamma, the vortex strength, Eq.(2-11)

```c
// calculate the tip vortex circulation: Gamma
    new_marker.Gamma = -(rotor_state.direction) * 2.0f * rotor_state.thrust / rotor_state.frame.n_blades / (rotor_state.frame.radius * rotor_state.frame.radius) / 1.185f / rotor_state.Omega; // tip vortex circulation
```

Calculate the position of this marker according to the rotor's position and attitude

Push back the marker



<span style="color:red">**3. WakesUpdate**</span>

**① Phase 1**: update velocity & position of markers

collect all vortex markers to a memory, for GPU computing  the markers are placed contiguously, fila to fila

Markers

```c
//array containing all of the markers, for convenience mem copying with GPU
VortexMarker_t *wake_markers;     // on-host
VortexMarker_t *dev_wake_markers; // on-device array ...
```

 **wake_markers** is the container storing all robots' wake markers

markers' index

```c
//array containing index of endpoint of all vortex filaments, for parallel computing
int *idx_end_marker_fila;      // on-host 
int *dev_idx_end_marker_robot; // on-device
```

Memory allocation

```c
**max_num_markers** += robots->at(idx_robot)->wakes.at(idx_rotor)->wake_state[idx_blade]->capacity(); // **max_num_markers**
//Allocate a page-locked host memory for wake_markers
cudaHostAlloc((void* * )&wake_markers, **max_num_markers** * sizeof(*wake_markers), ...)
// allocate device memory as big as the host's
cudaMalloc((void **) &dev_wake_markers, max_num_markers * sizeof(*dev_wake_markers))
```





Free stream

```c
typedef struct
{
    float v[3];
} Wake_FreeStreamVector_t;
//Free stream vector at robots' position
Wake_FreeStreamVector_t *free_stream;	  // on host
Wake_FreeStreamVector_t *dev_free_stream; // on-device 

```



**② Phase 2**: memory convertion from Host to Device

copy array wake_markers & idx_wake_markers to GPU's dev_wake_markers

```c
HANDLE_ERROR(cudaMemcpy(dev_wake_markers, wake_markers, addr_cp_markers * sizeof(VortexMarker_t), cudaMemcpyHostToDevice));
HANDLE_ERROR(cudaMemcpy(dev_idx_end_marker_fila, idx_end_marker_fila, num_blade * sizeof(int), cudaMemcpyHostToDevice));
...
```



**③ Phase 3**: parallel computing

= determine threads per block and blocks number, at present addr_cp_markers contains total num of markers

Invocation of kernel functions, but in the legacy way, for current version the execution configuration is set with grid and block with dim3 type.

```
CalculateVelofMarkers << < blocks, threads, (threads) * sizeof(VortexMarker_t) >> >
		(dev_wake_markers, dev_idx_end_marker_fila, num_blade, addr_cp_markers, p, q, dev_free_stream, dev_idx_end_marker_robot, robots->size());

```

a. calculate velocity of Lagrangian markers, n-1 state

**CalculateVelofMarkers** ( __ global __ ) calculate velocities of markers, running on gpu.



### FilaModel

**<span style='color:red'>1. RotorWake::maintain</span>**



**<span style='color:red'>2. FilaModel::update</span>**

**Step 1** update positions of filament

Displacement = Velocity * Time Interval

```c
// here the integrated velocity contains three parts:
//  wind[*]: the free stream wind field
//  vm_*: the random noise wind speed
//  state.at(i).vel[*]: the induced velocity (induced by wakes, updated in WakesIndVelatPlumePuffsUpdate)
state.at(i).pos[0] += (wind[0] + vm_x + state.at(i).vel[0]) * sim_state->dt;
state.at(i).pos[1] += (wind[1] + vm_y + state.at(i).vel[1]) * sim_state->dt;
state.at(i).pos[2] += (wind[2] + vm_z + state.at(i).vel[2]) * sim_state->dt;
if (state.at(i).pos[2] < 0.0)
    state.at(i).pos[2] = -state.at(i).pos[2];
```



## 机器人本体仿真



### 旋翼无人机仿真



#### 四旋翼无人机模型

实现类：QRdynamic

相关类图：

![](\docs\pics\QR-classes.png)



### 控制模型

目前支持的控制模型包括PID和ADRC两种经典控制模型。

```c
typedef enum
{
    QRcontroller_PID,
    QRcontroller_ADRC,
} QRcontroller_name_t;
```



