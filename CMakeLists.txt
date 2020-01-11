#====================================================
# CMakeLists configuration file for RAOS project
# this file aims to build the RAOS project
#
# Author: tao.jing @ tju
# Date:   2014-11-24 Create this file (previous solplot3d)
#         2016-02-01 Re-create this file
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

#====================================================
# ================ User can Access ==================
#====================================================
# RAOS features, comment out if not need the Aero-olfaction simulation of propellers
set(RAOS_FEATURE_WAKES "enable")

#====================================================
# ============= Don't touch the following ===========
#====================================================
# Project name
set(PRJ_NAME RAOS)
# Debug version
set(CMAKE_BUILD_TYPE Debug)
#   Color map
string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(ColourBold  "${Esc}[1m")
set(Red         "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")
set(Blue        "${Esc}[34m")
set(Magenta     "${Esc}[35m")
set(Cyan        "${Esc}[36m")
set(White       "${Esc}[37m")
set(BoldRed     "${Esc}[1;31m")
set(BoldGreen   "${Esc}[1;32m")
set(BoldYellow  "${Esc}[1;33m")
set(BoldBlue    "${Esc}[1;34m")
set(BoldMagenta "${Esc}[1;35m")
set(BoldCyan    "${Esc}[1;36m")
set(BoldWhite   "${Esc}[1;37m")
#   Detect platforms
if(APPLE)
    message("${BoldRed}Warning: ${ColourReset}MacOS is no longer supported since RAOS v2.0, aborting compilation.")
    return()
elseif(UNIX AND NOT APPLE)
    message("${BoldGreen}Compile on UNIX.${ColourReset}")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    #   compiler, use clang-3.8 instead of gcc-6 because cuda 8.0 does not support gcc-6 (2017-07-18) -std=c++11
    set (TOOL_DIR /usr)
    #set (CMAKE_C_COMPILER               "${TOOL_DIR}/bin/gcc-4.9")
    SET (CMAKE_C_COMPILER               "/usr/bin/gcc")
    SET (CMAKE_C_FLAGS                  "-Wall -std=c11 -fPIC -Wno-unused-but-set-variable -Wno-unused-variable -Wno-comment")
    SET (CMAKE_C_FLAGS_DEBUG            "-g")
    SET (CMAKE_C_FLAGS_MINSIZEREL       "-Os -DNDEBUG")
    SET (CMAKE_C_FLAGS_RELEASE          "-O4 -DNDEBUG")
    SET (CMAKE_C_FLAGS_RELWITHDEBINFO   "-O2 -g")
    #set (CMAKE_CXX_COMPILER             "${TOOL_DIR}/bin/g++-4.9")
    SET (CMAKE_CXX_COMPILER             "/usr/bin/g++")
    SET (CMAKE_CXX_FLAGS                "-Wall -std=c++11 -fPIC -std=gnu++11 -Wno-reorder -Wno-unused-but-set-variable -Wno-unused-variable -Wno-comment")
    SET (CMAKE_CXX_FLAGS_DEBUG          "-g")
    SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
    SET (CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
    SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
elseif(WIN32)
    message("${BoldGreen}Compile on Windows.${ColourReset}")
else()
    message("${BoldRed}Error: Platform not recognized.${ColourReset} RAOS only support Linux & Windows.")
    return()
endif()
#   manage definitions according to features
if (RAOS_FEATURE_WAKES)
    add_definitions(-DRAOS_FEATURE_WAKES)
endif()
#   random lib uses c++11 standard
set (CMAKE_CXX_STANDARD 11)
#   begin creating project
project(${PRJ_NAME})

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#============== Find Dependencies ================
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
# print out paths in order to check if goes wrong
message("Project Source Dir: \"${PROJECT_SOURCE_DIR}\"")
message("Project Binary Dir: \"${PROJECT_BINARY_DIR}\"")
#---- External FLTK related settings ----
# Set FLTK_DIR to the directory which contains FLTKConfig.cmake file
set(FLTK_DIR "${PROJECT_SOURCE_DIR}/3rdparty/fltk-1.3.3/build")

# "NO_MODULE" tells CMake to search only for the FLTKConfig file, not using the FindFLTK.cmake supplied with CMake
find_package(FLTK REQUIRED NO_MODULE)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED CONFIG)

# gl.h and glu.h
add_definitions(-DGL_HEADER="FL/gl.h" -DGLU_HEADER="FL/glu.h" -DGLUT_HEADER="FL/glut.H")
#---- External CUDA related settings ----
if(RAOS_FEATURE_WAKES)
    find_package(CUDA REQUIRED)
endif()
# nvcc flags
# Note: Modify the arch and code according to your GPU's compute capability
# Ex. RTX2060 - 7.5, GTX750M - 5.0
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode; arch=compute_75,code=sm_75;-use_fast_math;-std=c++11;)
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -ccbin gcc;)
#---- External HDF5 related, for data recording
find_package(HDF5 COMPONENTS C CXX REQUIRED)
find_package(HighFive 2.1 REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS})

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#============== Build the project ================
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#---- source files include path settings ----
# add source directory into include path
#   add root dir to search for header files of RAOS projects
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/model/)
include_directories(${PROJECT_SOURCE_DIR}/src/model/sensors/)
include_directories(${PROJECT_SOURCE_DIR}/src/commom/)
include_directories(${PROJECT_SOURCE_DIR}/src/commom/math/)
#   add cblas dir to search for cblas.h header for linear algebra
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/blas/CBLAS/include)
#   add lapack dir to search for lapacke.h header for solving linear equations
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/lapack-3.7.0/build/include)
#   add ziggurat source file to generate normal distribution random numbers
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/ziggurat)
# add assimp lib dir
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/assimp-3.3.1/include)
# add glfw include dir
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/glfw-3.3/include)
# add glm include dir
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
# add SOIL include dir
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/SOIL/)

#---- library files path settings ----
# add library dir into lib path
# add blas and cblas lib dir
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/blas/BLAS-3.6.0)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/blas/CBLAS/lib)
# add lapack lib dir
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/lapack-3.7.0/build/lib)
# add assimp lib dir
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/assimp-3.3.1/lib)
include_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/lib)

# for -lgfortran
link_directories(/usr/lib/gcc/x86_64-linux-gnu/5/)

# for -lglfw
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/glfw-3.3/build/src/)
# for -lSOIL
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/SOIL/lib)

#---- start compiling ----
set(LIB_UI_NAME ui)
set(LIB_MODEL_NAME model) # models
set(LIB_METHOD_NAME method) # methods
# make a library from quadrotor model files
if(RAOS_FEATURE_WAKES)
    cuda_add_library(${LIB_MODEL_NAME}
        src/model/SimModel.cxx
        src/model/plume.cxx
        src/model/wake_rotor.cxx src/model/wake.cu
        src/model/robot.cxx src/model/quadrotor.cxx src/model/ground_robot.cpp
        src/model/sensors/sensor_manager.cpp
        src/model/sensors/mox_sensor.cpp src/model/sensors/pid_sensor.cpp
        src/model/sensors/tdlas_sensor.cpp
        3rdparty/ziggurat/ziggurat.cpp
	src/model/windvector.cxx)
else()
    add_library(${LIB_MODEL_NAME}
        src/model/SimModel.cxx
        src/model/plume.cxx
        src/model/robot.cxx src/model/quadrotor.cxx src/model/ground_robot.cpp
        src/model/sensors/mox_sensor.cpp src/model/sensors/pid_sensor.cpp
        src/model/sensors/tdlas_sensor.cpp
        3rdparty/ziggurat/ziggurat.cpp src/model/windvector.cxx)
endif()

# make a library from ui files
if(RAOS_FEATURE_WAKES)
    add_library(${LIB_UI_NAME} src/ui/SimUI.cxx src/ui/SimView.cxx src/ui/agv.cxx
        src/ui/draw/DrawScene.cxx src/ui/draw/materials.cxx
        src/ui/draw/draw_robots.cxx src/ui/draw/draw_qr.cxx
        src/ui/draw/draw_arena.cxx src/ui/draw/draw_plume.cxx
        src/ui/draw/draw_wake.cxx src/ui/draw/draw_ref_point.cxx
	    src/ui/draw/cad/loadmodel.cxx src/ui/draw/draw_windvector.cxx
        src/ui/draw/draw_ground_robot.cpp
            )
else()
    add_library(${LIB_UI_NAME} src/ui/SimUI.cxx src/ui/SimView.cxx src/ui/agv.cxx
        src/ui/draw/DrawScene.cxx src/ui/draw/materials.cxx
        src/ui/draw/draw_robots.cxx src/ui/draw/draw_qr.cxx
        src/ui/draw/draw_arena.cxx src/ui/draw/draw_plume.cxx
        src/ui/draw/draw_ref_point.cxx src/ui/draw/cad/loadmodel.cxx
	    src/ui/draw/draw_windvector.cxx
        src/ui/draw/draw_ground_robot.cpp
            )
endif()

# make a library from method files
add_library(${LIB_METHOD_NAME} src/method/method.cxx
        src/method/gas_dist_mapping.cxx src/method/hover_measure.cxx
        )
# compile main file
add_executable(${PRJ_NAME} src/SimMain.cxx src/SimThread.cxx src/SimConfig.cxx src/SimRecord.cxx
        src/model/environment.cxx
        src/common/math/rotation.cxx src/common/math/linalg.cxx
        src/common/math/raos_math.cpp)
add_dependencies(${PRJ_NAME} ${LIB_MODEL_NAME} ${LIB_UI_NAME} ${LIB_METHOD_NAME})

#---- start linking ----
# Note: the former line depends on the next line
target_link_libraries(${LIB_UI_NAME} GLEW::GLEW)
# link model library created above
target_link_libraries(${PRJ_NAME} ${LIB_MODEL_NAME})
# link GUI library created above
target_link_libraries(${PRJ_NAME} ${LIB_UI_NAME})
# link method library created above
target_link_libraries(${PRJ_NAME} ${LIB_METHOD_NAME})
# link external FLTK library
target_link_libraries(${PRJ_NAME} fltk)
# link external OpenGL library
target_link_libraries(${PRJ_NAME} fltk_gl)
# link external image library
target_link_libraries(${PRJ_NAME} fltk_images)
# link external lapack library
target_link_libraries(${PRJ_NAME} lapacke)
target_link_libraries(${PRJ_NAME} lapack)
# link external blas library
target_link_libraries(${PRJ_NAME} cblas)
target_link_libraries(${PRJ_NAME} blas)
target_link_libraries(${PRJ_NAME} gfortran)
# link extern hdf5 library
target_link_libraries(${PRJ_NAME} ${HDF5_LIBRARIES})

#target_link_libraries(${PRJ_NAME} GLEW)
target_link_libraries(${PRJ_NAME} assimp)
target_link_libraries(${PRJ_NAME} glfw)
target_link_libraries(${PRJ_NAME} SOIL)

target_link_libraries(${PRJ_NAME} HighFive)
