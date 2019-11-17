####==================================================
#### Compile, Setup and install script of RAOS
####                For Linux Ubuntu
####                tao.jing @TJU-IRAS
####==================================================

#!/bin/sh

####=================================
#### Uninstall all build files
####=================================

# Get platform type
if [ "$(uname)" = "Darwin" ]; then
    # Do something under Mac OS X platform
    SYSTEM="APPLE"
elif [ "$(expr substr $(uname -s) 1 5)" = "Linux" ]; then
    # Do something under GNU/Linux platform
    SYSTEM="LINUX"
elif [ "$(expr substr $(uname -s) 1 10)" = "MINGW32_NT" ]; then
    # Do something under Windows NT platform
    SYSTEM="WIN32"
fi

# Get project top directory abs path
prjtop=$(cd "$(dirname "$0")"; pwd)
echo "Absolute path of project top directory is: "$prjtop
sleep 2

echo "Start cleaning ..."
sleep 1
# clear 3rd party software builds
if [ ${SYSTEM} = "LINUX" ]; then
    echo "Cleaning FLTK..."
    cd $prjtop/3rdparty
    rm -rf fltk-1.3.3
fi

if [ ${SYSTEM} = "LINUX" ]; then
    echo "Cleaning glew..."
    cd $prjtop/3rdparty
    rm -rf glew-2.1.0
fi

if [ ${SYSTEM} = "LINUX" ]; then
    echo "Cleaning glfw..."
    cd $prjtop/3rdparty
    rm -rf glfw-3.3
fi

if [ ${SYSTEM} = "LINUX" ]; then
    echo "Cleaning SOIL..."
    cd $prjtop/3rdparty/SOIL/lib
    rm -rf $prjtop/3rdparty/SOIL/lib/*
    cd $prjtop/3rdparty/SOIL/projects/makefile
    rm -rf obj
fi

echo "RAOS uninstall completed..."
